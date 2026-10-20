"""RTLS host: TCP ingest of tag range packets + web floor view.

Two listeners:
  - TCP :7007  — the tag connects here and streams newline-delimited JSON
    (one object per ranging round, see apps/tag). Each packet is trilaterated.
  - HTTP :8180 — "/" serves a canvas floor view fed by "/events" (SSE);
    "/state" returns the latest solution as JSON.

Stdlib-only besides numpy/scipy (no matplotlib): runs headless, view in a
browser at http://<host>:8180/.
"""

import argparse
import json
import queue
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np

from host import solver

FLOOR_PAGE = """<!DOCTYPE html>
<html><head><title>UWB RTLS floor view</title><style>
  body { font-family: sans-serif; background: #111; color: #ddd; margin: 20px; }
  canvas { background: #1c2230; border: 1px solid #445; }
  #info { margin: 8px 0; font-size: 14px; }
</style></head><body>
<h3>UWB RTLS floor view</h3>
<div id="info">waiting for data&hellip;</div>
<canvas id="floor"></canvas>
<script>
const canvas = document.getElementById('floor'), ctx = canvas.getContext('2d');
const info = document.getElementById('info');
let cfg = null, trail = [];
function px([x, y]) {  // room cm -> canvas px, y up
  const s = Math.min(canvas.width / cfg.room[0], canvas.height / cfg.room[1]);
  return [x * s, canvas.height - y * s];
}
function draw(state) {
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  cfg.anchors.forEach((a, i) => {
    const [x, y] = px(a);
    ctx.fillStyle = '#6cf';
    ctx.beginPath();
    ctx.moveTo(x, y - 7); ctx.lineTo(x - 6, y + 5); ctx.lineTo(x + 6, y + 5);
    ctx.fill();
    ctx.fillText('A' + (i + 1), x + 8, y);
  });
  ctx.strokeStyle = '#3a5';
  ctx.beginPath();
  trail.forEach((p, i) => { const [x, y] = px(p); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); });
  ctx.stroke();
  if (state.position) {
    const [x, y] = px(state.position);
    ctx.fillStyle = '#fa4';
    ctx.beginPath(); ctx.arc(x, y, 6, 0, 7); ctx.fill();
    info.textContent = 'tag ' + state.tag_id + ' @ (' +
      state.position[0].toFixed(1) + ', ' + state.position[1].toFixed(1) +
      ') cm — ranges [' + state.distances.map(d => d.toFixed(0)).join(', ') + ']';
  }
}
const es = new EventSource('/events');
es.onmessage = (e) => {
  const state = JSON.parse(e.data);
  if (!cfg) {
    cfg = state.config;
    canvas.width = Math.min(600, cfg.room[0] * 1.2);
    canvas.height = canvas.width * cfg.room[1] / cfg.room[0];
  }
  if (state.position) {
    trail.push(state.position);
    if (trail.length > 300) trail.shift();
  }
  draw(state);
};
</script></body></html>"""


class RTLS:
    """Shared state: latest solution + fan-out queues for SSE clients."""

    def __init__(self, anchors, room):
        self.anchors = np.asarray(anchors, float)
        self.room = tuple(room)
        self.lock = threading.Lock()
        self.latest = None
        self.subscribers = []

    def config(self):
        return {"anchors": self.anchors.tolist(), "room": list(self.room)}

    def ingest_line(self, line):
        """Parse one tag JSON line; solve; publish. Returns the state or None."""
        try:
            packet = json.loads(line)
            entries = packet["anchors"]
            distances = [float(entries["A%d" % (i + 1)]["distance"])
                         for i in range(len(self.anchors))]
        except (ValueError, KeyError):
            return None
        pos = solver.trilaterate(distances, self.anchors)
        state = {
            "tag_id": packet.get("tag_id"),
            "distances": distances,
            "position": None if pos is None else [float(pos[0]), float(pos[1])],
            "time": time.time(),
            "config": self.config(),
        }
        with self.lock:
            self.latest = state
            for q in self.subscribers:
                q.put(state)
        return state

    def subscribe(self):
        q = queue.Queue()
        with self.lock:
            self.subscribers.append(q)
            if self.latest:
                q.put(self.latest)
        return q

    def unsubscribe(self, q):
        with self.lock:
            if q in self.subscribers:
                self.subscribers.remove(q)


def tcp_ingest(rtls, sock):
    """Accept tag connections (one at a time) and feed lines to the solver."""
    while True:
        try:
            conn, addr = sock.accept()
        except OSError:  # socket closed on shutdown
            return
        print("[tcp] tag connected from %s:%d" % addr)
        with conn:
            buf = b""
            while True:
                try:
                    data = conn.recv(4096)
                except OSError:
                    break
                if not data:
                    break
                buf += data
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    if line.strip():
                        rtls.ingest_line(line.decode("utf-8", "replace"))
        print("[tcp] tag disconnected")


class Handler(BaseHTTPRequestHandler):
    rtls = None  # set by serve()

    def do_GET(self):
        if self.path == "/":
            body = FLOOR_PAGE.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path == "/state":
            with self.rtls.lock:
                state = self.rtls.latest or {"position": None,
                                             "config": self.rtls.config()}
            body = json.dumps(state).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path == "/events":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            q = self.rtls.subscribe()
            try:
                while True:
                    try:
                        state = q.get(timeout=15)
                        payload = "data: %s\n\n" % json.dumps(state)
                    except queue.Empty:
                        payload = ": keepalive\n\n"
                    self.wfile.write(payload.encode())
                    self.wfile.flush()
            except OSError:
                pass  # client went away
            finally:
                self.rtls.unsubscribe(q)
        else:
            self.send_error(404)

    def log_message(self, fmt, *args):  # quiet the per-request spam
        pass


def serve(anchors, room, tcp_port=7007, http_port=8180):
    """Start both listeners; returns (rtls, tcp_sock, httpd) for tests."""
    rtls = RTLS(anchors, room)
    tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    tcp_sock.bind(("0.0.0.0", tcp_port))
    tcp_sock.listen()
    threading.Thread(target=tcp_ingest, args=(rtls, tcp_sock), daemon=True).start()

    Handler.rtls = rtls
    httpd = ThreadingHTTPServer(("0.0.0.0", http_port), Handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    return rtls, tcp_sock, httpd


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--tcp-port", type=int, default=7007)
    ap.add_argument("--http-port", type=int, default=8180)
    ap.add_argument("--anchors", type=json.loads,
                    default=solver.DEFAULT_ANCHORS.tolist(),
                    help='anchor positions cm, e.g. "[[15,5],[290,5],[165,625]]"')
    ap.add_argument("--room", type=float, nargs=2,
                    default=list(solver.DEFAULT_ROOM), metavar=("W", "H"))
    args = ap.parse_args()
    _, tcp_sock, _ = serve(args.anchors, args.room, args.tcp_port, args.http_port)
    print("TCP ingest on :%d — point the tag's UWB_HOST_IP here" % args.tcp_port)
    print("Floor view on http://0.0.0.0:%d/" % args.http_port)
    try:
        while True:
            time.sleep(3600)
    except KeyboardInterrupt:
        tcp_sock.close()


if __name__ == "__main__":
    main()
