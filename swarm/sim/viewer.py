"""Browser visualization for the swarm sim: runs a scenario in real time and
streams frames over SSE to a canvas top-down view.

  bazel run //swarm/sim:viewer -- --widgets 5 --gps 0 --surveyed 3

Open http://<host>:8081/ — filled dots are estimates (with 2-sigma
covariance ellipses), hollow rings are ground truth, lines are the ranging
links active this frame. Surveyed widgets are green, GPS carriers blue,
IMU+UWB-only widgets orange.
"""

import argparse
import json
import queue
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from swarm.sim import world

PAGE = """<!DOCTYPE html>
<html><head><title>Station-free swarm view</title><style>
  body { font-family: sans-serif; background: #111; color: #ddd; margin: 20px; }
  canvas { background: #1c2230; border: 1px solid #445; }
  #info { margin: 8px 0; font-size: 13px; white-space: pre; }
</style></head><body>
<h3>Station-free swarm — truth vs on-device estimates</h3>
<label><input type="checkbox" id="relview"> relative view (remove the
unobservable common offset — use for anchorless swarms)</label>
<div id="info">waiting for data&hellip;</div>
<canvas id="view" width="720" height="720"></canvas>
<script>
const canvas = document.getElementById('view'), ctx = canvas.getContext('2d');
const info = document.getElementById('info');
const ARENA = %(arena)f, PAD = 30;
const trails = {};
function px(p) {
  const s = (canvas.width - 2 * PAD) / ARENA;
  return [PAD + p[0] * s, canvas.height - PAD - p[1] * s];
}
function colorOf(w) {
  return w.surveyed ? '#5c5' : (w.gps ? '#6af' : '#fa4');
}
function drawEllipse(p, cov, color) {
  // 2-sigma ellipse from the XY covariance block.
  const a = cov[0][0], b = cov[0][1], c = cov[1][1];
  const tr = a + c, det = a * c - b * b;
  const l1 = tr / 2 + Math.sqrt(Math.max(tr * tr / 4 - det, 0));
  const l2 = tr / 2 - Math.sqrt(Math.max(tr * tr / 4 - det, 0));
  const ang = Math.abs(b) < 1e-9 ? 0 : Math.atan2(l1 - a, b);
  const s = (canvas.width - 2 * PAD) / ARENA;
  ctx.strokeStyle = color; ctx.globalAlpha = 0.5;
  ctx.beginPath();
  ctx.ellipse(p[0], p[1], 2 * Math.sqrt(Math.max(l1, 0)) * s,
              2 * Math.sqrt(Math.max(l2, 0)) * s, -ang, 0, 7);
  ctx.stroke(); ctx.globalAlpha = 1;
}
function draw(f) {
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  if (document.getElementById('relview').checked) {
    // Recenter the estimated mesh on the true centroid: shows the relative
    // solution when the absolute gauge is unobservable.
    const n = f.widgets.length;
    const off = [0, 1, 2].map(k =>
      f.widgets.reduce((s, w) => s + w.est_p[k] - w.true_p[k], 0) / n);
    f = {...f, widgets: f.widgets.map(w =>
      ({...w, est_p: w.est_p.map((x, k) => x - off[k])}))};
  }
  const byId = {};
  f.widgets.forEach(w => byId[w.id] = w);
  ctx.strokeStyle = '#456'; ctx.lineWidth = 1;
  f.links.forEach(([i, j]) => {
    const a = px(byId[i].est_p), b = px(byId[j].est_p);
    ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke();
  });
  let lines = 't=' + f.t.toFixed(1) + 's\\n';
  f.widgets.forEach(w => {
    const col = colorOf(w);
    if (!trails[w.id]) trails[w.id] = [];
    trails[w.id].push(w.est_p);
    if (trails[w.id].length > 200) trails[w.id].shift();
    ctx.strokeStyle = col; ctx.globalAlpha = 0.25;
    ctx.beginPath();
    trails[w.id].forEach((p, i) => {
      const q = px(p); i ? ctx.lineTo(q[0], q[1]) : ctx.moveTo(q[0], q[1]);
    });
    ctx.stroke(); ctx.globalAlpha = 1;
    const tp = px(w.true_p), ep = px(w.est_p);
    ctx.strokeStyle = col;                       // truth: hollow ring + heading
    ctx.beginPath(); ctx.arc(tp[0], tp[1], 7, 0, 7); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(tp[0], tp[1]);
    ctx.lineTo(tp[0] + 12 * Math.cos(w.true_yaw), tp[1] - 12 * Math.sin(w.true_yaw));
    ctx.stroke();
    ctx.fillStyle = col;                         // estimate: filled dot + heading
    ctx.beginPath(); ctx.arc(ep[0], ep[1], 4, 0, 7); ctx.fill();
    ctx.beginPath(); ctx.moveTo(ep[0], ep[1]);
    ctx.lineTo(ep[0] + 10 * Math.cos(w.est_yaw), ep[1] - 10 * Math.sin(w.est_yaw));
    ctx.stroke();
    drawEllipse(ep, w.cov_xy, col);
    ctx.fillText('' + w.id, ep[0] + 8, ep[1] - 6);
    lines += 'w' + w.id + (w.surveyed ? ' [survey]' : (w.gps ? ' [gps]' : '  ')) +
             '  err ' + w.err.toFixed(2).padStart(6) + ' m' +
             '  sigma ' + Math.sqrt(w.pos_var).toFixed(2).padStart(6) + ' m\\n';
  });
  info.textContent = lines;
}
new EventSource('/events').onmessage = (e) => draw(JSON.parse(e.data));
</script></body></html>"""


class Hub:
    """Latest frame + fan-out queues for SSE clients."""

    def __init__(self):
        self.lock = threading.Lock()
        self.latest = None
        self.subscribers = []

    def publish(self, frame):
        with self.lock:
            self.latest = frame
            for q in self.subscribers:
                q.put(frame)

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


class Handler(BaseHTTPRequestHandler):
    hub = None    # set by serve()
    page = None

    def do_GET(self):
        if self.path == "/":
            body = self.page.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path == "/state":
            with self.hub.lock:
                state = self.hub.latest or {}
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
            q = self.hub.subscribe()
            try:
                while True:
                    try:
                        payload = "data: %s\n\n" % json.dumps(q.get(timeout=15))
                    except queue.Empty:
                        payload = ": keepalive\n\n"
                    self.wfile.write(payload.encode())
                    self.wfile.flush()
            except OSError:
                pass  # client went away
            finally:
                self.hub.unsubscribe(q)
        else:
            self.send_error(404)

    def log_message(self, fmt, *args):
        pass


def serve(w, http_port=8081, arena=30.0, realtime=True, frame_rate=10.0):
    """Start the HTTP server and step the world (real time by default).
    Returns (hub, httpd, sim_thread); the sim thread is a daemon."""
    hub = Hub()
    Handler.hub = hub
    Handler.page = PAGE % {"arena": arena}
    httpd = ThreadingHTTPServer(("0.0.0.0", http_port), Handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()

    def sim_loop():
        frame_period = max(1, round(w.cfg.imu_rate / frame_rate))
        start = time.time()
        while True:
            w.step()
            if w.step_count % frame_period == 0:
                hub.publish(w.frame())
            if realtime:
                lag = start + w.t - time.time()
                if lag > 0:
                    time.sleep(lag)

    th = threading.Thread(target=sim_loop, daemon=True)
    th.start()
    return hub, httpd, th


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--widgets", type=int, default=5)
    ap.add_argument("--gps", type=int, default=0)
    ap.add_argument("--surveyed", type=int, default=3)
    ap.add_argument("--seed", type=int, default=1234)
    ap.add_argument("--http-port", type=int, default=8081)
    ap.add_argument("--arena", type=float, default=30.0)
    args = ap.parse_args()
    specs = world.default_swarm(n=args.widgets, n_gps=args.gps,
                                n_surveyed=args.surveyed, arena=args.arena)
    w = world.World(specs, seed=args.seed)
    serve(w, args.http_port, args.arena)
    print("Swarm view on http://0.0.0.0:%d/" % args.http_port)
    try:
        while True:
            time.sleep(3600)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
