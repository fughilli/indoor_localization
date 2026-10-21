"""Live web view of a station-free swarm through any one node.

Plug any node (apps/node firmware) into this machine: it reports its own pose
at 100 Hz and every node it hears at 10 Hz, so one USB cable shows the whole
swarm. The page draws a top-down map (x/y, m) with each node's 1-sigma
position uncertainty, the plugged-in node's high-rate track, the ranging links
it is measuring, and a table; it also sends commands to the node (survey its
position, magnetometer calibration, raw capture).

  python3 swarm/tools/swarm_view.py --port /dev/cu.usbserial-110
  python3 swarm/tools/swarm_view.py --replay captures/swarm.jsonl   # no hardware

Runs on the machine the node is plugged into (pyserial), like capture.py and
range_view.py, and never resets the board on open. --capture appends every
record to a JSONL file (the records.py capture schema).
"""

import argparse
import json
import os
import queue
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

try:
    from swarm.tools import capture, range_view
except ImportError:  # run as a plain script from the repo root
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
    from swarm.tools import capture, range_view

STREAM_TYPES = {"self", "swarm", "range", "stats", "boot", "ack", "i2c_scan", "mpu", "qmc"}


class Hub:
    """Fans node records out to SSE subscribers; queues commands to the node."""

    def __init__(self, capture_path=None):
        self.lock = threading.Lock()
        self.subscribers = []
        self.capture_path = capture_path
        self.commands = queue.Queue()
        self.latest = {}  # type -> last record (sent to new subscribers)

    def ingest(self, rec):
        if self.capture_path:
            with open(self.capture_path, "a") as f:
                f.write(json.dumps(rec, separators=(",", ":")) + "\n")
        if rec["type"] not in STREAM_TYPES:
            return
        with self.lock:
            if rec["type"] in ("boot", "swarm", "stats", "i2c_scan"):
                self.latest[rec["type"]] = rec
            for q in self.subscribers:
                q.put(rec)

    def subscribe(self):
        q = queue.Queue()
        with self.lock:
            self.subscribers.append(q)
            for rec in self.latest.values():
                q.put(rec)
        return q

    def unsubscribe(self, q):
        with self.lock:
            if q in self.subscribers:
                self.subscribers.remove(q)

    def command(self, obj):
        if not isinstance(obj, dict) or not isinstance(obj.get("cmd"), str):
            raise ValueError("expected {\"cmd\": ...}")
        self.commands.put(json.dumps(obj, separators=(",", ":")))


def run_serial(hub, port, baud):
    import serial  # pyserial

    # Don't touch DTR/RTS: deasserting them resets the ESP32 (see capture.py).
    with serial.Serial(port, baud, timeout=0.05) as s:
        while True:
            while not hub.commands.empty():
                s.write((hub.commands.get() + "\n").encode())
            rec = capture.parse_line(s.readline())
            if rec is not None:
                hub.ingest(rec)


class Handler(BaseHTTPRequestHandler):
    hub = None

    def _json(self, code, obj):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/":
            body = PAGE.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
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
                pass
            finally:
                self.hub.unsubscribe(q)
        else:
            self.send_error(404)

    def do_POST(self):
        if self.path != "/cmd":
            return self.send_error(404)
        try:
            length = int(self.headers.get("Content-Length") or 0)
            self.hub.command(json.loads(self.rfile.read(length) or b"{}"))
            self._json(200, {"ok": True})
        except (ValueError, TypeError) as e:
            self._json(400, {"ok": False, "error": str(e)})

    def log_message(self, fmt, *args):
        pass


def serve(hub, host="127.0.0.1", http_port=8184):
    Handler.hub = hub
    httpd = ThreadingHTTPServer((host, http_port), Handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    return httpd


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--port", help="serial port of any node")
    src.add_argument("--replay", metavar="CAPTURE", help="replay a capture JSONL")
    ap.add_argument("--baud", type=int, default=460800)
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--capture", help="append every record here")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--http-port", type=int, default=8184)
    args = ap.parse_args()
    hub = Hub(args.capture)
    serve(hub, args.host, args.http_port)
    print("Swarm view on http://%s:%d/" % (args.host, args.http_port), flush=True)
    try:
        if args.port:
            run_serial(hub, args.port, args.baud)
        else:
            range_view.replay(hub, args.replay, args.speed)
    except KeyboardInterrupt:
        pass


PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Swarm View</title>
<style>
:root {
  color-scheme: light;
  --page: #f9f9f7; --surface: #fcfcfb; --ink: #0b0b0b; --ink-2: #52514e;
  --muted: #898781; --grid: #e1e0d9; --axis: #c3c2b7; --border: #e1e0d9;
  --s1: #2a78d6; --s2: #eb6834; --s3: #1baf7a; --s4: #eda100;
  --s5: #e87ba4; --s6: #008300; --s7: #6250d6; --s8: #e34948;
  --good: #0ca30c; --warning: #fab219; --critical: #d03b3b;
}
@media (prefers-color-scheme: dark) {
  :root:where(:not([data-theme="light"])) {
    color-scheme: dark;
    --page: #0d0d0d; --surface: #1a1a19; --ink: #f0efec; --ink-2: #c3c2b7;
    --muted: #898781; --grid: #2c2c2a; --axis: #383835; --border: #2c2c2a;
    --s1: #3987e5; --s2: #d95926; --s3: #199e70; --s4: #c98500;
    --s5: #d55181; --s6: #008300; --s7: #9085e9; --s8: #e66767;
  }
}
:root[data-theme="dark"] {
  color-scheme: dark;
  --page: #0d0d0d; --surface: #1a1a19; --ink: #f0efec; --ink-2: #c3c2b7;
  --muted: #898781; --grid: #2c2c2a; --axis: #383835; --border: #2c2c2a;
  --s1: #3987e5; --s2: #d95926; --s3: #199e70; --s4: #c98500;
  --s5: #d55181; --s6: #008300; --s7: #9085e9; --s8: #e66767;
}
* { box-sizing: border-box; }
body { margin: 0; background: var(--page); color: var(--ink);
  font: 14px/1.4 system-ui, -apple-system, "Segoe UI", sans-serif; }
main { max-width: 1200px; margin: 0 auto; padding: 20px 16px 40px; }
header { display: flex; align-items: baseline; gap: 12px; flex-wrap: wrap; margin-bottom: 14px; }
h1 { font-size: 20px; margin: 0; font-weight: 600; }
#status { color: var(--ink-2); font-size: 13px; }
#status::before { content: "●"; margin-right: 6px; color: var(--muted); }
#status.live::before { color: var(--good); }
#status.bad::before { color: var(--critical); }
.layout { display: grid; grid-template-columns: minmax(0, 3fr) minmax(0, 2fr); gap: 12px; }
@media (max-width: 860px) { .layout { grid-template-columns: minmax(0, 1fr); } }
.card { background: var(--surface); border: 1px solid var(--border); border-radius: 10px;
  padding: 12px 14px; position: relative; min-width: 0; margin-bottom: 12px; }
.card h2 { font-size: 14px; font-weight: 600; margin: 0 0 2px; }
.desc { color: var(--muted); font-size: 12px; margin-bottom: 8px; }
canvas { width: 100%; display: block; touch-action: none; }
#map { cursor: grab; }
#map:active { cursor: grabbing; }
.legend { display: flex; gap: 14px; flex-wrap: wrap; font-size: 12px; color: var(--ink-2); margin-bottom: 6px; }
.legend span { display: inline-flex; align-items: center; gap: 6px; }
.dot { width: 9px; height: 9px; border-radius: 50%; display: inline-block; }
.ring { width: 11px; height: 11px; border-radius: 50%; display: inline-block; border: 2px solid var(--ink); }
.tip { position: absolute; pointer-events: none; background: var(--surface); border: 1px solid var(--axis);
  border-radius: 8px; padding: 6px 9px; font-size: 12px; box-shadow: 0 2px 10px rgba(0,0,0,.15);
  display: none; white-space: nowrap; font-variant-numeric: tabular-nums; z-index: 2; }
.tip b { font-weight: 600; }
.tip .k { color: var(--ink-2); }
table { width: 100%; border-collapse: collapse; font-size: 12px; font-variant-numeric: tabular-nums; }
th, td { text-align: right; padding: 5px 6px; border-bottom: 1px solid var(--grid); white-space: nowrap; }
th { color: var(--ink-2); font-weight: 500; }
th:first-child, td:first-child { text-align: left; }
.scroll { overflow-x: auto; }
.badge { display: inline-flex; align-items: center; gap: 5px; }
.badge i { width: 8px; height: 8px; border-radius: 50%; display: inline-block; }
.stats { display: grid; grid-template-columns: repeat(auto-fit, minmax(110px, 1fr)); gap: 8px; }
.stat .label { color: var(--ink-2); font-size: 12px; }
.stat .value { font-size: 20px; font-weight: 600; font-variant-numeric: tabular-nums; }
.row { display: flex; gap: 8px; align-items: center; flex-wrap: wrap; margin: 6px 0; }
input { width: 72px; padding: 5px 7px; border: 1px solid var(--axis); border-radius: 6px;
  background: var(--page); color: var(--ink); font: inherit; }
button { padding: 5px 11px; border-radius: 6px; border: 1px solid var(--axis); background: var(--page);
  color: var(--ink); font: inherit; cursor: pointer; }
button.primary { background: var(--s1); border-color: var(--s1); color: #fff; }
#cmd-msg { color: var(--ink-2); font-size: 12px; }
</style></head>
<body><main>
<header><h1>Swarm</h1><span id="status">connecting…</span><span id="via" class="desc"></span></header>
<div class="layout">
<div>
  <section class="card">
    <h2>Map (top-down)</h2><div class="desc">Estimated positions, x/y in metres; circles are 1σ position uncertainty. Scroll or pinch to zoom, drag to pan, double-click to fit.</div>
    <div class="row"><div class="legend" id="legend"></div><span style="flex:1"></span>
      <button id="z-in" aria-label="Zoom in">+</button><button id="z-out" aria-label="Zoom out">−</button>
      <button id="z-fit">Fit</button></div>
    <canvas id="map" height="520"></canvas><div class="tip" id="tip"></div>
  </section>
</div>
<div>
  <section class="card">
    <h2>This node</h2><div class="desc" id="self-desc">Pose at 100 Hz</div>
    <div class="stats">
      <div class="stat"><div class="label">x / y / z (m)</div><div class="value" id="v-p">–</div></div>
      <div class="stat"><div class="label">σ position</div><div class="value" id="v-sd">–</div></div>
      <div class="stat"><div class="label">Lock</div><div class="value" id="v-lock">–</div></div>
      <div class="stat"><div class="label">Ranges / s</div><div class="value" id="v-rate">–</div></div>
    </div>
  </section>
  <section class="card">
    <h2>Nodes</h2><div class="desc">As this node knows them (advertised, dead-reckoned)</div>
    <div class="scroll"><table id="nodes"><thead><tr><th>Node</th><th>x</th><th>y</th><th>z</th>
      <th>σ (m)</th><th>Lock</th><th>Age (s)</th><th>Range (m)</th></tr></thead><tbody></tbody></table></div>
  </section>
  <section class="card">
    <h2>Radio</h2><div class="desc" id="radio-desc">Per second, from this node</div>
    <div class="stats" id="radio"></div>
  </section>
  <section class="card">
    <h2>Commands</h2>
    <div class="desc">Sent to the plugged-in node; survey and magcal persist on the node.</div>
    <div class="row">Survey this node at
      <input id="sx" type="number" step="0.01" placeholder="x">
      <input id="sy" type="number" step="0.01" placeholder="y">
      <input id="sz" type="number" step="0.01" placeholder="z"> m
      <button class="primary" id="b-survey">Survey</button><button id="b-unsurvey">Clear</button></div>
    <div class="row"><button id="b-cap-on">Raw IMU capture on</button><button id="b-cap-off">off</button>
      <button id="b-reboot">Reboot node</button></div>
    <div id="cmd-msg"></div>
  </section>
</div>
</div>
</main>
<script>
"use strict";
const $ = id => document.getElementById(id);
const css = n => getComputedStyle(document.documentElement).getPropertyValue(n).trim();
const SLOTS = ["--s1","--s2","--s3","--s4","--s5","--s6","--s7","--s8"];
let selfId = null, swarm = null, selfPose = null, stats = null;
const trail = [];            // [t, x, y] of the plugged-in node
const lastRange = {};        // peer -> {t, m}
const rangeTimes = [];       // completed range timestamps (rate)
const colorOf = {};          // node id -> slot var, fixed by first sight order of sorted ids
function color(id) {
  if (!(id in colorOf)) {
    const n = Object.keys(colorOf).length;
    colorOf[id] = n < SLOTS.length ? SLOTS[n] : "--muted";
  }
  return css(colorOf[id]);
}
const hex = id => "0x" + Number(id).toString(16).toUpperCase().padStart(4, "0");
const fmt = (v, d) => (v == null || isNaN(v)) ? "–" : Number(v).toFixed(d);

function setStatus(text, cls) { const el = $("status"); el.textContent = text; el.className = cls || ""; }

// ---- map
let view = null;
// Manual zoom/pan: {cx, cy, span} in metres, or null to auto-fit every frame.
let manual = null;
const PAD = 30, MAP_H = 520;
function drawMap() {
  const c = $("map"), dpr = window.devicePixelRatio || 1, w = c.clientWidth, h = MAP_H;
  c.width = w * dpr; c.height = h * dpr; c.style.height = h + "px";
  const ctx = c.getContext("2d"); ctx.setTransform(dpr, 0, 0, dpr, 0, 0); ctx.clearRect(0, 0, w, h);
  const nodes = swarm ? swarm.nodes : [];
  if (!nodes.length) { ctx.fillStyle = css("--muted"); ctx.font = "13px system-ui"; ctx.fillText("Waiting for the node's swarm report…", 16, 30); view = null; return; }
  // Fit: positions + 1σ, at least 2 m across.
  let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
  for (const n of nodes) { const s = Math.min(Math.sqrt(n.var || 0), 5);
    x0 = Math.min(x0, n.p[0] - s); x1 = Math.max(x1, n.p[0] + s); y0 = Math.min(y0, n.p[1] - s); y1 = Math.max(y1, n.p[1] + s); }
  for (const [, x, y] of trail) { x0 = Math.min(x0, x); x1 = Math.max(x1, x); y0 = Math.min(y0, y); y1 = Math.max(y1, y); }
  let span = Math.max(x1 - x0, y1 - y0, 2) * 1.15, cx = (x0 + x1) / 2, cy = (y0 + y1) / 2;
  if (manual) ({ cx, cy, span } = manual);
  const scale = Math.min(w - 2 * PAD, h - 2 * PAD) / span;
  const X = x => w / 2 + (x - cx) * scale, Y = y => h / 2 - (y - cy) * scale;
  // Grid: ~6-12 lines across at a 1-2-5 step.
  const steps = [0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000];
  const step = steps.find(s => span / s <= 12) || 10000;
  const dec = Math.max(0, -Math.floor(Math.log10(step) + 1e-9));
  ctx.strokeStyle = css("--grid"); ctx.lineWidth = 1; ctx.fillStyle = css("--muted"); ctx.font = "11px system-ui";
  for (let g = Math.ceil((cx - span) / step) * step; g <= cx + span; g += step) {
    const px = Math.round(X(g)) + .5; ctx.beginPath(); ctx.moveTo(px, 0); ctx.lineTo(px, h); ctx.stroke();
    ctx.fillText(g.toFixed(dec) + " m", px + 3, h - 6);
  }
  for (let g = Math.ceil((cy - span) / step) * step; g <= cy + span; g += step) {
    const py = Math.round(Y(g)) + .5; ctx.beginPath(); ctx.moveTo(0, py); ctx.lineTo(w, py); ctx.stroke();
    if (py > 14) ctx.fillText(g.toFixed(dec), 4, py - 3);
  }
  const me = nodes.find(n => n.self);
  // Links: ranges measured in the last second from this node.
  const now = swarm.t;
  if (me) for (const n of nodes) {
    const r = lastRange[n.id]; if (!r || now - r.t > 1) continue;
    ctx.strokeStyle = css("--axis"); ctx.lineWidth = 1.5; ctx.setLineDash([4, 4]);
    ctx.beginPath(); ctx.moveTo(X(me.p[0]), Y(me.p[1])); ctx.lineTo(X(n.p[0]), Y(n.p[1])); ctx.stroke(); ctx.setLineDash([]);
  }
  // High-rate track of this node.
  if (trail.length > 1) {
    ctx.strokeStyle = color(selfId); ctx.globalAlpha = 0.5; ctx.lineWidth = 2; ctx.beginPath();
    trail.forEach(([, x, y], i) => i ? ctx.lineTo(X(x), Y(y)) : ctx.moveTo(X(x), Y(y))); ctx.stroke(); ctx.globalAlpha = 1;
  }
  view = { X, Y, nodes, scale, cx, cy, span, w, h };
  for (const n of nodes) {
    const px = X(n.p[0]), py = Y(n.p[1]), col = color(n.id), sd = Math.sqrt(n.var || 0);
    if (sd * scale > 3) { ctx.fillStyle = col; ctx.globalAlpha = 0.12; ctx.beginPath(); ctx.arc(px, py, Math.min(sd * scale, 2000), 0, 7); ctx.fill(); ctx.globalAlpha = 1; }
    ctx.fillStyle = col; ctx.strokeStyle = css("--surface"); ctx.lineWidth = 2;
    ctx.beginPath(); ctx.arc(px, py, 7, 0, 7); ctx.fill(); ctx.stroke();
    if (n.self) { ctx.strokeStyle = css("--ink"); ctx.lineWidth = 2; ctx.beginPath(); ctx.arc(px, py, 11, 0, 7); ctx.stroke(); }
    if (n.flags & 1) { ctx.strokeStyle = css("--ink-2"); ctx.lineWidth = 1.5; ctx.strokeRect(px - 12, py - 12, 24, 24); }
    ctx.fillStyle = css("--ink"); ctx.font = "12px system-ui"; ctx.fillText(hex(n.id) + (n.self ? " (this node)" : ""), px + 14, py - 8);
  }
}
$("map").addEventListener("pointermove", e => {
  const tip = $("tip"); if (!view) return;
  const r = $("map").getBoundingClientRect(), mx = e.clientX - r.left, my = e.clientY - r.top;
  let best = null, bd = 400;
  for (const n of view.nodes) { const d = (view.X(n.p[0]) - mx) ** 2 + (view.Y(n.p[1]) - my) ** 2; if (d < bd) { bd = d; best = n; } }
  if (!best) { tip.style.display = "none"; return; }
  tip.replaceChildren();
  const line = (v, k) => { const d = document.createElement("div"); const b = document.createElement("b"); b.textContent = v;
    const s = document.createElement("span"); s.className = "k"; s.textContent = " " + k; d.append(b, s); tip.appendChild(d); };
  line(hex(best.id), best.self ? "this node" : (best.flags & 1 ? "surveyed anchor" : "peer"));
  line(best.p.map(v => v.toFixed(2)).join(", "), "m");
  line(fmt(Math.sqrt(best.var), 3) + " m", "1σ");
  line(fmt(best.lock, 2), "lock");
  if (!best.self) line(fmt(best.age, 2) + " s", "advert age");
  tip.style.display = "block";
  tip.style.left = Math.min(mx + 16, $("map").clientWidth - tip.offsetWidth) + "px";
  tip.style.top = (my + 40) + "px";
});
$("map").addEventListener("pointerleave", () => { $("tip").style.display = "none"; });

// Zoom by `factor` (>1 zooms in) keeping the map point under (mx, my) fixed.
function zoomAt(factor, mx, my) {
  if (!view) return;
  const wx = view.cx + (mx - view.w / 2) / view.scale, wy = view.cy - (my - view.h / 2) / view.scale;
  const span = Math.min(Math.max(view.span / factor, 0.05), 20000);
  const scale = view.scale * view.span / span;
  manual = { cx: wx - (mx - view.w / 2) / scale, cy: wy + (my - view.h / 2) / scale, span };
  drawMap();
}
function panBy(dx, dy) {
  if (!view) return;
  manual = { cx: view.cx - dx / view.scale, cy: view.cy + dy / view.scale, span: view.span };
  drawMap();
}
function fit() { manual = null; drawMap(); }
$("map").addEventListener("wheel", e => {
  e.preventDefault();
  const r = $("map").getBoundingClientRect();
  zoomAt(Math.exp(-e.deltaY * (e.deltaMode === 1 ? 0.05 : 0.0015)), e.clientX - r.left, e.clientY - r.top);
}, { passive: false });
$("map").addEventListener("dblclick", fit);
$("z-in").addEventListener("click", () => view && zoomAt(1.5, view.w / 2, view.h / 2));
$("z-out").addEventListener("click", () => view && zoomAt(1 / 1.5, view.w / 2, view.h / 2));
$("z-fit").addEventListener("click", fit);
// Drag to pan (one pointer), pinch to zoom (two).
const ptrs = new Map();
$("map").addEventListener("pointerdown", e => { $("map").setPointerCapture(e.pointerId); ptrs.set(e.pointerId, [e.clientX, e.clientY]); });
$("map").addEventListener("pointermove", e => {
  const prev = ptrs.get(e.pointerId); if (!prev) return;
  const cur = [e.clientX, e.clientY];
  if (ptrs.size === 1) { panBy(cur[0] - prev[0], cur[1] - prev[1]); }
  else if (ptrs.size === 2) {
    const other = [...ptrs].find(([id]) => id !== e.pointerId)[1];
    const d0 = Math.hypot(prev[0] - other[0], prev[1] - other[1]), d1 = Math.hypot(cur[0] - other[0], cur[1] - other[1]);
    const r = $("map").getBoundingClientRect();
    if (d0 > 0) zoomAt(d1 / d0, (cur[0] + other[0]) / 2 - r.left, (cur[1] + other[1]) / 2 - r.top);
  }
  ptrs.set(e.pointerId, cur);
});
for (const ev of ["pointerup", "pointercancel"]) $("map").addEventListener(ev, e => ptrs.delete(e.pointerId));

// ---- panels
function lockBadge(lock) {
  const span = document.createElement("span"); span.className = "badge";
  const i = document.createElement("i");
  const good = lock < 2, ok = lock < 6;
  i.style.background = css(good ? "--good" : ok ? "--warning" : "--critical");
  span.append(i, document.createTextNode(fmt(lock, 2) + (good ? " locked" : ok ? " converging" : " no lock")));
  return span;
}
function updatePanels() {
  if (selfPose) {
    $("v-p").textContent = selfPose.p.map(v => v.toFixed(2)).join(" / ");
    $("v-sd").textContent = fmt(Math.sqrt(selfPose.var), 3) + " m";
    $("v-lock").replaceChildren(lockBadge(selfPose.lock));
  }
  const tNow = rangeTimes.length ? rangeTimes[rangeTimes.length - 1] : 0;
  while (rangeTimes.length && rangeTimes[0] < tNow - 1) rangeTimes.shift();
  $("v-rate").textContent = String(rangeTimes.length);
  const tb = $("nodes").tBodies[0]; tb.replaceChildren();
  const leg = $("legend"); leg.replaceChildren();
  if (swarm) for (const n of [...swarm.nodes].sort((a, b) => a.id - b.id)) {
    const tr = tb.insertRow(), first = tr.insertCell(), dot = document.createElement("i");
    dot.className = "dot"; dot.style.background = color(n.id); dot.style.marginRight = "6px";
    first.append(dot, document.createTextNode(hex(n.id) + (n.self ? " ★" : "") + (n.flags & 1 ? " ⌖" : "")));
    for (const v of [fmt(n.p[0], 2), fmt(n.p[1], 2), fmt(n.p[2], 2), fmt(Math.sqrt(n.var), 3)]) tr.insertCell().textContent = v;
    tr.insertCell().appendChild(lockBadge(n.lock));
    tr.insertCell().textContent = n.self ? "–" : fmt(n.age, 2);
    const r = lastRange[n.id]; tr.insertCell().textContent = r && swarm.t - r.t < 2 ? r.m.toFixed(3) : "–";
    const s = document.createElement("span"), d = document.createElement("i");
    d.className = n.self ? "ring" : "dot"; if (!n.self) d.style.background = color(n.id);
    s.append(d, document.createTextNode(hex(n.id) + (n.self ? " this node" : (n.flags & 1 ? " anchor" : "")))); leg.appendChild(s);
  }
  if (stats) {
    const box = $("radio"); box.replaceChildren();
    const items = [["Slot", stats.slot], ["Peers", stats.peers], ["Sync master", hex(stats.sync_master)],
      ["Polls", stats.polls], ["Ranged (init)", stats.ranges_init], ["Ranged (resp)", stats.ranges_resp],
      ["Timeouts", stats.timeouts], ["RX errors", stats.rx_errors], ["Dropped lines", stats.dropped],
      ["Fusion max (µs)", stats.fusion_max_us]];
    for (const [k, v] of items) { const d = document.createElement("div"); d.className = "stat";
      const l = document.createElement("div"); l.className = "label"; l.textContent = k;
      const vv = document.createElement("div"); vv.className = "value"; vv.textContent = String(v); d.append(l, vv); box.appendChild(d); }
    $("radio-desc").textContent = "Cumulative counters since boot · last sync jump " + stats.sync_jump_us + " µs";
  }
}
let dirty = false;
function schedule() { if (!dirty) { dirty = true; requestAnimationFrame(() => { dirty = false; drawMap(); updatePanels(); }); } }
window.addEventListener("resize", schedule);

// ---- data
const es = new EventSource("/events");
es.onerror = () => setStatus("disconnected from viewer", "bad");
es.onmessage = ev => {
  const m = JSON.parse(ev.data);
  if (m.type === "boot") { selfId = m.id; $("via").textContent = "via node " + hex(m.id) + (m.imu ? " · IMU" : " · no IMU") + (m.mag ? " + mag" : ""); }
  else if (m.type === "self") {
    selfId = m.id; selfPose = m; setStatus("live", "live");
    trail.push([m.t, m.p[0], m.p[1]]); while (trail.length && trail[0][0] < m.t - 10) trail.shift();
  }
  else if (m.type === "swarm") { swarm = m; selfId = m.id; }
  else if (m.type === "range") { lastRange[m.peer] = { t: m.t, m: m.range_m }; rangeTimes.push(m.t); }
  else if (m.type === "stats") stats = m;
  else if (m.type === "ack") $("cmd-msg").textContent = (m.ok ? "✓ " : "✗ ") + m.cmd + (m.note ? " — " + m.note : "") + (m.error ? " — " + m.error : "");
  schedule();
};
async function send(obj) {
  const r = await fetch("/cmd", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(obj) });
  const j = await r.json(); if (!j.ok) $("cmd-msg").textContent = "✗ " + j.error;
}
$("b-survey").onclick = () => {
  const p = ["sx", "sy", "sz"].map(id => parseFloat($(id).value));
  if (p.some(isNaN)) { $("cmd-msg").textContent = "Enter x, y and z first."; return; }
  send({ cmd: "survey", p });
};
$("b-unsurvey").onclick = () => send({ cmd: "unsurvey" });
$("b-cap-on").onclick = () => send({ cmd: "capture", on: 1 });
$("b-cap-off").onclick = () => send({ cmd: "capture", on: 0 });
$("b-reboot").onclick = () => send({ cmd: "reboot" });
</script></body></html>
"""

if __name__ == "__main__":
    main()
