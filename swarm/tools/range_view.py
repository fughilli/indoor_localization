"""Live web view of UWB ranging from a mule initiator.

Reads the initiator's newline-JSON serial stream (or replays a capture) and
serves a browser page: current range with spread, success rate, range-vs-time
with a rolling median, a histogram, and RSSI / first-path RSSI. The page can
also record ground-truth segments, so it replaces capture.py while it holds
the serial port (only one program can open it):

  python3 swarm/tools/range_view.py --port /dev/cu.usbserial-110 \\
      --capture range.jsonl --manifest range.json
  # open http://127.0.0.1:8183/, enter the true distance, Start / Stop

  python3 swarm/tools/range_view.py --replay range.jsonl   # no hardware

Like capture.py it runs on the machine the board is plugged into (needs
pyserial for --port), and never resets the board on open.
"""

import argparse
import json
import os
import queue
import statistics
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

try:
    from swarm.tools import capture
except ImportError:  # run as a plain script from the repo root
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
    from swarm.tools import capture

STREAM_TYPES = {"range", "stats", "boot", "ready", "error"}


class Hub:
    """Fans records out to SSE subscribers and tracks segment recording."""

    def __init__(self, capture_path=None, manifest_path=None):
        self.lock = threading.Lock()
        self.subscribers = []
        self.capture_path = capture_path
        self.manifest_path = manifest_path
        self.recording = None  # {"true_range_m", "t0", "t1", "ranges"}
        self.summaries = {}    # t0 -> stats of segments recorded this session
        self.last_t = None
        self.status = {"type": "status", "text": "waiting for data"}

    def can_record(self):
        return bool(self.capture_path and self.manifest_path)

    def ingest(self, rec):
        if self.capture_path:
            with open(self.capture_path, "a") as f:
                f.write(json.dumps(rec, separators=(",", ":")) + "\n")
        with self.lock:
            if self.last_t is not None and rec["t"] < self.last_t:
                self.status = {"type": "status", "text": "board rebooted (clock restarted)"}
                self._publish(self.status)
                if self.recording:
                    self.recording = None  # its window would straddle the reboot
                    self._publish(self.state())
            self.last_t = rec["t"]
            if rec["type"] == "error":
                self.status = {"type": "status", "text": "bring-up error: %s" % rec.get("stage")}
                self._publish(self.status)
            elif rec["type"] in ("ready", "range") and self.status["text"] != "ranging":
                self.status = {"type": "status", "text": "ranging"}
                self._publish(self.status)
            if self.recording is not None:
                if self.recording["t0"] is None:
                    self.recording["t0"] = rec["t"]
                self.recording["t1"] = rec["t"]
                if rec["type"] == "range":
                    self.recording["ranges"].append(rec["range_m"])
            if rec["type"] in STREAM_TYPES:
                self._publish(rec)

    def _publish(self, msg):
        for q in self.subscribers:
            q.put(msg)

    def subscribe(self):
        q = queue.Queue()
        with self.lock:
            self.subscribers.append(q)
            q.put(self.status)
            q.put(self.state())
        return q

    def unsubscribe(self, q):
        with self.lock:
            if q in self.subscribers:
                self.subscribers.remove(q)

    def state(self):
        segments = []
        if self.manifest_path and os.path.exists(self.manifest_path):
            with open(self.manifest_path) as f:
                segments = json.load(f).get("segments", [])
        rec = self.recording
        return {
            "type": "state",
            "can_record": self.can_record(),
            "recording": None if rec is None else {
                "true_range_m": rec["true_range_m"], "n": len(rec["ranges"])},
            "segments": [dict(s, **self.summaries.get(s["t0"], {})) for s in segments],
        }

    def start_segment(self, true_range_m):
        with self.lock:
            if not self.can_record():
                raise ValueError("start the viewer with --capture and --manifest to record")
            if self.recording is not None:
                raise ValueError("already recording")
            self.recording = {"true_range_m": float(true_range_m), "t0": None,
                              "t1": None, "ranges": []}
            self._publish(self.state())

    def stop_segment(self):
        with self.lock:
            rec, self.recording = self.recording, None
            if rec is None:
                raise ValueError("not recording")
            if not rec["ranges"]:
                self._publish(self.state())
                raise ValueError("no ranges captured; segment discarded")
            seg = {"t0": rec["t0"], "t1": round(rec["t1"] + 1e-3, 3),
                   "true_range_m": rec["true_range_m"]}
            capture.append_segment(self.manifest_path, seg)
            med = statistics.median(rec["ranges"])
            stats = {"n": len(rec["ranges"]), "median_m": round(med, 4),
                     "bias_m": round(med - rec["true_range_m"], 4),
                     "sd_m": round(statistics.pstdev(rec["ranges"]), 4)}
            self.summaries[seg["t0"]] = stats
            seg_summary = dict(seg, **stats)
            self._publish(self.state())
            return seg_summary


def read_serial(hub, port, baud):
    import serial  # pyserial

    # Don't touch DTR/RTS: deasserting them resets the ESP32 (see capture.py).
    with serial.Serial(port, baud, timeout=0.2) as s:
        while True:
            rec = capture.parse_line(s.readline())
            if rec is not None:
                hub.ingest(rec)


def replay(hub, path, speed=1.0, loop=True):
    """Feed a capture file through the hub at its recorded pace, shifting
    times forward on each loop so the clock stays monotonic."""
    recs = [r for r in capture_records(path)]
    if not recs:
        raise SystemExit("no records in %s" % path)
    offset = 0.0
    span = recs[-1]["t"] - recs[0]["t"] + 1.0
    while True:
        start_wall, start_t = time.time(), recs[0]["t"]
        for r in recs:
            lag = (r["t"] - start_t) / speed - (time.time() - start_wall)
            if lag > 0:
                time.sleep(lag)
            hub.ingest(dict(r, t=round(r["t"] + offset, 3)))
        if not loop:
            return
        offset += span


def capture_records(path):
    with open(path) as f:
        for line in f:
            rec = capture.parse_line(line)
            if rec is not None:
                yield rec


class Handler(BaseHTTPRequestHandler):
    hub = None  # set by serve()

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
        elif self.path == "/state":
            with self.hub.lock:
                self._json(200, self.hub.state())
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

    def do_POST(self):
        try:
            length = int(self.headers.get("Content-Length") or 0)
            body = json.loads(self.rfile.read(length) or b"{}")
            if self.path == "/segment/start":
                self.hub.start_segment(body["true_range_m"])
                self._json(200, {"ok": True})
            elif self.path == "/segment/stop":
                self._json(200, {"ok": True, "segment": self.hub.stop_segment()})
            else:
                self.send_error(404)
        except (ValueError, KeyError, TypeError) as e:
            self._json(400, {"ok": False, "error": str(e)})

    def log_message(self, fmt, *args):
        pass


def serve(hub, host="127.0.0.1", http_port=8183):
    Handler.hub = hub
    httpd = ThreadingHTTPServer((host, http_port), Handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    return httpd


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--port", help="initiator serial port")
    src.add_argument("--replay", metavar="CAPTURE", help="replay a capture JSONL instead")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--speed", type=float, default=1.0, help="replay speed factor")
    ap.add_argument("--capture", help="append every record here (enables recording)")
    ap.add_argument("--manifest", help="ground-truth manifest for recorded segments")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--http-port", type=int, default=8183)
    args = ap.parse_args()
    if args.replay and args.capture and os.path.abspath(args.replay) == os.path.abspath(args.capture):
        ap.error("--capture must differ from --replay")

    hub = Hub(args.capture, args.manifest)
    serve(hub, args.host, args.http_port)
    print("Range view on http://%s:%d/" % (args.host, args.http_port), flush=True)
    try:
        if args.port:
            read_serial(hub, args.port, args.baud)
        else:
            replay(hub, args.replay, args.speed)
    except KeyboardInterrupt:
        pass


PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>UWB Range View</title>
<style>
:root {
  color-scheme: light;
  --page: #f9f9f7; --surface: #fcfcfb; --ink: #0b0b0b; --ink-2: #52514e;
  --muted: #898781; --grid: #e1e0d9; --axis: #c3c2b7; --border: #e1e0d9;
  --s1: #2a78d6; --s2: #eb6834; --s3: #1baf7a;
  --good: #0ca30c; --warning: #fab219; --critical: #d03b3b;
}
@media (prefers-color-scheme: dark) {
  :root:where(:not([data-theme="light"])) {
    color-scheme: dark;
    --page: #0d0d0d; --surface: #1a1a19; --ink: #f0efec; --ink-2: #c3c2b7;
    --muted: #898781; --grid: #2c2c2a; --axis: #383835; --border: #2c2c2a;
    --s1: #3987e5; --s2: #d95926; --s3: #199e70;
  }
}
:root[data-theme="dark"] {
  color-scheme: dark;
  --page: #0d0d0d; --surface: #1a1a19; --ink: #f0efec; --ink-2: #c3c2b7;
  --muted: #898781; --grid: #2c2c2a; --axis: #383835; --border: #2c2c2a;
  --s1: #3987e5; --s2: #d95926; --s3: #199e70;
}
* { box-sizing: border-box; }
body { margin: 0; background: var(--page); color: var(--ink);
  font: 14px/1.4 system-ui, -apple-system, "Segoe UI", sans-serif; }
main { max-width: 1100px; margin: 0 auto; padding: 20px 16px 40px; }
header { display: flex; align-items: baseline; gap: 12px; flex-wrap: wrap; margin-bottom: 16px; }
h1 { font-size: 20px; margin: 0; font-weight: 600; }
#status { color: var(--ink-2); font-size: 13px; }
#status::before { content: "●"; margin-right: 6px; color: var(--muted); }
#status.live::before { color: var(--good); }
#status.bad::before { color: var(--critical); }
.tiles { display: grid; grid-template-columns: repeat(auto-fit, minmax(150px, 1fr)); gap: 12px; margin-bottom: 16px; }
.tile, .card { background: var(--surface); border: 1px solid var(--border); border-radius: 10px; }
.tile { padding: 12px 14px; }
.tile .label { color: var(--ink-2); font-size: 12px; }
.tile .value { font-size: 28px; font-weight: 600; font-variant-numeric: tabular-nums; margin-top: 2px; }
.tile .value.hero { font-size: 40px; }
.tile .sub { color: var(--muted); font-size: 12px; font-variant-numeric: tabular-nums; }
.badge { display: inline-flex; align-items: center; gap: 4px; font-size: 12px; color: var(--ink-2); }
.badge i { width: 8px; height: 8px; border-radius: 50%; display: inline-block; }
.grid2 { display: grid; grid-template-columns: 2fr 1fr; gap: 12px; margin-bottom: 12px; }
@media (max-width: 760px) { .grid2 { grid-template-columns: 1fr; } .tile .value.hero { font-size: 30px; } }
.card { padding: 12px 14px; position: relative; min-width: 0; }
.card h2 { font-size: 14px; font-weight: 600; margin: 0 0 2px; }
.card .desc { color: var(--muted); font-size: 12px; margin-bottom: 8px; }
.legend { display: flex; gap: 14px; flex-wrap: wrap; font-size: 12px; color: var(--ink-2); margin-bottom: 6px; }
.legend span { display: inline-flex; align-items: center; gap: 6px; }
.key-line { width: 14px; height: 2px; display: inline-block; border-radius: 1px; }
.key-dot { width: 8px; height: 8px; border-radius: 50%; display: inline-block; }
.key-dash { width: 14px; height: 0; border-top: 2px dashed var(--ink-2); display: inline-block; }
canvas { width: 100%; display: block; }
.tip { position: absolute; pointer-events: none; background: var(--surface); border: 1px solid var(--axis);
  border-radius: 8px; padding: 6px 9px; font-size: 12px; box-shadow: 0 2px 10px rgba(0,0,0,.15);
  display: none; white-space: nowrap; font-variant-numeric: tabular-nums; z-index: 2; }
.tip .row { display: flex; align-items: center; gap: 6px; }
.tip b { font-weight: 600; }
.tip .k { color: var(--ink-2); }
.rec { display: flex; gap: 8px; align-items: center; flex-wrap: wrap; }
.rec input { width: 90px; padding: 6px 8px; border: 1px solid var(--axis); border-radius: 6px;
  background: var(--page); color: var(--ink); font: inherit; }
button { padding: 6px 12px; border-radius: 6px; border: 1px solid var(--axis); background: var(--page);
  color: var(--ink); font: inherit; cursor: pointer; }
button:disabled { opacity: .5; cursor: default; }
button.primary { background: var(--s1); border-color: var(--s1); color: #fff; }
#rec-msg { color: var(--ink-2); font-size: 12px; }
table { width: 100%; border-collapse: collapse; font-size: 12px; font-variant-numeric: tabular-nums; }
th, td { text-align: right; padding: 4px 6px; border-bottom: 1px solid var(--grid); }
th { color: var(--ink-2); font-weight: 500; }
th:first-child, td:first-child { text-align: left; }
details summary { cursor: pointer; color: var(--ink-2); font-size: 13px; margin: 4px 0; }
.scroll { overflow-x: auto; }
</style></head>
<body><main>
<header><h1>UWB range</h1><span id="status">connecting…</span></header>

<section class="tiles">
  <div class="tile"><div class="label">Range (1 s median)</div>
    <div class="value hero" id="v-range">–</div><div class="sub" id="v-range-sub">&nbsp;</div></div>
  <div class="tile"><div class="label">Spread (1 s σ)</div>
    <div class="value" id="v-sd">–</div><div class="sub" id="v-sd-sub">&nbsp;</div></div>
  <div class="tile"><div class="label">Exchange success</div>
    <div class="value" id="v-ok">–</div><div class="sub" id="v-ok-sub">&nbsp;</div></div>
  <div class="tile"><div class="label">RSSI / first path</div>
    <div class="value" id="v-rssi">–</div><div class="sub" id="v-los">&nbsp;</div></div>
</section>

<div class="grid2">
  <section class="card">
    <h2>Range over time</h2><div class="desc">Last 30 s of the initiator's clock</div>
    <div class="legend">
      <span><i class="key-dot" style="background:var(--s1)"></i>Each exchange</span>
      <span><i class="key-line" style="background:var(--ink)"></i>1 s rolling median</span>
      <span id="lg-truth" hidden><i class="key-dash"></i>True range</span>
      <span><i class="key-line" style="background:var(--critical);width:2px;height:10px"></i>Failed exchanges</span>
    </div>
    <canvas id="c-range" height="300"></canvas><div class="tip" id="tip-range"></div>
  </section>
  <section class="card">
    <h2>Distribution</h2><div class="desc">Last 10 s, 5 mm bins</div>
    <div class="legend">
      <span><i class="key-dot" style="background:var(--s1);border-radius:2px"></i>Exchanges per bin</span>
      <span><i class="key-line" style="background:var(--ink)"></i>Median</span>
    </div>
    <canvas id="c-hist" height="300"></canvas><div class="tip" id="tip-hist"></div>
  </section>
</div>

<section class="card" style="margin-bottom:12px">
  <h2>Signal strength</h2><div class="desc">A first-path level well below total RSSI (gap &gt; 6 dB) suggests a blocked direct path</div>
  <div class="legend">
    <span><i class="key-line" style="background:var(--s2)"></i>RSSI</span>
    <span><i class="key-line" style="background:var(--s3)"></i>First-path RSSI</span>
  </div>
  <canvas id="c-rssi" height="170"></canvas><div class="tip" id="tip-rssi"></div>
</section>

<section class="card" style="margin-bottom:12px">
  <h2>Record a ground-truth segment</h2>
  <div class="desc" id="rec-desc">Place the boards at a measured distance, enter it, then Start; Stop when you have enough samples.</div>
  <div class="rec">
    <label for="truth">True range</label><input id="truth" type="number" step="0.01" min="0" placeholder="m"> m
    <button class="primary" id="b-start">Start</button><button id="b-stop" disabled>Stop</button>
    <span id="rec-msg"></span>
  </div>
  <div class="scroll" style="margin-top:10px"><table id="t-seg"><thead><tr>
    <th>True range</th><th>Samples</th><th>Median</th><th>Bias</th><th>σ</th><th>Window (s)</th></tr></thead><tbody></tbody></table></div>
</section>

<details><summary>Table view: last 25 exchanges</summary>
  <div class="scroll"><table id="t-raw"><thead><tr><th>t (s)</th><th>Seq</th><th>Range (m)</th>
    <th>RSSI (dBm)</th><th>First path (dBm)</th><th>Clock offset</th></tr></thead><tbody></tbody></table></div>
</details>
</main>
<script>
"use strict";
const WINDOW = 30, HIST_WINDOW = 10, BIN = 0.005;
const samples = [];   // {t, r, rssi, fp, seq, co}
const fails = [];     // {t, n}
let okWin = [], truth = null, latestT = 0;

const $ = id => document.getElementById(id);
const css = n => getComputedStyle(document.documentElement).getPropertyValue(n).trim();
const fmt = (v, d) => (v == null || isNaN(v)) ? "–" : v.toFixed(d);
function median(a) { if (!a.length) return NaN; const s = [...a].sort((x, y) => x - y), m = s.length >> 1;
  return s.length % 2 ? s[m] : (s[m - 1] + s[m]) / 2; }
function sd(a) { if (a.length < 2) return 0; const m = a.reduce((x, y) => x + y, 0) / a.length;
  return Math.sqrt(a.reduce((x, y) => x + (y - m) ** 2, 0) / a.length); }

// ---- canvas plumbing
function setup(canvas) {
  const dpr = window.devicePixelRatio || 1, w = canvas.clientWidth, h = canvas.height / (canvas._dpr || 1);
  canvas._h = canvas._h || canvas.height;
  canvas.width = w * dpr; canvas.height = canvas._h * dpr; canvas.style.height = canvas._h + "px"; canvas._dpr = dpr;
  const ctx = canvas.getContext("2d"); ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { ctx, w, h: canvas._h };
}
function niceTicks(lo, hi, n, integer) {
  const span = hi - lo || 1, step0 = span / n, mag = Math.pow(10, Math.floor(Math.log10(step0)));
  const steps = [1, 2, 2.5, 5, 10].map(m => m * mag).filter(s => !integer || (s >= 1 && Number.isInteger(s)));
  const step = steps.find(s => span / s <= n) || Math.max(1, 10 * mag);
  const out = []; for (let v = Math.ceil(lo / step) * step; v <= hi + 1e-9; v += step) out.push(+v.toFixed(6));
  return { ticks: out, step };
}
function axes(ctx, box, xs, ys, xfmt, yfmt) {
  ctx.font = "11px system-ui, sans-serif"; ctx.lineWidth = 1;
  ctx.strokeStyle = css("--grid"); ctx.fillStyle = css("--muted");
  ctx.textAlign = "right"; ctx.textBaseline = "middle";
  for (const v of ys.ticks) { const y = ys.map(v); ctx.beginPath(); ctx.moveTo(box.l, y + .5); ctx.lineTo(box.r, y + .5); ctx.stroke();
    ctx.fillText(yfmt(v), box.l - 6, y); }
  ctx.textAlign = "center"; ctx.textBaseline = "top";
  for (const v of xs.ticks) ctx.fillText(xfmt(v), xs.map(v), box.b + 6);
  ctx.strokeStyle = css("--axis"); ctx.beginPath(); ctx.moveTo(box.l, box.b + .5); ctx.lineTo(box.r, box.b + .5); ctx.stroke();
}
function timeTicks(t1) {  // ticks at whole seconds before "now"
  const xs = niceTicks(-WINDOW, 0, 6); xs.ticks = xs.ticks.map(v => t1 + v); return xs;
}
const ageFmt = t1 => v => Math.round(v - t1) === 0 ? "now" : Math.round(v - t1) + " s";
const GAP = 0.5;  // s without data -> break the line instead of bridging it
function polyline(ctx, vis, xf, yf) {
  ctx.beginPath();
  vis.forEach((s, i) => { const x = xf(s), y = yf(s);
    (i && s.t - vis[i - 1].t <= GAP) ? ctx.lineTo(x, y) : ctx.moveTo(x, y); });
  ctx.stroke();
}
function scale(d0, d1, r0, r1) { const f = v => r0 + (v - d0) / (d1 - d0 || 1) * (r1 - r0); return f; }

// ---- range over time
function rollingMedian(arr, t) { return median(arr.filter(s => s.t > t - 1 && s.t <= t).map(s => s.r)); }
function drawRange() {
  const c = $("c-range"), { ctx, w, h } = setup(c);
  const box = { l: 52, r: w - 10, t: 8, b: h - 24 };
  ctx.clearRect(0, 0, w, h);
  const t1 = latestT, t0 = t1 - WINDOW;
  const vis = samples.filter(s => s.t >= t0);
  let lo = Math.min(...vis.map(s => s.r)), hi = Math.max(...vis.map(s => s.r));
  if (truth != null) { lo = Math.min(lo, truth); hi = Math.max(hi, truth); }
  if (!vis.length) { lo = 0; hi = 1; }
  const pad = Math.max((hi - lo) * 0.15, 0.02); lo -= pad; hi += pad;
  const ys = niceTicks(lo, hi, 5); ys.map = scale(lo, hi, box.b, box.t);
  const xs = timeTicks(t1); xs.map = scale(t0, t1, box.l, box.r);
  axes(ctx, box, xs, ys, ageFmt(t1), v => v.toFixed(ys.step < 0.01 ? 3 : 2) + " m");
  ctx.save(); ctx.beginPath(); ctx.rect(box.l, box.t, box.r - box.l, box.b - box.t); ctx.clip();
  if (truth != null) { ctx.setLineDash([6, 4]); ctx.strokeStyle = css("--ink-2"); ctx.lineWidth = 2;
    const y = ys.map(truth); ctx.beginPath(); ctx.moveTo(box.l, y); ctx.lineTo(box.r, y); ctx.stroke(); ctx.setLineDash([]); }
  ctx.fillStyle = css("--s1"); ctx.globalAlpha = 0.55;
  for (const s of vis) { ctx.beginPath(); ctx.arc(xs.map(s.t), ys.map(s.r), 2.5, 0, 7); ctx.fill(); }
  ctx.globalAlpha = 1; ctx.strokeStyle = css("--ink"); ctx.lineWidth = 2; ctx.lineJoin = "round";
  polyline(ctx, vis, s => xs.map(s.t), s => ys.map(s.med));
  ctx.restore();
  ctx.fillStyle = css("--critical");
  for (const f of fails) if (f.t >= t0) { const x = xs.map(f.t); const hh = Math.min(4 + f.n, 16); ctx.fillRect(x - 1, box.b - hh, 2, hh); }
  c._view = { box, xs, ys, vis };
}

// ---- histogram
function drawHist() {
  const c = $("c-hist"), { ctx, w, h } = setup(c);
  const box = { l: 40, r: w - 10, t: 8, b: h - 24 };
  ctx.clearRect(0, 0, w, h);
  const vals = samples.filter(s => s.t >= latestT - HIST_WINDOW).map(s => s.r);
  if (!vals.length) { c._view = null; return; }
  const med = median(vals);
  let lo = Math.floor(Math.min(...vals) / BIN) * BIN, hi = Math.ceil(Math.max(...vals) / BIN + 1e-9) * BIN;
  if (hi - lo < 10 * BIN) { const mid = (lo + hi) / 2; lo = mid - 5 * BIN; hi = mid + 5 * BIN; }
  const nb = Math.round((hi - lo) / BIN), bins = new Array(nb).fill(0);
  for (const v of vals) bins[Math.min(nb - 1, Math.max(0, Math.floor((v - lo) / BIN)))]++;
  const ymax = Math.max(...bins);
  const ys = niceTicks(0, ymax, 4, true); ys.map = scale(0, Math.max(ys.ticks[ys.ticks.length - 1], ymax), box.b, box.t);
  const xs = niceTicks(lo, hi, 4); xs.map = scale(lo, hi, box.l, box.r);
  axes(ctx, box, xs, ys, v => v.toFixed(2), v => v.toFixed(0));
  const bw = (box.r - box.l) / nb, gap = 2;
  ctx.fillStyle = css("--s1");
  bins.forEach((n, i) => { if (!n) return; const x = box.l + i * bw + gap / 2, y = ys.map(n), ww = Math.max(1, bw - gap);
    const r = Math.min(4, ww / 2, box.b - y); ctx.beginPath(); ctx.roundRect(x, y, ww, box.b - y, [r, r, 0, 0]); ctx.fill(); });
  ctx.strokeStyle = css("--ink"); ctx.lineWidth = 2; const mx = xs.map(med);
  ctx.beginPath(); ctx.moveTo(mx, box.t); ctx.lineTo(mx, box.b); ctx.stroke();
  if (truth != null && truth >= lo && truth <= hi) { ctx.setLineDash([6, 4]); ctx.strokeStyle = css("--ink-2");
    const tx = xs.map(truth); ctx.beginPath(); ctx.moveTo(tx, box.t); ctx.lineTo(tx, box.b); ctx.stroke(); ctx.setLineDash([]); }
  c._view = { box, bins, lo, bw, med };
}

// ---- RSSI
function drawRssi() {
  const c = $("c-rssi"), { ctx, w, h } = setup(c);
  const box = { l: 52, r: w - 10, t: 8, b: h - 24 };
  ctx.clearRect(0, 0, w, h);
  const t1 = latestT, t0 = t1 - WINDOW, vis = samples.filter(s => s.t >= t0);
  let lo = Math.min(...vis.map(s => Math.min(s.rssi, s.fp))), hi = Math.max(...vis.map(s => Math.max(s.rssi, s.fp)));
  if (!vis.length) { lo = -100; hi = -40; }
  lo -= 2; hi += 2;
  const ys = niceTicks(lo, hi, 4); ys.map = scale(lo, hi, box.b, box.t);
  const xs = timeTicks(t1); xs.map = scale(t0, t1, box.l, box.r);
  axes(ctx, box, xs, ys, ageFmt(t1), v => v.toFixed(0) + " dBm");
  for (const [key, color] of [["rssi", "--s2"], ["fp", "--s3"]]) {
    ctx.strokeStyle = css(color); ctx.lineWidth = 2; ctx.lineJoin = "round";
    polyline(ctx, vis, s => xs.map(s.t), s => ys.map(s[key]));
  }
  c._view = { box, xs, ys, vis };
}

// ---- hover
function row(tip, color, value, label, dashed) {
  const r = document.createElement("div"); r.className = "row";
  const k = document.createElement("i"); k.className = dashed ? "key-dash" : "key-line";
  if (!dashed) k.style.background = color; r.appendChild(k);
  const b = document.createElement("b"); b.textContent = value; r.appendChild(b);
  const s = document.createElement("span"); s.className = "k"; s.textContent = label; r.appendChild(s);
  tip.appendChild(r);
}
function hoverTime(canvasId, tipId, render) {
  const c = $(canvasId), tip = $(tipId);
  c.addEventListener("pointermove", e => {
    const v = c._view; if (!v || !v.vis || !v.vis.length) return;
    const rect = c.getBoundingClientRect(), px = e.clientX - rect.left;
    let best = v.vis[0], bd = Infinity;
    for (const s of v.vis) { const d = Math.abs(v.xs.map(s.t) - px); if (d < bd) { bd = d; best = s; } }
    tip.replaceChildren();
    const head = document.createElement("div"); head.className = "k"; head.textContent = "t = " + best.t.toFixed(2) + " s · seq " + best.seq; tip.appendChild(head);
    render(tip, best);
    tip.style.display = "block";
    const x = v.xs.map(best.t);
    tip.style.left = Math.min(x + 12 + 14, c.clientWidth - tip.offsetWidth) + "px";
    tip.style.top = (c.offsetTop + 10) + "px";
    c._hoverX = x; drawAll();
  });
  c.addEventListener("pointerleave", () => { tip.style.display = "none"; c._hoverX = null; drawAll(); });
}
function crosshair(canvasId) {
  const c = $(canvasId), v = c._view; if (c._hoverX == null || !v) return;
  const ctx = c.getContext("2d"); ctx.strokeStyle = css("--muted"); ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(c._hoverX + .5, v.box.t); ctx.lineTo(c._hoverX + .5, v.box.b); ctx.stroke();
}
hoverTime("c-range", "tip-range", (tip, s) => {
  row(tip, css("--s1"), s.r.toFixed(3) + " m", "exchange");
  row(tip, css("--ink"), s.med.toFixed(3) + " m", "1 s median");
  if (truth != null) row(tip, null, truth.toFixed(3) + " m", "true range", true);
});
hoverTime("c-rssi", "tip-rssi", (tip, s) => {
  row(tip, css("--s2"), s.rssi.toFixed(1) + " dBm", "RSSI");
  row(tip, css("--s3"), s.fp.toFixed(1) + " dBm", "first path");
});
(() => {
  const c = $("c-hist"), tip = $("tip-hist");
  c.addEventListener("pointermove", e => {
    const v = c._view; if (!v) return;
    const px = e.clientX - c.getBoundingClientRect().left, i = Math.floor((px - v.box.l) / v.bw);
    if (i < 0 || i >= v.bins.length) { tip.style.display = "none"; return; }
    tip.replaceChildren();
    const a = v.lo + i * BIN;
    row(tip, css("--s1"), String(v.bins[i]), "exchanges in " + a.toFixed(3) + "–" + (a + BIN).toFixed(3) + " m");
    tip.style.display = "block";
    tip.style.left = Math.min(v.box.l + i * v.bw + 14, c.clientWidth - tip.offsetWidth) + "px";
    tip.style.top = (c.offsetTop + 10) + "px";
  });
  c.addEventListener("pointerleave", () => { tip.style.display = "none"; });
})();

// ---- tiles + tables
function updateTiles() {
  const last = samples.filter(s => s.t > latestT - 1).map(s => s.r);
  $("v-range").textContent = last.length ? fmt(median(last), 3) + " m" : "–";
  $("v-range-sub").textContent = truth != null && last.length ? "bias " + fmt(median(last) - truth, 3) + " m vs true " + fmt(truth, 2) + " m" : " ";
  $("v-sd").textContent = last.length > 1 ? fmt(sd(last) * 100, 1) + " cm" : "–";
  $("v-sd-sub").textContent = last.length + " exchanges in the last second";
  const ok = okWin.reduce((a, s) => a + s.ok, 0), tot = okWin.reduce((a, s) => a + s.ok + s.fail, 0);
  if (tot) {
    const pct = 100 * ok / tot;
    $("v-ok").textContent = pct.toFixed(0) + "%";
    const sub = $("v-ok-sub"); sub.replaceChildren();
    const b = document.createElement("span"); b.className = "badge";
    const dot = document.createElement("i"); dot.style.background = css(pct >= 95 ? "--good" : pct >= 80 ? "--warning" : "--critical");
    b.appendChild(dot); b.appendChild(document.createTextNode((pct >= 95 ? "healthy" : pct >= 80 ? "degraded" : "poor") + " · " + ok + "/" + tot + " in 5 s"));
    sub.appendChild(b);
  }
  if (tot && !ok) {  // initiator is polling but nothing answers
    $("v-range").textContent = "no replies";
    $("v-range-sub").textContent = "initiator polling, responder silent: is it powered and up?";
  }
  const lastS = samples[samples.length - 1];
  if (lastS) {
    $("v-rssi").textContent = fmt(lastS.rssi, 0) + " / " + fmt(lastS.fp, 0);
    const recent = samples.filter(s => s.t > latestT - 1), gap = median(recent.map(s => s.rssi - s.fp));
    const los = $("v-los"); los.replaceChildren();
    const b = document.createElement("span"); b.className = "badge";
    const dot = document.createElement("i"); dot.style.background = css(gap > 6 ? "--warning" : "--good");
    b.appendChild(dot); b.appendChild(document.createTextNode("dBm · " + (gap > 6 ? "likely blocked path" : "line of sight") + " (gap " + fmt(gap, 1) + " dB)"));
    los.appendChild(b);
  }
  const tb = $("t-raw").tBodies[0]; tb.replaceChildren();
  for (const s of samples.slice(-25).reverse()) {
    const tr = tb.insertRow();
    for (const v of [s.t.toFixed(3), s.seq, s.r.toFixed(4), s.rssi.toFixed(1), s.fp.toFixed(1), s.co]) tr.insertCell().textContent = v;
  }
}
function renderSegments(segs) {
  const tb = $("t-seg").tBodies[0]; tb.replaceChildren();
  for (const s of segs) {
    const st = s.n != null ? s : null, tr = tb.insertRow();
    for (const v of [s.true_range_m.toFixed(3) + " m", st ? st.n : "–", st ? st.median_m.toFixed(3) + " m" : "–",
      st ? (st.bias_m >= 0 ? "+" : "") + st.bias_m.toFixed(3) + " m" : "–", st ? (st.sd_m * 100).toFixed(1) + " cm" : "–",
      s.t0.toFixed(1) + "–" + s.t1.toFixed(1)]) tr.insertCell().textContent = v;
  }
}

function drawAll() {
  drawRange(); crosshair("c-range"); drawHist(); drawRssi(); crosshair("c-rssi");
}
let dirty = false;
function schedule() { if (!dirty) { dirty = true; requestAnimationFrame(() => { dirty = false; drawAll(); updateTiles(); }); } }
window.addEventListener("resize", schedule);
matchMedia("(prefers-color-scheme: dark)").addEventListener("change", schedule);

// ---- data
function setStatus(text) {
  const el = $("status"); el.textContent = text;
  el.className = text === "ranging" ? "live" : /error|reboot/.test(text) ? "bad" : "";
}
function applyState(st) {
  $("b-start").disabled = !st.can_record || !!st.recording;
  $("b-stop").disabled = !st.recording;
  $("truth").disabled = !st.can_record || !!st.recording;
  if (!st.can_record) $("rec-desc").textContent = "Recording is off: restart the viewer with --capture and --manifest to record segments.";
  truth = st.recording ? st.recording.true_range_m : (parseFloat($("truth").value) >= 0 ? parseFloat($("truth").value) : null);
  $("lg-truth").hidden = truth == null;
  if (st.recording) $("rec-msg").textContent = "Recording at " + st.recording.true_range_m.toFixed(2) + " m…";
  renderSegments(st.segments);
  schedule();
}
$("truth").addEventListener("input", () => { const v = parseFloat($("truth").value); truth = v >= 0 ? v : null; $("lg-truth").hidden = truth == null; schedule(); });
async function post(path, body) {
  const r = await fetch(path, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body || {}) });
  return r.json();
}
$("b-start").addEventListener("click", async () => {
  const v = parseFloat($("truth").value);
  if (!(v >= 0)) { $("rec-msg").textContent = "Enter the measured distance first."; return; }
  const r = await post("/segment/start", { true_range_m: v });
  if (!r.ok) $("rec-msg").textContent = r.error;
});
$("b-stop").addEventListener("click", async () => {
  const r = await post("/segment/stop");
  if (!r.ok) { $("rec-msg").textContent = r.error; return; }
  const s = r.segment;
  $("rec-msg").textContent = "Saved: " + s.n + " samples, bias " + (s.bias_m >= 0 ? "+" : "") + s.bias_m.toFixed(3) + " m, σ " + (s.sd_m * 100).toFixed(1) + " cm";
  fetch("/state").then(r => r.json()).then(applyState);
});

const es = new EventSource("/events");
es.onerror = () => setStatus("disconnected from viewer");
es.onmessage = ev => {
  const m = JSON.parse(ev.data);
  if (m.type === "status") { setStatus(m.text); return; }
  if (m.type === "state") { applyState(m); return; }
  if (m.t < latestT - 1) { samples.length = 0; fails.length = 0; okWin = []; latestT = m.t; }  // clock restarted
  latestT = Math.max(latestT, m.t);
  if (m.type === "range") {
    const s = { t: m.t, r: m.range_m, rssi: m.rssi, fp: m.fp_rssi, seq: m.seq, co: m.clock_offset };
    samples.push(s); s.med = rollingMedian(samples.slice(-60), m.t);
  } else if (m.type === "stats") {
    const fail = (m.timeout || 0) + (m.rx_error || 0) + (m.protocol || 0);
    if (fail) fails.push({ t: m.t, n: fail });
    okWin.push({ t: m.t, ok: m.ok, fail }); okWin = okWin.filter(s => s.t > m.t - 5);
  }
  while (samples.length && samples[0].t < latestT - WINDOW - 1) samples.shift();
  while (fails.length && fails[0].t < latestT - WINDOW - 1) fails.shift();
  schedule();
};
</script></body></html>
"""

if __name__ == "__main__":
    main()
