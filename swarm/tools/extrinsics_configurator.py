"""Graphical extrinsics configurator: a browser UI for placing every sensor
frame on the widget (axis mapping + fine Euler trim + offset), with a live
3D preview, persisted to the standard widget-extrinsics JSON schema and
exportable as the firmware C header or WEXT blob.

  bazel run //swarm/tools:extrinsics_configurator -- --config my_widget.json

Open http://<host>:8082/. Drag to orbit. RGB triads = sensor x/y/z axes in
the widget frame; the grey triad is the widget reference frame itself.
Export buttons download the C header (build-time baking) and the WEXT blob
(dynamic load; parsed by swarm/core/src/extrinsics.rs).
"""

import argparse
import json
import os
import shutil
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from swarm.tools import extrinsics

PAGE = """<!DOCTYPE html>
<html><head><title>Widget extrinsics configurator</title><style>
  body { font-family: sans-serif; background: #111; color: #ddd; margin: 20px; }
  canvas { background: #1c2230; border: 1px solid #445; float: left; }
  #panel { margin-left: 560px; }
  table { border-collapse: collapse; font-size: 13px; }
  td, th { padding: 3px 6px; border-bottom: 1px solid #333; }
  input[type=number] { width: 60px; background: #222; color: #ddd;
                       border: 1px solid #444; }
  select { background: #222; color: #ddd; border: 1px solid #444; }
  #err { color: #f66; min-height: 1.2em; }
  a { color: #6af; }
</style></head><body>
<h3>Widget extrinsics configurator</h3>
<canvas id="view" width="540" height="540"></canvas>
<div id="panel">
  <div id="err"></div>
  <table id="tbl"><thead><tr>
    <th>sensor</th><th>+x&rarr;</th><th>+z&rarr;</th>
    <th>roll&deg;</th><th>pitch&deg;</th><th>yaw&deg;</th>
    <th>off x mm</th><th>off y mm</th><th>off z mm</th>
  </tr></thead><tbody></tbody></table>
  <p>
    <a href="/export/header" download="widget_extrinsics.h">export C header</a>
    &nbsp;|&nbsp;
    <a href="/export/blob" download="widget_extrinsics.wext">export WEXT blob</a>
  </p>
  <p style="font-size:12px;color:#889">Convention: v_widget = R &middot;
  v_sensor + offset. Fine Euler trim (roll/pitch/yaw about sensor x/y/z)
  is applied in the sensor frame before the axis mapping. Saved to the
  config file on every change.</p>
</div>
<script>
const AXES = ['+x','-x','+y','-y','+z','-z'];
const canvas = document.getElementById('view'), ctx = canvas.getContext('2d');
let cfg = null, yawV = 0.6, pitchV = 0.4;

function qrot(q, v) {  // rotate v by quaternion [w,x,y,z]
  const [w,x,y,z] = q, [vx,vy,vz] = v;
  const tx = 2*(y*vz - z*vy), ty = 2*(z*vx - x*vz), tz = 2*(x*vy - y*vx);
  return [vx + w*tx + y*tz - z*ty,
          vy + w*ty + z*tx - x*tz,
          vz + w*tz + x*ty - y*tx];
}
function project(p) {  // widget mm -> screen px, orbit view, y-up world z-up
  const cy = Math.cos(yawV), sy = Math.sin(yawV);
  const cp = Math.cos(pitchV), sp = Math.sin(pitchV);
  const x1 = cy*p[0] + sy*p[1], y1 = -sy*p[0] + cy*p[1];
  const y2 = cp*y1 - sp*p[2],  z2 = sp*y1 + cp*p[2];
  const s = 4.0;
  return [270 + s*x1, 270 - s*z2, y2];
}
function line(a, b, color, width) {
  const pa = project(a), pb = project(b);
  ctx.strokeStyle = color; ctx.lineWidth = width || 1.5;
  ctx.beginPath(); ctx.moveTo(pa[0], pa[1]); ctx.lineTo(pb[0], pb[1]);
  ctx.stroke();
}
function draw() {
  ctx.clearRect(0, 0, 540, 540);
  if (!cfg) return;
  // Widget body: 60 x 40 x 15 mm box.
  const hx = 30, hy = 20, hz = 7.5;
  const corners = [];
  for (const sx of [-1,1]) for (const sy of [-1,1]) for (const sz of [-1,1])
    corners.push([sx*hx, sy*hy, sz*hz]);
  const edges = [[0,1],[2,3],[4,5],[6,7],[0,2],[1,3],[4,6],[5,7],
                 [0,4],[1,5],[2,6],[3,7]];
  edges.forEach(([i,j]) => line(corners[i], corners[j], '#556', 1));
  // Widget reference frame triad (grey, at a body corner).
  const o = [-hx, -hy, -hz];
  const axc = ['#999', '#777', '#bbb'];
  [[1,0,0],[0,1,0],[0,0,1]].forEach((ax, i) => {
    line(o, [o[0]+ax[0]*18, o[1]+ax[1]*18, o[2]+ax[2]*18], axc[i], 2);
  });
  ctx.fillStyle = '#999'; ctx.font = '11px sans-serif';
  const lx = project([o[0]+22, o[1], o[2]]), ly = project([o[0], o[1]+22, o[2]]),
        lz = project([o[0], o[1], o[2]+22]);
  ctx.fillText('X', lx[0], lx[1]); ctx.fillText('Y', ly[0], ly[1]);
  ctx.fillText('Z', lz[0], lz[1]);
  // Sensor triads: RGB = sensor x/y/z mapped into the widget frame.
  for (const [name, s] of Object.entries(cfg.sensors)) {
    const off = (s.offset_m || [0,0,0]).map(v => v * 1000);
    const q = s.rotation_wxyz || [1,0,0,0];
    const cols = ['#f55', '#5d5', '#59f'];
    [[1,0,0],[0,1,0],[0,0,1]].forEach((ax, i) => {
      const d = qrot(q, ax);
      line(off, [off[0]+d[0]*25, off[1]+d[1]*25, off[2]+d[2]*25], cols[i], 2.5);
    });
    const lp = project(off);
    ctx.fillStyle = '#ddd';
    ctx.fillText(name, lp[0] + 6, lp[1] - 6);
  }
}
let drag = null;
canvas.onmousedown = (e) => drag = [e.clientX, e.clientY];
window.onmouseup = () => drag = null;
window.onmousemove = (e) => {
  if (!drag) return;
  yawV += (e.clientX - drag[0]) * 0.01;
  pitchV = Math.max(-1.5, Math.min(1.5, pitchV + (e.clientY - drag[1]) * 0.01));
  drag = [e.clientX, e.clientY];
  draw();
};

function buildTable() {
  const tb = document.querySelector('#tbl tbody');
  tb.innerHTML = '';
  for (const [name, s] of Object.entries(cfg.sensors)) {
    const tr = document.createElement('tr');
    const sel = (val, key) => {
      const el = document.createElement('select');
      AXES.forEach(a => {
        const op = document.createElement('option');
        op.value = a; op.textContent = a;
        if (a === val) op.selected = true;
        el.appendChild(op);
      });
      el.onchange = () => { s.mapping[key] = el.value; push(); };
      return el;
    };
    const num = (val, setter, step) => {
      const el = document.createElement('input');
      el.type = 'number'; el.step = step; el.value = val;
      el.onchange = () => { setter(parseFloat(el.value) || 0); push(); };
      return el;
    };
    const cells = [document.createTextNode(name),
                   sel(s.mapping.x, 'x'), sel(s.mapping.z, 'z')];
    s.euler_deg = s.euler_deg || [0,0,0];
    s.offset_m = s.offset_m || [0,0,0];
    for (let i = 0; i < 3; i++)
      cells.push(num(s.euler_deg[i], v => s.euler_deg[i] = v, 0.5));
    for (let i = 0; i < 3; i++)
      cells.push(num(Math.round(s.offset_m[i]*1000*100)/100,
                     v => s.offset_m[i] = v / 1000, 0.5));
    cells.forEach(c => {
      const td = document.createElement('td'); td.appendChild(c);
      tr.appendChild(td);
    });
    tb.appendChild(tr);
  }
}
async function push() {
  const resp = await fetch('/config', {method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify(cfg)});
  const body = await resp.json();
  if (resp.ok) {
    document.getElementById('err').textContent = '';
    cfg = body;
    draw();
  } else {
    document.getElementById('err').textContent = body.error;
  }
}
fetch('/config').then(r => r.json()).then(c => {
  cfg = c; buildTable(); draw();
});
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    config_path = None
    lock = threading.Lock()

    def _send(self, code, body, ctype="application/json", download=None):
        if isinstance(body, str):
            body = body.encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        if download:
            self.send_header("Content-Disposition",
                             'attachment; filename="%s"' % download)
        self.end_headers()
        self.wfile.write(body)

    def _config(self):
        with self.lock:
            return extrinsics.load(self.config_path)

    def do_GET(self):
        if self.path == "/":
            self._send(200, PAGE, "text/html; charset=utf-8")
        elif self.path == "/config":
            self._send(200, json.dumps(extrinsics.derive(self._config())))
        elif self.path == "/export/header":
            self._send(200, extrinsics.c_header(self._config()),
                       "text/plain; charset=utf-8", "widget_extrinsics.h")
        elif self.path == "/export/blob":
            self._send(200, extrinsics.encode_blob(self._config()),
                       "application/octet-stream", "widget_extrinsics.wext")
        else:
            self.send_error(404)

    def do_POST(self):
        if self.path != "/config":
            self.send_error(404)
            return
        n = int(self.headers.get("Content-Length", 0))
        try:
            config = extrinsics.derive(json.loads(self.rfile.read(n)))
        except (ValueError, KeyError) as e:
            self._send(400, json.dumps({"error": str(e)}))
            return
        with self.lock:
            extrinsics.save(config, self.config_path)
        self._send(200, json.dumps(config))

    def log_message(self, fmt, *args):
        pass


def serve(config_path, http_port=8082):
    Handler.config_path = config_path
    httpd = ThreadingHTTPServer(("0.0.0.0", http_port), Handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    return httpd


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default="/tmp/widget_extrinsics.json",
                    help="config JSON to edit (seeded from the example if "
                         "missing)")
    ap.add_argument("--http-port", type=int, default=8082)
    args = ap.parse_args()
    if not os.path.exists(args.config):
        example = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               "examples", "widget_v1.json")
        shutil.copy(example, args.config)
        print("seeded %s from the widget-v1 example" % args.config)
    serve(args.config, args.http_port)
    print("Extrinsics configurator on http://0.0.0.0:%d/  (config: %s)"
          % (args.http_port, args.config))
    try:
        while True:
            time.sleep(3600)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
