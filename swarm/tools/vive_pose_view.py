"""Live window of Vive Tracker positions from libsurvive.

  python3 swarm/tools/vive_pose_view.py [--out truth.jsonl] [--map T20=F694]

Starts vive_truth.py (survive-cli as a subprocess) sending poses to this
window over local UDP, and draws a top-down view with a trail, the height,
and a numeric readout per tracker. --out and --map are passed through, so the
same run records ground truth. With --udp-in PORT it only listens (start
vive_truth.py --udp-out 127.0.0.1:PORT yourself, e.g. on another machine).

libsurvive needs ~15 s with the tracker still and in view to locate the
base station(s) the first time. On macOS the tracker sometimes resets when
survive-cli first switches its light mode, and survive-cli doesn't recover;
the window restarts it when no pose has arrived for --restart-after seconds.
"""

import argparse
import json
import math
import os
import queue
import signal
import socket
import subprocess
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CLI = os.path.expanduser("~/Projects/git_checkouts/libsurvive/build-hid/survive-cli")
COLORS = ["#2a7de1", "#e1572a", "#2aa876", "#a54de1"]
PLANES = {"x-y (z up)": (0, 1, 2), "x-z (y up)": (0, 2, 1), "y-z (x up)": (1, 2, 0)}


def euler_deg(q):
    """(yaw, pitch, roll) in degrees from q = [w, x, y, z], z-up convention."""
    w, x, y, z = q
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    return [math.degrees(a) for a in (yaw, pitch, roll)]


def rotate(q, v):
    w, x, y, z = q
    vx, vy, vz = v
    # v + 2w(u x v) + 2u x (u x v), u = (x, y, z)
    cx, cy, cz = y * vz - z * vy, z * vx - x * vz, x * vy - y * vx
    ccx, ccy, ccz = y * cz - z * cy, z * cx - x * cz, x * cy - y * cx
    return [vx + 2 * (w * cx + ccx), vy + 2 * (w * cy + ccy), vz + 2 * (w * cz + ccz)]


class Source:
    """vive_truth.py child process (unless listen-only) plus the UDP listener."""

    def __init__(self, args, q):
        self.args, self.q = args, q
        self.proc = None
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", args.udp_in or 0))
        self.port = self.sock.getsockname()[1]
        threading.Thread(target=self.listen, daemon=True).start()

    def listen(self):
        while True:
            data, _ = self.sock.recvfrom(65536)
            try:
                rec = json.loads(data)
            except ValueError:
                continue
            if isinstance(rec, dict) and rec.get("type") == "truth":
                self.q.put(rec)

    def start(self):
        if self.args.udp_in:
            return
        self.stop()
        cmd = [sys.executable, os.path.join(HERE, "vive_truth.py"), "--source", "cli",
               "--survive-cli", self.args.survive_cli, "--udp-out", "127.0.0.1:%d" % self.port,
               "--rate", str(self.args.rate)]
        if self.args.out:
            cmd += ["--out", self.args.out]
        for m in self.args.map:
            cmd += ["--map", m]
        cmd += ["--"] + self.args.survive_args
        self.proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                     start_new_session=True)

    def stop(self):
        if self.proc and self.proc.poll() is None:
            os.killpg(self.proc.pid, 15)
            try:
                self.proc.wait(5)
            except subprocess.TimeoutExpired:
                os.killpg(self.proc.pid, 9)
        self.proc = None


class App:
    def __init__(self, root, args):
        self.root, self.args = root, args
        self.q = queue.Queue()
        self.src = Source(args, self.q)
        self.tracks = {}  # dev -> dict(last=rec, trail=[(ht, p)], n=count, color)
        self.origin = [0.0, 0.0, 0.0]
        self.started = time.time()
        self.last_pose = 0.0
        self.restarts = 0
        self.plane = tk.StringVar(value=args.plane)
        self.trail_s = tk.DoubleVar(value=10.0)
        self.build()
        self.src.start()
        root.protocol("WM_DELETE_WINDOW", self.quit)
        # survive-cli runs in its own process group; take it down with us.
        signal.signal(signal.SIGTERM, lambda *_: self.quit())
        signal.signal(signal.SIGINT, lambda *_: self.quit())
        root.after(30, self.pump)
        root.after(1000, self.watchdog)

    def build(self):
        r = self.root
        r.title("Vive tracker positions")
        top = ttk.Frame(r)
        top.pack(fill="x", padx=6, pady=4)
        ttk.Button(top, text="Zero here", command=self.zero).pack(side="left")
        ttk.Button(top, text="Clear zero", command=lambda: self.set_origin([0.0, 0.0, 0.0])).pack(side="left")
        ttk.Button(top, text="Clear trail", command=self.clear).pack(side="left")
        ttk.Button(top, text="Restart libsurvive", command=self.restart).pack(side="left")
        ttk.Label(top, text="  plane:").pack(side="left")
        ttk.OptionMenu(top, self.plane, self.plane.get(), *PLANES).pack(side="left")
        ttk.Label(top, text="  trail s:").pack(side="left")
        ttk.Spinbox(top, from_=0, to=600, width=5, textvariable=self.trail_s).pack(side="left")
        body = ttk.Frame(r)
        body.pack(fill="both", expand=True, padx=6, pady=4)
        self.canvas = tk.Canvas(body, width=720, height=720, background="white", highlightthickness=0)
        self.canvas.pack(side="left", fill="both", expand=True)
        self.info = tk.Text(body, font=("Menlo", 12), width=44)
        self.info.pack(side="right", fill="y")
        self.status = ttk.Label(r, text="")
        self.status.pack(fill="x", padx=6, pady=(0, 4))

    def set_origin(self, p):
        self.origin = list(p)
        self.clear()

    def zero(self):
        if self.tracks:
            first = sorted(self.tracks)[0]
            self.set_origin(self.tracks[first]["last"]["p"])

    def clear(self):
        for tr in self.tracks.values():
            tr["trail"] = []

    def restart(self):
        self.restarts += 1
        self.started = time.time()
        self.src.start()

    def quit(self):
        self.src.stop()
        self.root.destroy()

    def pump(self):
        while True:
            try:
                rec = self.q.get_nowait()
            except queue.Empty:
                break
            dev = rec.get("dev", "?")
            tr = self.tracks.setdefault(dev, dict(trail=[], n=0, color=COLORS[len(self.tracks) % len(COLORS)],
                                                  rate_n=0, rate=0.0, rate_t=time.time()))
            tr["last"] = rec
            tr["n"] += 1
            tr["rate_n"] += 1
            tr["trail"].append((rec["ht"], rec["p"]))
            self.last_pose = time.time()
        keep = self.trail_s.get()
        for tr in self.tracks.values():
            if keep <= 0:
                tr["trail"] = tr["trail"][-1:]
            else:
                cut = time.time() - keep
                i = 0
                while i < len(tr["trail"]) and tr["trail"][i][0] < cut:
                    i += 1
                tr["trail"] = tr["trail"][i:]
            now = time.time()
            if now - tr["rate_t"] >= 1.0:
                tr["rate"] = tr["rate_n"] / (now - tr["rate_t"])
                tr["rate_n"], tr["rate_t"] = 0, now
        self.draw()
        self.root.after(30, self.pump)

    def watchdog(self):
        quiet = time.time() - max(self.last_pose, self.started)
        if not self.args.udp_in and quiet > self.args.restart_after:
            self.restart()
        self.root.after(1000, self.watchdog)

    def rel(self, p):
        return [p[i] - self.origin[i] for i in range(3)]

    def draw(self):
        c = self.canvas
        c.delete("all")
        w, h = max(c.winfo_width(), 100), max(c.winfo_height(), 100)
        a, b, up = PLANES[self.plane.get()]
        pts = [self.rel(p) for tr in self.tracks.values() for _, p in tr["trail"]]
        span = 2.0
        cx = cy = 0.0
        if pts:
            xs, ys = [p[a] for p in pts], [p[b] for p in pts]
            span = max(2.0, (max(xs) - min(xs)) * 1.3, (max(ys) - min(ys)) * 1.3)
            cx, cy = (max(xs) + min(xs)) / 2, (max(ys) + min(ys)) / 2
        scale = min(w, h) / span

        def px(p):
            return w / 2 + (p[a] - cx) * scale, h / 2 - (p[b] - cy) * scale

        step = 0.1 if span < 3 else 0.5 if span < 12 else 1.0
        g = math.floor((cx - span) / step) * step
        while g < cx + span:
            x = w / 2 + (g - cx) * scale
            c.create_line(x, 0, x, h, fill="#e4e4e4" if abs(g / 1.0 - round(g)) > 1e-6 else "#c8c8c8")
            g += step
        g = math.floor((cy - span) / step) * step
        while g < cy + span:
            y = h / 2 - (g - cy) * scale
            c.create_line(0, y, w, y, fill="#e4e4e4" if abs(g / 1.0 - round(g)) > 1e-6 else "#c8c8c8")
            g += step
        ox, oy = px([0.0, 0.0, 0.0])
        c.create_line(ox - 8, oy, ox + 8, oy, fill="#888")
        c.create_line(ox, oy - 8, ox, oy + 8, fill="#888")
        axes = "xyz"
        c.create_text(8, h - 8, anchor="sw", fill="#666", font=("Menlo", 11),
                      text="grid %g m   right = +%s   up = +%s" % (step, axes[a], axes[b]))
        lines = []
        for dev in sorted(self.tracks):
            tr = self.tracks[dev]
            col = tr["color"]
            trail = [px(self.rel(p)) for _, p in tr["trail"]]
            if len(trail) > 1:
                c.create_line(*[v for xy in trail for v in xy], fill=col, width=2)
            rec = tr["last"]
            p = self.rel(rec["p"])
            x, y = px(p)
            fwd = rotate(rec["q"], [1.0, 0.0, 0.0])
            ang = math.atan2(fwd[b], fwd[a])
            c.create_line(x, y, x + 28 * math.cos(ang), y - 28 * math.sin(ang), fill=col, width=3, arrow="last")
            c.create_oval(x - 7, y - 7, x + 7, y + 7, fill=col, outline="white", width=2)
            c.create_text(x + 10, y + 10, anchor="nw", text=dev, fill=col, font=("Menlo", 12, "bold"))
            yaw, pitch, roll = euler_deg(rec["q"])
            age = time.time() - self.last_pose
            lines += ["%s%s" % (dev, "  -> node %s" % rec["node"] if rec.get("node") is not None else ""),
                      "  x %8.1f cm" % (p[0] * 100), "  y %8.1f cm" % (p[1] * 100), "  z %8.1f cm" % (p[2] * 100),
                      "  yaw %6.1f  pitch %6.1f  roll %6.1f" % (yaw, pitch, roll),
                      "  %.0f Hz   %d poses" % (tr["rate"], tr["n"]), ""]
        if self.origin != [0.0, 0.0, 0.0]:
            lines.append("zeroed at libsurvive (%.3f, %.3f, %.3f)" % tuple(self.origin))
        self.info.delete("1.0", "end")
        self.info.insert("end", "\n".join(lines) if lines else
                         "Waiting for libsurvive...\n\nKeep the tracker still and in view\nof the base station; the first fix\ntakes ~15 s.")
        if self.args.udp_in:
            st = "listening on udp %d" % self.src.port
        else:
            st = "libsurvive %s" % ("running" if self.src.proc and self.src.proc.poll() is None else "stopped")
            st += "   restarts %d" % self.restarts
        if self.last_pose:
            st += "   last pose %.1f s ago" % (time.time() - self.last_pose)
        if self.args.out:
            st += "   recording to %s" % self.args.out
        self.status.configure(text=st)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--survive-cli", default=DEFAULT_CLI)
    ap.add_argument("--out", help="also record truth records here (vive_truth.py --out)")
    ap.add_argument("--map", action="append", default=[], metavar="DEV=NODE")
    ap.add_argument("--rate", type=float, default=50.0, help="poses per second per tracker")
    ap.add_argument("--udp-in", type=int, metavar="PORT", help="only listen here; don't start libsurvive")
    ap.add_argument("--restart-after", type=float, default=30.0,
                    help="restart libsurvive after this many seconds without a pose")
    ap.add_argument("--plane", choices=list(PLANES), default="x-y (z up)")
    ap.add_argument("survive_args", nargs="*", help="after --: passed to survive-cli",
                    default=["--lighthouse-gen", "1"])
    a = ap.parse_args()
    root = tk.Tk()
    App(root, a)
    root.mainloop()


if __name__ == "__main__":
    main()
