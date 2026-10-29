"""Splanc tracker demo: a Vive Tracker 3.0 drives a Splanc LED fixture's plane_slice effect.

  # offline: simulated tracker + simulated fixture, nothing leaves the machine
  python3 lighthouse/demos/splanc/app.py --synthetic

  # live poses from swarm/tools/vive_web (must already be running), OSC dry run (logged only)
  python3 lighthouse/demos/splanc/app.py --map pixielamp.csv

  # the real fixture
  python3 lighthouse/demos/splanc/app.py --map pixielamp.csv --fixture 192.168.1.42 --mount-name lamp-top

Serves a local page at http://127.0.0.1:8770 (bound to localhost only): the 3D
workspace, the calibration wizard (§5) and the plane controls. See README.md.
"""

import argparse
import json
import os
import queue
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np

import calib
import frames as fr
import osc
import poses
import probe
from ledmap import LedMap

HERE = os.path.dirname(os.path.abspath(__file__))
MOUNTS = os.path.join(HERE, "mounts")
SESSIONS = os.path.join(HERE, "sessions")
STATE_HZ = 20.0
PARAM_KEEPALIVE = 1.0      # s: /width, /halo, /glow re-send period (they also go out on change)
MAX_RUBS = 20000
GATE_AFTER = 3     # touches; from then on a hold must be near the picked LED's predicted position
GATE_DIST = 0.03   # m
# Tracker button: a press confirms the pose from just before it (the click itself jolts the
# tracker), within these spreads; a second press within DOUBLE_S makes a double press.
BTN_LOOKBACK = (1.5, 0.1)  # s before the press: search the stillest window in this span...
BTN_HOLD = 0.3             # ...of this length; the latest one within the spreads wins
BTN_POS = 0.002            # m, contact point
BTN_ROT = 1.5              # degrees
DOUBLE_S = 0.45
FIXTURE_POSE_MAX_AGE = 0.05  # s: a stylus pose is paired with the fixture tracker's latest pose
                             # only if that is at most this old
TABLE_CLEARANCE = 0.035    # m: LEDs this close above the fixture's lowest LED (in W) aren't
                           # picked: the tracker's body would hit the table


def _f(x):
    return None if x is None else np.asarray(x).tolist()


def _finite(o):
    """NaN/inf -> None: browsers' JSON.parse rejects Python's bare NaN (e.g. the touch RMS
    before the first touch), which would freeze the page."""
    if isinstance(o, float):
        return o if np.isfinite(o) else None
    if isinstance(o, dict):
        return {k: _finite(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_finite(v) for v in o]
    return o


def to_json(obj, **kw):
    return json.dumps(_finite(obj), allow_nan=False, **kw)


class App:
    def __init__(self, ledmap, source, streamer, transport_desc, scale_mode="auto", s_fixed=1.0, device=None,
                 mount_path=None, mount_name="default", fake=None, auto_next=True,
                 table_clearance=TABLE_CLEARANCE, stylus=None):
        self.ledmap, self.source, self.streamer = ledmap, source, streamer
        self.transport_desc = transport_desc
        self.scale_mode, self.s_fixed = scale_mode, s_fixed
        self.device = device
        self.mount_name = mount_name
        self.fake = fake
        self.auto_next = auto_next
        self.table_clearance = table_clearance
        self.lock = threading.RLock()
        self.mode = "idle"
        self.latest = {}             # dev -> (t, M, host time)
        self.rubs = []
        self.touches = []
        self.skip_rows = set()
        self.pick_row = None
        self.fit = None
        self.prev_fit = None
        self.conv = {"why": "no fit yet"}
        self.converged = False
        self.fitting = False
        self.refit_pending = False
        self.hold = calib.HoldDetector()
        self.mount_armed = False
        self.T_TF, self.mount_doc, self.mount_path = None, None, None
        self.events = []             # recent UI events (accept, fit, mount), with a sequence number
        self.seq = 0
        self.rub_version = 0
        self.plane_W = (np.array([0.0, 0.0, 1.0]), 1.0)
        self.plane_F = None
        self.shape = "plane"         # what's streamed live: "plane" (/plane) or "sphere" (/sphere)
        self.sphere_W = (np.array([0.0, 0.0, 1.0]), 0.08)
        self.sphere_F = None
        self.width, self.halo, self.glow = 0.01, 0.03, (1.0, 0.4, 0.1)
        self.T_WF_live = None
        self.T_WF_source = None
        self.recent = deque()        # (host time, T_WT) of the followed tracker, last 2 s
        self.button_control = False  # set by the first press: presses confirm, holds don't
        self.touch_after_fit = False
        self.press_timer = None
        self.clock = time.time       # host time for presses and the poses before them (tests swap it)
        # Stylus calibration (probe.py): `device` is the tracker on the fixture, `stylus` the
        # probe. Its poses are kept relative to the fixture tracker (M = T_WT^-1 T_WS).
        self.stylus = stylus
        self.press_dev = None
        self.recent_rel = deque()    # (host time, M) of the stylus in T, last 2 s
        self.piv = []                # samples of the pivot in progress
        self.piv_row = None          # the LED it pivots on
        self.pivots = []             # [(row, [M...])]
        self.c_tip = None            # stylus tip in its frame, from the pivot (then the fit)
        self.ptouches = []           # [(row, M)]
        self.pfit = self.pprev = None
        self.pconv, self.pconverged = {"why": "no fit yet"}, False
        if mount_path:
            self.load_mount(mount_path)

    # ---------------------------------------------------------------- helpers
    def event(self, kind, **kw):
        with self.lock:
            self.seq += 1
            self.events = (self.events + [{"seq": self.seq, "kind": kind, "t": time.time(), **kw}])[-20:]

    def current_c(self):
        return self.fit.params.c if self.fit else calib.C0

    def static_T_WF(self):
        return self.fit.params.T_WF if self.fit else None

    # ---------------------------------------------------------------- poses
    def on_pose(self, ps):
        M = ps.matrix()
        with self.lock:
            self.latest[ps.dev] = (ps.t, M, time.time())
            if self.device is None and ps.dev != self.stylus:
                self.device = ps.dev
            if ps.dev == self.stylus and ps.dev != self.device:
                self.on_stylus(M)
                return
            if ps.dev != self.device:
                return
            now = self.clock()
            self.recent.append((now, M))
            while self.recent[0][0] < now - BTN_LOOKBACK[0] - 0.5:
                self.recent.popleft()
            mode = self.mode
            if mode == "rub":
                if len(self.rubs) < MAX_RUBS:
                    self.rubs.append(M)
                    if len(self.rubs) % 60 == 0:
                        self.rub_version += 1
            elif mode == "touch":
                if self.pick_row is not None and not self.fitting and not self.button_control:
                    held = self.hold.feed(ps.t, M, self.current_c())
                    if held is not None:
                        far = self.distance_to_pick(held)
                        if len(self.touches) >= GATE_AFTER and far is not None and far > GATE_DIST:
                            # Probably the tracker put down somewhere, not a touch.
                            self.event("reject", row=self.pick_row, dist_mm=far * 1e3)
                        else:
                            self.accept_touch(held, ps.t)
            elif mode == "mount":
                if self.mount_armed:
                    held = self.hold.feed(ps.t, M, np.zeros(3))
                    if held is not None:
                        self.mount_armed = False
                        self.record_mount(held)
            elif mode == "live":
                if self.T_TF is not None:
                    self.T_WF_live = calib.live_fixture_pose(M, self.T_TF)
                    self.T_WF_source = "mount"
                    self.send_plane()

    # ---------------------------------------------------------------- modes
    def set_mode(self, mode):
        with self.lock:
            if mode not in ("idle", "rub", "touch", "mount", "live", "pivot", "probe"):
                raise ValueError(mode)
            if mode in ("pivot", "probe") and (self.stylus is None or self.device is None or
                                               self.stylus == self.device):
                raise ValueError("stylus calibration needs two trackers: --device (on the fixture) and --stylus")
            if mode == "probe" and self.c_tip is None:
                raise ValueError("pivot first")
            if mode == "touch" and self.fit is None:
                raise ValueError("record rubs and fit first")
            if mode == "mount" and self.fit is None:
                raise ValueError("calibrate first")
            if mode == "live" and self.T_TF is None and self.fit is None:
                raise ValueError("no mount record and no fit: nothing places the fixture")
            prev, self.mode = self.mode, mode
            self.hold.reset()
            self.mount_armed = False
            if mode in ("rub", "touch", "mount", "pivot", "probe"):
                # Calibration runs led_pick on the fixture: only /pick matters.
                self.streamer.clear("/plane", "/sphere", "/width", "/halo", "/glow")
                if mode == "touch" and self.pick_row is None:
                    self.next_pick()
                elif mode == "pivot":
                    self.piv = []
                    self.piv_row = self.probe_next_row()
                    self.set_pick(self.piv_row)
                elif mode == "probe":
                    if self.pick_row is None and not self.pconverged:
                        self.probe_next()
                elif mode != "touch":
                    self.streamer.clear("/pick")
            elif mode == "live":
                self.streamer.clear("/pick")
                if self.T_TF is None:
                    self.T_WF_live, self.T_WF_source = self.static_T_WF(), "fit (fixture assumed not moved)"
                self.send_params()
                self.send_plane()
            else:
                self.streamer.clear("/pick", "/plane", "/sphere", "/width", "/halo", "/glow")
            if prev == "rub" and mode != "rub" and self.rubs:
                self.request_fit()
            self.event("mode", mode=mode)

    # ---------------------------------------------------------------- calibration
    def reset_calibration(self):
        with self.lock:
            self.rubs, self.touches, self.skip_rows = [], [], set()
            self.pick_row, self.fit, self.prev_fit = None, None, None
            self.conv, self.converged = {"why": "no fit yet"}, False
            self.rub_version += 1
            self.hold.reset()
            self.streamer.clear("/pick")
            self.event("reset")

    def request_fit(self, multistart=None):
        with self.lock:
            if self.fitting:
                self.refit_pending = True
                return
            self.fitting = True
        threading.Thread(target=self._fit_worker, args=(multistart,), daemon=True, name="fit").start()

    def _fit_worker(self, multistart):
        while True:
            with self.lock:
                rubs = np.array(self.rubs).reshape(-1, 4, 4)
                touches = list(self.touches)
                prev = self.fit
                self.refit_pending = False
            try:
                ms = multistart if multistart is not None else (prev is None or len(touches) <= 3)
                res = calib.fit(self.ledmap, rubs, touches, prev=prev.params if prev else None,
                                scale_mode=self.scale_mode, s_fixed=self.s_fixed, multistart=ms)
                err = None
            except Exception as e:  # noqa: BLE001
                res, err = None, str(e)
            with self.lock:
                if res is not None:
                    self.prev_fit, self.fit = prev, res
                    self.converged, self.conv = calib.converged(self.ledmap, prev, res, len(touches),
                                                                self.scale_mode)
                    if self.mode == "live" and self.T_TF is None:
                        self.T_WF_live = res.params.T_WF
                    self.event("fit", touch_rms_mm=res.touch_rms * 1e3, rub_rms_mm=res.rub_rms * 1e3,
                               converged=self.converged)
                    if self.touch_after_fit and self.mode == "idle":
                        self.touch_after_fit = False
                        self.set_mode("touch")
                else:
                    self.touch_after_fit = False
                    self.event("fit_error", error=err)
                if not self.refit_pending:
                    self.fitting = False
                    if res is not None and self.mode == "touch" and self.auto_next and not self.converged \
                            and self.pick_row is None:
                        self.next_pick()
                    return

    def low_rows(self):
        """LEDs too near the table to touch, from the current fit (W is gravity-aligned)."""
        if self.fit is None or self.table_clearance <= 0:
            return set()
        z = fr.apply(self.fit.params.T_WF, self.ledmap.p)[:, 2]
        return set(np.nonzero(z - z.min() < self.table_clearance)[0].tolist())

    def next_pick(self):
        with self.lock:
            row = calib.next_led(self.ledmap, [t.row for t in self.touches], self.skip_rows | self.low_rows())
            self.set_pick(row)

    def set_pick(self, row):
        with self.lock:
            self.pick_row = row
            self.hold.reset(keep_arm=True)
            if row is not None:
                self.streamer.set("/pick", float(self.ledmap.index[row]))
            self.event("pick", row=row, led=None if row is None else int(self.ledmap.index[row]))

    def distance_to_pick(self, T_WT):
        """Distance from the contact point to where the current fit puts the picked LED."""
        if self.fit is None or self.pick_row is None:
            return None
        s, R, t = fr.sim_parts(self.fit.params.T_WF)
        target = s * R @ self.ledmap.p[self.pick_row] + t + self.fit.params.o * R @ self.ledmap.n[self.pick_row]
        return float(np.linalg.norm(fr.apply(T_WT, self.fit.params.c) - target))

    def arm_mount(self):
        with self.lock:
            if self.mode != "mount":
                raise ValueError("not in mount mode")
            self.hold.reset()
            self.mount_armed = True
            self.event("mount_armed")

    def skip_pick(self):
        with self.lock:
            if self.pick_row is not None:
                self.skip_rows.add(self.pick_row)
            self.next_pick()

    def accept_touch(self, T_WT, t):
        with self.lock:
            row = self.pick_row
            self.touches.append(calib.Touch(row, T_WT, t))
            self.pick_row = None
            self.event("accept", row=row, led=int(self.ledmap.index[row]), n=len(self.touches))
            self.request_fit()

    def accept_now(self):
        """Manual accept: the mean of the last second, whatever its spread."""
        with self.lock:
            buf = [b[3] for b in self.hold.buf]
            if self.pick_row is None or len(buf) < 5:
                raise ValueError("no pick or no recent poses")
            self.accept_touch(fr.mean_pose(np.array(buf)), time.time())

    def drop_touch(self, i, redo=False):
        with self.lock:
            tc = self.touches.pop(int(i))
            self.event("drop", row=tc.row)
            if redo:
                self.set_pick(tc.row)
            self.request_fit()

    def set_scale(self, mode, s_fixed=None):
        with self.lock:
            if mode not in ("auto", "on", "off"):
                raise ValueError(mode)
            self.scale_mode = mode
            if s_fixed is not None:
                self.s_fixed = float(s_fixed)
        if self.fit is not None:
            self.request_fit()

    # ---------------------------------------------------------------- stylus calibration
    def on_stylus(self, M_WS):
        """A stylus pose: relative to the fixture tracker's latest (lock held)."""
        fx = self.latest.get(self.device)
        if fx is None or time.time() - fx[2] > FIXTURE_POSE_MAX_AGE:
            return
        M = probe.relative(fx[1], M_WS)
        now = self.clock()
        self.recent_rel.append((now, M))
        while self.recent_rel[0][0] < now - BTN_LOOKBACK[0] - 0.5:
            self.recent_rel.popleft()
        if self.mode == "pivot" and len(self.piv) < 5000:
            self.piv.append(M)

    def probe_rows(self):
        return [r for r, _ in self.ptouches] + [r for r, _ in self.pivots if r is not None]

    def probe_next_row(self):
        return calib.next_led(self.ledmap, self.probe_rows(), self.skip_rows, reachable_only=False)

    def probe_next(self):
        with self.lock:
            self.set_pick(self.probe_next_row())

    def finish_pivot(self):
        with self.lock:
            if len(self.piv) < 3:
                raise ValueError("no stylus poses yet (are both trackers tracking?)")
            c, q, rms, spread = probe.pivot(self.piv)
            why = probe.pivot_ok(len(self.piv), rms, spread)
            if why:
                self.event("reject", why="pivot: " + why)
                self.piv = []
                return
            self.pivots.append((self.piv_row, list(self.piv)))
            self.c_tip = c
            self.event("pivot", rms_mm=rms * 1e3, spread_deg=spread, tip_mm=(c * 1e3).round(1).tolist())
            self.piv, self.pick_row = [], None
            self.set_mode("probe")

    def confirm_probe(self, t0):
        """Accept the picked LED from the stylus poses before t0: the latest BTN_HOLD window
        within BTN_LOOKBACK where the tip (in the fixture tracker's frame) held still."""
        with self.lock:
            if self.pick_row is None:
                raise ValueError("no LED picked")
            buf = [(h, M) for h, M in self.recent_rel if t0 - BTN_LOOKBACK[0] <= h <= t0 - BTN_LOOKBACK[1]]
            best, end = None, t0 - BTN_LOOKBACK[1]
            while end - BTN_HOLD >= t0 - BTN_LOOKBACK[0]:
                win = [M for h, M in buf if end - BTN_HOLD <= h <= end]
                if len(win) >= 8:
                    dpos, drot = fr.pose_spread(win, self.c_tip)
                    if dpos <= BTN_POS and drot <= BTN_ROT:
                        row = self.pick_row
                        self.ptouches.append((row, fr.mean_pose(np.array(win))))
                        self.pick_row = None
                        self.event("accept", row=row, led=int(self.ledmap.index[row]), n=len(self.ptouches))
                        self.probe_fit()
                        if not self.pconverged:
                            self.probe_next()
                        return
                    if best is None or dpos < best[0]:
                        best = (dpos, drot)
                end -= 0.05
            why = "no stylus poses before the press" if best is None else \
                "moving before the press: stillest %.1f s moved %.1f mm, %.1f°" % (BTN_HOLD, best[0] * 1e3, best[1])
            self.event("reject", row=self.pick_row, why=why)

    def probe_fit(self):
        """Refit after a touch (milliseconds: done inline)."""
        with self.lock:
            rows = self.probe_rows()
            if len(rows) < probe.MIN_TOUCHES:
                return
            est = calib.should_estimate_scale(self.ledmap, [calib.Touch(r, None, 0) for r in rows], self.scale_mode)
            prev = self.pfit if self.pfit is not None and self.pfit.scale_estimated == est else None
            try:
                cur = probe.fit(self.ledmap, self.ptouches, self.pivots, self.c_tip, est, self.s_fixed, prev=prev)
            except Exception as e:  # noqa: BLE001
                self.event("fit_error", error=str(e))
                return
            self.pprev, self.pfit = self.pfit, cur
            self.c_tip = cur.c
            self.pconverged, self.pconv = probe.converged(self.ledmap, self.pprev, cur, len(rows), self.scale_mode)
            self.event("fit", touch_rms_mm=cur.rms * 1e3, h_mm=cur.h * 1e3, s=cur.s, converged=self.pconverged)

    def drop_probe_touch(self, i):
        with self.lock:
            row, _ = self.ptouches.pop(int(i))
            self.event("drop", row=row)
            self.pfit = self.pprev = None
            self.pconverged = False
            self.probe_fit()

    def save_probe_mount(self):
        """The fit is T_TF: save it as the mount and go live."""
        with self.lock:
            if self.pfit is None:
                raise ValueError("no stylus fit yet")
            T_TF = self.pfit.T_TF
            os.makedirs(MOUNTS, exist_ok=True)
            path = os.path.join(MOUNTS, "%s.json" % self.mount_name)
            doc = calib.save_mount(path, T_TF, None, self.ledmap, device=self.device, name=self.mount_name, extra={
                "calibration": self.pfit.to_json(), "method": "stylus", "stylus": self.stylus,
                "converged": self.pconverged, "convergence": self.pconv,
                "probe_touches": [{"row": int(r), "led": int(self.ledmap.index[r]), "M_TS": M.tolist()}
                                  for r, M in self.ptouches],
                "pivots": [{"row": None if r is None else int(r), "n": len(P)} for r, P in self.pivots]})
            self.T_TF, self.mount_doc, self.mount_path = T_TF, doc, path
            self.pick_row = None
            self.event("mount", path=os.path.relpath(path, HERE))
            self.set_mode("live")

    def reset_probe(self):
        with self.lock:
            self.piv, self.pivots, self.ptouches = [], [], []
            self.c_tip = self.pfit = self.pprev = None
            self.pconv, self.pconverged = {"why": "no fit yet"}, False
            self.skip_rows, self.pick_row = set(), None
            self.streamer.clear("/pick")
            if self.mode in ("pivot", "probe"):
                self.set_mode("idle")
            self.event("reset")

    def probe_state(self):
        fx = self.latest.get(self.device)
        st = self.latest.get(self.stylus) if self.stylus else None
        tip_W = None
        if st is not None and self.c_tip is not None:
            tip_W = fr.apply(st[1], self.c_tip).tolist()
        return {
            "stylus": self.stylus, "fixture_tracker": self.device,
            "pivot_samples": len(self.piv), "pivot_row": self.piv_row,
            "pivots": [{"row": r, "n": len(P)} for r, P in self.pivots],
            "c_tip_mm": None if self.c_tip is None else (self.c_tip * 1e3).round(2).tolist(),
            "tip_W": tip_W,
            "touches": [{"i": i, "row": int(r), "led": int(self.ledmap.index[r]),
                         "res_mm": None if self.pfit is None or i >= len(self.pfit.residuals)
                         else float(np.linalg.norm(self.pfit.residuals[i]) * 1e3)}
                        for i, (r, _) in enumerate(self.ptouches)],
            "fit": None if self.pfit is None else self.pfit.to_json(),
            "converged": self.pconverged, "conv": self.pconv,
            "T_WF": None if fx is None or self.pfit is None else _f(fx[1] @ self.pfit.T_TF),
        }

    # ---------------------------------------------------------------- tracker button
    def on_button(self, dev, n=1):
        """n new presses of the followed tracker's button. A single press waits DOUBLE_S for
        a second one; the pose it confirms is taken from before the first press either way."""
        with self.lock:
            if dev == self.stylus and self.stylus is not None:
                if self.mode not in ("idle", "pivot", "probe"):
                    return
            elif self.device is not None and dev != self.device:
                return
            self.press_dev = dev
            now = self.clock()
            self.button_control = True
            if self.press_timer is not None:
                self.press_timer.cancel()
                t0, self.press_timer = self.press_timer.t0, None
                self._press(2, t0)
            elif n >= 2:
                self._press(2, now)
            else:
                self.press_timer = threading.Timer(DOUBLE_S, self._single, args=(now,))
                self.press_timer.t0 = now
                self.press_timer.daemon = True
                self.press_timer.start()

    def _single(self, t0):
        with self.lock:
            self.press_timer = None
            self._press(1, t0)

    def _press(self, n, t0):
        """What a press does depends on the step: rub → touches → mount → record."""
        with self.lock:
            mode = self.mode
            self.event("button", n=n, mode=mode)
            try:
                if self.press_dev == self.stylus and self.stylus is not None:
                    self._press_stylus(n, t0)
                    return
                if n == 2:
                    if mode == "touch" and self.pick_row is not None:
                        self.skip_pick()
                    return
                if mode == "idle":
                    self.set_mode("rub")
                elif mode == "rub":
                    # Stop & fit; the fit then starts the touches.
                    self.touch_after_fit = bool(self.rubs)
                    self.set_mode("idle")
                elif mode == "touch":
                    if self.pick_row is not None:
                        self.confirm_touch(t0)
                    elif self.converged:
                        self.set_mode("mount")
                    elif not self.fitting:
                        self.next_pick()
                elif mode == "mount":
                    if not self.mount_armed:
                        self.arm_mount()
            except ValueError as e:
                self.event("warning", text=str(e))

    def _press_stylus(self, n, t0):
        """Stylus button: idle → pivot → (press) finish → probe: press confirms the blinking
        LED, double press skips it; converged: press saves the mount and goes live."""
        mode = self.mode
        if mode == "idle":
            if n == 1:
                self.set_mode("pivot" if self.c_tip is None else "probe")
        elif mode == "pivot":
            if n == 2:
                self.piv = []          # start the rocking again
                self.event("pivot_restart")
            else:
                self.finish_pivot()
        elif mode == "probe":
            if n == 2:
                if self.pick_row is not None:
                    self.skip_rows.add(self.pick_row)
                    self.probe_next()
            elif self.pick_row is not None:
                self.confirm_probe(t0)
            elif self.pconverged:
                self.save_probe_mount()
            else:
                self.probe_next()

    def confirm_touch(self, t0):
        """Accept the pick from the poses before the press at host time t0: the latest
        BTN_HOLD window within BTN_LOOKBACK that is still enough (pushing the button down
        rocks the tracker for a moment before the click registers)."""
        with self.lock:
            buf = [(h, M) for h, M in self.recent if t0 - BTN_LOOKBACK[0] <= h <= t0 - BTN_LOOKBACK[1]]
            best = None
            end = t0 - BTN_LOOKBACK[1]
            while end - BTN_HOLD >= t0 - BTN_LOOKBACK[0]:
                win = [M for h, M in buf if end - BTN_HOLD <= h <= end]
                if len(win) >= 8:
                    dpos, drot = fr.pose_spread(win, self.current_c())
                    if dpos <= BTN_POS and drot <= BTN_ROT:
                        self.accept_touch(fr.mean_pose(np.array(win)), t0)
                        return
                    if best is None or dpos < best[0]:
                        best = (dpos, drot)
                end -= 0.05
            why = "no poses before the press" if best is None else \
                "moving before the press: stillest %.1f s moved %.1f mm, %.1f°" % (BTN_HOLD, best[0] * 1e3, best[1])
            self.event("reject", row=self.pick_row, why=why)

    def button_hint(self):
        """What the next press does, for the page."""
        mode = self.mode
        if mode == "pivot":
            return "stylus: rest the tip on LED %s and rock it round (±20° or more), press: done · double: restart" % (
                "?" if self.piv_row is None else int(self.ledmap.index[self.piv_row]))
        if mode == "probe":
            if self.pick_row is not None:
                return "stylus: tip on LED %d, hold still, press: confirm · double: skip" % int(
                    self.ledmap.index[self.pick_row])
            return "converged · stylus press: save mount & go live" if self.pconverged else "stylus press: next LED"
        if mode == "idle":
            if self.stylus is not None:
                return "stylus press: start (pivot)" if self.c_tip is None else "stylus press: continue probing"
            return "fitting…" if self.touch_after_fit else "press: start rubbing"
        if mode == "rub":
            return "press: stop & fit, then touches"
        if mode == "touch":
            if self.pick_row is not None:
                return "hold on LED %d, press: confirm · double: skip" % int(self.ledmap.index[self.pick_row])
            if self.converged:
                return "converged · stick the tracker on the fixture, press: mount mode"
            return "fitting…" if self.fitting else "press: pick next LED"
        if mode == "mount":
            return "hold still 1 s…" if self.mount_armed else "attach the tracker, press: record (then hold still)"
        return "live"

    # ---------------------------------------------------------------- mount
    def record_mount(self, T_WT):
        with self.lock:
            T_TF = calib.mount_transform(T_WT, self.fit.params.T_WF)
            os.makedirs(MOUNTS, exist_ok=True)
            path = os.path.join(MOUNTS, "%s.json" % self.mount_name)
            doc = calib.save_mount(path, T_TF, self.fit, self.ledmap, device=self.device, name=self.mount_name,
                                   touches=self.touches,
                                   extra={"converged": self.converged, "convergence": self.conv,
                                          "T_WT_at_mount": T_WT.tolist()})
            self.T_TF, self.mount_doc, self.mount_path = T_TF, doc, path
            self.event("mount", path=os.path.relpath(path, HERE))
            self.set_mode("live")

    def load_mount(self, path):
        T_TF, doc = calib.load_mount(path)
        with self.lock:
            self.T_TF, self.mount_doc, self.mount_path = T_TF, doc, path
            if doc.get("device") and self.device is None:
                self.device = doc["device"]
            m = doc.get("map") or {}
            if m.get("source", {}) and self.ledmap.source and m["source"].get("sha256_16") != \
                    self.ledmap.source.get("sha256_16"):
                self.event("warning", text="mount file was calibrated against a different LED map")

    def save_session(self):
        os.makedirs(SESSIONS, exist_ok=True)
        with self.lock:
            path = os.path.join(SESSIONS, time.strftime("%Y%m%d-%H%M%S") + "_calib.json")
            doc = {"map": self.ledmap.summary(), "device": self.device,
                   "rubs": np.array(self.rubs).reshape(-1, 4, 4).tolist(),
                   "touches": [t.to_json(self.ledmap) for t in self.touches],
                   "fit": self.fit.to_json() if self.fit else None, "scale_mode": self.scale_mode}
        with open(path, "w") as f:
            json.dump(doc, f)
        return path

    # ---------------------------------------------------------------- plane
    def set_plane(self, n, d):
        n = np.asarray(n, float)
        norm = np.linalg.norm(n)
        if not norm > 0:
            raise ValueError("zero normal")
        with self.lock:
            self.plane_W = (n / norm, float(d) / norm)
            self.send_plane()

    def set_sphere(self, c, r):
        c = np.asarray(c, float)
        if c.shape != (3,) or not float(r) > 0:
            raise ValueError("sphere needs a centre (3) and a radius > 0")
        with self.lock:
            self.sphere_W = (c, float(r))
            self.send_plane()

    def set_shape(self, shape):
        if shape not in ("plane", "sphere"):
            raise ValueError(shape)
        with self.lock:
            self.shape = shape
            self.send_plane()

    def set_params(self, width=None, halo=None, glow=None):
        with self.lock:
            if width is not None:
                self.width = float(width)
            if halo is not None:
                self.halo = float(halo)
            if glow is not None:
                self.glow = tuple(float(g) for g in glow)
            self.send_params()

    def send_params(self):
        if self.mode == "live":
            # Settings: on change, and once a second in case a datagram was lost.
            self.streamer.set("/width", self.width, keepalive=PARAM_KEEPALIVE)
            self.streamer.set("/halo", self.halo, keepalive=PARAM_KEEPALIVE)
            self.streamer.set("/glow", *self.glow, keepalive=PARAM_KEEPALIVE)

    def send_plane(self):
        """Streams the live shape (plane or sphere) in F; the fixture runs the matching effect."""
        T_WF = self.T_WF_live
        if T_WF is None:
            self.plane_F = self.sphere_F = None
            return
        n_F, d_F = fr.plane_to_fixture(T_WF, *self.plane_W)
        self.plane_F = (n_F, d_F)
        c_F, r_F = fr.sphere_to_fixture(T_WF, *self.sphere_W)
        self.sphere_F = (c_F, r_F)
        if self.mode == "live":
            if self.shape == "sphere":
                self.streamer.clear("/plane")
                self.streamer.set("/sphere", c_F[0], c_F[1], c_F[2], r_F)
            else:
                self.streamer.clear("/sphere")
                self.streamer.set("/plane", n_F[0], n_F[1], n_F[2], d_F)

    # ---------------------------------------------------------------- state for the page
    def state(self):
        with self.lock:
            now = time.time()
            dev = self.latest.get(self.device)
            pst = self.probe_state()
            if self.mode == "live":
                T_WF, T_WF_src = self.T_WF_live, self.T_WF_source
            elif self.mode in ("pivot", "probe") and pst["T_WF"] is not None:
                T_WF, T_WF_src = np.array(pst["T_WF"]), "stylus fit"
            elif self.mode == "idle" and self.T_TF is not None and dev is not None:
                T_WF, T_WF_src = calib.live_fixture_pose(dev[1], self.T_TF), "mount"
            else:
                T_WF, T_WF_src = self.static_T_WF(), ("fit" if self.fit else None)
            span = calib.scale_span(self.ledmap, [t.row for t in self.touches])
            pick = None if self.pick_row is None else {"row": self.pick_row, "led": int(self.ledmap.index[self.pick_row])}
            pred = None
            if self.pick_row is not None and self.fit is not None:
                s, R, t = fr.sim_parts(self.fit.params.T_WF)
                pred = (s * R @ self.ledmap.p[self.pick_row] + t + self.fit.params.o * R @
                        self.ledmap.n[self.pick_row]).tolist()
            contact = None
            if dev is not None:
                contact = fr.apply(dev[1], self.current_c()).tolist()
            st = {
                "type": "state", "now": now, "mode": self.mode, "device": self.device,
                "devices": {d: round(now - v[2], 2) for d, v in self.latest.items()},
                "source": self.source.status(),
                "pose": None if dev is None else {"M": dev[1].tolist(), "age": now - dev[2], "contact": contact},
                "T_WF": _f(T_WF),
                "T_WF_source": T_WF_src,
                "rubs": len(self.rubs), "rub_version": self.rub_version,
                "fitting": self.fitting,
                "fit": self.fit.to_json() if self.fit else None,
                "converged": self.converged, "conv": self.conv,
                "touches": [{"i": i, "row": t.row, "led": int(self.ledmap.index[t.row]),
                             "res_mm": None if t.residual is None else float(np.linalg.norm(t.residual) * 1e3),
                             "res_vec_mm": None if t.residual is None else (t.residual * 1e3).round(2).tolist(),
                             "tilt_deg": float(np.degrees(np.arccos(np.clip(
                                 -(t.T_WT[:3, :3] @ calib.CONTACT_AXIS_T) @ (self._normal_W(t.row)), -1, 1))))
                             if self.fit else None}
                            for i, t in enumerate(self.touches)],
                "pick": pick, "pick_pred_W": pred, "hold": self.hold.status,
                "scale": {"mode": self.scale_mode, "s_fixed": self.s_fixed, "span": span,
                          "span_needed": calib.SCALE_MIN_SPAN,
                          "estimated": bool(self.fit and self.fit.scale_estimated)},
                "plane": {"n_W": self.plane_W[0].tolist(), "d_W": self.plane_W[1],
                          "n_F": None if self.plane_F is None else self.plane_F[0].tolist(),
                          "d_F": None if self.plane_F is None else self.plane_F[1],
                          "width": self.width, "halo": self.halo, "glow": list(self.glow)},
                "shape": self.shape,
                "probe": pst,
                "sphere": {"c_W": self.sphere_W[0].tolist(), "r_W": self.sphere_W[1],
                           "c_F": None if self.sphere_F is None else self.sphere_F[0].tolist(),
                           "r_F": None if self.sphere_F is None else self.sphere_F[1]},
                "osc": {"target": self.transport_desc, "datagrams": self.streamer.sent,
                        "values": {k: list(v) for k, v in self.streamer.values.items()},
                        "recent": getattr(getattr(self, "dry_log", None), "recent", [])[-6:]},
                "mount": None if self.T_TF is None else {
                    "path": self.mount_path, "name": (self.mount_doc or {}).get("name"),
                    "created": (self.mount_doc or {}).get("created"), "s": fr.sim_parts(self.T_TF)[0]},
                "mount_name": self.mount_name, "mount_armed": self.mount_armed,
                "pick_dist_mm": None if dev is None or self.mode != "touch" or self.distance_to_pick(dev[1]) is None
                else self.distance_to_pick(dev[1]) * 1e3,
                "events": self.events[-10:],
                "button": {"control": self.button_control, "hint": self.button_hint()},
                "fake": None if self.fake is None else {"lit": self.fake.lit().tolist(),
                                                        "received": self.fake.received,
                                                        "pick_row": self.fake.pick_row()},
            }
        return st

    def _normal_W(self, row):
        s, R, t = fr.sim_parts(self.fit.params.T_WF)
        return R @ self.ledmap.n[row]

    def rub_points(self, max_n=2000):
        with self.lock:
            if not self.rubs:
                return []
            idx = np.linspace(0, len(self.rubs) - 1, min(max_n, len(self.rubs))).astype(int)
            R = np.array([self.rubs[i] for i in idx])
            c = self.current_c()
        return (np.einsum("nij,j->ni", R[:, :3, :3], c) + R[:, :3, 3]).round(5).tolist()


# ---------------------------------------------------------------- HTTP

def make_handler(app):
    index = os.path.join(HERE, "static", "index.html")

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _json(self, obj, code=200):
            body = to_json(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                body = open(index, "rb").read()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif self.path == "/map":
                m = app.ledmap
                self._json({"index": m.index.tolist(), "p": m.p.round(6).tolist(), "n": m.n.round(4).tolist(),
                            "mapped": m.mapped.astype(int).tolist(), "reachable": m.reachable.astype(int).tolist(),
                            "summary": m.summary()})
            elif self.path == "/rubs":
                self._json({"points": app.rub_points(), "version": app.rub_version})
            elif self.path == "/state":
                self._json(app.state())
            elif self.path == "/events":
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                try:
                    while True:
                        msg = to_json(app.state(), separators=(",", ":"))
                        self.wfile.write(b"data: " + msg.encode() + b"\n\n")
                        self.wfile.flush()
                        time.sleep(1.0 / STATE_HZ)
                except (BrokenPipeError, ConnectionResetError):
                    pass
            else:
                self.send_error(404)

        def do_POST(self):
            n = int(self.headers.get("Content-Length") or 0)
            try:
                body = json.loads(self.rfile.read(n) or b"{}")
            except ValueError:
                body = {}
            p = self.path
            try:
                out = {}
                if p == "/api/mode":
                    app.set_mode(body["mode"])
                elif p == "/api/device":
                    with app.lock:
                        app.device = body["dev"]
                elif p == "/api/reset":
                    app.reset_calibration()
                elif p == "/api/fit":
                    app.request_fit(multistart=True)
                elif p == "/api/next":
                    app.next_pick()
                elif p == "/api/pick":
                    app.set_pick(app.ledmap.row_of(body["led"]))
                elif p == "/api/mount_record":
                    app.arm_mount()
                elif p == "/api/skip":
                    app.skip_pick()
                elif p == "/api/button":
                    app.on_button(app.device, int(body.get("n", 1)))
                elif p == "/api/accept":
                    app.accept_now()
                elif p == "/api/drop":
                    app.drop_touch(body["i"], redo=bool(body.get("redo")))
                elif p == "/api/scale":
                    app.set_scale(body["mode"], body.get("s_fixed"))
                elif p == "/api/auto_next":
                    app.auto_next = bool(body["on"])
                elif p == "/api/mount_name":
                    app.mount_name = "".join(ch for ch in str(body["name"]) if ch.isalnum() or ch in "-_") or "default"
                elif p == "/api/plane":
                    app.set_plane(body["n"], body["d"])
                elif p == "/api/probe/finish_pivot":
                    app.finish_pivot()
                elif p == "/api/probe/accept":
                    app.confirm_probe(app.clock())
                elif p == "/api/probe/skip":
                    with app.lock:
                        if app.pick_row is not None:
                            app.skip_rows.add(app.pick_row)
                        app.probe_next()
                elif p == "/api/probe/drop":
                    app.drop_probe_touch(body["i"])
                elif p == "/api/probe/save":
                    app.save_probe_mount()
                elif p == "/api/probe/reset":
                    app.reset_probe()
                elif p == "/api/stylus":
                    with app.lock:
                        app.stylus = body["dev"] or None
                elif p == "/api/sphere":
                    app.set_sphere(body["c"], body["r"])
                elif p == "/api/shape":
                    app.set_shape(body["shape"])
                elif p == "/api/params":
                    app.set_params(body.get("width"), body.get("halo"), body.get("glow"))
                elif p == "/api/session":
                    out["path"] = app.save_session()
                else:
                    return self.send_error(404)
                self._json({"ok": True, **out})
            except (ValueError, KeyError, IndexError) as e:
                self._json({"ok": False, "error": str(e)}, 400)

    return Handler


def parse_target(s):
    host, _, port = s.partition(":")
    return host, int(port or osc.OSC_PORT)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--map", help="LED map CSV from Splanc export_map (format csv)")
    ap.add_argument("--table-clearance", type=float, default=TABLE_CLEARANCE,
                    help="m: don't pick LEDs this close above the fixture's lowest LED (0: off)")
    ap.add_argument("--exclude", default="",
                    help="LED indices never to pick for touches, e.g. 0-12,240 (hidden inside the fixture)")
    ap.add_argument("--synthetic", action="store_true",
                    help="simulated tracker and fixture (synthetic cylinder lamp unless --map is given)")
    ap.add_argument("--source", default="http://127.0.0.1:8765",
                    help="pose source: vive_web server URL (SSE, read-only), or a file of POSE lines to replay")
    ap.add_argument("--world-up", choices=["z", "y"], default="z",
                    help="up axis of the source's world: z (libsurvive, default) or y (SteamVR; converted to z)")
    ap.add_argument("--device", help="tracker id to follow (default: the first one seen, or the mount file's); "
                                     "with --stylus, the tracker mounted on the fixture")
    ap.add_argument("--stylus", help="tracker id of the probe stylus: calibrate by touching LEDs with its tip "
                                     "(pivot, then probe), no rubbing")
    ap.add_argument("--fixture", help="fixture address host[:port] for OSC (port 9000 by default). "
                                      "Without it nothing is sent on the network (dry run).")
    ap.add_argument("--dry-run", action="store_true", help="log OSC datagrams (also when --fixture is given)")
    ap.add_argument("--osc-log", default=os.path.join(HERE, "osc-dry-run.log"), help="dry-run log file")
    ap.add_argument("--max-hz", type=float, default=osc.MAX_HZ, help="OSC rate cap (<= 120)")
    ap.add_argument("--keepalive-hz", type=float, default=10.0, help="OSC re-send rate while idle (>= 5)")
    ap.add_argument("--mount", help="mount record JSON to load (T_TF)")
    ap.add_argument("--mount-name", default="default",
                    help="name for the mount record (mounts/<name>.json; loaded at startup if it exists)")
    ap.add_argument("--scale", choices=["auto", "on", "off"], default="auto",
                    help="estimate the map scale s: auto (once touches span the fixture), on, off (use --s)")
    ap.add_argument("--s", type=float, default=1.0, help="map scale when not estimated (e.g. from a ruler)")
    ap.add_argument("--port", type=int, default=8770)
    a = ap.parse_args()

    fake = None
    if a.synthetic:
        import synth
        lm, true_p, true_n, geom = synth.cylinder_lamp()
        if a.map:
            print("--synthetic with --map: the simulation uses its own cylinder lamp; ignoring --map")
        ledmap = lm
        truth = synth.Truth(geom)
        sim = synth.Sim(truth, true_p, true_n)
        fake = synth.FakeFixture(ledmap)
    else:
        if not a.map:
            ap.error("--map is required (or --synthetic)")
        ledmap = LedMap.from_csv(a.map)
    for part in filter(None, a.exclude.split(",")):
        lo, _, hi = part.partition("-")
        for i in range(int(lo), int(hi or lo) + 1):
            ledmap.reachable[ledmap.row_of(i)] = False

    dry = osc.LogTransport(a.osc_log) if (a.dry_run or not a.fixture) else None
    udp = osc.UdpTransport(*parse_target(a.fixture)) if a.fixture else None
    fake_t = osc.CallbackTransport(fake.receive, "synthetic fixture") if fake else None
    transport = osc.Tee(udp, fake_t, dry)
    streamer = osc.OscStreamer(transport, max_hz=a.max_hz, keepalive_hz=a.keepalive_hz).start()

    mount_path = a.mount
    if not mount_path:
        cand = os.path.join(MOUNTS, "%s.json" % a.mount_name)
        if os.path.exists(cand) and not a.synthetic:
            mount_path = cand

    holder = {}
    if a.synthetic:
        script = lambda: holder["app"].mode  # noqa: E731
        source = poses.SyntheticSource(sim, script, fake.pick_row)
    elif a.source.startswith("http://") or a.source.startswith("https://"):
        source = poses.SseSource(a.source, world_up=a.world_up)
    else:
        source = poses.ReplaySource(a.source)

    app = App(ledmap, source, streamer, transport.describe(), scale_mode=a.scale, s_fixed=a.s, device=a.device,
              mount_path=mount_path, mount_name=a.mount_name, fake=fake, table_clearance=a.table_clearance,
              stylus=a.stylus)
    app.dry_log = dry
    holder["app"] = app
    source.start(app.on_pose, app.on_button)

    server = ThreadingHTTPServer(("127.0.0.1", a.port), make_handler(app))
    server.daemon_threads = True
    print("map: %s" % json.dumps(ledmap.summary()))
    print("pose source: %s" % source.name)
    print("OSC: %s" % transport.describe())
    if mount_path:
        print("mount record: %s" % mount_path)
    print("serving http://127.0.0.1:%d/" % a.port, flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        source.stop()
        streamer.stop()


if __name__ == "__main__":
    main()
