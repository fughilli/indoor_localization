import sys

p = sys.argv[1]
s = open(p).read()


def rep(a, b, n=1):
    global s
    assert s.count(a) == n, (s.count(a), a[:80])
    s = s.replace(a, b)


rep("import poses\n", "import poses\nimport probe\n")
rep("TABLE_CLEARANCE = 0.035", """FIXTURE_POSE_MAX_AGE = 0.05  # s: a stylus pose is paired with the fixture tracker's latest pose
                             # only if that is at most this old
TABLE_CLEARANCE = 0.035""")
rep("""                 mount_path=None, mount_name="default", fake=None, auto_next=True,
                 table_clearance=TABLE_CLEARANCE):""", """                 mount_path=None, mount_name="default", fake=None, auto_next=True,
                 table_clearance=TABLE_CLEARANCE, stylus=None):""")
rep("""        self.clock = time.time       # host time for presses and the poses before them (tests swap it)
""", """        self.clock = time.time       # host time for presses and the poses before them (tests swap it)
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
""")
rep("""            if self.device is None:
                self.device = ps.dev
            if ps.dev != self.device:
                return""", """            if self.device is None and ps.dev != self.stylus:
                self.device = ps.dev
            if ps.dev == self.stylus and ps.dev != self.device:
                self.on_stylus(M)
                return
            if ps.dev != self.device:
                return""")
rep("""            if mode not in ("idle", "rub", "touch", "mount", "live"):
                raise ValueError(mode)""", """            if mode not in ("idle", "rub", "touch", "mount", "live", "pivot", "probe"):
                raise ValueError(mode)
            if mode in ("pivot", "probe") and (self.stylus is None or self.device is None or
                                               self.stylus == self.device):
                raise ValueError("stylus calibration needs two trackers: --device (on the fixture) and --stylus")
            if mode == "probe" and self.c_tip is None:
                raise ValueError("pivot first")""")
rep("""            if mode in ("rub", "touch", "mount"):
                # Calibration runs led_pick on the fixture: only /pick matters.
                self.streamer.clear("/plane", "/sphere", "/width", "/halo", "/glow")
                if mode == "touch" and self.pick_row is None:
                    self.next_pick()
                elif mode != "touch":
                    self.streamer.clear("/pick")""", """            if mode in ("rub", "touch", "mount", "pivot", "probe"):
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
                    self.streamer.clear("/pick")""")
rep("""    # ---------------------------------------------------------------- tracker button""", '''    # ---------------------------------------------------------------- stylus calibration
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
            why = "no stylus poses before the press" if best is None else \\
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

    # ---------------------------------------------------------------- tracker button''')
rep("""            if self.device is not None and dev != self.device:
                return
            now = self.clock()""", """            if dev == self.stylus and self.stylus is not None:
                if self.mode not in ("idle", "pivot", "probe"):
                    return
            elif self.device is not None and dev != self.device:
                return
            self.press_dev = dev
            now = self.clock()""")
rep("""            try:
                if n == 2:
                    if mode == "touch" and self.pick_row is not None:
                        self.skip_pick()
                    return
                if mode == "idle":""", """            try:
                if self.press_dev == self.stylus and self.stylus is not None:
                    self._press_stylus(n, t0)
                    return
                if n == 2:
                    if mode == "touch" and self.pick_row is not None:
                        self.skip_pick()
                    return
                if mode == "idle":""")
rep("""    def confirm_touch(self, t0):""", '''    def _press_stylus(self, n, t0):
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

    def confirm_touch(self, t0):''')
rep('''        mode = self.mode
        if mode == "idle":
            return "fitting…" if self.touch_after_fit else "press: start rubbing"''', '''        mode = self.mode
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
            return "fitting…" if self.touch_after_fit else "press: start rubbing"''')
rep('''            if self.mode == "live":
                T_WF, T_WF_src = self.T_WF_live, self.T_WF_source''', '''            pst = self.probe_state()
            if self.mode == "live":
                T_WF, T_WF_src = self.T_WF_live, self.T_WF_source
            elif self.mode in ("pivot", "probe") and pst["T_WF"] is not None:
                T_WF, T_WF_src = np.array(pst["T_WF"]), "stylus fit"''')
rep('''                "shape": self.shape,''', '''                "shape": self.shape,
                "probe": pst,''')
rep('''                elif p == "/api/sphere":''', '''                elif p == "/api/probe/finish_pivot":
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
                elif p == "/api/sphere":''')
rep('''    ap.add_argument("--device", help="tracker id to follow (default: the first one seen, or the mount file's)")''',
    '''    ap.add_argument("--device", help="tracker id to follow (default: the first one seen, or the mount file's); "
                                     "with --stylus, the tracker mounted on the fixture")
    ap.add_argument("--stylus", help="tracker id of the probe stylus: calibrate by touching LEDs with its tip "
                                     "(pivot, then probe), no rubbing")''')
rep('''              mount_path=mount_path, mount_name=a.mount_name, fake=fake, table_clearance=a.table_clearance)''',
    '''              mount_path=mount_path, mount_name=a.mount_name, fake=fake, table_clearance=a.table_clearance,
              stylus=a.stylus)''')
open(p, "w").write(s)
print("patched")
