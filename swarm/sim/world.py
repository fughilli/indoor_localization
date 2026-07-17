"""Station-free swarm simulation world.

Models N widgets moving through an arena, each carrying a 9-axis IMU, a UWB
radio (two-way ranging + AoA), and optionally GPS / altimeter. Every widget
runs its own Rust ESKF (swarm/core via ctypes) fed only what real firmware
would see:

  - raw IMU samples at `imu_rate`,
  - UWB range + AoA measurements produced by a TDMA slot schedule (one
    ranging pair per slot, round-robin over all pairs in radio range),
  - the *advertised* pose (position + mean position variance) of the ranging
    peer, as broadcast at `broadcast_rate` — never the peer's true state,
  - GPS fixes / altimeter / magnetometer per its equipage.

Truth kinematics are closed-form trajectories differentiated numerically, so
new trajectory shapes only need a position function. Attitude truth is
yaw-only (constant spin), which exercises the attitude filter without full
6-DOF flight dynamics.
"""

import dataclasses
import math

import numpy as np

from swarm.sim import estimator

GRAVITY = 9.80665
G_WORLD = np.array([0.0, 0.0, -GRAVITY])
# World magnetic field direction (unit): north-ish with downward dip.
MAG_WORLD = np.array([1.0, 0.0, -0.45]) / np.linalg.norm([1.0, 0.0, -0.45])


@dataclasses.dataclass
class SensorConfig:
    """Per-sample noise magnitudes (not densities) and rates."""
    # Accept a relative (range/AoA) update only when the peer's advertised
    # position variance is below this fraction of my own — information flows
    # "downhill" from absolutely-anchored widgets. This blocks the circular
    # double-counting that collapses a naive-KF swarm onto a mutually-
    # consistent-but-wrong consensus. 0 disables the rule (unstable; kept
    # only for A/B experiments).
    downhill_ratio: float = 0.5
    imu_rate: float = 100.0
    accel_sigma: float = 0.05        # m/s^2 per sample
    gyro_sigma: float = 0.002        # rad/s per sample
    accel_bias_sigma: float = 0.05   # constant per-run bias draw, m/s^2
    gyro_bias_sigma: float = 0.01    # constant per-run bias draw, rad/s
    mag_sigma: float = 0.05          # unit-vector components
    mag_rate: float = 20.0
    uwb_range_sigma: float = 0.10    # m
    uwb_multipath_prob: float = 0.05
    uwb_multipath_scale: float = 1.0  # Exp() mean of the positive range bias, m
    uwb_aoa_sigma: float = 0.12      # rad-ish, on bearing components
    uwb_slot_rate: float = 100.0     # ranging slots per second, network-wide
    uwb_max_range: float = 60.0      # m, radio range cutoff
    gps_sigma_h: float = 1.5         # m
    gps_sigma_v: float = 3.0         # m
    gps_rate: float = 1.0
    alt_sigma: float = 0.3           # m
    alt_rate: float = 10.0
    broadcast_rate: float = 10.0     # pose advertisements per second per widget


@dataclasses.dataclass
class WidgetSpec:
    """Truth-side description of one widget."""
    wid: int
    kind: str = "orbit"              # "orbit" | "static"
    center: tuple = (0.0, 0.0, 1.0)
    radius: float = 5.0
    omega: float = 0.2               # rad/s along the orbit
    phase: float = 0.0
    z_amp: float = 0.0               # vertical bobbing amplitude, m
    yaw_rate: float = 0.1            # rad/s constant spin
    has_gps: bool = False
    has_alt: bool = True
    has_aoa: bool = True
    # A surveyed widget knows its own (static) absolute position — the
    # "any widget that knows its absolute position provides it" case.
    surveyed: bool = False


def _true_pos(spec, t):
    cx, cy, cz = spec.center
    if spec.kind == "static":
        return np.array([cx, cy, cz])
    a = spec.omega * t + spec.phase
    return np.array([
        cx + spec.radius * math.cos(a),
        cy + spec.radius * math.sin(a),
        cz + spec.z_amp * math.sin(0.5 * a),
    ])


def _yaw_quat(yaw):
    return np.array([math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)])


def _quat_from_mat(r):
    """Scalar-first quaternion from a rotation matrix (Shepperd's method)."""
    tr = r[0, 0] + r[1, 1] + r[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        q = [0.25 * s, (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s,
             (r[1, 0] - r[0, 1]) / s]
    elif r[0, 0] >= r[1, 1] and r[0, 0] >= r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
        q = [(r[2, 1] - r[1, 2]) / s, 0.25 * s, (r[0, 1] + r[1, 0]) / s,
             (r[0, 2] + r[2, 0]) / s]
    elif r[1, 1] >= r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
        q = [(r[0, 2] - r[2, 0]) / s, (r[0, 1] + r[1, 0]) / s, 0.25 * s,
             (r[1, 2] + r[2, 1]) / s]
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
        q = [(r[1, 0] - r[0, 1]) / s, (r[0, 2] + r[2, 0]) / s,
             (r[1, 2] + r[2, 1]) / s, 0.25 * s]
    return np.array(q) / np.linalg.norm(q)


def triad(up_body, mag_body, mag_world):
    """Coarse body-to-world attitude from one accel + mag sample (TRIAD),
    exactly what firmware does to initialize the filter before fusing."""
    def frame(primary, secondary):
        t1 = primary / np.linalg.norm(primary)
        t2 = np.cross(t1, secondary)
        t2 /= np.linalg.norm(t2)
        return np.column_stack([t1, t2, np.cross(t1, t2)])
    body = frame(up_body, mag_body)
    ref = frame(np.array([0.0, 0.0, 1.0]), mag_world)
    return _quat_from_mat(ref @ body.T)


def _rot_z(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class TruthWidget:
    """Closed-form truth kinematics for one widget."""

    def __init__(self, spec):
        self.spec = spec

    def pos(self, t):
        return _true_pos(self.spec, t)

    def vel(self, t, h=1e-3):
        return (self.pos(t + h) - self.pos(t - h)) / (2 * h)

    def acc(self, t, h=1e-3):
        return (self.pos(t + h) - 2 * self.pos(t) + self.pos(t - h)) / (h * h)

    def yaw(self, t):
        return self.spec.yaw_rate * t + 0.7 * self.spec.wid

    def quat(self, t):
        return _yaw_quat(self.yaw(t))

    @property
    def wid(self):
        return self.spec.wid


class SimWidget:
    """One widget: truth + sensors + its onboard estimator instance."""

    def __init__(self, spec, sensors, rng):
        self.truth = TruthWidget(spec)
        self.spec = spec
        self.cfg = sensors
        self.rng = rng
        self.accel_bias = rng.normal(0.0, sensors.accel_bias_sigma, 3)
        self.gyro_bias = rng.normal(0.0, sensors.gyro_bias_sigma, 3)
        # Filter noise densities from per-sample sigmas: density = sigma/sqrt(rate).
        sqrt_rate = math.sqrt(sensors.imu_rate)
        self.est = estimator.Estimator(
            accel_noise=sensors.accel_sigma / sqrt_rate,
            gyro_noise=sensors.gyro_sigma / sqrt_rate,
        )
        # Coarse attitude init from one accel+mag sample; position unknown
        # unless surveyed.
        accel, _ = self.imu_sample(0.0)
        mag_body = (_rot_z(self.truth.yaw(0.0)).T @ MAG_WORLD
                    + rng.normal(0, sensors.mag_sigma, 3))
        q0 = triad(accel, mag_body, MAG_WORLD)
        if spec.surveyed:
            self.est.set_pose(self.truth.pos(0.0), (0, 0, 0), q0,
                              pos_sigma=0.02, vel_sigma=0.05, att_sigma=0.3)
        else:
            self.est.set_pose((0, 0, 0), (0, 0, 0), q0,
                              pos_sigma=100.0, vel_sigma=1.5, att_sigma=0.3)
        # Advertised pose as the network last heard it: (position, mean var).
        self.advertised = (np.zeros(3), 1e4)
        self.accepted_ranges = 0
        self.rejected_ranges = 0

    def imu_sample(self, t):
        """Raw accel (specific force) + gyro in the body frame, with errors."""
        r_wb = _rot_z(self.truth.yaw(t))          # body -> world
        f_world = self.truth.acc(t) - G_WORLD     # specific force, world
        f_body = r_wb.T @ f_world
        accel = f_body + self.accel_bias + self.rng.normal(0, self.cfg.accel_sigma, 3)
        gyro = (np.array([0.0, 0.0, self.spec.yaw_rate]) + self.gyro_bias
                + self.rng.normal(0, self.cfg.gyro_sigma, 3))
        return accel, gyro

    def broadcast(self, t):
        if self.spec.surveyed:
            self.advertised = (self.truth.pos(t), 1e-4)
            return
        s = self.est.state()
        self.advertised = (np.array(s["p"]), s["pos_var"])

    def pos_error(self, t):
        return float(np.linalg.norm(np.array(self.est.state()["p"]) - self.truth.pos(t)))


class World:
    """Steps truth, sensors, the TDMA schedule, and every onboard filter."""

    def __init__(self, specs, sensors=None, seed=1234):
        self.cfg = sensors or SensorConfig()
        self.rng = np.random.default_rng(seed)
        self.widgets = [SimWidget(s, self.cfg, self.rng) for s in specs]
        self.t = 0.0
        self.dt = 1.0 / self.cfg.imu_rate
        self.step_count = 0
        self._pairs = [(i, j) for i in range(len(specs))
                       for j in range(i + 1, len(specs))]
        self._pair_idx = 0
        self._slot_acc = 0.0
        self.last_links = []  # (i, j) pairs ranged in the most recent step

    def _every(self, rate):
        """True on steps that hit the given event rate."""
        period = max(1, round(self.cfg.imu_rate / rate))
        return self.step_count % period == 0

    def step(self):
        """Advance one IMU period."""
        t = self.t + self.dt
        cfg = self.cfg
        for w in self.widgets:
            accel, gyro = w.imu_sample(t)
            w.est.predict(accel, gyro, self.dt)
            # Quasi-static gravity tilt aiding (gated on |f| ~ g).
            if self.step_count % 5 == 0 and abs(np.linalg.norm(accel) - GRAVITY) < 0.5:
                w.est.update_gravity(accel, 0.1)
            if self._every(cfg.mag_rate):
                r_wb = _rot_z(w.truth.yaw(t))
                mag_body = r_wb.T @ MAG_WORLD + self.rng.normal(0, cfg.mag_sigma, 3)
                w.est.update_mag(mag_body, MAG_WORLD, cfg.mag_sigma)
            if w.spec.has_gps and self._every(cfg.gps_rate):
                fix = w.truth.pos(t) + np.concatenate([
                    self.rng.normal(0, cfg.gps_sigma_h, 2),
                    self.rng.normal(0, cfg.gps_sigma_v, 1)])
                w.est.update_gps(fix, cfg.gps_sigma_h, cfg.gps_sigma_v)
            if w.spec.has_alt and self._every(cfg.alt_rate):
                z = w.truth.pos(t)[2] + self.rng.normal(0, cfg.alt_sigma)
                w.est.update_alt(z, cfg.alt_sigma)
            if w.spec.surveyed and self._every(cfg.alt_rate):
                # A surveyed widget re-pins itself to its known position.
                w.est.update_gps(w.truth.pos(t), 0.02, 0.02)

        # UWB TDMA: run the slots that fall inside this step.
        self.last_links = []
        self._slot_acc += cfg.uwb_slot_rate * self.dt
        while self._slot_acc >= 1.0 and self._pairs:
            self._slot_acc -= 1.0
            self._run_slot(t)

        if self._every(cfg.broadcast_rate):
            for w in self.widgets:
                w.broadcast(t)

        self.t = t
        self.step_count += 1

    def _run_slot(self, t):
        """One TDMA ranging slot: the next round-robin pair in radio range."""
        cfg = self.cfg
        for _ in range(len(self._pairs)):
            i, j = self._pairs[self._pair_idx]
            self._pair_idx = (self._pair_idx + 1) % len(self._pairs)
            wi, wj = self.widgets[i], self.widgets[j]
            true_d = float(np.linalg.norm(wi.truth.pos(t) - wj.truth.pos(t)))
            if true_d <= cfg.uwb_max_range:
                break
        else:
            return
        # Two-way ranging: one shared range value for both sides.
        rng_meas = true_d + self.rng.normal(0, cfg.uwb_range_sigma)
        if self.rng.random() < cfg.uwb_multipath_prob:
            rng_meas += self.rng.exponential(cfg.uwb_multipath_scale)
        for me, other in ((wi, wj), (wj, wi)):
            if me.spec.surveyed:
                continue  # already knows where it is
            peer_p, peer_var = other.advertised
            my_var = me.est.state()["pos_var"]
            if (cfg.downhill_ratio > 0.0
                    and peer_var > cfg.downhill_ratio * my_var):
                continue
            # Cold-start damping: floor the peer variance at half my own, so
            # a badly-localized widget takes bounded, re-linearized steps
            # (damped Gauss-Newton) instead of unity-gain snaps onto a tight
            # peer from a garbage linearization point — those diverge.
            nvar_eff = max(peer_var,
                           0.5 * my_var - cfg.uwb_range_sigma ** 2)
            ok = me.est.update_range(peer_p, nvar_eff, rng_meas, cfg.uwb_range_sigma)
            me.accepted_ranges += ok
            me.rejected_ranges += not ok
            if me.spec.has_aoa and my_var < 25.0:
                # True bearing to the peer in my body frame, plus noise.
                d_w = other.truth.pos(t) - me.truth.pos(t)
                u_b = _rot_z(me.truth.yaw(t)).T @ (d_w / np.linalg.norm(d_w))
                meas = u_b + self.rng.normal(0, cfg.uwb_aoa_sigma, 3)
                meas /= np.linalg.norm(meas)
                me.est.update_aoa(peer_p, peer_var, meas, cfg.uwb_aoa_sigma)
        self.last_links.append((i, j))

    def run(self, seconds, frame_cb=None, frame_rate=10.0):
        """Run for sim-seconds; optionally emit frames at frame_rate."""
        steps = int(seconds * self.cfg.imu_rate)
        frame_period = max(1, round(self.cfg.imu_rate / frame_rate))
        for _ in range(steps):
            self.step()
            if frame_cb and self.step_count % frame_period == 0:
                frame_cb(self.frame())

    def frame(self):
        """Snapshot for viewers/metrics: truth vs estimate per widget."""
        out = {"t": round(self.t, 3), "widgets": [],
               "links": list(self.last_links)}
        for w in self.widgets:
            s = w.est.state()
            cov = w.est.pos_cov()
            out["widgets"].append({
                "id": w.spec.wid,
                "gps": w.spec.has_gps,
                "surveyed": w.spec.surveyed,
                "true_p": [round(float(x), 3) for x in w.truth.pos(self.t)],
                "true_yaw": round(w.truth.yaw(self.t) % (2 * math.pi), 3),
                "est_p": [round(float(x), 3) for x in s["p"]],
                "est_yaw": round(_quat_yaw(s["q"]), 3),
                "pos_var": round(float(s["pos_var"]), 4),
                "cov_xy": [[round(cov[r][c], 4) for c in range(2)] for r in range(2)],
                "err": round(w.pos_error(self.t), 3),
            })
        return out

    def errors(self):
        return [w.pos_error(self.t) for w in self.widgets]


def _quat_yaw(q):
    w, x, y, z = q
    # Yaw of the body +x axis in the world frame.
    fx = 1.0 - 2.0 * (y * y + z * z)
    fy = 2.0 * (x * y + w * z)
    return math.atan2(fy, fx) % (2 * math.pi)


def default_swarm(n=6, n_gps=2, n_surveyed=0, arena=30.0):
    """n mobile widgets orbiting around a ring (the first n_gps carrying
    GPS), plus n_surveyed static widgets at known positions on the arena
    edge."""
    specs = []
    for i in range(n):
        ang = 2 * math.pi * i / n
        specs.append(WidgetSpec(
            wid=i,
            kind="orbit",
            center=(arena / 2 + 0.35 * arena * math.cos(ang),
                    arena / 2 + 0.35 * arena * math.sin(ang),
                    1.0 + 0.3 * i),
            radius=3.0 + 0.5 * i,
            omega=0.15 + 0.03 * i,
            phase=ang,
            z_amp=0.3,
            yaw_rate=0.05 + 0.02 * i,
            has_gps=i < n_gps,
        ))
    for k in range(n_surveyed):
        ang = 2 * math.pi * k / max(1, n_surveyed) + 0.4
        specs.append(WidgetSpec(
            wid=n + k,
            kind="static",
            center=(arena / 2 + 0.48 * arena * math.cos(ang),
                    arena / 2 + 0.48 * arena * math.sin(ang),
                    0.5 + 1.0 * (k % 3)),
            yaw_rate=0.0,
            surveyed=True,
        ))
    return specs
