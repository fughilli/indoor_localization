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

Two backends (SensorConfig.backend):
  - "estimator": the Rust ESKF + swarm_core::coop driven from Python (the
    design-study harness: GPS, AoA, altimeter).
  - "node": the firmware's node runtime (swarm/node via node_backend) per
    widget — stationary detection, auto-anchoring, planar mode, centroid
    bleed, settings — fed synthetic IMU at the firmware rate, ranges on the
    TDMA schedule and peers' encoded adverts. No GPS / AoA, like the boards.
"""

import dataclasses
import math

import numpy as np

from swarm.sim import estimator
from swarm.sim import node_backend

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
    # In the symmetric (mean-only) regime, AoA's job is pinning the mesh's
    # slowly-drifting global orientation — range triangulation already makes
    # the mesh shape rigid. Its per-measurement noise (sigma_aoa * r meters
    # tangential) would shake the mesh at relaxation gain, so its gain is
    # divided by ~this factor (K ~ 1/(1+f)).
    mean_only_aoa_damping: float = 30.0
    # Innovation-annealed mean-only gain: mean-only updates never shrink the
    # covariance (the absolute frame is unobservable), so the Kalman gain
    # cannot anneal on its own and every range would keep kicking the state
    # at relaxation gain (~2/3) forever — the post-convergence jitter. The
    # estimator's range_innov_ratio (EWMA of |innovation|/sigma) measures
    # *relative* convergence instead: as it falls from lock_ratio_none to
    # lock_ratio_full, the mean-only damping factor is interpolated from 0.5
    # (K ~ 2/3, fast pull-in) up to mean_only_range_damping (K ~ 1/(1+f),
    # IMU-dominated hold). Innovations re-inflate on real motion or mesh
    # shifts, so the gain recovers automatically. 0 disables annealing.
    mean_only_range_damping: float = 30.0
    lock_ratio_full: float = 2.0     # at/below: fully annealed (locked)
    lock_ratio_none: float = 6.0     # at/above: cold-start gain
    # Refuse mean-only pulls from a peer whose advertised lock ratio is this
    # many times worse than mine: a cold joiner "knows nothing" but its
    # absolute variance is indistinguishable from a converged anchorless
    # veteran's (both honestly huge), so variance alone cannot protect the
    # mesh from a joiner's garbage advertisements — lock quality can.
    peer_lock_factor: float = 3.0
    # Ignore advertisements older than this (departed or silent widgets keep
    # dead-reckoning forever otherwise).
    max_advert_age: float = 1.0
    # AoA in the downhill regime only once my own position variance is
    # below this (m^2); before that a bearing can't be linearized usefully.
    aoa_downhill_max_var: float = 25.0
    # Mean-only range updates correct position only (CoopConfig field).
    mean_only_pos_only: bool = False
    # Weak zero-velocity gauge prior (see Estimator::update_zero_vel): pins
    # the swarm frame when no widget has an absolute position source. Sigma
    # is far above real widget speeds so it barely biases motion.
    zero_vel_sigma: float = 3.0      # m/s
    zero_vel_rate: float = 2.0       # updates per second
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
    # Per-device constant range offset (uncalibrated antenna delay): a range
    # between i and j reads true + bias_i + bias_j. 0 = off.
    uwb_device_bias_sigma: float = 0.0
    uwb_slot_rate: float = 100.0     # ranging slots per second, network-wide
    uwb_max_range: float = 60.0      # m, radio range cutoff
    gps_sigma_h: float = 1.5         # m
    gps_sigma_v: float = 3.0         # m
    gps_rate: float = 1.0
    alt_sigma: float = 0.3           # m
    alt_rate: float = 10.0
    broadcast_rate: float = 10.0     # pose advertisements per second per widget
    # IMU realism beyond white noise + constant bias (0 = off, the design
    # study's defaults). Scale: per-axis gain error draw (an MPU6050 here read
    # 14% low; the firmware's boot calibration fixes the magnitude only).
    accel_scale_sigma: float = 0.0
    accel_bias_rw: float = 0.0       # m/s^2 per sqrt(s)
    gyro_bias_rw: float = 0.0        # rad/s per sqrt(s)
    # Seconds every widget holds still at the start (the firmware calibrates
    # its IMU in its first second, if still).
    still_start: float = 0.0
    backend: str = "estimator"       # "estimator" | "node"
    # "node" backend: settings by registry name (swarm/node/src/settings.rs),
    # applied to every widget like a swarm-layer bundle.
    node_settings: dict = dataclasses.field(default_factory=dict)


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
    # Translational-motion controls (see _motion_time). motion_rate scales
    # the orbit + bob speed (1.0 = nominal). Devices don't move every
    # timestep: each cycle of move_period seconds spends move_duty of its
    # time in motion and the rest paused (velocity smoothly ramps to zero),
    # offset by move_phase so the swarm doesn't stop in unison. move_period
    # <= 0 or move_duty >= 1 means continuous motion.
    motion_rate: float = 1.0
    move_period: float = 0.0         # s per move/pause cycle; 0 = no pauses
    move_duty: float = 1.0           # fraction of each cycle spent moving
    move_phase: float = 0.0          # cycle offset in [0,1) for desync
    has_gps: bool = False
    has_alt: bool = True
    has_aoa: bool = True
    # A surveyed widget knows its own (static) absolute position — the
    # "any widget that knows its absolute position provides it" case.
    surveyed: bool = False
    # Truth holds still (at its t=0 pose) this long before moving.
    start_delay: float = 0.0
    # Hand-carried motion while moving: roll/pitch sway (rad) and vertical
    # bob (m) at the step rate (Hz). 0 = a flat-gliding widget.
    sway: float = 0.0
    bob: float = 0.0
    step_hz: float = 1.8
    # Hand handling while carried: slow multi-axis wrist rotation (rad
    # amplitude) and a lift (m) from where it rested.
    handling: float = 0.0
    lift: float = 0.0


# Each move/pause transition ramps the speed linearly over this fraction of
# the cycle (per side) so velocity is continuous and the numerical IMU
# acceleration has no impulse at stop/start. Capped at half the moving
# window for short duties.
MOVE_RAMP_FRAC = 0.05


def _cycle_area(x, m, g):
    """Integral of the trapezoidal speed envelope over phase [0, x], for a
    cycle that ramps up on [0,g), holds full speed on [g,m-g), ramps down on
    [m-g,m), then pauses on [m,1). Full-cycle area is m-g."""
    if x <= 0.0:
        return 0.0
    up = min(x, g)
    area = 0.5 * up * up / g
    if x <= g:
        return area
    plateau_end = m - g
    area += min(x, plateau_end) - g
    if x <= plateau_end:
        return area
    xd = min(x, m)
    f = lambda p: (m * p - 0.5 * p * p) / g
    area += f(xd) - f(plateau_end)
    return area


def _motion_time(spec, t):
    """Effective elapsed motion time (s): motion_rate * t warped by the
    move/pause duty cycle. Reduces to motion_rate * t under continuous
    motion. Position/velocity/accel are all derived from this, so a pause is
    a genuine stationary interval (velocity ramps to zero, not a jump)."""
    rate = spec.motion_rate
    if spec.move_period <= 0.0 or spec.move_duty >= 1.0:
        return rate * t
    m = min(max(spec.move_duty, 0.0), 1.0)
    g = min(MOVE_RAMP_FRAC, m / 2.0)
    phi = t / spec.move_period + spec.move_phase
    cycles = math.floor(phi)
    frac = phi - cycles
    phase_area = cycles * (m - g) + _cycle_area(frac, m, g)
    return rate * spec.move_period * phase_area


def _true_pos(spec, t):
    cx, cy, cz = spec.center
    if spec.kind == "static":
        return np.array([cx, cy, cz])
    a = spec.omega * _motion_time(spec, t) + spec.phase
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


def _rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rot_z(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class TruthWidget:
    """Closed-form truth kinematics for one widget."""

    def __init__(self, spec):
        self.spec = spec

    def pos(self, t):
        # Clamp only with a start delay: the numerical derivatives sample
        # t +- h, and clamping at 0 would distort every widget's first IMU
        # sample (and so its attitude init) for nothing.
        te = max(0.0, t - self.spec.start_delay) if self.spec.start_delay > 0 else t
        p = _true_pos(self.spec, te)
        if self.spec.bob or self.spec.lift:
            g = self._gait(t)
            p = p + np.array([0.0, 0.0, self.spec.lift * g
                              + self.spec.bob * g * math.sin(2 * math.pi * self.spec.step_hz * t)])
        return p

    def _gait(self, t):
        """0..1: how much the widget is being carried (its translational
        speed relative to full speed), so sway and bob stop when it does."""
        if self.spec.kind == "static" or t < self.spec.start_delay:
            return 0.0
        te = t - self.spec.start_delay
        h = 1e-3
        dm = (_motion_time(self.spec, te + h) - _motion_time(self.spec, te - h)) / (2 * h)
        return min(1.0, dm / max(self.spec.motion_rate, 1e-9))

    def rot(self, t):
        """Body -> world rotation: yaw spin, plus roll/pitch sway when
        carried."""
        yaw = self.yaw(t)
        roll = pitch = 0.0
        if self.spec.sway or self.spec.handling:
            g = self._gait(t)
            w = 2 * math.pi * self.spec.step_hz
            roll = self.spec.sway * g * math.sin(w * t)
            pitch = 0.6 * self.spec.sway * g * math.sin(0.5 * w * t + 0.7)
            h = self.spec.handling * g
            yaw += h * math.sin(2 * math.pi * 0.31 * t)
            roll += 0.7 * h * math.sin(2 * math.pi * 0.53 * t + 1.0)
            pitch += 0.6 * h * math.sin(2 * math.pi * 0.71 * t + 2.0)
        r = _rot_z(yaw)
        if roll or pitch:
            r = r @ _rot_y(pitch) @ _rot_x(roll)
        return r

    def body_rate(self, t, h=1e-3):
        """Gyro truth: body angular rate from the rotation's derivative."""
        if not (self.spec.sway or self.spec.handling):
            return np.array([0.0, 0.0, self.spec.yaw_rate if t >= self.spec.start_delay else 0.0])
        r0, r1 = self.rot(t - h), self.rot(t + h)
        rd = (r1 - r0) / (2 * h)
        w = self.rot(t).T @ rd
        return np.array([w[2, 1], w[0, 2], w[1, 0]])

    def vel(self, t, h=1e-3):
        return (self.pos(t + h) - self.pos(t - h)) / (2 * h)

    def acc(self, t, h=1e-3):
        return (self.pos(t + h) - 2 * self.pos(t) + self.pos(t - h)) / (h * h)

    def yaw(self, t):
        if (self.spec.sway or self.spec.handling) and self.spec.kind == "orbit":
            # Carried: facing along the path, turning only while moving.
            te = max(0.0, t - self.spec.start_delay)
            return self.spec.omega * _motion_time(self.spec, te) + self.spec.phase + math.pi / 2
        return self.spec.yaw_rate * max(0.0, t - self.spec.start_delay) + 0.7 * self.spec.wid

    def quat(self, t):
        return _yaw_quat(self.yaw(t))

    @property
    def wid(self):
        return self.spec.wid


class SimWidget:
    """One widget: truth + sensors + its onboard estimator instance."""

    def __init__(self, spec, sensors, rng, t0=0.0):
        self.truth = TruthWidget(spec)
        self.spec = spec
        self.cfg = sensors
        self.rng = rng
        self.accel_bias = rng.normal(0.0, sensors.accel_bias_sigma, 3)
        self.gyro_bias = rng.normal(0.0, sensors.gyro_bias_sigma, 3)
        self.accel_scale = (1.0 + rng.normal(0.0, sensors.accel_scale_sigma, 3)
                            if sensors.accel_scale_sigma > 0 else np.ones(3))
        self.range_bias = (rng.normal(0.0, sensors.uwb_device_bias_sigma)
                           if sensors.uwb_device_bias_sigma > 0 else 0.0)
        self.node = None
        self.accepted_ranges = 0
        self.rejected_ranges = 0
        if sensors.backend == "node":
            # Firmware runtime; ids offset so none is 0.
            self.node = node_backend.Node(spec.wid + 100, True, sensors.node_settings)
            self.est = None
            return
        # Filter noise densities from per-sample sigmas: density = sigma/sqrt(rate).
        sqrt_rate = math.sqrt(sensors.imu_rate)
        self.est = estimator.Estimator(
            accel_noise=sensors.accel_sigma / sqrt_rate,
            gyro_noise=sensors.gyro_sigma / sqrt_rate,
        )
        # Coarse attitude init from one accel+mag sample; position unknown
        # unless surveyed. t0 > 0 = a widget joining a running network cold.
        accel, _ = self.imu_sample(t0)
        mag_body = (_rot_z(self.truth.yaw(t0)).T @ MAG_WORLD
                    + rng.normal(0, sensors.mag_sigma, 3))
        q0 = triad(accel, mag_body, MAG_WORLD)
        if spec.surveyed:
            self.est.set_pose(self.truth.pos(t0), (0, 0, 0), q0,
                              pos_sigma=0.02, vel_sigma=0.05, att_sigma=0.3)
        else:
            self.est.set_pose((0, 0, 0), (0, 0, 0), q0,
                              pos_sigma=100.0, vel_sigma=1.5, att_sigma=0.3)
        # Advertised pose as the network last heard it:
        # (position, velocity, mean var, lock ratio, timestamp).
        self.advertised = (np.zeros(3), np.zeros(3), 1e4,
                           self.est.range_innov_ratio(), t0)
        self.accepted_ranges = 0
        self.rejected_ranges = 0

    def imu_sample(self, t):
        """Raw accel (specific force) + gyro in the body frame, with errors."""
        r_wb = self.truth.rot(t)                  # body -> world
        f_world = self.truth.acc(t) - G_WORLD     # specific force, world
        f_body = r_wb.T @ f_world
        accel = (self.accel_scale * f_body + self.accel_bias
                 + self.rng.normal(0, self.cfg.accel_sigma, 3))
        gyro = (self.truth.body_rate(t) + self.gyro_bias
                + self.rng.normal(0, self.cfg.gyro_sigma, 3))
        return accel, gyro

    def drift_biases(self, dt):
        """Bias random walk (SensorConfig accel_bias_rw / gyro_bias_rw)."""
        if self.cfg.accel_bias_rw > 0:
            self.accel_bias += self.rng.normal(0, self.cfg.accel_bias_rw * math.sqrt(dt), 3)
        if self.cfg.gyro_bias_rw > 0:
            self.gyro_bias += self.rng.normal(0, self.cfg.gyro_bias_rw * math.sqrt(dt), 3)

    def est_state(self, t):
        """Estimated state in one shape for both backends."""
        if self.node is not None:
            return self.node.state(t)
        s = self.est.state()
        return {"p": s["p"], "v": s["v"], "q": s["q"], "pos_var": s["pos_var"],
                "lock": self.est.range_innov_ratio(), "still": False, "anchored": False,
                "initialized": True, "regains": 0}

    def broadcast(self, t):
        if self.spec.surveyed:
            # An absolute source is locked by definition.
            self.advertised = (self.truth.pos(t), np.zeros(3), 1e-4, 1.0, t)
            return
        s = self.est.state()
        lock = 1.0 if self.spec.has_gps else self.est.range_innov_ratio()
        self.advertised = (np.array(s["p"]), np.array(s["v"]),
                           s["pos_var"], lock, t)

    def advertised_at(self, t):
        """Peer view of me: last broadcast dead-reckoned by its velocity —
        the advertisement payload carries v for exactly this. Returns
        (position, variance, lock ratio, age)."""
        p, v, var, lock, t0 = self.advertised
        return p + v * (t - t0), var, lock, t - t0

    def pos_error(self, t):
        return float(np.linalg.norm(np.array(self.est_state(t)["p"]) - self.truth.pos(t)))


class World:
    """Steps truth, sensors, the TDMA schedule, and every onboard filter."""

    def __init__(self, specs, sensors=None, seed=1234):
        self.cfg = sensors or SensorConfig()
        self.rng = np.random.default_rng(seed)
        self.widgets = [SimWidget(s, self.cfg, self.rng) for s in specs]
        self.t = 0.0
        self.dt = 1.0 / self.cfg.imu_rate
        self.step_count = 0
        self._rebuild_pairs()
        self._coop = estimator.coop_config(
            self.cfg.downhill_ratio, self.cfg.mean_only_aoa_damping,
            self.cfg.mean_only_range_damping, self.cfg.lock_ratio_full,
            self.cfg.lock_ratio_none, self.cfg.peer_lock_factor,
            self.cfg.max_advert_age, self.cfg.aoa_downhill_max_var,
            self.cfg.mean_only_pos_only)
        self._slot_acc = 0.0
        self.last_links = []  # (wid, wid) pairs ranged in the most recent step

    def _rebuild_pairs(self):
        self._pairs = [(i, j) for i in range(len(self.widgets))
                       for j in range(i + 1, len(self.widgets))]
        self._pair_idx = 0

    def add_widget(self, spec):
        """A widget joins the network cold at the current sim time: position
        unknown (100 m sigma), no lock — exactly the fresh-boot state."""
        self.widgets.append(SimWidget(spec, self.cfg, self.rng, t0=self.t))
        self._rebuild_pairs()

    def remove_widget(self, wid):
        """A widget leaves (power-off / out of range): it stops ranging and
        broadcasting; peers age its last advertisement out."""
        self.widgets = [w for w in self.widgets if w.spec.wid != wid]
        self._rebuild_pairs()

    def _every(self, rate):
        """True on steps that hit the given event rate."""
        period = max(1, round(self.cfg.imu_rate / rate))
        return self.step_count % period == 0

    def step(self):
        """Advance one IMU period."""
        if self.cfg.backend == "node":
            return self._step_node()
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
            if cfg.zero_vel_sigma > 0.0 and self._every(cfg.zero_vel_rate):
                w.est.update_zero_vel(cfg.zero_vel_sigma)

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

    def _step_node(self):
        """Node backend: what the firmware's tasks see, at the IMU rate."""
        t = self.t + self.dt
        cfg = self.cfg
        for w in self.widgets:
            w.drift_biases(self.dt)
            accel, gyro = w.imu_sample(t)
            w.node.imu(t, accel, gyro)
        self.last_links = []
        self._slot_acc += cfg.uwb_slot_rate * self.dt
        while self._slot_acc >= 1.0 and self._pairs:
            self._slot_acc -= 1.0
            self._run_slot(t)
        if self._every(cfg.broadcast_rate):
            # Adverts ride on POLLs: everyone in range hears every widget's.
            for w in self.widgets:
                data = w.node.advert_tx()
                for o in self.widgets:
                    if o is not w and np.linalg.norm(o.truth.pos(t) - w.truth.pos(t)) <= cfg.uwb_max_range:
                        o.node.advert_rx(t, data)
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
        rng_meas = true_d + self.rng.normal(0, cfg.uwb_range_sigma) + wi.range_bias + wj.range_bias
        if self.rng.random() < cfg.uwb_multipath_prob:
            rng_meas += self.rng.exponential(cfg.uwb_multipath_scale)
        if cfg.backend == "node":
            # DS-TWR: both ends fold the same measurement.
            for me, other in ((wi, wj), (wj, wi)):
                code = me.node.range(t, other.spec.wid + 100, rng_meas)
                me.accepted_ranges += code == 1
                me.rejected_ranges += code == 0
            self.last_links.append((wi.spec.wid, wj.spec.wid))
            return
        for me, other in ((wi, wj), (wj, wi)):
            if me.spec.surveyed:
                continue  # already knows where it is
            peer_p, peer_var, peer_lock, advert_age = other.advertised_at(t)
            # The cooperative rules (downhill / mean-only / skip, cold-start
            # damping, lock annealing, advert aging, anchored widgets refusing
            # consensus pulls) live in swarm_core::coop — the same code the
            # firmware runs; see coop.rs for the reasoning behind each.
            code, decision = me.est.coop_range_update(
                self._coop, peer_p, peer_var, peer_lock, advert_age, rng_meas,
                cfg.uwb_range_sigma, anchored=me.spec.has_gps)
            if code < 0:
                continue  # skipped: stale advert / anchored / cold peer
            me.accepted_ranges += code == 1
            me.rejected_ranges += code == 0
            # Same condition coop::aoa_update applies (mean-only, or downhill
            # once roughly localized), mirrored only to keep the noise draws
            # (and so seeded runs) identical.
            mean_only = decision[0] == 2.0
            if me.spec.has_aoa and (mean_only or decision[2] < cfg.aoa_downhill_max_var):
                # True bearing to the peer in my body frame, plus noise.
                d_w = other.truth.pos(t) - me.truth.pos(t)
                u_b = _rot_z(me.truth.yaw(t)).T @ (d_w / np.linalg.norm(d_w))
                meas = u_b + self.rng.normal(0, cfg.uwb_aoa_sigma, 3)
                meas /= np.linalg.norm(meas)
                me.est.coop_aoa_update(self._coop, peer_p, decision, meas,
                                       cfg.uwb_aoa_sigma)
        self.last_links.append((wi.spec.wid, wj.spec.wid))

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
        aligned = self.aligned_estimates() if self.cfg.backend == "node" else None
        for k, w in enumerate(self.widgets):
            s = w.est_state(self.t)
            if w.est is not None:
                cov = w.est.pos_cov()
            else:
                cov = [[s["pos_var"], 0.0], [0.0, s["pos_var"]]]
            out["widgets"].append({
                "id": w.spec.wid,
                "gps": w.spec.has_gps,
                "surveyed": w.spec.surveyed,
                "true_p": [round(float(x), 3) for x in w.truth.pos(self.t)],
                "true_yaw": round(w.truth.yaw(self.t) % (2 * math.pi), 3),
                "est_p": [round(float(x), 3) for x in s["p"]],
                "est_v": [round(float(x), 3) for x in s["v"]],
                "true_v": [round(float(x), 3) for x in w.truth.vel(self.t)],
                "est_yaw": round(_quat_yaw(s["q"]), 3),
                "pos_var": round(float(s["pos_var"]), 4),
                "lock": round(float(s["lock"]), 2),
                "cov_xy": [[round(cov[r][c], 4) for c in range(2)] for r in range(2)],
                "err": round(w.pos_error(self.t), 3),
                "still": s["still"],
                "anchored": s["anchored"],
            })
            if aligned is not None:
                out["widgets"][-1]["aligned_p"] = [round(float(x), 3) for x in aligned[k]]
                out["widgets"][-1]["aligned_err"] = round(
                    float(np.linalg.norm(aligned[k] - w.truth.pos(self.t))), 3)
        return out

    def aligned_estimates(self):
        """Estimates after the best rigid fit (Kabsch) onto truth: an
        anchorless swarm's frame is arbitrary, so its error is the shape's.
        With fewer than 3 widgets, translation only."""
        est = np.array([w.est_state(self.t)["p"] for w in self.widgets], dtype=float)
        tru = np.array([w.truth.pos(self.t) for w in self.widgets], dtype=float)
        ce, ct = est.mean(axis=0), tru.mean(axis=0)
        if len(est) < 3:
            return est - ce + ct
        h = (est - ce).T @ (tru - ct)
        u, _, vt = np.linalg.svd(h)
        d = np.sign(np.linalg.det(vt.T @ u.T))
        r = vt.T @ np.diag([1.0, 1.0, d]) @ u.T
        return (r @ (est - ce).T).T + ct

    def errors(self):
        return [w.pos_error(self.t) for w in self.widgets]


def _quat_yaw(q):
    w, x, y, z = q
    # Yaw of the body +x axis in the world frame.
    fx = 1.0 - 2.0 * (y * y + z * z)
    fy = 2.0 * (x * y + w * z)
    return math.atan2(fy, fx) % (2 * math.pi)


def room_swarm(walker=True, still=4.0):
    """The bench setup: four widgets in a room at one height. Widget 1 sits
    roughly mid-triangle but nearly on the 0-2 edge (as on the bench, where
    three widgets came out within ~5 deg of collinear); widget 3, if
    `walker`, is carried around the room with stops."""
    specs = [
        WidgetSpec(wid=0, kind="static", center=(0.0, 0.0, 0.0), yaw_rate=0.0, has_alt=False, has_aoa=False),
        WidgetSpec(wid=1, kind="static", center=(0.85, 0.12, 0.0), yaw_rate=0.0, has_alt=False, has_aoa=False),
        WidgetSpec(wid=2, kind="static", center=(3.35, 0.45, 0.0), yaw_rate=0.0, has_alt=False, has_aoa=False),
    ]
    if walker:
        specs.append(WidgetSpec(
            wid=3, kind="orbit", center=(1.2, 2.0, 0.0), radius=1.8, omega=0.35, yaw_rate=0.2,
            move_period=20.0, move_duty=0.6, has_alt=False, has_aoa=False, start_delay=still,
            sway=0.15, bob=0.03, handling=0.5, lift=0.25))
    else:
        specs.append(WidgetSpec(wid=3, kind="static", center=(-1.0, 4.0, 0.0), yaw_rate=0.0,
                                has_alt=False, has_aoa=False))
    return specs


def room_sensors(**overrides):
    """Bench-like sensors for the node backend: MPU6050-class IMU at the
    firmware's 200 Hz with scale errors and bias drift, DW3000 ranging at the
    firmware's TDMA rate (4 x 33 exchanges/s), adverts on every POLL."""
    cfg = SensorConfig(
        backend="node",
        imu_rate=200.0,
        accel_sigma=0.04,            # ~400 ug/sqrt(Hz) at 200 Hz
        gyro_sigma=0.003,            # ~0.005 dps/sqrt(Hz) at 200 Hz, plus margin
        accel_bias_sigma=0.04,
        gyro_bias_sigma=0.02,
        accel_scale_sigma=0.03,
        accel_bias_rw=0.002,
        gyro_bias_rw=0.0002,
        still_start=4.0,
        uwb_range_sigma=0.05,
        uwb_multipath_prob=0.02,
        uwb_multipath_scale=0.3,
        uwb_device_bias_sigma=0.08,
        uwb_slot_rate=133.0,
        broadcast_rate=33.0,
        node_settings={"planar.on": True},
    )
    for k, v in overrides.items():
        setattr(cfg, k, v)
    return cfg


def default_swarm(n=6, n_gps=2, n_surveyed=0, arena=30.0,
                  motion_rate=1.0, move_period=0.0, move_duty=1.0):
    """n mobile widgets orbiting around a ring (the first n_gps carrying
    GPS), plus n_surveyed static widgets at known positions on the arena
    edge. motion_rate scales orbit speed; move_period/move_duty add
    stop-and-go pauses (see WidgetSpec), desynced per widget."""
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
            motion_rate=motion_rate,
            move_period=move_period,
            move_duty=move_duty,
            move_phase=(0.37 * i) % 1.0,
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
