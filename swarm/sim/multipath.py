"""UWB multipath channel for the swarm sim: fixed reflectors and faders, what a
DW3000 reports about them, and what a PDoA antenna array adds.

The scene holds two kinds of fixed landmarks:

  Reflector  a point scatterer. Every link with a transmitter a and receiver b gets
             a second path of length |a-m| + |m-b|, with amplitude (relative to a
             clear line of sight) rho * d_ab / (|a-m| |m-b|)  [rho in metres].
  Fader      a sphere (centre, radius) that costs a path loss_db * chord/(2r) dB for
             each leg of any path that passes through it (so a diametral crossing
             costs the full loss_db). It shadows the line of sight, and so the
             first-path power, and can hide the first path altogether.

What the radio reports for one exchange (one frame received by one end):

  first arrival only   rng (the first detected path's length, so a faded line of
                       sight reads long) and gap_db (total minus first-path power:
                       ~N(3.5, 1.2) clear, ~N(18, 3.5) when the line of sight is
                       lost; the same numbers as SensorConfig.uwb_los_gap_db /
                       uwb_nlos_gap_db, and in between when it is only faded)
  CIR (multipath)      every path the detector finds in the channel impulse
                       response: length, SNR and (optionally) phase. The DW3000 can
                       read its CIR accumulator (1016 taps at 64 MHz PRF, 1.0016 ns
                       = 0.30 m per tap) and first-path diagnostics over SPI; two
                       paths closer than `resolution` merge into one peak.
  PDoA                 the phase difference between the two antennas of a pair, per
                       path (the CIR is read for both receive antennas): k * b . u,
                       with b the baseline in the receiver's body frame and u the
                       unit vector toward where the path came from (the transmitter
                       for the line of sight, the reflector for a reflection).

Everything here is truth-side; landmarks.py sees only LinkObs. Physical parameters
are plausible DW3000 numbers, not yet bench fits (see docs/uwb-landmarks.md).
"""

import dataclasses
import math
from typing import Optional

import numpy as np

C = 299_792_458.0
# The DW3000 does channel 5 and channel 9 only (libs/dw3000: CHANNEL_5, CHANNEL_9).
CHANNELS = {5: 6489.6e6, 9: 7987.2e6}


def wavelength(channel):
    return C / CHANNELS[channel]


def wrap(x):
    """Wrap to (-pi, pi]."""
    return np.angle(np.exp(1j * np.asarray(x)))


@dataclasses.dataclass
class Reflector:
    pos: tuple
    rho: float = 1.5            # m: path amplitude rho * d_ab / (d1 d2) vs clear LOS


@dataclasses.dataclass
class Fader:
    pos: tuple
    radius: float = 0.4         # m
    loss_db: float = 15.0       # for a path through the centre


@dataclasses.dataclass
class Scene:
    reflectors: list = dataclasses.field(default_factory=list)
    faders: list = dataclasses.field(default_factory=list)


@dataclasses.dataclass
class AntennaPair:
    """Two receive patches on one channel: the vector antenna 1 -> antenna 2 in the
    body frame, and the direction the patches face. Half a wavelength apart is the
    usual choice (unambiguous over +-90 degrees); `pair_for` builds that. A patch
    hears the front hemisphere: see `patch_gain_db`."""
    channel: int
    baseline: tuple
    boresight: tuple = (0.0, 0.0, 1.0)


def pair_for(channel, axis, spacing_lambda=0.5, boresight=(0.0, 0.0, 1.0)):
    b = np.zeros(3)
    b[axis] = spacing_lambda * wavelength(channel)
    return AntennaPair(channel, tuple(b), tuple(boresight))


def patch_gain_db(cos_theta, exponent=1.5, floor_db=-20.0):
    """Receive gain of a patch relative to boresight, for an arrival at angle theta
    from it: 10 n log10(cos), floored at -20 dB, which is also what anything from
    behind gets. (A round-number model, not a measured pattern.)"""
    if cos_theta <= 0.0:
        return floor_db
    return max(10.0 * exponent * math.log10(cos_theta), floor_db)


def flat_array():
    """The board lies flat (patches face up): CH5 pair along body x, CH9 pair along
    body y. Both baselines are horizontal, so neither sees elevation."""
    up = (0.0, 0.0, 1.0)
    return [pair_for(5, 0, boresight=up), pair_for(9, 1, boresight=up)]


def upright_array():
    """The board stands vertical, patches facing body +y: CH5 pair along body x
    (horizontal), CH9 pair along body z (vertical). Two axes of bearing, but the
    board's front/back mirror is only broken by the patch pattern."""
    fwd = (0.0, 1.0, 0.0)
    return [pair_for(5, 0, boresight=fwd), pair_for(9, 2, boresight=fwd)]


@dataclasses.dataclass
class Exchange:
    """One ranging exchange: the channel used and the antenna pair whose phase is
    read (index into the array, or -1 for none: range and CIR only)."""
    channel: int = 5
    pair: int = -1


@dataclasses.dataclass
class RadioModel:
    snr1_db: float = 60.0          # CIR peak SNR of a clear line of sight at 1 m
    detect_thr_db: float = 10.0    # path detection: 50% at this SNR ...
    detect_slope_db: float = 2.0   # ... logistic width
    resolution: float = 0.9        # m: closer paths merge (500 MHz bandwidth, ~3 ns)
    sigma_shared: float = 0.07     # m: error common to every path of an exchange
    sigma_floor: float = 0.05      # m: per-path delay noise at high SNR
    k_len: float = 0.6             # m: per-path delay noise sigma = k_len / sqrt(snr)
    sigma_amp_db: float = 1.5
    sigma_phase0: float = 0.05     # rad, high SNR
    k_phase: float = 1.0           # rad: phase noise sigma = k_phase / sqrt(snr)
    fade_db: float = 3.0           # per-exchange, per-reflection amplitude jitter
    cal_phase_sigma: float = 0.05  # rad: constant PDoA calibration error per (node, pair)
    false_peaks: float = 0.2       # mean spurious CIR peaks per frame
    false_window: float = 20.0     # m beyond the line of sight they can land in
    gap_los: tuple = (3.5, 1.2)    # dB, as SensorConfig.uwb_los_gap_db
    gap_nlos: tuple = (18.0, 3.5)  # dB, as SensorConfig.uwb_nlos_gap_db: the first path is not the LOS
    max_range: float = 60.0


def chord(a, b, c, r):
    """Length of segment a->b inside the sphere (c, r). Arrays (n,3) or (3,)."""
    a, b, c = np.atleast_2d(a), np.atleast_2d(b), np.asarray(c, float)
    d = b - a
    length = np.linalg.norm(d, axis=1)
    u = d / np.maximum(length, 1e-12)[:, None]
    w = c - a
    s = np.einsum("ij,ij->i", w, u)
    h2 = np.einsum("ij,ij->i", w, w) - s * s
    half = np.sqrt(np.maximum(r * r - h2, 0.0))
    lo = np.maximum(s - half, 0.0)
    hi = np.minimum(s + half, length)
    return np.where(h2 < r * r, np.maximum(hi - lo, 0.0), 0.0)


def fader_loss_db(scene, a, b):
    return sum(f.loss_db * float(chord(a, b, f.pos, f.radius)[0]) / (2 * f.radius) for f in scene.faders)


@dataclasses.dataclass
class LinkObs:
    """What the mesh knows about one received frame. p_tx, p_rx and R_rx are the
    mesh's *believed* poses (mesh frame, body -> mesh rotation); the truth is not
    in here. `label` is ground truth per path (-1 line of sight, k reflector k,
    -2 spurious), for scoring only."""
    t: float
    tx: int
    rx: int
    p_tx: np.ndarray
    p_rx: np.ndarray
    R_rx: np.ndarray
    channel: int
    pair: int
    rng: float                 # first detected path length
    gap_db: float
    L: np.ndarray              # detected path lengths, ascending
    snr: np.ndarray            # dB
    phase: np.ndarray          # rad, NaN without a pair
    label: np.ndarray


class Channel:
    """Generates LinkObs from true poses."""

    def __init__(self, scene, radio=None, array=(), seed=0):
        self.scene = scene
        self.radio = radio or RadioModel()
        self.array = list(array)
        self.rng = np.random.default_rng(seed)
        self._cal = {}

    def _cal_bias(self, node, pair):
        key = (node, pair)
        if key not in self._cal:
            self._cal[key] = self.rng.normal(0.0, self.radio.cal_phase_sigma)
        return self._cal[key]

    def paths(self, tx, rx):
        """True paths of a link: [(length, amplitude vs clear LOS, source point, label)]."""
        rd, rng = self.radio, self.rng
        d = float(np.linalg.norm(tx - rx))
        out = [(d, 10 ** (-fader_loss_db(self.scene, tx, rx) / 20), tx, -1)]
        for k, rf in enumerate(self.scene.reflectors):
            m = np.asarray(rf.pos, float)
            d1, d2 = float(np.linalg.norm(tx - m)), float(np.linalg.norm(m - rx))
            a = rf.rho * d / max(d1 * d2, 1e-6) * 10 ** (rng.normal(0, rd.fade_db) / 20)
            a *= 10 ** (-(fader_loss_db(self.scene, tx, m) + fader_loss_db(self.scene, m, rx)) / 20)
            out.append((d1 + d2, a, m, k))
        return out

    def observe(self, t, tx_id, rx_id, tx, rx, R_rx, believed, ex, sync_len=0.0):
        """One frame from tx received at rx (true world poses). `believed` =
        (p_tx, p_rx, R_rx) in the mesh frame. Returns a LinkObs, or None if no
        path was detected (an exchange that times out). `sync_len` (m) is the receiver's
        clock error against the transmitter's, which a one-way (broadcast) timestamp
        carries into every path length and a two-way exchange cancels."""
        rd, rng = self.radio, self.rng
        d = float(np.linalg.norm(tx - rx))
        if d > rd.max_range:
            return None
        cands = self.paths(tx, rx)
        if ex.pair >= 0:
            # The patches' pattern scales each path by where it arrives from.
            bs = np.asarray(self.array[ex.pair].boresight, float)
            scaled = []
            for L, a, src, lab in cands:
                u_b = R_rx.T @ ((src - rx) / max(np.linalg.norm(src - rx), 1e-9))
                scaled.append((L, a * 10 ** (patch_gain_db(float(bs @ u_b)) / 20), src, lab))
            cands = scaled
        snr_of = lambda a: rd.snr1_db - 20 * math.log10(max(d, 0.1)) + 20 * math.log10(max(a, 1e-9))
        det = []
        for L, a, src, lab in cands:
            snr = snr_of(a)
            if rng.random() < 0.5 * (1.0 + math.tanh((snr - rd.detect_thr_db) / (2 * rd.detect_slope_db))):
                det.append([L, a, snr, src, lab, 1.0])
        for _ in range(rng.poisson(rd.false_peaks)):
            det.append([d + rng.uniform(0, rd.false_window), 0.0, rd.detect_thr_db + rng.uniform(0, 6),
                        None, -2, 1.0])
        if not det:
            return None
        det.sort(key=lambda x: x[0])
        merged = [det[0]]
        for p in det[1:]:                      # peaks closer than the resolution merge
            q = merged[-1]
            if p[0] - q[0] < rd.resolution:
                strong, weak = (q, p) if q[2] >= p[2] else (p, q)
                strong[0] += 0.5 * 10 ** ((weak[2] - strong[2]) / 20) * (weak[0] - strong[0])
                strong[5] = 2.0                # an interfered peak: noisier phase
                merged[-1] = strong
            else:
                merged.append(p)
        merged.sort(key=lambda x: x[0])
        e_shared = rng.normal(0, rd.sigma_shared)
        n = len(merged)
        L = np.empty(n)
        snr = np.empty(n)
        phase = np.full(n, np.nan)
        label = np.empty(n, int)
        for i, (Li, a, s, src, lab, boost) in enumerate(merged):
            sl = 10 ** (s / 20)
            L[i] = Li + e_shared + sync_len + rng.normal(0, math.hypot(rd.sigma_floor, rd.k_len / sl))
            snr[i] = s + rng.normal(0, rd.sigma_amp_db)
            label[i] = lab
            if ex.pair >= 0:
                pr = self.array[ex.pair]
                if src is None:
                    phase[i] = rng.uniform(-math.pi, math.pi)
                else:
                    u_w = (src - rx) / max(np.linalg.norm(src - rx), 1e-9)
                    u_b = R_rx.T @ u_w
                    k = 2 * math.pi / wavelength(pr.channel)
                    sig = boost * math.hypot(rd.sigma_phase0, rd.k_phase / sl)
                    phase[i] = wrap(k * float(np.asarray(pr.baseline) @ u_b)
                                    + self._cal_bias(rx_id, ex.pair) + rng.normal(0, sig))
        total = sum(a * a for _, a, *_ in cands)
        first_a = max(merged[0][1], 1e-3)
        if merged[0][4] == -1:
            # The line of sight was timed: its (possibly faded) power against the lot.
            gap = rd.gap_los[0] + 10 * math.log10(max(total, 1e-9) / (first_a ** 2)) + rng.normal(0, rd.gap_los[1])
        else:
            # A reflection (or a spurious peak) was timed. The DW3000 reports the gap
            # between total and first-path power at ~18 dB then (bench fit, world.py).
            gap = rng.normal(*rd.gap_nlos)
        p_tx, p_rx, R_b = believed
        return LinkObs(t, tx_id, rx_id, p_tx, p_rx, R_b, ex.channel, ex.pair, float(L[0]), float(gap),
                       L, snr, phase, label)


@dataclasses.dataclass
class MeshError:
    """How wrong the mesh's idea of its own geometry is."""
    pos_sigma: float = 0.10        # m, constant per node (the swarm's residual shape error)
    jitter: float = 0.03           # m, per observation
    yaw_sigma: float = 0.05        # rad, constant per node (nodes with an IMU)
    noimu_yaw_sigma: float = 0.30  # rad: a node with no IMU, placed by eye (see landmarks.calibrate_yaw)
    bias_tau: float = 0.0          # s: per-node shape error decorrelates over this time (0 = constant)
    gauge_drift: float = 0.0       # m/sqrt(s): the mesh frame wanders against the world
    gauge_yaw_drift: float = 0.0   # rad/sqrt(s)


@dataclasses.dataclass
class SyncModel:
    """Residual clock error between nodes after whatever sync the firmware does,
    in metres of path (1 ns = 0.30 m). Per node, correlated over `tau` seconds (an
    OU process), so it is not averaged away by taking more frames."""
    sigma: float = 0.0
    tau: float = 5.0


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class MeshSampler:
    """Standalone harness: widgets follow their closed-form trajectories
    (world.TruthWidget). Each TDMA slot is either a pairwise two-way exchange
    (mode "pairwise": round-robin over pairs, observed at both ends, clock errors
    cancel) or a broadcast (mode "broadcast": round-robin over transmitters, every
    other node in range receives it, and each carries the transmitter-receiver
    clock error `sync` into its path lengths). The mesh's frame is the world frame
    moved by a gauge transform world = Rg(t) p_mesh + tg(t) (random walk), plus
    per-node shape error; `gauge_at()` is the truth the P2 solve is scored against.

    Static widgets have no IMU: their attitude is whatever they were placed with
    (`face`: they point their patches at that point), known only to
    `MeshError.noimu_yaw_sigma`."""

    def __init__(self, specs, channel, schedule=(Exchange(),), mesh=None, slot_rate=100.0,
                 gauge0=(0.0, 0.0, 0.0, 0.0), seed=1, mode="pairwise", sync=None, face=None):
        from swarm.sim import world
        self.truth = [world.TruthWidget(s) for s in specs]
        self.ids = [s.wid for s in specs]
        self.ch, self.schedule = channel, list(schedule)
        self.mesh = mesh or MeshError()
        self.slot_rate = slot_rate
        self.mode = mode
        self.sync = sync or SyncModel()
        self.rng = np.random.default_rng(seed + 7919)
        n = len(specs)
        self.has_imu = [s.kind != "static" for s in specs]
        self.static_yaw = {}
        for i, s in enumerate(specs):
            if s.kind == "static" and face is not None:
                p = np.asarray(s.center, float)       # body +y toward `face`
                self.static_yaw[i] = math.atan2(p[0] - face[0], face[1] - p[1])
        self.pos_bias = self.rng.normal(0, self.mesh.pos_sigma, (n, 3))
        sig = np.where(self.has_imu, self.mesh.yaw_sigma, self.mesh.noimu_yaw_sigma)
        self.yaw_bias = self.rng.normal(0, 1, n) * sig
        self.clock = self.rng.normal(0, self.sync.sigma, n)
        self.pairs = [(i, j) for i in range(n) for j in range(i + 1, n)]
        self.gauge = np.array(gauge0, float)        # tx, ty, tz, yaw
        self.counter = {}
        self.slot = 0
        self.window_gauge = self.gauge.copy()     # mean gauge over the last run()

    def gauge_at(self):
        return self.gauge.copy()

    def no_imu(self):
        """Ids of the nodes whose attitude the mesh does not measure."""
        return {self.ids[i] for i, h in enumerate(self.has_imu) if not h}

    def _to_mesh(self, p_w, R_w):
        Rg = rot_z(self.gauge[3])
        return Rg.T @ (p_w - self.gauge[:3]), Rg.T @ R_w

    def _pose(self, i, t):
        R = rot_z(self.static_yaw[i]) if i in self.static_yaw else self.truth[i].rot(t)
        return self.truth[i].pos(t), R

    def _believed(self, x, p_w, R_w):
        p_m, R_m = self._to_mesh(p_w, R_w)
        return (p_m + self.pos_bias[x] + self.rng.normal(0, self.mesh.jitter, 3),
                rot_z(self.yaw_bias[x]) @ R_m)

    def run(self, seconds, t0=0.0):
        obs, dt = [], 1.0 / self.slot_rate
        m, sy = self.mesh, self.sync
        trace = []
        for step in range(int(seconds * self.slot_rate)):
            t = t0 + step * dt
            if m.bias_tau > 0:      # slowly varying shape error: OU with the same stationary sigma
                a = dt / m.bias_tau
                self.pos_bias += -a * self.pos_bias + m.pos_sigma * math.sqrt(2 * a) * self.rng.normal(size=self.pos_bias.shape)
            if sy.sigma > 0:
                a = dt / sy.tau
                self.clock += -a * self.clock + sy.sigma * math.sqrt(2 * a) * self.rng.normal(size=self.clock.shape)
            self.gauge[:3] += self.rng.normal(0, m.gauge_drift * math.sqrt(dt), 3) * [1, 1, 0]
            self.gauge[3] += self.rng.normal(0, m.gauge_yaw_drift * math.sqrt(dt))
            trace.append(self.gauge.copy())
            if self.mode == "broadcast":
                self._broadcast_slot(t, obs)
            else:
                self._pairwise_slot(t, step, obs)
            self.slot += 1
        if trace:
            self.window_gauge = np.mean(trace, axis=0)
        return obs

    def _pairwise_slot(self, t, step, obs):
        i, j = self.pairs[step % len(self.pairs)]
        k = self.counter[(i, j)] = self.counter.get((i, j), -1) + 1
        ex = self.schedule[k % len(self.schedule)]
        pose = [self._pose(x, t) for x in (i, j)]
        bel = [self._believed(x, *pose[q]) for q, x in enumerate((i, j))]
        for tx, rx in ((0, 1), (1, 0)):
            o = self.ch.observe(t, self.ids[(i, j)[tx]], self.ids[(i, j)[rx]], pose[tx][0], pose[rx][0],
                                pose[rx][1], (bel[tx][0], bel[rx][0], bel[rx][1]), ex)
            if o is not None:
                obs.append(o)

    def _broadcast_slot(self, t, obs):
        n = len(self.truth)
        tx = self.slot % n
        ex = self.schedule[(self.slot // n) % len(self.schedule)]      # one channel per slot, all rx use it
        poses = [self._pose(x, t) for x in range(n)]
        bel = [self._believed(x, *poses[x]) for x in range(n)]
        for rx in range(n):
            if rx == tx:
                continue
            o = self.ch.observe(t, self.ids[tx], self.ids[rx], poses[tx][0], poses[rx][0], poses[rx][1],
                                (bel[tx][0], bel[rx][0], bel[rx][1]), ex,
                                sync_len=float(self.clock[rx] - self.clock[tx]))
            if o is not None:
                obs.append(o)
