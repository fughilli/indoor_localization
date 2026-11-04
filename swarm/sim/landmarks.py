"""Joint estimation of fixed reflectors and faders against the UWB mesh.

Two problems on the same measurements (multipath.LinkObs):

  P1  map      where are the reflectors and faders, in the mesh's own frame?
               `map_reflectors` / `map_faders`.
  P2  locate   where is the mesh, given reflectors/faders at known positions in the
               room? `register` finds the gauge transform (yaw + translation: the
               mesh knows up from gravity) and `solve_gauge` refines it from the raw
               observations, per time window, so the mesh's drift is held to the
               fixed references instead of wandering.

What each observation says (L = a reported path length, a/b the endpoints, m a
reflector):

  length   |a - m| + |m - b| = L          an ellipsoid with foci a, b
  phase    k * baseline . R_b^T (m-b)/|m-b|  = PDoA        a cone about the baseline
  gap/SNR  a fader c shadows a link a-b by loss * chord(a, b, c, r) / (2r) dB

Tiers (SolveConfig.use_cir / use_phase): with first arrival only the reflector shows
up only when a fader has made the radio time a reflection as its first path; with
the CIR every resolved path is a constraint; with PDoA each path also carries a
bearing, so a single link fixes a reflector.
"""

import dataclasses
import math

import numpy as np
from scipy.optimize import least_squares, linear_sum_assignment

from swarm.sim import multipath as mp


@dataclasses.dataclass
class SolveConfig:
    use_cir: bool = True           # per-path lengths from the CIR (else first arrival only)
    use_phase: bool = True         # PDoA phase of each path
    snr1_db: float = 60.0          # radio constants the receiver is calibrated for
    detect_thr_db: float = 10.0
    sigma_shared: float = 0.07
    sigma_floor: float = 0.05
    k_len: float = 0.6
    sigma_phase0: float = 0.05
    k_phase: float = 1.0
    sigma_mesh: float = 0.15       # m: the mesh's own position error, in a path length
    sigma_att: float = 0.05        # rad: its attitude error (and PDoA calibration), in a phase
    sigma_first: float = 0.5       # m: a first-arrival-only range that read long
    los_gate: float = 0.6          # m: a path this much longer than |a-b| is not the LOS
    nlos_gap_db: float = 10.0      # first-arrival only: gap above this = LOS lost
    gate: float = 3.0              # association gate, in sigmas
    cell: float = 0.6              # grid cell for the vote, m
    margin: float = 8.0            # grid extends this far beyond the nodes (horizontally)
    margin_z: float = 3.0          # ... and vertically
    vote_sample: int = 1200        # detections voting per pass
    min_support: int = 20          # detections for a landmark
    min_rel_support: float = 0.15  # ... and this fraction of the best landmark's
    peak_sigma: float = 6.0        # the vote peak must clear the background level by this many sqrt(level)
    min_links: int = 6             # ... from this many different node pairs
    max_landmarks: int = 8
    fader_radius: float = 0.4      # assumed fader size, m
    fader_cell: float = 0.5
    fader_min_gain: float = 40.0   # chi^2 gain to accept a fader
    fader_sigma_db: float = 3.0
    fader_sample: int = 3000       # frames used for the centre search
    seed: int = 0


# --- detections -----------------------------------------------------------------------

@dataclasses.dataclass
class Detections:
    """Every non-line-of-sight path report, flat. `link` indexes the node pair."""
    ptx: np.ndarray
    prx: np.ndarray
    R: np.ndarray
    L: np.ndarray
    sL: np.ndarray
    phi: np.ndarray
    sphi: np.ndarray
    base: np.ndarray
    k: np.ndarray
    link: np.ndarray
    label: np.ndarray
    t: np.ndarray

    def __len__(self):
        return len(self.L)

    def take(self, idx):
        return Detections(*(getattr(self, f.name)[idx] for f in dataclasses.fields(self)))

    @property
    def has_phase(self):
        return np.isfinite(self.phi)


def sigma_len(cfg, snr_db):
    return np.hypot(cfg.sigma_mesh, np.hypot(cfg.sigma_shared,
                    np.hypot(cfg.sigma_floor, cfg.k_len / 10 ** (np.asarray(snr_db) / 20))))


def sigma_phase(cfg, snr_db):
    return np.hypot(cfg.sigma_att * math.pi, np.hypot(cfg.sigma_phase0, cfg.k_phase / 10 ** (np.asarray(snr_db) / 20)))


def build_detections(obs, cfg, array):
    rows = []
    for o in obs:
        d = float(np.linalg.norm(o.p_tx - o.p_rx))
        if cfg.use_cir:
            sel = np.nonzero(o.L - d > cfg.los_gate)[0]
        else:
            # First arrival only: the one range, and only when the radio says the
            # line of sight is gone and the range read long.
            sel = [0] if (o.gap_db > cfg.nlos_gap_db and o.rng - d > cfg.los_gate) else []
        for i in sel:
            ph = o.phase[i] if (cfg.use_cir and cfg.use_phase and o.pair >= 0) else np.nan
            snr = o.snr[i] if cfg.use_cir else 0.0
            pr = array[o.pair] if o.pair >= 0 else None
            rows.append((o.p_tx, o.p_rx, o.R_rx, o.L[i],
                         sigma_len(cfg, snr) if cfg.use_cir else cfg.sigma_first,
                         ph, sigma_phase(cfg, snr) if np.isfinite(ph) else np.nan,
                         np.asarray(pr.baseline) if pr else np.zeros(3),
                         2 * math.pi / mp.wavelength(pr.channel) if pr else 0.0,
                         min(o.tx, o.rx) * 1000 + max(o.tx, o.rx), o.label[i], o.t))
    if not rows:
        return Detections(*(np.empty((0,) + s) for s in
                            [(3,), (3,), (3, 3), (), (), (), (), (3,), (), (), (), ()]))
    cols = list(zip(*rows))
    return Detections(*(np.array(c, float if j not in (9, 10) else int) for j, c in enumerate(cols)))


# --- residuals ------------------------------------------------------------------------

def reflector_residuals(m, D, T=None):
    """Normalised length (and phase) residuals of detections D against reflector m
    (world frame if T=(Rg, tg) maps mesh -> world). Returns (len_res, phase_res);
    phase_res is NaN where the detection has none."""
    ptx, prx, R = D.ptx, D.prx, D.R
    if T is not None:
        Rg, tg = T
        ptx, prx, R = ptx @ Rg.T + tg, prx @ Rg.T + tg, Rg @ R
    d1 = np.linalg.norm(m - ptx, axis=-1)
    v = m - prx
    d2 = np.maximum(np.linalg.norm(v, axis=-1), 1e-9)
    rl = (d1 + d2 - D.L) / D.sL
    ub = np.einsum("nji,nj->ni", R, v / d2[:, None])
    pred = D.k * np.einsum("ni,ni->n", D.base, ub)
    rp = mp.wrap(pred - D.phi) / D.sphi
    return rl, rp


def stacked(m, D, T=None):
    rl, rp = reflector_residuals(m, D, T)
    return np.concatenate([rl, rp[D.has_phase]])


# --- reflectors (P1) -------------------------------------------------------------------

@dataclasses.dataclass
class Landmark:
    pos: np.ndarray
    cov: np.ndarray
    support: int
    links: int
    members: np.ndarray            # indices into the Detections it explains

    @property
    def sigma(self):
        return math.sqrt(max(float(np.trace(self.cov)), 0.0))


def _vote(grid, D, chunk=1500):
    """Vote of detections D for each grid point: how many ellipsoids (and, with
    phase, cones) pass through it."""
    score = np.empty(len(grid))
    for s in range(0, len(grid), chunk):
        g = grid[s:s + chunk][:, None, :]
        d1 = np.linalg.norm(g - D.ptx[None], axis=-1)
        v = g - D.prx[None]
        d2 = np.maximum(np.linalg.norm(v, axis=-1), 1e-9)
        w = np.exp(-0.5 * ((d1 + d2 - D.L) / D.sL) ** 2)
        hp = D.has_phase
        if hp.any():
            ub = np.einsum("nji,cnj->cni", D.R[hp], v[:, hp] / d2[:, hp, None])
            pred = D.k[hp] * np.einsum("ni,cni->cn", D.base[hp], ub)
            wp = np.exp(-0.5 * (mp.wrap(pred - D.phi[hp]) / D.sphi[hp]) ** 2)
            w[:, hp] *= 0.15 + 0.85 * wp
        score[s:s + chunk] = w.sum(1)
    return score


def _grid(lo, hi, cell):
    axes = [np.arange(l, h + cell, cell) for l, h in zip(lo, hi)]
    return np.stack(np.meshgrid(*axes, indexing="ij"), -1).reshape(-1, 3)


def node_bounds(obs, margin, margin_z=None):
    pts = np.array([p for o in obs for p in (o.p_tx, o.p_rx)])
    m = np.array([margin, margin, margin if margin_z is None else margin_z])
    return pts.min(0) - m, pts.max(0) + m


def _refine(m0, D, sel):
    f = lambda m: stacked(m, D.take(sel))
    r = least_squares(f, m0, loss="soft_l1", f_scale=1.5)
    J = r.jac
    cov = np.linalg.pinv(J.T @ J)
    return r.x, cov


def map_reflectors(obs, cfg, array, bounds=None):
    """P1 for reflectors: grid-vote for the best-supported point, refine it on the
    detections it explains, remove those, repeat."""
    D = build_detections(obs, cfg, array)
    if len(D) == 0:
        return [], D
    lo, hi = bounds or node_bounds(obs, cfg.margin, cfg.margin_z)
    rng = np.random.default_rng(cfg.seed)
    grid = _grid(lo, hi, cfg.cell)
    free = np.ones(len(D), bool)
    found = []
    for _ in range(cfg.max_landmarks):
        idx = np.nonzero(free)[0]
        if len(idx) < cfg.min_support:
            break
        vote = rng.choice(idx, min(len(idx), cfg.vote_sample), replace=False)
        score = _vote(grid, D.take(vote))
        # Chance alignments of clutter put a floor under every cell's vote; a real
        # landmark has to stand clear of that floor, whatever the clutter rate.
        floor = float(np.percentile(score, 90))
        if score.max() < floor + cfg.peak_sigma * math.sqrt(floor) or score.max() < 2 * floor:
            break
        m = grid[int(np.argmax(score))]
        # Fine pass around the peak, then least squares on what it explains.
        fine = _grid(m - cfg.cell, m + cfg.cell, cfg.cell / 5)
        m = fine[int(np.argmax(_vote(fine, D.take(vote))))]
        first = _explained(m, D, idx, cfg)
        sel, cov = None, None
        for _ in range(3):
            cand = _explained(m, D, idx, cfg)
            if len(cand) < cfg.min_support:
                break
            m, cov = _refine(m, D, cand)
            sel = _explained(m, D, idx, cfg)
        if sel is not None and len(sel) >= cfg.min_support:
            links = len(np.unique(D.link[sel]))
            strongest = max([l.support for l in found], default=len(sel))
            if links >= cfg.min_links and len(sel) >= cfg.min_rel_support * strongest:
                found.append(Landmark(m, cov, len(sel), links, sel))
            free[sel] = False
        elif len(first):
            free[first] = False        # a chance alignment: drop its votes, look again
        else:
            break
    return found, D


def _explained(m, D, idx, cfg):
    """Indices (of idx) of detections consistent with a reflector at m."""
    rl, rp = reflector_residuals(m, D.take(idx))
    ok = (np.abs(rl) < cfg.gate) & (~np.isfinite(rp) | (np.abs(rp) < cfg.gate + 0.5))
    return idx[ok]


# --- faders (P1) -----------------------------------------------------------------------

@dataclasses.dataclass
class LinkAtten:
    a: np.ndarray
    b: np.ndarray
    y: np.ndarray          # observed attenuation, dB
    sigma: float
    censored: np.ndarray   # True: y is only a lower bound (the line of sight was lost)


def link_attenuation(obs, cfg):
    """Per-frame estimate of how much the line of sight was shadowed."""
    A, B, Y, C = [], [], [], []
    for o in obs:
        d = float(np.linalg.norm(o.p_tx - o.p_rx))
        if cfg.use_cir:
            expect = cfg.snr1_db - 20 * math.log10(max(d, 0.1))
            hit = np.nonzero(np.abs(o.L - d) < cfg.los_gate)[0]
            if len(hit):
                y, c = expect - float(o.snr[hit[0]]), False
            else:
                y, c = expect - cfg.detect_thr_db, True
        else:
            y, c = max(0.0, o.gap_db - 3.5), False
        A.append(o.p_tx); B.append(o.p_rx); Y.append(y); C.append(c)
    sigma = cfg.fader_sigma_db if cfg.use_cir else 1.5 * cfg.fader_sigma_db
    return LinkAtten(np.array(A), np.array(B), np.array(Y), sigma, np.array(C))


def _pred_atten(c, loss, la, r):
    return loss * mp.chord(la.a, la.b, c, r) / (2 * r)


def _atten_res(la, pred):
    """Residuals in sigmas. A censored y (line of sight lost) is a lower bound:
    only a prediction below it counts."""
    d = pred - la.y
    return np.where(la.censored, np.minimum(d, 0.0), d) / la.sigma


def map_faders(obs, cfg, bounds=None, max_faders=4):
    """P1 for faders: greedy tomography. Each pass grid-searches the sphere centre
    that most reduces the attenuation misfit (loss amplitude by least squares),
    then refines centre and loss together."""
    la = link_attenuation(obs, cfg)
    rng = np.random.default_rng(cfg.seed)
    sub = np.sort(rng.choice(len(la.y), min(len(la.y), cfg.fader_sample), replace=False))
    lo, hi = bounds or node_bounds(obs, 2.0)
    grid = _grid(lo, hi, cfg.fader_cell)
    r = cfg.fader_radius
    resid = la.y.copy()
    keep = (~la.censored).astype(float)     # a lower bound isn't a measurement
    found = []
    for _ in range(max_faders):
        best, best_gain = None, 0.0
        for c in grid:
            if any(np.linalg.norm(c - f[0]) < 1.0 for f in found):
                continue
            g = mp.chord(la.a[sub], la.b[sub], c, r) / (2 * r) * keep[sub]
            gg = float(g @ g)
            if gg < 1e-6:
                continue
            gain = float(g @ resid[sub]) ** 2 / gg / la.sigma ** 2 * (len(la.y) / len(sub))
            if gain > best_gain:
                best, best_gain = c, gain
        if best is None or best_gain < cfg.fader_min_gain:
            break
        g = mp.chord(la.a, la.b, best, r) / (2 * r) * keep
        loss0 = max(float(g @ resid) / float(g @ g), 1.0)
        prior = np.zeros(len(la.y))
        for f in found:
            prior += _pred_atten(f[0], f[1], la, r)
        fun = lambda x: _atten_res(la, prior + _pred_atten(x[:3], x[3], la, r))
        sol = least_squares(fun, np.append(best, loss0), loss="soft_l1", f_scale=1.5)
        cov = np.linalg.pinv(sol.jac.T @ sol.jac)
        found.append((sol.x[:3], float(sol.x[3]), cov[:3, :3]))
        resid = la.y - prior - _pred_atten(sol.x[:3], sol.x[3], la, r)
    return found


# --- scoring ---------------------------------------------------------------------------

def match(est, truth, gate=4.0):
    """Hungarian match of estimated to true positions within `gate` metres.
    Returns (est index per true landmark, -1 if missed; indices of unmatched
    estimates)."""
    est, truth = np.atleast_2d(est).reshape(-1, 3), np.atleast_2d(truth).reshape(-1, 3)
    idx = np.full(len(truth), -1)
    if len(est) == 0 or len(truth) == 0:
        return idx, np.arange(len(est))
    cost = np.linalg.norm(est[:, None] - truth[None], axis=-1)
    for i, j in zip(*linear_sum_assignment(cost)):
        if cost[i, j] <= gate:
            idx[j] = i
    return idx, np.array([i for i in range(len(est)) if i not in set(idx)], int)


# --- P2: the mesh against references -------------------------------------------------------

def gauge_matrix(g):
    """(Rg, tg) of a gauge g = (tx, ty, tz, yaw): world = Rg p_mesh + tg."""
    return mp.rot_z(g[3]), np.asarray(g[:3], float)


def register(mesh_pts, ref_pts, tol=1.0):
    """Gauge g (tx, ty, tz, yaw) taking mesh-frame landmark positions onto the known
    references, by trying every pair-to-pair match and keeping the one most other
    landmarks agree with. Needs >= 2 of each; returns (g, inliers) or (None, 0)."""
    P, Q = np.asarray(mesh_pts, float), np.asarray(ref_pts, float)
    best, best_n, best_e = None, 0, np.inf
    for i in range(len(P)):
        for j in range(len(P)):
            if i == j:
                continue
            dp = np.linalg.norm(P[j, :2] - P[i, :2])
            if dp < 1e-6:
                continue
            for a in range(len(Q)):
                for b in range(len(Q)):
                    if a == b or abs(np.linalg.norm(Q[b, :2] - Q[a, :2]) - dp) > tol:
                        continue
                    yaw = (math.atan2(*(Q[b, :2] - Q[a, :2])[::-1]) - math.atan2(*(P[j, :2] - P[i, :2])[::-1]))
                    Rg = mp.rot_z(yaw)
                    tg = Q[a] - Rg @ P[i]
                    d = np.linalg.norm((P @ Rg.T + tg)[:, None] - Q[None], axis=-1).min(1)
                    n, e = int((d < tol).sum()), float(d[d < tol].sum())
                    if n > best_n or (n == best_n and e < best_e):
                        best, best_n, best_e = np.array([*tg, yaw]), n, e
    return best, best_n


def solve_gauge(obs, refs, cfg, array, g0, prior_sigma=(5.0, 5.0, 5.0, 0.5), iters=3):
    """P2: refine the gauge against reference reflectors `refs` (world positions) from
    the raw observations. Each pass re-associates every detection to the reference
    whose path it fits best (within the gate), then least-squares on (tx, ty, tz,
    yaw). The weak prior keeps the solve defined when the references don't pin all
    four degrees of freedom (a single reflector leaves a ring; see the covariance).
    Returns (g, cov, n_used)."""
    D = build_detections(obs, cfg, array)
    refs = np.atleast_2d(np.asarray(refs, float)).reshape(-1, 3)
    g = np.array(g0, float)
    prior = np.array(prior_sigma)
    cov = np.eye(4)
    n_used = 0
    for _ in range(iters):
        T = gauge_matrix(g)
        best = np.full(len(D), -1)
        bestr = np.full(len(D), np.inf)
        for k, m in enumerate(refs):
            rl, rp = reflector_residuals(m, D, T)
            ok = (np.abs(rl) < cfg.gate) & (~np.isfinite(rp) | (np.abs(rp) < cfg.gate + 0.5))
            better = ok & (np.abs(rl) < bestr)
            best[better], bestr[better] = k, np.abs(rl)[better]
        used = np.nonzero(best >= 0)[0]
        n_used = len(used)
        if n_used < 4:
            break
        Du, asg = D.take(used), best[used]

        def fun(x):
            Tx = gauge_matrix(x)
            out = []
            for k in range(len(refs)):
                s = asg == k
                if s.any():
                    out.append(stacked(refs[k], Du.take(np.nonzero(s)[0]), Tx))
            return np.concatenate(out + [(x - g0) / prior])
        sol = least_squares(fun, g, loss="soft_l1", f_scale=1.5)
        g = sol.x
        cov = np.linalg.pinv(sol.jac.T @ sol.jac)
    return g, cov, n_used
