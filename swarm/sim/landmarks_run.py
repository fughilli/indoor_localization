"""Compare what each radio capability buys for locating reflectors and faders.

  python3 -m swarm.sim.landmarks_run                # P1 over the four tiers
  python3 -m swarm.sim.landmarks_run --p2           # P2: the mesh against references
  python3 -m swarm.sim.landmarks_run --seeds 5 --seconds 90

Tiers (the same scene, trajectories and noise seeds in each):
  first     first arrival only: range + first-path power gap
  cir       + every resolved path from the CIR (length, SNR)
  pdoa2     + PDoA from one pair of patches on channel 5 (baseline along body x)
  pdoa4     + a second pair on channel 9 (baseline along body z), one pair per
            exchange, alternating (the QM33120 with diplexed CH5/CH9 patches); the
            board stands upright, patches facing body +y
  pdoa4flat the same array lying flat, patches facing up (both baselines horizontal)
"""

import argparse
import dataclasses
import sys
import time

import numpy as np

from swarm.sim import landmarks as lm
from swarm.sim import multipath as mp
from swarm.sim import world

UPRIGHT, FLAT = mp.upright_array(), mp.flat_array()
TIERS = {
    "first": dict(array=[], schedule=[mp.Exchange(5, -1)], use_cir=False, use_phase=False),
    "cir": dict(array=[], schedule=[mp.Exchange(5, -1)], use_cir=True, use_phase=False),
    "pdoa2": dict(array=UPRIGHT[:1], schedule=[mp.Exchange(5, 0)], use_cir=True, use_phase=True),
    "pdoa4": dict(array=UPRIGHT, schedule=[mp.Exchange(5, 0), mp.Exchange(9, 1)], use_cir=True, use_phase=True),
    "pdoa4flat": dict(array=FLAT, schedule=[mp.Exchange(5, 0), mp.Exchange(9, 1)], use_cir=True, use_phase=True),
}


def default_scene():
    return mp.Scene(
        reflectors=[mp.Reflector((1.0, 15.0, 2.0)), mp.Reflector((18.5, 2.0, 1.0)),
                    mp.Reflector((12.0, 19.0, 3.0))],
        faders=[mp.Fader((10.0, 10.0, 1.5), 0.4, 20.0), mp.Fader((6.0, 13.0, 1.2), 0.4, 20.0)])


def sample(tier, seed, seconds, scene=None, mesh=None, n=6, gauge0=(0, 0, 0, 0), motion="orbit"):
    t = TIERS[tier]
    scene = scene or default_scene()
    ch = mp.Channel(scene, array=t["array"], seed=seed)
    smp = mp.MeshSampler(swarm_specs(motion, n), ch, t["schedule"], mesh=mesh,
                         seed=seed, gauge0=gauge0)
    return smp, smp.run(seconds), scene


def p1(tier, seed, seconds, **kw):
    smp, obs, scene = sample(tier, seed, seconds, **kw)
    t = TIERS[tier]
    cfg = lm.SolveConfig(use_cir=t["use_cir"], use_phase=t["use_phase"], seed=seed)
    # The map is in the mesh frame, which here is the world frame (no gauge offset).
    t0 = time.time()
    refl, D = lm.map_reflectors(obs, cfg, t["array"])
    faders = lm.map_faders(obs, cfg)
    truth_r = np.array([r.pos for r in scene.reflectors])
    truth_f = np.array([f.pos for f in scene.faders])
    est_r = np.array([l.pos for l in refl]).reshape(-1, 3)
    est_f = np.array([f[0] for f in faders]).reshape(-1, 3)
    ir, extra_r = lm.match(est_r, truth_r)
    jf, extra_f = lm.match(est_f, truth_f)
    return dict(frames=len(obs), dets=len(D), false_r=len(extra_r), false_f=len(extra_f),
                dr=errors(est_r, ir, truth_r), df=errors(est_f, jf, truth_f),
                pred=[l.sigma for l in refl], secs=time.time() - t0)


def errors(est, idx, truth):
    """(n_true, 3) error vectors est - truth, NaN where missed."""
    out = np.full(truth.shape, np.nan)
    for j, i in enumerate(idx):
        if i >= 0:
            out[j] = est[i] - truth[j]
    return out


def fmt(e):
    """Found count, horizontal and vertical rms of (n, 3) error vectors."""
    ok = e[np.isfinite(e[:, 0])]
    if not len(ok):
        return "0/%d found" % len(e)
    return "%d/%d found, horiz %.2f m, vert %.2f m" % (
        len(ok), len(e), np.sqrt((ok[:, :2] ** 2).sum(1).mean()), np.sqrt((ok[:, 2] ** 2).mean()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=3)
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--p2", action="store_true")
    ap.add_argument("--study", action="store_true", help="with --p2: the error-budget / motion study")
    args = ap.parse_args()
    if args.p2:
        return p2_main(args)
    print("P1: landmark positions in the mesh frame (3 reflectors, 2 faders), %d seeds x %.0f s" %
          (args.seeds, args.seconds))
    for tier in TIERS:
        er, ef, fr, ff = [], [], 0, 0
        for s in range(args.seeds):
            r = p1(tier, s + 1, args.seconds)
            er.append(r["dr"]); ef.append(r["df"]); fr += r["false_r"]; ff += r["false_f"]
        print("%-6s reflectors: %-44s faders: %-44s false: %d refl, %d fader" %
              (tier, fmt(np.concatenate(er)), fmt(np.concatenate(ef)), fr, ff))


def p2(tier, seed, windows=6, win_s=10.0, drift=(0.03, 0.004), gauge0=(1.7, -2.3, 0.4, 0.12), n_refs=3):
    """The mesh against references at known room positions. The mesh frame is the
    room moved by `gauge0` and then wandering (a random walk). Acquire: map the
    reflectors in the first window and register them onto the references. Track:
    refine the gauge per window against the references. Baseline: the gauge frozen
    where it was acquired (what a mesh with no fixed references has)."""
    t = TIERS[tier]
    scene = default_scene()
    refs = np.array([x.pos for x in scene.reflectors])[:n_refs]
    ch = mp.Channel(scene, array=t["array"], seed=seed)
    mesh = mp.MeshError(gauge_drift=drift[0], gauge_yaw_drift=drift[1])
    smp = mp.MeshSampler(world.default_swarm(6, 0, 0, 20.0), ch, t["schedule"], mesh=mesh, seed=seed,
                         gauge0=gauge0)
    cfg = lm.SolveConfig(use_cir=t["use_cir"], use_phase=t["use_phase"], seed=seed)
    obs = smp.run(win_s)
    mapped, _ = lm.map_reflectors(obs, cfg, t["array"])
    g0, inl = lm.register(np.array([l.pos for l in mapped]).reshape(-1, 3), refs)
    out = dict(acquired=g0 is not None, inliers=inl, truth0=smp.gauge_at(), g0=g0, rows=[])
    if g0 is None:
        return out
    g_prev, g_frozen = g0.copy(), g0.copy()
    for w in range(windows):
        obs = smp.run(win_s, t0=(w + 1) * win_s)
        g, cov, n = lm.solve_gauge(obs, refs, cfg, t["array"], g_prev)
        truth = smp.gauge_at()
        out["rows"].append(dict(
            t=(w + 2) * win_s, n=n,
            err=_gauge_err(g, truth), frozen=_gauge_err(g_frozen, truth),
            sigma=np.sqrt(np.diag(cov))))
        g_prev = g
    return out


def swarm_specs(motion, n=6):
    """still: nobody moves. orbit: the default sim swarm (large smooth orbits, yaw
    spin only). handheld: carried by hand, stop-and-go, with the wrist wobble and
    lift of world.room_swarm, so the board's attitude changes too."""
    specs = world.default_swarm(n, 0, 0, 20.0)
    for s in specs:
        if motion == "still":
            s.center, s.kind, s.yaw_rate = tuple(world._true_pos(s, 0.0)), "static", 0.0
        elif motion == "handheld":
            s.move_period, s.move_duty = 20.0, 0.6
            s.sway, s.bob, s.handling, s.lift = 0.15, 0.03, 0.5, 0.25
    return specs


MOUNTS = {"none": [], "flat": mp.flat_array(), "upright": mp.upright_array()}


def p2_track(motion="orbit", mount="upright", pos_sigma=0.10, bias_tau=0.0, windows=6, win_s=10.0,
             drift=(0.03, 0.004), start_err=(0.25, -0.2, 0.1, 0.03), seed=1, gauge0=(1.7, -2.3, 0.4, 0.12),
             n_refs=3, cfg_kw=None, n=6):
    """Tracking only: the gauge starts `start_err` off (an earlier acquisition) and is
    refined against the references each window. Scored against the *mean* true
    gauge over the window, since that is what a window's data can know."""
    array = MOUNTS[mount]
    cir_only = not array
    schedule = [mp.Exchange(5, -1)] if cir_only else [mp.Exchange(5, 0), mp.Exchange(9, 1)]
    scene = default_scene()
    refs = np.array([x.pos for x in scene.reflectors])[:n_refs]
    ch = mp.Channel(scene, array=array, seed=seed)
    mesh = mp.MeshError(pos_sigma=pos_sigma, bias_tau=bias_tau, gauge_drift=drift[0], gauge_yaw_drift=drift[1])
    smp = mp.MeshSampler(swarm_specs(motion, n), ch, schedule, mesh=mesh, seed=seed, gauge0=gauge0)
    cfg = lm.SolveConfig(use_cir=True, use_phase=not cir_only, seed=seed, **(cfg_kw or {}))
    g = np.array(gauge0, float) + start_err
    rows = []
    for w in range(windows):
        obs = smp.run(win_s, t0=w * win_s)
        g, cov, n = lm.solve_gauge(obs, refs, cfg, array, g)
        rows.append(dict(n=n, err=_gauge_err(g, smp.window_gauge), now=_gauge_err(g, smp.gauge_at()),
                         sigma=np.sqrt(np.diag(cov))))
    return rows


def _gauge_err(g, truth):
    d = np.asarray(g, float) - truth
    d[3] = (d[3] + np.pi) % (2 * np.pi) - np.pi
    return np.array([np.hypot(d[0], d[1]), abs(d[2]), abs(np.degrees(d[3]))])


def study(args):
    """What limits the P2 tracking error, and does motion help?"""
    def row(label, **kw):
        errs = np.array([x["err"] for s in range(args.seeds) for x in p2_track(seed=s + 1, **kw)[1:]])
        print("%-46s xy %.3f m   z %.3f m   yaw %.2f deg" % ((label,) + tuple(np.sqrt((errs ** 2).mean(0)))),
              flush=True)
    print("P2 tracking, window 10 s, %d seeds, first window (settling) dropped" % args.seeds)
    print("-- error budget (upright mount, orbit motion)")
    row("no mesh shape error", pos_sigma=0.0)
    row("shape error 0.10 m, constant per node", pos_sigma=0.10)
    row("shape error 0.10 m, decorrelating over 15 s", pos_sigma=0.10, bias_tau=15.0)
    print("-- does motion help? (shape error 0.10 m constant | decorrelating over 15 s)")
    for motion in ("still", "orbit", "handheld"):
        row("%-8s constant" % motion, motion=motion)
        row("%-8s decorrelating" % motion, motion=motion, bias_tau=15.0)
    print("-- what the array adds to the P2 solve (orbit motion)")
    for mount in ("none", "flat", "upright"):
        row("mount: %s" % mount, mount=mount)
    print("-- more nodes (shape error is per node, so the mean over nodes should shrink as 1/sqrt(N))")
    for n in (4, 6, 12):
        row("%2d widgets" % n, n=n)
    print("-- window length, shape error decorrelating over 15 s, 240 s of data; 'vs now' = vs the gauge at the window's end")
    for win in (5.0, 10.0, 20.0, 40.0, 80.0):
        rows = [x for s in range(args.seeds) for x in p2_track(seed=s + 1, bias_tau=15.0, win_s=win,
                                                               windows=int(240 / win))[1:]]
        e = np.array([x["err"] for x in rows])
        nw = np.array([x["now"] for x in rows])
        print("window %4.0f s:  vs window mean xy %.3f z %.3f   |   vs now xy %.3f z %.3f yaw %.2f deg" %
              ((win,) + tuple(np.sqrt((e ** 2).mean(0))[:2]) + tuple(np.sqrt((nw ** 2).mean(0)))), flush=True)


def p2_main(args):
    if args.study:
        return study(args)
    print("P2: tracking the mesh gauge against reference reflectors (%d seeds, %.0f s windows)" %
          (args.seeds, args.seconds / 6))
    for tier in ("cir", "pdoa4"):
        print("--", tier)
        for s in range(args.seeds):
            o = p2(tier, s + 1, win_s=args.seconds / 6)
            if not o["acquired"]:
                print("   seed %d: registration failed (%d inliers)" % (s + 1, o["inliers"]))
                continue
            e = np.array([x["err"] for x in o["rows"]])
            f = np.array([x["frozen"] for x in o["rows"]])
            print("   seed %d: referenced  xy %.2f m, z %.2f m, yaw %.2f deg | no references (frozen) xy %.2f m, "
                  "z %.2f m, yaw %.2f deg" % ((s + 1,) + tuple(np.sqrt((e ** 2).mean(0))) + tuple(np.sqrt((f ** 2).mean(0)))))


if __name__ == "__main__":
    main()
