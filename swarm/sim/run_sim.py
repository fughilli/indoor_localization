"""Headless swarm-sim runner: prints per-widget convergence + error metrics.

  bazel run //swarm/sim:run_sim -- --widgets 6 --gps 2 --seconds 60

Metrics: position error at end, RMSE over the last quarter of the run, and
time-to-converge (first time the error stays below --converged-m).
"""

import argparse
import json
import math

import numpy as np

from swarm.sim import world
from swarm.tools import widget_config


def apply_intrinsics(sensors, config):
    """Map a WidgetConfig's intrinsics onto the sim's SensorConfig — the
    same mapping firmware applies to its estimator Params and measurement
    noise (characterized values replace the conservative defaults)."""
    sqrt_rate = math.sqrt(sensors.imu_rate)
    intr = config.intrinsics
    imu = intr.imu
    if imu.HasField("accel_noise_density"):
        sensors.accel_sigma = imu.accel_noise_density * sqrt_rate
    if imu.HasField("gyro_noise_density"):
        sensors.gyro_sigma = imu.gyro_noise_density * sqrt_rate
    rng = intr.uwb_range
    if rng.HasField("sigma_m"):
        sensors.uwb_range_sigma = rng.sigma_m
    if rng.HasField("outlier_frac"):
        sensors.uwb_multipath_prob = rng.outlier_frac
    if intr.uwb_aoa.HasField("sigma_rad"):
        sensors.uwb_aoa_sigma = intr.uwb_aoa.sigma_rad
    gps = intr.gps
    if gps.HasField("sigma_h_m"):
        sensors.gps_sigma_h = gps.sigma_h_m
    if gps.HasField("sigma_v_m"):
        sensors.gps_sigma_v = gps.sigma_v_m
    if intr.alt.HasField("sigma_m"):
        sensors.alt_sigma = intr.alt.sigma_m
    return sensors


def run(n_widgets, n_gps, seconds, seed=1234, converged_m=1.0, aoa=True,
        frame_cb=None, trace=False, n_surveyed=0, downhill_ratio=0.5,
        sensor_overrides=None, static=False, intrinsics_path=None,
        motion_rate=1.0, move_period=0.0, move_duty=1.0):
    specs = world.default_swarm(n=n_widgets, n_gps=n_gps, n_surveyed=n_surveyed,
                                motion_rate=motion_rate,
                                move_period=move_period, move_duty=move_duty)
    for s in specs:
        s.has_aoa = aoa
        if static and not s.surveyed:
            s.kind = "static"
    sensors = world.SensorConfig(downhill_ratio=downhill_ratio,
                                 **(sensor_overrides or {}))
    if intrinsics_path:
        apply_intrinsics(sensors, widget_config.load(intrinsics_path))
    w = world.World(specs, sensors=sensors, seed=seed)
    n_widgets = len(specs)
    history = []  # (t, [abs_err_i...], [rel_err_i...])

    def record(frame):
        # Relative error: absolute error after removing the swarm-mean
        # (centroid) offset — the metric for anchorless swarms, where common
        # translation is unobservable by design.
        est = np.array([wg["est_p"] for wg in frame["widgets"]])
        true = np.array([wg["true_p"] for wg in frame["widgets"]])
        offset = (est - true).mean(axis=0)
        rel = np.linalg.norm((est - offset) - true, axis=1)
        # Rotation-aligned residual (Kabsch): what remains after the best
        # rigid translation+rotation — separates "mesh globally rotated"
        # from "mesh deformed".
        ec = est - est.mean(axis=0)
        tc = true - true.mean(axis=0)
        u, _, vt = np.linalg.svd(ec.T @ tc)
        d = np.sign(np.linalg.det(u @ vt))
        rot = (u @ np.diag([1.0, 1.0, d]) @ vt).T
        rot_rel = np.linalg.norm((rot @ ec.T).T - tc, axis=1)
        # est - true per widget: frame-to-frame changes of this deviation
        # are the visible "jitter" (truth motion subtracted out).
        history.append((frame["t"], [wg["err"] for wg in frame["widgets"]],
                        list(rel), list(rot_rel), est - true))
        if trace and abs(frame["t"] % 1.0) < 0.05:
            verr = [np.linalg.norm(np.array(wg["est_v"]) - wg["true_v"])
                    for wg in frame["widgets"]]
            print("t=%5.1f  relerr %5.2f  " % (frame["t"], float(np.mean(rel)))
                  + "  ".join(
                      "e%.1f/s%.1f/v%.2f" % (wg["err"], wg["pos_var"] ** 0.5, ve)
                      for wg, ve in zip(frame["widgets"], verr)))
        if frame_cb:
            frame_cb(frame)

    w.run(seconds, frame_cb=record, frame_rate=10.0)

    errs = np.array([h[1] for h in history])
    rels = np.array([h[2] for h in history])
    rot_rels = np.array([h[3] for h in history])
    devs = np.array([h[4] for h in history])   # frames x widgets x 3
    times = np.array([h[0] for h in history])
    tail = errs[int(len(errs) * 0.75):]
    rel_tail = rels[int(len(rels) * 0.75):]
    rot_tail = rot_rels[int(len(rot_rels) * 0.75):]
    # Post-convergence jitter: mean per-frame step of (est - true).
    dev_tail = devs[int(len(devs) * 0.75):]
    jitter = np.linalg.norm(np.diff(dev_tail, axis=0), axis=2).mean(axis=0)
    report = {"widgets": [], "seconds": seconds, "seed": seed}
    for i in range(n_widgets):
        # Convergence: last time the error crosses down below the threshold.
        above = np.where(errs[:, i] >= converged_m)[0]
        conv_t = 0.0 if len(above) == 0 else (
            None if above[-1] == len(errs) - 1 else float(times[above[-1] + 1]))
        report["widgets"].append({
            "id": i,
            "gps": specs[i].has_gps,
            "surveyed": specs[i].surveyed,
            "final_err_m": round(float(errs[-1, i]), 3),
            "tail_rmse_m": round(float(np.sqrt(np.mean(tail[:, i] ** 2))), 3),
            "tail_rel_rmse_m":
                round(float(np.sqrt(np.mean(rel_tail[:, i] ** 2))), 3),
            "tail_rot_rel_rmse_m":
                round(float(np.sqrt(np.mean(rot_tail[:, i] ** 2))), 3),
            "tail_jitter_m": round(float(jitter[i]), 4),
            "lock": round(float(w.widgets[i].est.range_innov_ratio()), 2),
            "converge_s": conv_t,
            "accepted_ranges": w.widgets[i].accepted_ranges,
            "rejected_ranges": w.widgets[i].rejected_ranges,
        })
    return report, w


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--widgets", type=int, default=6)
    ap.add_argument("--gps", type=int, default=2, help="how many widgets have GPS")
    ap.add_argument("--surveyed", type=int, default=0,
                    help="add N static widgets that know their position")
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--seed", type=int, default=1234)
    ap.add_argument("--no-aoa", action="store_true", help="disable AoA updates")
    ap.add_argument("--static", action="store_true",
                    help="hold all widgets stationary (they still spin)")
    ap.add_argument("--motion-rate", type=float, default=1.0,
                    help="scale orbit/translation speed (1.0 = nominal)")
    ap.add_argument("--move-period", type=float, default=0.0,
                    help="stop-and-go cycle length, s (0 = always moving)")
    ap.add_argument("--move-duty", type=float, default=1.0,
                    help="fraction of each cycle spent moving (<1 = pauses)")
    ap.add_argument("--downhill", type=float, default=0.5,
                    help="peer-var/my-var acceptance ratio, 0=off")
    ap.add_argument("--converged-m", type=float, default=1.0)
    ap.add_argument("--slot-rate", type=float, default=None,
                    help="override UWB ranging slots/s")
    ap.add_argument("--range-sigma", type=float, default=None,
                    help="override UWB range noise, m")
    ap.add_argument("--broadcast-rate", type=float, default=None,
                    help="override pose advertisements/s")
    ap.add_argument("--intrinsics", default=None,
                    help="WidgetConfig textproto: use characterized sensor "
                         "quality instead of the built-in defaults")
    ap.add_argument("--no-multipath", action="store_true",
                    help="disable UWB multipath outliers")
    ap.add_argument("--aoa-damping", type=float, default=None,
                    help="override mean-only AoA gain divisor")
    ap.add_argument("--range-damping", type=float, default=None,
                    help="override annealed mean-only range damping "
                         "(0 disables annealing)")
    ap.add_argument("--mean-only-pos-only", action="store_true",
                    help="mean-only range updates correct position only")
    ap.add_argument("--zero-vel-sigma", type=float, default=None,
                    help="override the zero-velocity gauge prior sigma, m/s")
    ap.add_argument("--zero-vel-rate", type=float, default=None,
                    help="override the zero-velocity prior rate, Hz")
    ap.add_argument("--json", action="store_true", help="emit the report as JSON")
    ap.add_argument("--trace", action="store_true",
                    help="print per-second err/sigma per widget")
    args = ap.parse_args()
    report, _ = run(args.widgets, args.gps, args.seconds, args.seed,
                    args.converged_m, aoa=not args.no_aoa, trace=args.trace,
                    n_surveyed=args.surveyed, downhill_ratio=args.downhill,
                    static=args.static, intrinsics_path=args.intrinsics,
                    motion_rate=args.motion_rate, move_period=args.move_period,
                    move_duty=args.move_duty,
                    sensor_overrides={k: v for k, v in {
                        "uwb_slot_rate": args.slot_rate,
                        "uwb_range_sigma": args.range_sigma,
                        "broadcast_rate": args.broadcast_rate,
                        "mean_only_aoa_damping": args.aoa_damping,
                        "mean_only_range_damping": args.range_damping,
                        "uwb_multipath_prob":
                            0.0 if args.no_multipath else None,
                        "zero_vel_sigma": args.zero_vel_sigma,
                        "mean_only_pos_only":
                            True if args.mean_only_pos_only else None,
                        "zero_vel_rate": args.zero_vel_rate,
                    }.items() if v is not None})
    if args.json:
        print(json.dumps(report, indent=2))
        return
    print("%d widgets (%d GPS), %.0f s, seed %d" %
          (args.widgets, args.gps, args.seconds, args.seed))
    print("%3s %4s %12s %12s %12s %12s %10s %6s %11s %9s" %
          ("id", "gps", "final err m", "tail RMSE m", "rel RMSE m",
           "rot-rel m", "jitter m", "lock", "converge s", "rejects"))
    for wg in report["widgets"]:
        print("%3d %4s %12.3f %12.3f %12.3f %12.3f %10.4f %6.1f %11s %9d" %
              (wg["id"], "*" if wg["gps"] else ("S" if wg["surveyed"] else ""),
               wg["final_err_m"],
               wg["tail_rmse_m"], wg["tail_rel_rmse_m"],
               wg["tail_rot_rel_rmse_m"], wg["tail_jitter_m"], wg["lock"],
               "never" if wg["converge_s"] is None else "%.1f" % wg["converge_s"],
               wg["rejected_ranges"]))


if __name__ == "__main__":
    main()
