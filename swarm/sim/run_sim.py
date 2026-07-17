"""Headless swarm-sim runner: prints per-widget convergence + error metrics.

  bazel run //swarm/sim:run_sim -- --widgets 6 --gps 2 --seconds 60

Metrics: position error at end, RMSE over the last quarter of the run, and
time-to-converge (first time the error stays below --converged-m).
"""

import argparse
import json

import numpy as np

from swarm.sim import world


def run(n_widgets, n_gps, seconds, seed=1234, converged_m=1.0, aoa=True,
        frame_cb=None, trace=False, n_surveyed=0, downhill_ratio=0.5,
        sensor_overrides=None):
    specs = world.default_swarm(n=n_widgets, n_gps=n_gps, n_surveyed=n_surveyed)
    for s in specs:
        s.has_aoa = aoa
    sensors = world.SensorConfig(downhill_ratio=downhill_ratio,
                                 **(sensor_overrides or {}))
    w = world.World(specs, sensors=sensors, seed=seed)
    n_widgets = len(specs)
    history = []  # (t, [err_i...])

    def record(frame):
        history.append((frame["t"], [wg["err"] for wg in frame["widgets"]]))
        if trace and abs(frame["t"] % 1.0) < 0.05:
            print("t=%5.1f  " % frame["t"] + "  ".join(
                "e%.1f/s%.1f" % (wg["err"], wg["pos_var"] ** 0.5)
                for wg in frame["widgets"]))
        if frame_cb:
            frame_cb(frame)

    w.run(seconds, frame_cb=record, frame_rate=10.0)

    errs = np.array([e for _, e in history])
    times = np.array([t for t, _ in history])
    tail = errs[int(len(errs) * 0.75):]
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
    ap.add_argument("--downhill", type=float, default=0.5,
                    help="peer-var/my-var acceptance ratio, 0=off")
    ap.add_argument("--converged-m", type=float, default=1.0)
    ap.add_argument("--slot-rate", type=float, default=None,
                    help="override UWB ranging slots/s")
    ap.add_argument("--range-sigma", type=float, default=None,
                    help="override UWB range noise, m")
    ap.add_argument("--broadcast-rate", type=float, default=None,
                    help="override pose advertisements/s")
    ap.add_argument("--json", action="store_true", help="emit the report as JSON")
    ap.add_argument("--trace", action="store_true",
                    help="print per-second err/sigma per widget")
    args = ap.parse_args()
    report, _ = run(args.widgets, args.gps, args.seconds, args.seed,
                    args.converged_m, aoa=not args.no_aoa, trace=args.trace,
                    n_surveyed=args.surveyed, downhill_ratio=args.downhill,
                    sensor_overrides={k: v for k, v in {
                        "uwb_slot_rate": args.slot_rate,
                        "uwb_range_sigma": args.range_sigma,
                        "broadcast_rate": args.broadcast_rate,
                    }.items() if v is not None})
    if args.json:
        print(json.dumps(report, indent=2))
        return
    print("%d widgets (%d GPS), %.0f s, seed %d" %
          (args.widgets, args.gps, args.seconds, args.seed))
    print("%3s %4s %12s %12s %11s %9s" %
          ("id", "gps", "final err m", "tail RMSE m", "converge s", "rejects"))
    for wg in report["widgets"]:
        print("%3d %4s %12.3f %12.3f %11s %9d" %
              (wg["id"], "*" if wg["gps"] else ("S" if wg["surveyed"] else ""),
               wg["final_err_m"],
               wg["tail_rmse_m"],
               "never" if wg["converge_s"] is None else "%.1f" % wg["converge_s"],
               wg["rejected_ranges"]))


if __name__ == "__main__":
    main()
