"""Sensor characterization CLI: turn a raw capture + ground-truth manifest
into precision/bias/calibration reports.

  bazel run //swarm/tools:characterize -- aoa   --capture c.jsonl --manifest m.json
  bazel run //swarm/tools:characterize -- range --capture c.jsonl --manifest m.json
  bazel run //swarm/tools:characterize -- imu   --capture c.jsonl --fs 100
  bazel run //swarm/tools:characterize -- mag   --capture c.jsonl

Every subcommand accepts --demo to generate a synthetic capture (known
truth) into --capture/--manifest first and then analyze it — a dry run of
the full pipeline, and the reference for what a hardware capture must look
like (see swarm/tools/records.py for the record schema).

--intrinsics FILE additionally merges the analysis result into the
intrinsics section of a widget.v1 WidgetConfig textproto (created if
missing) — the standard schema a manufacturer ships with a widget so the
firmware can use measured sensor quality instead of conservative defaults
(swarm/proto/widget_config.proto, swarm/tools/widget_config.py).

AoA specifically answers: in which body-frame plane does the radio measure
its angle (fitted plane normal), with what sign convention, mounting offset,
and precision — and how fast precision degrades out of plane.
"""

import argparse
import json
import os

import numpy as np

from swarm.tools import analysis, records, synth, widget_config


def cmd_range(args):
    caps = records.load_capture(args.capture)
    manifest = records.load_manifest(args.manifest)
    segments = []
    for seg in records.segments_with(manifest, "true_range_m"):
        rs = records.select(caps, "range", seg["t0"], seg["t1"])
        if rs:
            segments.append((seg["true_range_m"],
                             [r["range_m"] for r in rs]))
    if not segments:
        raise SystemExit("no range segments with true_range_m ground truth")
    return analysis.range_stats(segments)


def cmd_aoa(args):
    caps = records.load_capture(args.capture)
    manifest = records.load_manifest(args.manifest)
    dirs, angles = [], []
    for seg in records.segments_with(manifest, "true_dir_body"):
        for r in records.select(caps, "aoa", seg["t0"], seg["t1"]):
            dirs.append(seg["true_dir_body"])
            angles.append(r["angle_rad"])
    if len(angles) < 30:
        raise SystemExit("need >=30 AoA samples across segments with "
                         "true_dir_body ground truth")
    return analysis.fit_aoa_plane(np.array(dirs), np.array(angles))


def cmd_imu(args):
    caps = records.load_capture(args.capture)
    imu = records.select(caps, "imu")
    if len(imu) < 1000:
        raise SystemExit("need >=1000 static IMU samples")
    accel = [r["accel"] for r in imu]
    gyro = [r["gyro"] for r in imu]
    fs = args.fs
    if fs is None:
        ts = np.array([r["t"] for r in imu])
        fs = float(1.0 / np.median(np.diff(ts)))
    return analysis.imu_static_report(accel, gyro, fs)


def cmd_mag(args):
    caps = records.load_capture(args.capture)
    mag = records.select(caps, "mag")
    if len(mag) < 200:
        raise SystemExit("need >=200 mag samples with good orientation "
                         "coverage (wave the widget around)")
    return analysis.fit_mag_ellipsoid([r["mag"] for r in mag])


DEMOS = {
    "range": synth.make_range_capture,
    "aoa": synth.make_aoa_capture,
    "imu": synth.make_imu_capture,
    "mag": synth.make_mag_capture,
}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("kind", choices=["range", "aoa", "imu", "mag"])
    ap.add_argument("--capture", default="/tmp/capture.jsonl")
    ap.add_argument("--manifest", default="/tmp/manifest.json")
    ap.add_argument("--fs", type=float, default=None,
                    help="IMU sample rate override, Hz")
    ap.add_argument("--demo", action="store_true",
                    help="generate a synthetic capture first, then analyze")
    ap.add_argument("--intrinsics", metavar="FILE",
                    help="merge the result into this WidgetConfig textproto "
                         "(created if missing)")
    args = ap.parse_args()
    if args.demo:
        DEMOS[args.kind](args.capture, args.manifest)
        print("# synthetic %s capture -> %s" % (args.kind, args.capture))
    report = {"range": cmd_range, "aoa": cmd_aoa,
              "imu": cmd_imu, "mag": cmd_mag}[args.kind](args)
    print(json.dumps(report, indent=2))
    if args.intrinsics:
        import widget_config_pb2 as pb
        cfg = (widget_config.load(args.intrinsics)
               if os.path.exists(args.intrinsics) else pb.WidgetConfig())
        widget_config.save(
            widget_config.merge_report(cfg, args.kind, report),
            args.intrinsics)
        print("# merged %s intrinsics -> %s" % (args.kind, args.intrinsics))


if __name__ == "__main__":
    main()
