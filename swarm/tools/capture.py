"""Record a widget's serial telemetry into a capture + manifest pair.

Reads the newline-JSON stream a mule/widget prints on USB serial, appends
every schema record (see records.py) to a capture JSONL, prints a live
per-second summary, and — given ground truth — appends a manifest segment
spanning the records it captured. Run once per physical setup, e.g. a range
sweep:

  python3 swarm/tools/capture.py --port /dev/cu.usbserial-110 \\
      --capture range.jsonl --manifest range.json --true-range 1.0 --seconds 30
  # move the responder to 2 m, re-measure, repeat with --true-range 2.0 ...
  bazel run //swarm/tools:characterize -- range \\
      --capture $PWD/range.jsonl --manifest $PWD/range.json

Runs on the machine the board is plugged into (needs pyserial), not under
Bazel: Docker on macOS cannot pass USB serial into a container. The board is
not reset on open, so the widget clock — and with it segment times — stays
continuous across runs; if it does reboot mid-session the clock restarts and
the tool refuses to write an overlapping segment.
"""

import argparse
import json
import os
import statistics
import sys
import time


def parse_line(raw):
    """Schema record dict from one serial line, or None for anything else
    (driver log text, boot ROM noise, truncated lines)."""
    line = raw.decode("utf-8", errors="replace").strip() if isinstance(raw, bytes) else raw.strip()
    if not line.startswith("{"):
        return None
    try:
        rec = json.loads(line)
    except json.JSONDecodeError:
        return None
    if not isinstance(rec, dict) or "t" not in rec or "type" not in rec:
        return None
    return rec


def append_segment(manifest_path, segment):
    """Append a ground-truth segment to the manifest (created if missing).
    Refuses a segment overlapping an existing one: that means the widget
    clock restarted, and the analysis would mix the two setups."""
    manifest = {"segments": []}
    if os.path.exists(manifest_path):
        with open(manifest_path) as f:
            manifest = json.load(f)
    for s in manifest.get("segments", []):
        if segment["t0"] < s["t1"] and s["t0"] < segment["t1"]:
            raise ValueError(
                "segment [%.3f, %.3f) overlaps existing [%.3f, %.3f) — did the "
                "board reboot? Start a fresh capture/manifest pair."
                % (segment["t0"], segment["t1"], s["t0"], s["t1"]))
    manifest.setdefault("segments", []).append(segment)
    with open(manifest_path, "w") as f:
        json.dump(manifest, f, indent=2)
        f.write("\n")


class Summary:
    """Per-interval range statistics plus the initiator's failure counters."""

    def __init__(self):
        self.ranges = []
        self.fails = {}

    def add(self, rec):
        if rec["type"] == "range":
            self.ranges.append(rec["range_m"])
        elif rec["type"] == "stats":
            for k in ("timeout", "rx_error", "protocol"):
                self.fails[k] = self.fails.get(k, 0) + rec.get(k, 0)

    def line(self):
        n = len(self.ranges)
        if n:
            med = statistics.median(self.ranges)
            sd = statistics.pstdev(self.ranges) if n > 1 else 0.0
            text = "n=%3d  median=%.3f m  sd=%.3f m" % (n, med, sd)
        else:
            text = "n=  0"
        fails = ", ".join("%s=%d" % kv for kv in sorted(self.fails.items()) if kv[1])
        return text + ("  fails: " + fails if fails else "")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--capture", required=True, help="capture JSONL (appended)")
    ap.add_argument("--manifest", help="manifest JSON to append a segment to")
    ap.add_argument("--true-range", type=float, metavar="M",
                    help="ground-truth range for this segment (needs --manifest)")
    ap.add_argument("--seconds", type=float, default=None,
                    help="stop after this long (default: until Ctrl-C)")
    ap.add_argument("--settle", type=float, default=1.0,
                    help="seconds to discard first, after moving a board")
    ap.add_argument("--verbose", action="store_true",
                    help="echo non-record lines (driver logs) to stderr")
    args = ap.parse_args()
    if args.true_range is not None and not args.manifest:
        ap.error("--true-range needs --manifest")

    import serial  # pyserial; imported late so parse helpers test without it

    # Opening the port must not reset the board (that would restart the widget
    # clock between segments). The ESP32 auto-reset circuit only pulls EN low
    # while RTS is asserted and DTR is not; the OS asserts both on open, which
    # is harmless, but deasserting them (pyserial drops DTR before RTS) passes
    # through exactly that state. So leave both lines alone.
    port = serial.Serial(args.port, args.baud, timeout=0.2)

    t_first = t_last = None
    count = 0
    summary = Summary()
    start = time.time()
    next_report = start + 1.0
    try:
        with open(args.capture, "a") as out:
            while args.seconds is None or time.time() - start < args.seconds + args.settle:
                raw = port.readline()
                rec = parse_line(raw) if raw else None
                if raw and rec is None and args.verbose:
                    sys.stderr.write(raw.decode("utf-8", errors="replace"))
                if rec is not None and time.time() - start >= args.settle:
                    if t_last is not None and rec["t"] < t_last:
                        print("warning: widget clock went backwards (%.3f -> %.3f); "
                              "board rebooted?" % (t_last, rec["t"]), file=sys.stderr)
                    out.write(json.dumps(rec, separators=(",", ":")) + "\n")
                    count += 1
                    if t_first is None:
                        t_first = rec["t"]
                    t_last = rec["t"]
                    summary.add(rec)
                if time.time() >= next_report:
                    print("[%5.1fs] %s" % (time.time() - start, summary.line()), flush=True)
                    summary = Summary()
                    next_report += 1.0
    except KeyboardInterrupt:
        pass
    finally:
        port.close()

    print("captured %d records to %s" % (count, args.capture))
    if args.true_range is not None:
        if t_first is None:
            sys.exit("no records captured; manifest unchanged")
        seg = {"t0": t_first, "t1": round(t_last + 1e-3, 3), "true_range_m": args.true_range}
        append_segment(args.manifest, seg)
        print("appended segment %s to %s" % (json.dumps(seg), args.manifest))


if __name__ == "__main__":
    main()
