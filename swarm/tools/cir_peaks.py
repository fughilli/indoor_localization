"""Look at the CIR windows a node logs with {"cmd":"cir","on":1} (with sync_log on).

Each `cir` record holds n taps of the Ipatov channel impulse response, 6 bytes a tap
(24-bit little-endian signed I, then Q), starting at tap `first`, plus `fp`, the chip's
raw first-path index (tap = fp >> 6, assuming 6 fractional bits). One tap is about 1 ns,
0.3 m of path.

  python3 swarm/tools/cir_peaks.py capture.jsonl

Prints, per sender, how many captures there are, and for each: the magnitude peak, where
the magnitude first crosses a noise-based threshold, and the chip's first-path tap. If the
two first-path estimates disagree by a constant, the fixed-point assumption or the tap
offset unit in libs/dw3000 readCir is wrong; if they scatter, trust neither yet.
"""

import argparse
import collections
import json

import numpy as np

FP_FRAC_BITS = 6
SNR_K = 6.0   # threshold = SNR_K x noise sigma, from the window's first taps


def s24(b):
    v = b[0] | (b[1] << 8) | (b[2] << 16)
    return v - (1 << 24) if v & (1 << 23) else v


def decode(rec):
    raw = bytes.fromhex(rec["hex"])
    n = rec["n"]
    iq = np.array([[s24(raw[6 * k:6 * k + 3]), s24(raw[6 * k + 3:6 * k + 6])] for k in range(n)], float)
    return iq[:, 0] + 1j * iq[:, 1]


def first_crossing(mag, pre):
    """Index of the first tap above SNR_K x the noise estimated before the first path."""
    noise = mag[:max(4, pre // 2)]
    thr = SNR_K * max(np.std(noise), 1.0) + np.mean(noise)
    above = np.nonzero(mag > thr)[0]
    return int(above[0]) if len(above) else None


def analyze(records):
    rows = []
    for r in records:
        if r.get("type") != "cir":
            continue
        mag = np.abs(decode(r))
        fp_tap = r["fp"] / (1 << FP_FRAC_BITS)
        cross = first_crossing(mag, int(fp_tap) - r["first"])
        rows.append({"from": r["from"], "seq": r["seq"], "first": r["first"], "fp_tap": fp_tap,
                     "peak_tap": r["first"] + int(np.argmax(mag)),
                     "cross_tap": None if cross is None else r["first"] + cross,
                     "peak": float(np.max(mag))})
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("log")
    args = ap.parse_args()
    with open(args.log) as f:
        recs = []
        for line in f:
            try:
                recs.append(json.loads(line))
            except json.JSONDecodeError:
                pass
    rows = analyze(recs)
    by = collections.defaultdict(list)
    for r in rows:
        by[r["from"]].append(r)
    for s, rs in sorted(by.items()):
        print("from %d: %d captures" % (s, len(rs)))
        d = [r["cross_tap"] - r["fp_tap"] for r in rs if r["cross_tap"] is not None]
        if d:
            print("  threshold crossing minus chip first path (taps): median %.1f, spread %.1f"
                  % (np.median(d), np.std(d)))
        for r in rs[:5]:
            print("  seq %3d  window %4d..  chip fp %7.2f  crossing %s  peak at %d (%.0f)"
                  % (r["seq"], r["first"], r["fp_tap"], r["cross_tap"], r["peak_tap"], r["peak"]))


if __name__ == "__main__":
    main()
