"""How well do the nodes' clocks agree? From the firmware's sync log.

With `{"cmd":"sync_log","on":1}` each node prints a `tx` record (its own DW3000
transmit timestamp) for the frames it sends and an `rx` record (receive timestamp,
raw carrier clock offset, power) for the frames it hears. This joins them across nodes
and, for nodes at surveyed positions, turns every sender -> receiver frame into

    y = (ts_rx - ts_tx) * 15.65 ps  -  d / c        [metres, times c]

which for perfectly synchronised clocks would be a constant (the antenna delays). What
is left after removing a clock offset and drift fitted over a window of W seconds is
the timing error a broadcast (TDoA) scheme would still carry at that update interval:
the `sigma_sync` of swarm/sim/multipath.SyncModel and landmarks.SolveConfig.

  python3 swarm/tools/sync_residual.py --log 1=a.jsonl --log 2=b.jsonl --log 3=c.jsonl \\
      --pos 1=0,0,0 --pos 2=4.2,0,0 --pos 3=0,3.1,0.5

Each log is the node's own serial capture (capture.py). Nodes need not have been
started together: the offset between two nodes' boot clocks is estimated from the data.
"""

import argparse
import collections
import json
import math
import sys

import numpy as np

C = 299_792_458.0
TICK_S = 1.0 / (499.2e6 * 128)          # DW3000 device time unit, 15.65 ps
WRAP = 1 << 40


def read_log(path):
    tx, rx = [], []
    with open(path) as f:
        for line in f:
            try:
                r = json.loads(line)
            except json.JSONDecodeError:
                continue
            if r.get("type") == "tx":
                tx.append(r)
            elif r.get("type") == "rx":
                rx.append(r)
    return tx, rx


def join(tx, rx, bin_s=0.05):
    """Pair a sender's tx records with a receiver's rx records of that sender.
    The 8-bit sequence wraps every few seconds, so a (type, seq) match alone is
    ambiguous; the right matches are the ones that agree on the boot-clock offset
    between the two nodes, found as the fullest bin of the candidate offsets."""
    by_key = collections.defaultdict(list)
    for r in tx:
        by_key[(r["ft"], r["seq"])].append(r)
    cands = []
    for r in rx:
        for t in by_key.get((r["ft"], r["seq"]), ()):
            cands.append((r["t"] - t["t"], t, r))
    if not cands:
        return []
    offs = np.array([c[0] for c in cands])
    hist = collections.Counter(np.round(offs / bin_s).astype(int))
    mode = max(hist, key=hist.get) * bin_s
    keep = [c for c in cands if abs(c[0] - mode) < 2 * bin_s]
    return [(t, r) for _, t, r in keep]


def tick_diff(later, earlier):
    return ((int(later) - int(earlier)) % WRAP)


def pair_series(joined, d_m):
    """(t, y) for one directed pair: the receive-minus-transmit time in metres, less the
    true flight, at the receiver's device time t (s)."""
    t, y = [], []
    for tx, rx in joined:
        t.append(rx["ts"] * TICK_S)
        y.append(tick_diff(rx["ts"], tx["ts"]) * TICK_S * C - d_m)
    t, y = np.array(t), np.array(y)
    # the 40-bit receive timestamp wraps every 17.2 s; make t monotonic
    step = np.diff(t)
    t = t[0] + np.concatenate([[0.0], np.cumsum(np.where(step < -1.0, step + WRAP * TICK_S, step))])
    return t, y


def residual(t, y, window_s):
    """RMS (m) of y after removing a straight line fitted in each window of window_s."""
    if len(t) < 4:
        return float("nan")
    res, start = [], t[0]
    while start < t[-1]:
        sel = (t >= start) & (t < start + window_s)
        if sel.sum() >= 4:
            tt = t[sel] - t[sel].mean()
            p = np.polyfit(tt, y[sel], 1)
            res.append(y[sel] - np.polyval(p, tt))
        start += window_s
    return float(np.sqrt(np.mean(np.concatenate(res) ** 2))) if res else float("nan")


def analyze(logs, pos, windows=(0.5, 1.0, 2.0, 5.0, 10.0), ft=None):
    """logs: {node id: (tx records, rx records)}; pos: {node id: (x, y, z)}.
    Returns {(sender, receiver): {"n": frames, window: residual m, "drift_ppm": ...}}."""
    out = {}
    for s, (tx_s, _) in logs.items():
        for r, (_, rx_r) in logs.items():
            if s == r or s not in pos or r not in pos:
                continue
            mine = [x for x in rx_r if x["from"] == s and (ft is None or x["ft"] == ft)]
            tx = [x for x in tx_s if ft is None or x["ft"] == ft]
            joined = join(tx, mine)
            if len(joined) < 20:
                continue
            d = float(np.linalg.norm(np.subtract(pos[s], pos[r])))
            t, y = pair_series(joined, d)
            row = {"n": len(joined), "d": d}
            for w in windows:
                row[w] = residual(t, y, w)
            # drift in ppm: the slope of y (metres per second) over the whole run
            row["drift_ppm"] = float(np.polyfit(t - t.mean(), y, 1)[0] / C * 1e6)
            clk = np.array([j[1]["clk"] for j in joined], float)
            row["clk_raw"] = float(np.median(clk))
            out[(s, r)] = row
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--log", action="append", required=True, metavar="ID=FILE")
    ap.add_argument("--pos", action="append", required=True, metavar="ID=x,y,z")
    ap.add_argument("--ft", type=int, default=1, help="frame type to use (1 = POLL)")
    args = ap.parse_args()
    logs = {int(a.split("=")[0]): read_log(a.split("=", 1)[1]) for a in args.log}
    pos = {int(a.split("=")[0]): [float(v) for v in a.split("=")[1].split(",")] for a in args.pos}
    windows = (0.5, 1.0, 2.0, 5.0, 10.0)
    res = analyze(logs, pos, windows, args.ft)
    if not res:
        sys.exit("no sender -> receiver pair with enough joined frames (need >= 20 and surveyed positions)")
    print("sync residual after a fitted offset + drift, metres (1 ns = 0.30 m)")
    print("%-9s %6s %6s " % ("pair", "n", "d (m)") + " ".join("W=%-4gs" % w for w in windows) + "  drift ppm   clk raw")
    for (s, r), row in sorted(res.items()):
        print("%3d->%-3d %6d %6.2f " % (s, r, row["n"], row["d"]) + " ".join("%7.3f" % row[w] for w in windows)
              + "  %9.2f  %8.0f" % (row["drift_ppm"], row["clk_raw"]))
    for w in windows:
        v = [row[w] for row in res.values() if math.isfinite(row[w])]
        print("W=%gs: median %.3f m, worst %.3f m" % (w, np.median(v), max(v)))


if __name__ == "__main__":
    main()
