"""Score a swarm capture against tracker ground truth.

Takes a swarm_view.py capture (node records stamped with host time "ht") and
a vive_truth.py truth file (tracker poses on the same host clock), and
reports how far the swarm's estimates are from where the trackers say the
nodes were, plus how the radio's ranges compare with the true distances:

  python3 swarm/tools/truth_eval.py --capture captures/swarm_v28.jsonl \\
      --truth captures/truth.jsonl --lever 63124=0,0,-0.04

The swarm's frame is its own (anchorless: arbitrary origin, heading and
handedness), so positions are compared three ways:
  pair      |est_i - est_j| - |true_i - true_j| for every tracked pair: no
            frame fit at all, so nothing can hide in it.
  global    one 2-D rigid fit (mirror allowed: ranges can't tell a layout
            from its mirror image) over the whole session: includes frame
            drift.
  snapshot  a rigid fit per swarm record: shape error alone (needs 3+
            tracked nodes to mean much).
Positions are compared in the horizontal plane: truth is projected along
its up axis (--up; auto = the axis the trackers moved least along).

The tracker sits on the node, not at its UWB antenna: --lever gives the
antenna's offset in the tracker's frame (m) per node. Node ids may be hex
(F694) or decimal. --dt shifts truth in time; --fit-dt searches it (from
the ranges, so the swarm filter's own lag still counts as error).
Ranges (only the USB node's own, the ones in the capture) are compared with
the true antenna-to-antenna distance, binned by the first-path gap the
radio reported: the data the NLOS model (nlos.* settings) is fitted to.
"""

import argparse
import json
import math
import sys

import numpy as np

GAP_BINS = [0.0, 5.0, 10.0, 15.0, 20.0, 25.0, float("inf")]


def node_id(s):
    s = str(s).strip()
    if s.lower().startswith("0x"):
        return int(s, 16)
    try:
        return int(s)
    except ValueError:
        return int(s, 16)


def parse_levers(items):
    out = {}
    for item in items:
        node, _, vec = item.partition("=")
        v = [float(x) for x in vec.split(",")]
        if len(v) != 3:
            raise ValueError("expected NODE=x,y,z, got %r" % item)
        out[node_id(node)] = np.array(v)
    return out


def load_capture(lines):
    """Swarm snapshots [(ht, {id: p})] and the USB node's ranges
    [(ht, peer, range_m, gap_db)] from capture JSONL lines."""
    snaps, ranges, self_id, stamped = [], [], None, 0
    for line in lines:
        try:
            d = json.loads(line)
        except ValueError:
            continue
        if not isinstance(d, dict) or "ht" not in d:
            continue
        stamped += 1
        if d.get("type") == "swarm":
            self_id = d.get("id", self_id)
            snaps.append((d["ht"], {n["id"]: np.array(n["p"], dtype=float) for n in d.get("nodes", [])
                                    if "p" in n and n.get("ready", True)}))
        elif d.get("type") == "range" and "peer" in d and "range_m" in d:
            gap = (d["rssi"] - d["fp_rssi"]) if "rssi" in d and "fp_rssi" in d else float("nan")
            ranges.append((d["ht"], d["peer"], d["range_m"], gap))
    if not stamped:
        raise ValueError("no host-stamped records: record with swarm_view.py --capture "
                         "(this version stamps 'ht')")
    return snaps, ranges, self_id


def load_truth(lines):
    """{node: (ht[N], p[N,3], q[N,4] wxyz)} from truth JSONL lines (records
    without a node mapping are skipped)."""
    acc = {}
    for line in lines:
        try:
            d = json.loads(line)
        except ValueError:
            continue
        if not isinstance(d, dict) or d.get("type") != "truth" or d.get("node") is None:
            continue
        acc.setdefault(int(d["node"]), []).append((d["ht"], d["p"], d["q"]))
    out = {}
    for node, rows in acc.items():
        rows.sort(key=lambda r: r[0])
        out[node] = (np.array([r[0] for r in rows]), np.array([r[1] for r in rows], dtype=float),
                     np.array([r[2] for r in rows], dtype=float))
    return out


def quat_rotate(q, v):
    """Rotate v by unit quaternion q = (w, x, y, z)."""
    w, u = q[0], q[1:]
    return v + 2.0 * np.cross(u, np.cross(u, v) + w * v)


class Truth:
    """Antenna positions from tracker poses, interpolated in host time."""

    def __init__(self, tracks, levers=None, dt=0.0, max_gap=0.1):
        self.tracks = tracks
        self.levers = levers or {}
        self.dt = dt
        self.max_gap = max_gap

    def nodes(self):
        return set(self.tracks)

    def at(self, node, ht):
        tr = self.tracks.get(node)
        if tr is None:
            return None
        ts, ps, qs = tr
        t = ht - self.dt
        i = int(np.searchsorted(ts, t))
        if i == 0 or i >= len(ts) or ts[i] - ts[i - 1] > self.max_gap:
            return None
        a = (t - ts[i - 1]) / (ts[i] - ts[i - 1])
        p = ps[i - 1] + a * (ps[i] - ps[i - 1])
        q0, q1 = qs[i - 1], qs[i]
        if np.dot(q0, q1) < 0:
            q1 = -q1
        q = q0 + a * (q1 - q0)
        q /= np.linalg.norm(q)
        lever = self.levers.get(node)
        return p + quat_rotate(q, lever) if lever is not None else p


def up_axis(truth, choice="auto"):
    if choice != "auto":
        return "xyz".index(choice)
    allp = np.concatenate([tr[1] for tr in truth.tracks.values()])
    return int(np.argmin(allp.var(axis=0)))


def horizontal(p, up):
    return np.delete(p, up)


def fit2d(est, true, mirror=None):
    """Rigid 2-D fit est -> true: (R, t, mirrored) minimizing the summed
    squared residual, the mirror image of est tried too unless `mirror` is
    given."""
    best = None
    for m in ([False, True] if mirror is None else [mirror]):
        e = est * np.array([1.0, -1.0]) if m else est
        ce, ct = e.mean(axis=0), true.mean(axis=0)
        u, _, vt = np.linalg.svd((e - ce).T @ (true - ct))
        d = np.sign(np.linalg.det(u @ vt)) or 1.0
        r = (u @ np.diag([1.0, d]) @ vt).T
        res = np.linalg.norm((r @ (e - ce).T).T + ct - true, axis=1)
        cost = float((res ** 2).sum())
        if best is None or cost < best[3]:
            best = (r, ct - r @ ce, m, cost)
    return best[:3]


def apply2d(fit, est):
    r, t, m = fit
    e = est * np.array([1.0, -1.0]) if m else est
    return (r @ e.T).T + t


def stats(errs):
    a = np.abs(np.asarray(errs, dtype=float))
    if not len(a):
        return None
    return {"n": int(len(a)), "rms": float(np.sqrt((a ** 2).mean())), "p95": float(np.percentile(a, 95)),
            "max": float(a.max())}


def paired(snaps, truth, up):
    """[(ht, ids, est_xy[k,2], true_xy[k,2])] for snapshots with 2+ nodes
    that have both an estimate and truth."""
    out = []
    for ht, nodes in snaps:
        ids, e, t = [], [], []
        for nid, p in nodes.items():
            tp = truth.at(nid, ht)
            if tp is not None:
                ids.append(nid)
                e.append(p[:2])
                t.append(horizontal(tp, up))
        if len(ids) >= 2:
            out.append((ht, ids, np.array(e), np.array(t)))
    return out


def pair_errors(pairs):
    errs = {}
    for _, ids, e, t in pairs:
        for i in range(len(ids)):
            for j in range(i + 1, len(ids)):
                key = tuple(sorted((ids[i], ids[j])))
                errs.setdefault(key, []).append(
                    float(np.linalg.norm(e[i] - e[j]) - np.linalg.norm(t[i] - t[j])))
    return errs


def evaluate(snaps, ranges, self_id, truth, up="auto"):
    ax = up_axis(truth, up)
    pairs = paired(snaps, truth, ax)
    report = {"up_axis": "xyz"[ax], "snapshots": len(pairs), "tracked": sorted(truth.nodes())}
    if not pairs:
        report["error"] = "no snapshot has 2+ nodes with truth (check --map ids, clocks, --dt)"
        return report
    pe = pair_errors(pairs)
    report["pair"] = {"%x-%x" % k: dict(stats(v), mean=float(np.mean(v))) for k, v in sorted(pe.items())}
    # Global: one fit over every (node, snapshot) sample.
    e_all = np.concatenate([p[2] for p in pairs])
    t_all = np.concatenate([p[3] for p in pairs])
    g = fit2d(e_all, t_all)
    report["global_mirrored"] = bool(g[2])
    per_g, per_s = {}, {}
    for _, ids, e, t in pairs:
        rg = np.linalg.norm(apply2d(g, e) - t, axis=1)
        snap = len(ids) >= 3
        rs = np.linalg.norm(apply2d(fit2d(e, t, mirror=g[2]), e) - t, axis=1) if snap else None
        for k, nid in enumerate(ids):
            per_g.setdefault(nid, []).append(rg[k])
            if snap:
                per_s.setdefault(nid, []).append(rs[k])
    report["global"] = {"%x" % k: stats(v) for k, v in sorted(per_g.items())}
    report["snapshot"] = {"%x" % k: stats(v) for k, v in sorted(per_s.items())}
    # The USB node's ranges against the true antenna distance.
    rows = []
    for ht, peer, r, gap in ranges:
        a, b = truth.at(self_id, ht), truth.at(peer, ht)
        if a is not None and b is not None:
            rows.append((peer, gap, r - float(np.linalg.norm(a - b))))
    report["ranges"] = range_table(rows)
    return report


def range_table(rows):
    out = {"by_gap": [], "by_peer": {}}
    for lo, hi in zip(GAP_BINS[:-1], GAP_BINS[1:]):
        errs = [e for _, g, e in rows if lo <= g < hi]
        if errs:
            out["by_gap"].append({"gap_db": [lo, hi], "n": len(errs), "median": float(np.median(errs)),
                                  "sd": float(np.std(errs)), "p90": float(np.percentile(errs, 90))})
    for peer in sorted({p for p, _, _ in rows}):
        errs = [e for p, _, e in rows if p == peer]
        gaps = [g for p, g, _ in rows if p == peer and not math.isnan(g)]
        out["by_peer"]["%x" % peer] = {"n": len(errs), "median": float(np.median(errs)),
                                       "sd": float(np.std(errs)),
                                       "gap_median": float(np.median(gaps)) if gaps else None}
    return out


def fit_dt(snaps, ranges, self_id, truth, up="auto", span=0.5, step=0.01):
    """The truth time shift (s) that best lines truth up with the capture
    clock. From the ranges when they reach tracked peers (raw measurements:
    the time-varying part of range - true distance, per-link offsets
    removed, is smallest at the right shift), so the swarm's own filter lag
    stays in the position scores; else from the pair distances (which then
    absorb that lag)."""
    ax = up_axis(truth, up)
    base = truth.dt

    def range_cost():
        per = {}
        for ht, peer, r, _ in ranges:
            a, b = truth.at(self_id, ht), truth.at(peer, ht)
            if a is not None and b is not None:
                per.setdefault(peer, []).append(r - float(np.linalg.norm(a - b)))
        res = [e - np.median(v) for v in per.values() if len(v) > 10 for e in v]
        return float(np.sqrt(np.mean(np.square(res)))) if res else None

    def pair_cost():
        errs = [x for v in pair_errors(paired(snaps, truth, ax)).values() for x in v]
        return float(np.sqrt(np.mean(np.square(errs)))) if errs else None

    truth.dt = base
    cost = range_cost if range_cost() is not None else pair_cost
    best = (None, float("inf"))
    for dt in np.arange(-span, span + step / 2, step):
        truth.dt = base + float(dt)
        c = cost()
        if c is not None and c < best[1]:
            best = (truth.dt, c)
    truth.dt = base
    return best[0]


def format_report(rep):
    out = ["up axis %s, %d snapshots, tracked %s" % (
        rep["up_axis"], rep["snapshots"], " ".join("%x" % n for n in rep["tracked"]))]
    if "error" in rep:
        return "\n".join(out + [rep["error"]])

    def cm(s):
        return "%5.1f cm rms  p95 %5.1f  max %5.1f  (n %d)" % (100 * s["rms"], 100 * s["p95"],
                                                                100 * s["max"], s["n"])
    out.append("pair distance error (est - true):")
    for k, s in rep["pair"].items():
        out.append("  %-12s mean %+6.1f cm  %s" % (k, 100 * s["mean"], cm(s)))
    out.append("global rigid fit%s:" % (" (mirrored)" if rep["global_mirrored"] else ""))
    for k, s in rep["global"].items():
        out.append("  %-6s %s" % (k, cm(s)))
    if rep["snapshot"]:
        out.append("per-snapshot fit:")
        for k, s in rep["snapshot"].items():
            out.append("  %-6s %s" % (k, cm(s)))
    rg = rep["ranges"]
    if rg["by_gap"]:
        out.append("range error (measured - true) by first-path gap:")
        for b in rg["by_gap"]:
            out.append("  %4.0f-%-4.0f dB  n %5d  median %+6.1f cm  sd %5.1f  p90 %+6.1f" % (
                b["gap_db"][0], b["gap_db"][1], b["n"], 100 * b["median"], 100 * b["sd"], 100 * b["p90"]))
        for k, s in rg["by_peer"].items():
            gap = "" if s["gap_median"] is None else "  gap %.1f dB" % s["gap_median"]
            out.append("  to %-6s n %5d  median %+6.1f cm  sd %5.1f%s" % (
                k, s["n"], 100 * s["median"], 100 * s["sd"], gap))
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--capture", required=True, help="swarm_view.py --capture JSONL")
    ap.add_argument("--truth", required=True, help="vive_truth.py JSONL")
    ap.add_argument("--lever", action="append", default=[], metavar="NODE=x,y,z",
                    help="UWB antenna offset in the tracker's frame, m")
    ap.add_argument("--up", choices=["auto", "x", "y", "z"], default="auto")
    ap.add_argument("--dt", type=float, default=0.0, help="truth time shift, s")
    ap.add_argument("--fit-dt", action="store_true", help="search the truth time shift (+-0.5 s)")
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()
    with open(args.capture) as f:
        snaps, ranges, self_id = load_capture(f)
    with open(args.truth) as f:
        truth = Truth(load_truth(f), parse_levers(args.lever), args.dt)
    if args.fit_dt:
        dt = fit_dt(snaps, ranges, self_id, truth, args.up)
        if dt is not None:
            truth.dt = dt
            print("fitted dt %+.3f s" % dt, file=sys.stderr)
    rep = evaluate(snaps, ranges, self_id, truth, args.up)
    rep["dt"] = truth.dt
    print(json.dumps(rep, indent=1) if args.json else format_report(rep))


if __name__ == "__main__":
    main()
