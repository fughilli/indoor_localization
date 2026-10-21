"""Synthetic node capture for swarm_view (--replay): runs the swarm sim and
writes what an apps/node firmware would print over USB, from widget 0's
point of view — self pose at 100 Hz, its swarm table at 10 Hz, ranges, stats.

  bazel run //swarm/tools:swarm_view_demo -- --out /tmp/swarm_demo.jsonl
"""

import argparse
import json
import math

from swarm.sim import world


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", required=True)
    ap.add_argument("--seconds", type=float, default=40.0)
    args = ap.parse_args()

    specs = world.default_swarm(n=1, n_gps=0, n_surveyed=3, arena=8.0)
    w = world.World(specs, seed=7)
    me = w.widgets[0]
    ids = {x.spec.wid: 0xC248 + 0x111 * x.spec.wid for x in w.widgets}
    out = open(args.out, "w")

    def emit(rec):
        out.write(json.dumps(rec, separators=(",", ":")) + "\n")

    emit({"t": 0.0, "type": "boot", "fw": "node", "id": ids[0], "imu": True, "mag": False,
          "radio": True})
    ranged = 0
    for step in range(int(args.seconds * w.cfg.imu_rate)):
        w.step()
        t = round(w.t, 4)
        s = me.est.state()
        if step % 1 == 0:
            emit({"t": t, "type": "self", "id": ids[0], "p": [round(x, 4) for x in s["p"]],
                  "v": [round(x, 3) for x in s["v"]], "q": [round(x, 4) for x in s["q"]],
                  "var": s["pos_var"], "lock": round(me.est.range_innov_ratio(), 2)})
        for a, b in w.last_links:
            if 0 in (a, b):
                peer = b if a == 0 else a
                d = float(sum((x - y) ** 2 for x, y in zip(me.truth.pos(w.t), w.widgets[peer].truth.pos(w.t))) ** 0.5)
                emit({"t": t, "type": "range", "peer": ids[peer], "range_m": round(d, 4),
                      "rssi": -60.0, "fp_rssi": -61.0, "init": int(a == 0), "result": 1})
                ranged += 1
        if step % 10 == 0:
            nodes = [{"id": ids[0], "self": True, "ready": True, "p": [round(x, 3) for x in s["p"]],
                      "var": s["pos_var"], "lock": round(me.est.range_innov_ratio(), 2), "age": 0}]
            for o in w.widgets[1:]:
                p, var, lock, age = o.advertised_at(w.t)
                nodes.append({"id": ids[o.spec.wid], "p": [round(float(x), 3) for x in p],
                              "v": [0, 0, 0], "var": var, "lock": lock, "age": round(age, 3),
                              "flags": 3 if o.spec.surveyed else 0, "seq": step})
            emit({"t": t, "type": "swarm", "id": ids[0], "nodes": nodes})
        if step % 100 == 0:
            emit({"t": t, "type": "stats", "id": ids[0], "slot": 3, "peers": 3,
                  "sync_master": ids[0], "sync_jump_us": 12, "polls": step // 3,
                  "ranges_init": ranged // 2, "ranges_resp": ranged - ranged // 2, "timeouts": 0,
                  "rx_errors": 0, "foreign": 0, "beacons": 0, "fusion_max_us": 900,
                  "dropped": 0, "heap": 180000})
    out.close()


if __name__ == "__main__":
    main()
