"""A/B a set of run_sim configurations over many seeds, in parallel.

  python3 swarm/sim/ab.py '{"v28": [], "check off": ["--set", "anchor.check_ratio=0"]}' \\
      24 room 120

Arguments: a JSON object of {name: [extra run_sim arguments]}, the number of
seeds (1..N), the scenario and the seconds per run. Prints, per config: the
walker's static-frame error (mean, max, p95, rotational jitter), the still
widgets' error and how many ended anchored, regains, and the range-bias
estimate's error. PERSEED=1 also prints every seed. Runs
./bazel-bin/swarm/sim/run_sim (build it first) from the repo root.
"""

import json
import os
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

import numpy as np

BIN = "./bazel-bin/swarm/sim/run_sim"


def one(name, args, seed, scen, secs):
    out = subprocess.run([BIN, "--scenario", scen, "--seconds", secs, "--seed", str(seed), "--json"] + args,
                         capture_output=True, text=True)
    if out.returncode != 0:
        raise RuntimeError("%s seed %d: %s" % (name, seed, out.stderr[-500:]))
    return name, seed, json.loads(out.stdout)


def main():
    configs = json.loads(sys.argv[1])
    seeds = range(1, int(sys.argv[2]) + 1) if len(sys.argv) > 2 else range(1, 6)
    scen = sys.argv[3] if len(sys.argv) > 3 else "room"
    secs = sys.argv[4] if len(sys.argv) > 4 else "120"
    jobs = [(n, a, s) for n, a in configs.items() for s in seeds]
    with ThreadPoolExecutor(10) as ex:
        res = list(ex.map(lambda j: one(*j, scen, secs), jobs))
    for n in configs:
        rs = [r for nn, _, r in res if nn == n]
        walk = [w for r in rs for w in r["widgets"] if w["kind"] == "orbit"]
        st = [w for r in rs for w in r["widgets"] if w["kind"] == "static"]
        berr = [abs(w["range_bias_est"] - w["range_bias_true"]) for r in rs for w in r["widgets"]]
        line = "%-28s" % n
        if walk:
            line += " walker %5.1fcm max %5.1f p95 %5.1f jit %4.2fcm" % (
                np.mean([w["tail_frame_rmse_m"] for w in walk]) * 100,
                max(w["tail_frame_rmse_m"] for w in walk) * 100,
                np.mean([w["frame_p95_m"] for w in walk]) * 100,
                np.mean([w["tail_rot_jitter_m"] for w in walk]) * 100)
        line += " | static %5.1fcm max %5.1f anch %d/%d regains %d | bias err %4.1fcm" % (
            np.mean([w["tail_frame_rmse_m"] for w in st]) * 100,
            max(w["tail_frame_rmse_m"] for w in st) * 100,
            sum(w["anchored"] for w in st), len(st),
            sum(w["regains"] for r in rs for w in r["widgets"]), np.mean(berr) * 100)
        print(line, flush=True)
    if os.environ.get("PERSEED"):
        for n in configs:
            print(n)
            for nn, s, r in sorted(res, key=lambda x: x[1]):
                if nn != n:
                    continue
                print("  seed %2d " % s + " ".join(
                    "%s:%.2f/r%d/rej%d/b%+.2f:%+.2f" % (
                        w["kind"][0], w["tail_frame_rmse_m"], w["regains"], w["rejected_ranges"],
                        w["range_bias_true"], w["range_bias_est"]) for w in r["widgets"]))


if __name__ == "__main__":
    main()
