"""Room-scenario trace (diagnostics): every node's aligned error, lock,
variance, still/anchored flags over time.

  trace_room [seed=N] [every=S] [NAME=VALUE ...]
"""
import sys
import numpy as np
from swarm.sim import run_sim

sets = dict(kv.split("=", 1) for kv in sys.argv[1:])
seed = int(sets.pop("seed", 1))
every = float(sets.pop("every", 2.0))
secs = float(sets.pop("seconds", 60))
frames = []
run_sim.run(4, 0, secs, seed=seed, scenario="room", node_set=sets, frame_cb=frames.append)
last = -1e9
for f in frames:
    if f["t"] - last < every - 1e-6:
        continue
    last = f["t"]
    cells = []
    for w in f["widgets"]:
        tag = ("A" if w["anchored"] else "") + ("s" if w["still"] else "")
        spd = np.linalg.norm(w["true_v"])
        cells.append("%d:%5.2fm L%4.1f sd%5.2f %-2s%s" % (w["id"], w["aligned_err"], w["lock"], w["pos_var"] ** 0.5,
                                                          tag, " v%.1f" % spd if w["id"] == 3 else ""))
    print("t%5.1f  " % f["t"] + " | ".join(cells))
