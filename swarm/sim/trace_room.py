"""Frame-by-frame trace of the room scenario's walker (diagnostics)."""
import sys, json, math
import numpy as np
from swarm.sim import run_sim

sets = dict(kv.split("=", 1) for kv in sys.argv[1:])
frames = []
run_sim.run(4, 0, 120, seed=int(sets.pop("seed", 1)), scenario="room", node_set=sets,
            frame_cb=lambda f: frames.append(f))
walk = [f for f in frames if f["t"] > 60]
def wk(f): return [w for w in f["widgets"] if w["id"] == 3][0]
rows = []
prev = None
for f in walk:
    w = wk(f)
    ve = np.linalg.norm(np.array(w["est_v"]) - np.array(w["true_v"]))
    speed = np.linalg.norm(w["true_v"])
    dev = np.array(w["aligned_p"]) - np.array(w["true_p"])
    step = np.linalg.norm(dev - prev) if prev is not None else 0
    prev = dev
    rows.append((f["t"], speed, ve, np.linalg.norm(dev), step, w["lock"], w["pos_var"], w["still"]))
for moving in (True, False):
    R = [r for r in rows[1:] if (r[1] > 0.05) == moving]
    if R:
        print("%s: n=%d  vel err %.3f m/s  aligned err %.3f m  step %.4f m/frame  var %.4f  still %d%%" % (
            "walking" if moving else "stopped", len(R), np.mean([r[2] for r in R]), np.mean([r[3] for r in R]),
            np.mean([r[4] for r in R]), np.median([r[6] for r in R]), 100 * np.mean([r[7] for r in R])))
for r in rows[::10][:12]:
    print("t %5.1f speed %.2f velerr %.2f err %.3f step %.3f lock %.2f var %.4f still %d" % r)
