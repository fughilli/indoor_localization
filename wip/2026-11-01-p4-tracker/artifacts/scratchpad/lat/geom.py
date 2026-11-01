"""Geometry metrics (vive_diag) of lh_replay outputs: whole vs streamed sweeps."""
import sys, json
sys.path.insert(0, "/Users/kevin/Projects/git_checkouts/indoor_localization/swarm/tools")
import numpy as np
import vive_diag as vd
for f in sys.argv[1:]:
    d = vd.parse(f)
    pose = d["pose"]; segs = vd.segments(d["imu"])
    moving = np.zeros(len(pose), bool)
    for seg in segs:
        if not seg[2]:
            moving |= vd.in_seg(pose[:, 0], seg)
    kind = vd.session_kind(f)
    P = pose[moving, 1:4]
    g = vd.fit_circle(P) if kind == "circle" else vd.fit_line(P) if kind == "line" else None
    st = [vd.still_stats(pose, s) for s in segs if s[2]]
    st = [s for s in st if s]
    j = vd.jumps(pose[moving]) if moving.sum() > 50 else None
    print(f.split("/")[-1], "poses", len(pose), "geometry", json.dumps(g))
    print("   still pos sd mm", [s["pos_sd_mm"] for s in st][:4], "moving jumps", j and (j["count"], j["median_step_err_mm"]))
