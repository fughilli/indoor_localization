"""Offline base-station calibration from recorded sessions (libsurvive LH1 model).

  python3 swarm/tools/vive_calib.py captures/vive/A.rec.gz ... --config GOOD.config.json \\
      --out new.config.json [--exclude SESSION:SEG,...] [--fix-lh 1]

The stations must not have moved between the recordings. From each recording,
every still stretch (found from the IMU) becomes one scene: per sensor, station
and sweep axis, the median angle over the stretch, plus the tracker's mean
accelerometer reading. Solved in stages, so a badly calibrated station can't drag the scenes off:
(1) scene poses from the reference station alone, (2) each other station
against those scenes (multi-start), (3) one joint robust fit of:

  - every scene's tracker pose (6 dof each),
  - the poses of the stations not held fixed (--fix-lh: the reference, whose
    position and heading define the world, as in libsurvive),
  - a small world tilt (2 dof) shared by all stations,
  - with --fit-fcal, corrections to each station's OOTX phase and tilt (per axis),

against the sweep angles (libsurvive's reprojection: OOTX phase, tilt, curve and
gibbous corrections, same as survive_reproject.c) and gravity: the tracker's
accelerometer in each scene and each station's own accel (OOTX) must point
along world +z. The result is written as a libsurvive config.json (station poses
replaced, everything else kept), ready for --configfile / live use.

Angles come from replaying the recordings (vive_diag.replay, cached); initial
scene poses from the reference station's single-station replay.
"""

import argparse
import collections
import gzip
import json
import math
import os
import re
import sys

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as Rot

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import vive_diag  # noqa: E402

SIG_ANGLE = 1.0e-4  # rad, per median angle
SIG_TRACKER_UP = 0.01  # rad, tracker accelerometer direction while still
SIG_LH_UP = 0.03  # rad, station accel (8-bit OOTX vector)


# ---------------------------------------------------------------- config i/o

def load_libsurvive_config(path):
    """libsurvive writes almost-JSON (missing commas between top-level items)."""
    text = open(path).read()
    fixed = re.sub(r'"\s*\n\s*"', '",\n"', text)
    fixed = re.sub(r'}\s*\n\s*"', '},\n"', fixed)
    return json.loads("{" + fixed + "}"), text


def save_libsurvive_config(text, poses, path):
    """Replace "pose" in each lighthouseN block of the original text."""
    out = text
    for idx, pose in poses.items():
        block = re.search(r'"lighthouse%d":\{.*?\n\}' % idx, out, re.S)
        new = re.sub(r'"pose":\[[^\]]*\]', '"pose":[%s]' % ",".join('"%.12f"' % v for v in pose), block.group(0))
        out = out[:block.start()] + new + out[block.end():]
    open(path, "w").write(out)


def bsd_cal(lh):
    f = lambda k: [float(v) for v in lh[k]]  # noqa: E731
    return [dict(phase=f("fcalphase")[a], tilt=f("fcaltilt")[a], curve=f("fcalcurve")[a],
                 gibpha=f("fcalgibpha")[a], gibmag=f("fcalgibmag")[a]) for a in range(2)]


# ---------------------------------------------------------------- model

def reproject_axis(cal, axis_value, other, Z, invert):
    """survive_reproject_axis, vectorised."""
    ang = math.pi / 2 - (-1.0 if invert else 1.0) * np.arctan2(axis_value, Z)
    mag = np.sqrt(axis_value ** 2 + Z ** 2)
    ang = ang - cal["phase"]
    ang = ang - np.arcsin(np.clip(cal["tilt"] * other / mag, -1, 1))
    ang = ang - np.cos(cal["gibpha"] + ang) * cal["gibmag"]
    ang = ang + cal["curve"] * np.arctan2(other, Z) ** 2
    return ang


def reproject(cal, p_lh, axis):
    """Angle for points in the station frame (N,3); axis 0 = x sweep, 1 = y."""
    if axis == 0:
        return reproject_axis(cal[0], p_lh[:, 0], p_lh[:, 1], -p_lh[:, 2], False) - math.pi / 2
    return reproject_axis(cal[1], p_lh[:, 1], p_lh[:, 0], -p_lh[:, 2], True) - math.pi / 2


def pose_to_rt(p):
    """libsurvive [x y z qw qx qy qz] -> (Rotation, t)."""
    return Rot.from_quat([p[4], p[5], p[6], p[3]]), np.array(p[:3], float)


def rt_to_pose(R, t):
    q = R.as_quat()
    return [t[0], t[1], t[2], q[3], q[0], q[1], q[2]]


def frame_from_entry(entry):
    x, z = np.array(entry["plus_x"], float), np.array(entry["plus_z"], float)
    y = np.cross(z, x)
    M = np.c_[x / np.linalg.norm(x), y / np.linalg.norm(y), z / np.linalg.norm(z)]
    return Rot.from_matrix(M), np.array(entry.get("position", [0, 0, 0]), float)


# ---------------------------------------------------------------- data

def tracker_config(rec):
    with gzip.open(rec, "rt") as f:
        for line in f:
            p = line.split(None, 3)
            if len(p) >= 4 and p[2] == "CONFIG" and p[1].startswith(("WM", "T2", "TR")):
                return json.loads(line.split("CONFIG", 1)[1])
    raise SystemExit("no tracker CONFIG in %s" % rec)


def scenes_from(rec, cfg_path, min_n=30):
    """Still stretches of one recording: median angles, mean accel, init pose."""
    name = os.path.basename(rec).replace(".rec.gz", "")
    d = vive_diag.parse(vive_diag.replay(rec, cfg_path, "default"))
    segs = vive_diag.segments(d["imu"])
    out = []
    for si, seg in enumerate(segs):
        if not seg[2]:
            continue
        groups = collections.defaultdict(list)
        for t, s, ax, lh, a in d["ang"]:
            if seg[0] <= t <= seg[1]:
                groups[(lh, ax, s)].append(a)
        meas = []
        for (lh, ax, s), v in groups.items():
            if len(v) < min_n:
                continue
            v = np.array(v)
            med = float(np.median(v))
            spread = float(np.percentile(v, 95) - np.percentile(v, 5))
            meas.append((lh, ax, s, med, spread, len(v)))
        m = vive_diag.in_seg(d["imu"][:, 0], seg)
        acc = d["imu"][m, 1:4].mean(axis=0)
        out.append({"id": "%s:%d" % (name.split("_", 1)[-1], si), "rec": rec, "seg": seg, "meas": meas,
                    "acc_imu": acc})
    return out


# ---------------------------------------------------------------- solve

class Problem:
    def __init__(self, scenes, points, cals, lh_init, lh_up, fix_lh, R_imu, fit_fcal=False):
        self.scenes, self.points, self.cals = scenes, points, cals
        self.fit_fcal = fit_fcal
        self.lh_ids = sorted(lh_init)
        self.free_lh = [i for i in self.lh_ids if i != fix_lh]
        self.lh_init, self.lh_up, self.fix_lh = lh_init, lh_up, fix_lh
        self.R_imu = R_imu  # trackref_from_imu rotation
        self.use_lh = set(self.lh_ids)

    def unpack(self, x):
        k = 0
        tilt = Rot.from_rotvec([x[0], x[1], 0.0])
        k = 2
        lh = {}
        for i in self.lh_ids:
            R0, t0 = self.lh_init[i]
            if i in self.free_lh:
                lh[i] = (tilt * Rot.from_rotvec(x[k:k + 3]) * R0, tilt.apply(t0 + x[k + 3:k + 6]))
                k += 6
            else:
                lh[i] = (tilt * R0, tilt.apply(t0))
        sc = []
        for _ in self.scenes:
            sc.append((Rot.from_rotvec(x[k:k + 3]), x[k + 3:k + 6]))
            k += 6
        cals = self.cals
        if self.fit_fcal:
            cals = {}
            for i in self.lh_ids:
                cals[i] = [dict(self.cals[i][ax], phase=self.cals[i][ax]["phase"] + x[k + 2 * ax],
                                tilt=self.cals[i][ax]["tilt"] + x[k + 2 * ax + 1]) for ax in (0, 1)]
                k += 4
        self.cur_cals = cals
        return lh, sc

    def n_params(self):
        return 2 + 6 * len(self.free_lh) + 6 * len(self.scenes) + (4 * len(self.lh_ids) if self.fit_fcal else 0)

    def scene_slice(self, j):
        k = 2 + 6 * len(self.free_lh) + 6 * j
        return slice(k, k + 6)

    def lh_slice(self, i):
        k = 2 + 6 * self.free_lh.index(i)
        return slice(k, k + 6)

    def residuals(self, x, split=False):
        lh, sc = self.unpack(x)
        res, parts = [], collections.defaultdict(list)
        for scene, (Rs, ts) in zip(self.scenes, sc):
            for (i, ax), rows in scene["by_lh_ax"].items():
                if not rows:
                    continue
                sens = np.array([r[0] for r in rows])
                meas = np.array([r[1] for r in rows])
                pw = Rs.apply(self.points[sens]) + ts
                if i not in self.use_lh:
                    continue
                Rl, tl = lh[i]
                plh = Rl.inv().apply(pw - tl)
                r = (reproject(self.cur_cals[i], plh, ax) - meas) / SIG_ANGLE
                res.append(r)
                parts["angle lh%d ax%d" % (i, ax)].append(r * SIG_ANGLE)
            up = (Rs * self.R_imu).apply(scene["acc_imu"] / np.linalg.norm(scene["acc_imu"]))
            r = up[:2] / SIG_TRACKER_UP
            res.append(r)
            parts["tracker up"].append(up[:2])
        for i in sorted(self.use_lh):
            up = lh[i][0].apply(self.lh_up[i])
            res.append(up[:2] / SIG_LH_UP)
            parts["lh%d up" % i].append(up[:2])
        out = np.concatenate(res)
        return (out, parts) if split else out


def report(prob, x, title):
    _, parts = prob.residuals(x, split=True)
    print("--", title)
    for k in sorted(parts):
        v = np.concatenate(parts[k])
        if k.startswith("angle"):
            print("   %-16s n %5d  rms %8.1f urad  p95 %8.1f  max %9.1f" % (
                k, len(v), np.sqrt(np.mean(v ** 2)) * 1e6, np.percentile(np.abs(v), 95) * 1e6, np.abs(v).max() * 1e6))
        else:
            ang = np.degrees(np.arcsin(np.clip(np.linalg.norm(v.reshape(-1, 2), axis=1), 0, 1)))
            print("   %-16s n %5d  tilt from world z: mean %.2f deg, max %.2f deg" % (k, len(ang), ang.mean(), ang.max()))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("recordings", nargs="+")
    ap.add_argument("--config", required=True, help="calibration to start from (and replay with)")
    ap.add_argument("--out", required=True)
    ap.add_argument("--fix-lh", type=int, default=1, help="reference station index (held fixed up to tilt)")
    ap.add_argument("--exclude", default="", help="comma list of scene ids (session:seg) to leave out")
    ap.add_argument("--min-sensors", type=int, default=4, help="per station, to use it in a scene")
    ap.add_argument("--fit-fcal", action="store_true", help="also refine OOTX phase and tilt per station axis")
    ap.add_argument("--max-spread", type=float, default=600e-6,
                    help="drop sensor/axis readings whose 5-95%% spread while still exceeds this (rad)")
    ap.add_argument("--reject", type=float, default=2e-3,
                    help="after each joint fit, drop readings with a residual above this (rad), refit")
    a = ap.parse_args()

    cfg, cfg_text = load_libsurvive_config(a.config)
    lhs = {int(v["index"]): v for k, v in cfg.items() if k.startswith("lighthouse")}
    cals = {i: bsd_cal(v) for i, v in lhs.items()}
    lh_init = {i: pose_to_rt([float(p) for p in v["pose"]]) for i, v in lhs.items()}
    lh_up = {i: np.array([float(c) for c in v["accel"]]) / np.linalg.norm([float(c) for c in v["accel"]])
             for i, v in lhs.items()}
    floor = float(cfg.get("floor-offset", 0))

    tcfg = tracker_config(a.recordings[0])
    points = np.array(tcfg["lighthouse_config"]["modelPoints"], float)
    R_imu, _ = frame_from_entry(tcfg["imu"])  # trackref_from_imu
    R_head, t_head = frame_from_entry(tcfg["head"])  # trackref_from_head

    exclude = set(filter(None, a.exclude.split(",")))
    scenes = []
    for rec in a.recordings:
        meta_path = rec[:-len(".rec.gz")] + ".json"
        for s in scenes_from(rec, a.config):
            if s["id"] in exclude:
                continue
            by = collections.defaultdict(list)
            for lh, ax, sens, med, spread, n in s["meas"]:
                # A sensor whose angle wanders by more than this while the tracker is still is
                # being hit by something other than the direct sweep (reflections; libsurvive
                # keeps each sensor's longest pulse per sweep).
                if spread <= a.max_spread:
                    by[(lh, ax)].append((sens, med))
            counts = {i: len({r[0] for ax in (0, 1) for r in by.get((i, ax), [])}) for i in lhs}
            for (lh, ax) in list(by):
                if counts[lh] < a.min_sensors:
                    del by[(lh, ax)]
            if counts.get(a.fix_lh, 0) < a.min_sensors:
                print("skip %s: reference station sees %d sensors" % (s["id"], counts.get(a.fix_lh, 0)))
                continue
            s["by_lh_ax"] = by
            s["counts"] = counts
            scenes.append(s)
    print("scenes:", ", ".join("%s(%s)" % (s["id"], "/".join("%d" % s["counts"][i] for i in sorted(lhs)))
                               for s in scenes), "(sensors per station)")

    # Initial scene poses: the reference station's single-station replay, converted
    # from libsurvive's output (head frame, floor offset applied) to trackref.
    for s in scenes:
        d = vive_diag.parse(vive_diag.replay(s["rec"], a.config, "lh%d" % a.fix_lh))
        m = vive_diag.in_seg(d["pose"][:, 0], s["seg"])
        pm = np.median(d["pose"][m, 1:4], axis=0)
        q = d["pose"][m, 4:8].mean(axis=0)
        R_hw = Rot.from_quat([q[1], q[2], q[3], q[0]])
        R_tw = R_hw * R_head.inv()
        t_tw = R_hw.apply(-R_head.inv().apply(t_head)) + pm + np.array([0, 0, floor])
        s["init"] = (R_tw, t_tw)

    prob = Problem(scenes, points, cals, lh_init, lh_up, a.fix_lh, R_imu, a.fit_fcal)
    x = np.zeros(prob.n_params())
    for j, sc in enumerate(scenes):
        x[prob.scene_slice(j)] = np.r_[sc["init"][0].as_rotvec(), sc["init"][1]]

    def solve_subset(x, idx, loss="linear"):
        def f(v):
            y = x.copy()
            y[idx] = v
            return prob.residuals(y)
        r = least_squares(f, x[idx], loss=loss, f_scale=20.0, max_nfev=2000)
        y = x.copy()
        y[idx] = r.x
        return y, r.cost

    prob.use_lh = set(prob.lh_ids)
    report(prob, x, "as configured (scene poses from libsurvive's reference-station replay)")

    # (1) Scene poses against the reference station only (falls back to both stations
    # for scenes where the reference sees too little).
    prob.use_lh = {a.fix_lh}
    for j in range(len(scenes)):
        x, _ = solve_subset(x, np.arange(prob.n_params())[prob.scene_slice(j)], "soft_l1")
    report(prob, x, "stage 1: scenes from the reference station")

    # (2) Each other station against those scenes, multi-start over orientation.
    for i in prob.free_lh:
        prob.use_lh = {i}
        idx = np.arange(prob.n_params())[prob.lh_slice(i)]
        best = None
        R0, t0 = lh_init[i]
        starts = [np.zeros(6)] + [np.r_[Rot.from_euler("zyx", e, degrees=True).as_rotvec(), 0, 0, 0]
                                  for e in [(dz, dy, dx) for dz in (-60, -30, 30, 60, 180)
                                            for dy in (-20, 0, 20) for dx in (-20, 0, 20)]]
        for st in starts:
            y = x.copy()
            # Starts rotate the station about its own position: x holds a left-multiplied delta.
            y[idx] = st
            y, cost = solve_subset(y, idx, "soft_l1")
            if best is None or cost < best[1]:
                best = (y, cost)
        x = best[0]
        prob.use_lh = {a.fix_lh, i}
        report(prob, x, "stage 2: lh%d against the stage-1 scenes" % i)

    # (3) Everything, rejecting readings the fit can't explain, until none are left.
    prob.use_lh = set(prob.lh_ids)
    for it in range(6):
        r2 = least_squares(prob.residuals, x, loss="soft_l1", f_scale=20.0, max_nfev=3000)
        x = r2.x
        _, sc_poses = prob.unpack(x)
        lh_poses, _ = prob.unpack(x)
        dropped = 0
        for scene, (Rs, ts) in zip(scenes, sc_poses):
            for (i, ax), rows in list(scene["by_lh_ax"].items()):
                keep = []
                for sens, med in rows:
                    pw = Rs.apply(points[[sens]]) + ts
                    Rl, tl = lh_poses[i]
                    r = reproject(prob.cur_cals[i], Rl.inv().apply(pw - tl), ax)[0] - med
                    if abs(r) <= a.reject:
                        keep.append((sens, med))
                dropped += len(rows) - len(keep)
                scene["by_lh_ax"][(i, ax)] = keep
        print("   joint fit pass %d: dropped %d readings" % (it + 1, dropped))
        if not dropped:
            break
    report(prob, r2.x, "stage 3: joint fit" + (" (with OOTX phase/tilt)" if a.fit_fcal else ""))
    if a.fit_fcal:
        prob.unpack(r2.x)
        for i in prob.lh_ids:
            print("   lh%d fcal corrections: phase %s urad, tilt %s" % (
                i, [round((prob.cur_cals[i][ax]["phase"] - cals[i][ax]["phase"]) * 1e6, 1) for ax in (0, 1)],
                [round(prob.cur_cals[i][ax]["tilt"] - cals[i][ax]["tilt"], 5) for ax in (0, 1)]))

    lh, _ = prob.unpack(r2.x)
    print("-- station changes")
    new_poses = {}
    for i in prob.lh_ids:
        R0, t0 = lh_init[i]
        R1, t1 = lh[i]
        print("   lh%d: moved %.1f mm %s, rotated %.2f deg" % (
            i, np.linalg.norm(t1 - t0) * 1000, np.round((t1 - t0) * 1000, 1).tolist(),
            np.degrees((R1 * R0.inv()).magnitude())))
        new_poses[i] = rt_to_pose(R1, t1)
    ids = prob.lh_ids
    print("   distance lh%d-lh%d: %.1f mm (was %.1f)" % (ids[0], ids[1],
          np.linalg.norm(lh[ids[0]][1] - lh[ids[1]][1]) * 1000,
          np.linalg.norm(lh_init[ids[0]][1] - lh_init[ids[1]][1]) * 1000))
    print("   world tilt correction: %.2f deg" % np.degrees(np.linalg.norm(r2.x[:2])))
    save_libsurvive_config(cfg_text, new_poses, a.out)
    print("wrote", a.out)


if __name__ == "__main__":
    main()
