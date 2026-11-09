"""Diagnose libsurvive pose errors from recorded sessions (vive_web recorder).

  python3 swarm/tools/vive_diag.py captures/vive/*.rec.gz [--config GOOD.config.json]
      [--variants default,lh0,lh1,tw,svps,raw] [--jobs 8] [--json out.json]

Replays each recording through survive-cli (--playback, --disable-calibrate,
a scratch copy of the calibration) once per variant, caching the output next
to the scratch dir, and reports per session:

  segments   still / moving stretches found from the IMU (gyro and accel)
  still      position and rotation spread while still, the jitter spectrum's
             strongest peaks, and per-sensor angle noise
  jumps      pose steps that a constant-velocity prediction doesn't explain
  geometry   turntable sessions: plane + circle fit to the moving stretches
             (radial and out-of-plane error, axis tilt from world z);
             slider: line fit (perpendicular error, stroke, tilt)
  variants   the same numbers per variant, plus, while still, how far apart
             the single-station solutions (lh0, lh1) put the tracker

Variants (extra survive-cli arguments):
  default  as recorded
  lh0/lh1  --disable-lighthouse 1 / 0: one station only
  tw       --time-window 1700000: ~35 ms, covers a full b/c cycle with margin
  svps     --sensor-variance-per-sec 1.0: discount older angles in the fit
  raw      --use-raw-obs 1: the optical solver's poses, no Kalman filtering
"""

import argparse
import collections
import concurrent.futures
import glob
import gzip
import hashlib
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile

import numpy as np

CLI = os.path.expanduser("~/Projects/git_checkouts/libsurvive/build-hid/survive-cli")
CACHE = os.path.join(tempfile.gettempdir(), "vive_diag_cache")
VARIANTS = {
    "default": [],
    "lh0": ["--disable-lighthouse", "1"],
    "lh1": ["--disable-lighthouse", "0"],
    "tw": ["--time-window", "1700000"],
    "svps": ["--sensor-variance-per-sec", "1.0"],
    "raw": ["--use-raw-obs", "1"],
}


# ---------------------------------------------------------------- replay

def replay(rec, config, variant):
    """survive-cli output for one recording and variant (cached by content)."""
    args = VARIANTS[variant]
    key = hashlib.sha1(("%s|%s|%s|%s" % (os.path.abspath(rec), os.path.getmtime(rec), open(config, "rb").read(),
                                         args)).encode() + open(CLI, "rb").read()[:4096]).hexdigest()[:16]
    os.makedirs(CACHE, exist_ok=True)
    out = os.path.join(CACHE, "%s.%s.%s.out" % (os.path.basename(rec).replace(".rec.gz", ""), variant, key))
    if os.path.exists(out) and os.path.getsize(out) > 0:
        return out
    tmpcfg = tempfile.NamedTemporaryFile(suffix=".json", delete=False).name
    shutil.copyfile(config, tmpcfg)
    cmd = [CLI, "--playback", rec, "--configfile", tmpcfg, "--playback-factor", "0", "--record-stdout",
           "--record-rawlight", "0", "--record-imu", "0", "--record-cal-imu", "1", "--record-angle", "1",
           "--lighthouse-gen", "1", "--disable-calibrate", "1"] + args
    with open(out + ".tmp", "w") as f:
        subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT, timeout=1800)
    os.unlink(tmpcfg)
    os.rename(out + ".tmp", out)
    return out


def parse(path):
    """Poses, calibrated IMU, angles and log lines from a replay."""
    pose, imu, ang, log = [], [], [], []
    with open(path, errors="replace") as f:
        for line in f:
            p = line.split()
            if len(p) < 3:
                continue
            k = p[2]
            try:
                if k == "POSE" and len(p) >= 10:
                    pose.append([float(x) for x in [p[0]] + p[3:10]])
                elif k == "I" and len(p) >= 14:
                    imu.append([float(p[0])] + [float(x) for x in p[5:11]])
                elif k == "A" and len(p) == 9:
                    ang.append((float(p[0]), int(p[3]), int(p[4]) & 1, int(p[8]), float(p[7])))
                elif p[1] == "INFO" and k == "LOG":
                    log.append((float(p[0]), line.split("LOG", 1)[1].strip()))
            except ValueError:
                pass
    return {"pose": np.array(pose).reshape(-1, 8), "imu": np.array(imu).reshape(-1, 7), "ang": ang, "log": log}


# ---------------------------------------------------------------- segments

def segments(imu, min_len=3.0, trim=0.5):
    """[(t0, t1, still)] from the calibrated IMU: still when the gyro and the
    accel's short-term spread are both quiet."""
    t = imu[:, 0]
    acc, gyr = imu[:, 1:4], imu[:, 4:7]
    win = max(5, int(0.25 * len(t) / max(1e-3, t[-1] - t[0])))
    kern = np.ones(win) / win
    gn = np.convolve(np.linalg.norm(gyr, axis=1), kern, "same")
    an = np.linalg.norm(acc, axis=1)
    am = np.convolve(an, kern, "same")
    asd = np.sqrt(np.maximum(0, np.convolve(an * an, kern, "same") - am * am))
    still = (gn < 0.03) & (asd < 0.008)
    out = []
    i = 0
    while i < len(t):
        j = i
        while j + 1 < len(t) and still[j + 1] == still[i]:
            j += 1
        t0, t1 = t[i] + trim, t[j] - trim
        if t1 - t0 >= min_len:
            out.append((t0, t1, bool(still[i])))
        i = j + 1
    return out


def in_seg(t, seg):
    return (t >= seg[0]) & (t <= seg[1])


# ---------------------------------------------------------------- metrics

def quat_angle(q, ref):
    d = np.abs(np.sum(q * ref, axis=1)).clip(0, 1)
    return 2 * np.arccos(d)


def still_stats(pose, seg):
    m = in_seg(pose[:, 0], seg)
    if m.sum() < 20:
        return None
    t, p, q = pose[m, 0], pose[m, 1:4], pose[m, 4:8]
    qm = q.mean(axis=0)
    qm /= np.linalg.norm(qm)
    rot = np.degrees(quat_angle(q, qm))
    # Spectrum: resample to 120 Hz, detrend, rfft per axis.
    grid = np.arange(t[0], t[-1], 1 / 120.0)
    peaks = []
    if len(grid) > 240:
        mag = np.zeros(len(grid) // 2 + 1)
        for k in range(3):
            x = np.interp(grid, t, p[:, k])
            x = x - np.polyval(np.polyfit(grid, x, 1), grid)
            mag += np.abs(np.fft.rfft(x * np.hanning(len(x)))) ** 2
        f = np.fft.rfftfreq(len(grid), 1 / 120.0)
        amp = np.sqrt(mag) * 2 / (len(grid) * 0.5) * 1000  # mm, hann-corrected
        sel = f > 0.2
        idx = np.argsort(amp[sel])[::-1]
        fs, am = f[sel], amp[sel]
        for i in idx:
            if all(abs(fs[i] - pf) > 0.5 for pf, _ in peaks):
                peaks.append((round(float(fs[i]), 2), round(float(am[i]), 3)))
            if len(peaks) == 4:
                break
    return {"seconds": round(float(t[-1] - t[0]), 1), "rate_hz": round(len(t) / float(t[-1] - t[0]), 1),
            "mean": p.mean(axis=0).tolist(),
            "pos_sd_mm": (p.std(axis=0) * 1000).round(2).tolist(),
            "pos_p2p_mm": ((p.max(axis=0) - p.min(axis=0)) * 1000).round(2).tolist(),
            "rot_sd_deg": round(float(rot.std()), 3), "rot_p2p_deg": round(float(rot.max() - rot.min()), 3),
            "spectrum_peaks_hz_mm": peaks}


def jumps(pose, thresh_mm=3.0):
    """Steps a constant-velocity prediction misses by more than thresh (or 8 MADs)."""
    t, p = pose[:, 0], pose[:, 1:4]
    if len(t) < 5:
        return {"count": 0}
    dt = np.diff(t)
    v = np.diff(p, axis=0) / np.maximum(dt, 1e-4)[:, None]
    pred = p[1:-1] + v[:-1] * dt[1:, None]
    err = np.linalg.norm(p[2:] - pred, axis=1) * 1000
    mad = np.median(np.abs(err - np.median(err))) + 1e-6
    th = max(thresh_mm, np.median(err) + 8 * 1.4826 * mad)
    idx = np.where(err > th)[0]
    # Merge hits within 50 ms into one event.
    events = []
    for i in idx:
        ti = t[i + 2]
        if events and ti - events[-1][0] < 0.05:
            events[-1][1] = max(events[-1][1], err[i])
        else:
            events.append([ti, err[i]])
    dur = t[-1] - t[0]
    return {"threshold_mm": round(float(th), 2), "count": len(events), "per_min": round(len(events) / dur * 60, 2),
            "median_step_err_mm": round(float(np.median(err)), 3),
            "largest": [(round(float(a), 3), round(float(b), 1)) for a, b in sorted(events, key=lambda e: -e[1])[:8]]}


def fit_plane(P):
    c = P.mean(axis=0)
    _, s, vt = np.linalg.svd(P - c)
    return c, vt[2], vt[0], vt[1]


def fit_circle(P):
    """Plane + geometric circle fit: radius, radial and out-of-plane residuals, axis tilt."""
    c, n, e1, e2 = fit_plane(P)
    x, y = (P - c) @ e1, (P - c) @ e2
    A = np.c_[2 * x, 2 * y, np.ones_like(x)]
    sol, *_ = np.linalg.lstsq(A, x * x + y * y, rcond=None)
    cx, cy = sol[0], sol[1]
    r = math.sqrt(max(1e-12, sol[2] + cx * cx + cy * cy))
    for _ in range(20):  # Gauss-Newton on geometric distance
        dx, dy = x - cx, y - cy
        d = np.hypot(dx, dy) + 1e-12
        J = np.c_[-dx / d, -dy / d, -np.ones_like(d)]
        res = d - r
        step, *_ = np.linalg.lstsq(J, -res, rcond=None)
        cx, cy, r = cx + step[0], cy + step[1], r + step[2]
    rad = np.hypot(x - cx, y - cy) - r
    off = (P - c) @ n
    return {"radius_mm": round(r * 1000, 2), "radial_rms_mm": round(float(np.sqrt(np.mean(rad ** 2))) * 1000, 2),
            "radial_p95_mm": round(float(np.percentile(np.abs(rad), 95)) * 1000, 2),
            "radial_max_mm": round(float(np.abs(rad).max()) * 1000, 2),
            "plane_rms_mm": round(float(np.sqrt(np.mean(off ** 2))) * 1000, 2),
            "axis_tilt_deg": round(math.degrees(math.acos(min(1, abs(n[2])))), 2), "n": int(len(P))}


def fit_line(P):
    c = P.mean(axis=0)
    _, s, vt = np.linalg.svd(P - c)
    d = vt[0]
    along = (P - c) @ d
    perp = np.linalg.norm((P - c) - np.outer(along, d), axis=1)
    return {"stroke_mm": round(float(np.percentile(along, 99.5) - np.percentile(along, 0.5)) * 1000, 1),
            "perp_rms_mm": round(float(np.sqrt(np.mean(perp ** 2))) * 1000, 2),
            "perp_p95_mm": round(float(np.percentile(perp, 95)) * 1000, 2),
            "perp_max_mm": round(float(perp.max()) * 1000, 2),
            "tilt_from_horizontal_deg": round(math.degrees(math.asin(min(1, abs(d[2])))), 2), "n": int(len(P))}


def angle_noise(ang, segs):
    """Per (lh, axis, sensor) angle spread inside still segments, in microradians."""
    groups = collections.defaultdict(list)
    for t, s, ax, lh, a in ang:
        for si, seg in enumerate(segs):
            if seg[2] and seg[0] <= t <= seg[1]:
                groups[(lh, ax, s, si)].append(a)
    rows = []
    for (lh, ax, s, si), v in groups.items():
        if len(v) < 50:
            continue
        v = np.array(v)
        mad = np.median(np.abs(v - np.median(v))) * 1.4826
        rows.append({"lh": lh, "axis": ax, "sensor": s, "seg": si, "n": len(v),
                     "sd_urad": round(float(v.std()) * 1e6, 1), "mad_urad": round(float(mad) * 1e6, 1),
                     "outliers": int(np.sum(np.abs(v - np.median(v)) > 10 * max(mad, 2e-5)))})
    return rows


def session_kind(name):
    if "turntable" in name:
        return "circle"
    if "slider" in name:
        return "line"
    return "other"


def analyze(rec, config, variants, jobs):
    name = os.path.basename(rec).replace(".rec.gz", "")
    with concurrent.futures.ThreadPoolExecutor(jobs) as ex:
        outs = dict(zip(variants, ex.map(lambda v: replay(rec, config, v), variants)))
    data = {v: parse(o) for v, o in outs.items()}
    base = data["default"] if "default" in data else next(iter(data.values()))
    segs = segments(base["imu"])
    res = {"name": name, "segments": [(round(a, 2), round(b, 2), s) for a, b, s in segs], "variants": {}}
    for v, d in data.items():
        pose = d["pose"]
        r = {"poses": int(len(pose)), "still": [], "jumps": jumps(pose) if len(pose) else None,
             "log": collections.Counter(msg.split(" for ")[0][:60] for _, msg in d["log"]).most_common(6)}
        for seg in segs:
            if seg[2]:
                st = still_stats(pose, seg)
                if st:
                    r["still"].append(st)
        moving = np.zeros(len(pose), bool)
        for seg in segs:
            if not seg[2]:
                moving |= in_seg(pose[:, 0], seg)
        kind = session_kind(name)
        if moving.sum() > 50:
            P = pose[moving, 1:4]
            r["geometry"] = fit_circle(P) if kind == "circle" else fit_line(P) if kind == "line" else None
            r["moving_jumps"] = jumps(pose[moving])
        res["variants"][v] = r
    if "lh0" in data and "lh1" in data:
        gaps = []
        for i, seg in enumerate(segs):
            if not seg[2]:
                continue
            a, b = still_stats(data["lh0"]["pose"], seg), still_stats(data["lh1"]["pose"], seg)
            if a and b:
                gaps.append({"seg": i, "lh0_minus_lh1_mm": (np.subtract(a["mean"], b["mean"]) * 1000).round(1).tolist(),
                             "dist_mm": round(float(np.linalg.norm(np.subtract(a["mean"], b["mean"])) * 1000), 1)})
        res["lh0_vs_lh1_still"] = gaps
    if "default" in data:
        noise = angle_noise(data["default"]["ang"], segs)
        sd = [r["mad_urad"] for r in noise]
        res["angle_noise"] = {
            "groups": len(noise),
            "median_mad_urad": round(float(np.median(sd)), 1) if sd else None,
            "worst": sorted(noise, key=lambda r: -r["mad_urad"])[:6],
            "per_lh_axis_median_mad_urad": {
                "%d/%d" % k: round(float(np.median([r["mad_urad"] for r in noise if (r["lh"], r["axis"]) == k])), 1)
                for k in sorted({(r["lh"], r["axis"]) for r in noise})},
            "outlier_hits": int(sum(r["outliers"] for r in noise)),
            "hits": int(sum(r["n"] for r in noise))}
    return res


def fmt_session(res):
    L = ["", "=" * 78, res["name"], "segments: " + ", ".join(
        "%s %.0f-%.0fs" % ("still" if s else "move", a, b) for a, b, s in res["segments"])]
    for v, r in res["variants"].items():
        L.append("-- %-8s poses %6d" % (v, r["poses"]))
        for st in r["still"]:
            L.append("   still %5.1fs @%5.0fHz  pos sd %s mm  p2p %s mm  rot sd %.3f deg  peaks %s" % (
                st["seconds"], st["rate_hz"], st["pos_sd_mm"], st["pos_p2p_mm"], st["rot_sd_deg"],
                " ".join("%.1fHz:%.2fmm" % pk for pk in st["spectrum_peaks_hz_mm"][:3])))
        j = r["jumps"]
        if j:
            L.append("   jumps %d (%.1f/min, thr %.1f mm, median step err %.2f mm) largest %s" % (
                j["count"], j["per_min"], j["threshold_mm"], j["median_step_err_mm"], j["largest"][:4]))
        if r.get("geometry"):
            L.append("   geometry " + json.dumps(r["geometry"]))
    if res.get("lh0_vs_lh1_still"):
        L.append("-- lh0-only vs lh1-only while still: " + "; ".join(
            "seg %d: %s mm (|d| %.1f)" % (g["seg"], g["lh0_minus_lh1_mm"], g["dist_mm"]) for g in res["lh0_vs_lh1_still"]))
    an = res.get("angle_noise")
    if an:
        L.append("-- angle noise (still): median MAD %s urad over %d sensor/axis groups; per lh/axis %s; "
                 "outlier hits %d of %d" % (an["median_mad_urad"], an["groups"], an["per_lh_axis_median_mad_urad"],
                                            an["outlier_hits"], an["hits"]))
        L.append("   worst: " + "; ".join("lh%d ax%d s%d %.0furad (n %d, %d out)" % (
            w["lh"], w["axis"], w["sensor"], w["mad_urad"], w["n"], w["outliers"]) for w in an["worst"]))
    return "\n".join(L)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("recordings", nargs="+")
    ap.add_argument("--config", help="calibration to replay with (default: each recording's use_config or its own)")
    ap.add_argument("--variants", default=",".join(VARIANTS))
    ap.add_argument("--jobs", type=int, default=6)
    ap.add_argument("--json")
    a = ap.parse_args()
    variants = a.variants.split(",")
    results = []
    for rec in a.recordings:
        base = rec[:-len(".rec.gz")]
        cfg = a.config
        if not cfg:
            meta = json.load(open(base + ".json")) if os.path.exists(base + ".json") else {}
            cfg = os.path.join(os.path.dirname(rec), meta["use_config"]) if meta.get("use_config") else base + ".config.json"
        res = analyze(rec, cfg, variants, a.jobs)
        results.append(res)
        print(fmt_session(res), flush=True)
    if a.json:
        json.dump(results, open(a.json, "w"), indent=1, default=float)


if __name__ == "__main__":
    main()
