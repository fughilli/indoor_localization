"""Numerics for sensor characterization: AoA plane fitting, range statistics,
IMU Allan deviation, magnetometer ellipsoid calibration. numpy/scipy only.
"""

import numpy as np
from scipy import optimize


def wrap_angle(a):
    """Wrap to (-pi, pi]."""
    return np.mod(np.asarray(a) + np.pi, 2 * np.pi) - np.pi


# ---------------------------------------------------------------- AoA plane

def _plane_basis(normal):
    """Deterministic in-plane orthonormal basis (e1, e2) for a unit normal."""
    ref = np.array([0.0, 0.0, 1.0])
    if abs(normal @ ref) > 0.9:
        ref = np.array([1.0, 0.0, 0.0])
    e1 = np.cross(normal, ref)
    e1 /= np.linalg.norm(e1)
    return e1, np.cross(normal, e1)


def _aoa_residuals(normal, sign, dirs, angles):
    """(residuals-with-offset-removed, offset, in-plane weights)."""
    e1, e2 = _plane_basis(normal)
    az = np.arctan2(dirs @ e2, dirs @ e1)
    r = wrap_angle(angles - sign * az)
    # In-plane component: azimuth is unobservable for directions along the
    # normal, so weight those out.
    w = np.linalg.norm(dirs - np.outer(dirs @ normal, normal), axis=1)
    offset = np.arctan2(np.sum(w * np.sin(r)), np.sum(w * np.cos(r)))
    return wrap_angle(r - offset), offset, w


def fit_aoa_plane(dirs, angles):
    """Discover the AoA measurement convention from ground-truth pairs.

    dirs: (N,3) unit true directions to the peer in the body frame.
    angles: (N,) raw reported arrival angles, radians.

    Model: angle = sign * azimuth_of(dir projected into the plane) + offset,
    where the plane is defined by a unit normal in the body frame. Coarse
    grid over the hemisphere, then Nelder-Mead refinement.

    Returns a dict: normal (body frame), sign, offset_rad, sigma_rad
    (weighted RMS residual), n_used, and out_of_plane_bins — residual sigma
    binned by out-of-plane angle, which shows how fast AoA degrades off-axis.
    """
    dirs = np.asarray(dirs, float)
    angles = np.asarray(angles, float)

    def score(normal, sign):
        n = np.asarray(normal, float)
        nn = np.linalg.norm(n)
        if nn < 1e-6:
            return np.inf, None
        n = n / nn
        r, _, w = _aoa_residuals(n, sign, dirs, angles)
        if np.sum(w) < 1e-6:
            return np.inf, None
        return float(np.sum(w * r * r) / np.sum(w)), n

    best = (np.inf, None, 1.0)
    for theta in np.linspace(0, np.pi / 2, 7):        # hemisphere is enough:
        for phi in np.linspace(0, 2 * np.pi, 13):     # -n == n with sign flip
            n = np.array([np.sin(theta) * np.cos(phi),
                          np.sin(theta) * np.sin(phi), np.cos(theta)])
            for sign in (1.0, -1.0):
                sc, nn = score(n, sign)
                if sc < best[0]:
                    best = (sc, nn, sign)
    _, n0, sign = best
    res = optimize.minimize(lambda x: score(x, sign)[0], n0,
                            method="Nelder-Mead",
                            options={"xatol": 1e-4, "fatol": 1e-8})
    normal = res.x / np.linalg.norm(res.x)
    resid, offset, w = _aoa_residuals(normal, sign, dirs, angles)
    good = w > 0.3
    sigma = float(np.sqrt(np.sum((w * resid**2)[good]) / np.sum(w[good])))

    # Degradation vs out-of-plane angle.
    oop = np.abs(np.degrees(np.arcsin(np.clip(dirs @ normal, -1, 1))))
    bins = []
    for lo in range(0, 90, 15):
        m = (oop >= lo) & (oop < lo + 15) & good
        if np.count_nonzero(m) >= 5:
            bins.append({"oop_deg": [lo, lo + 15],
                         "sigma_deg": round(float(np.degrees(
                             np.sqrt(np.mean(resid[m] ** 2)))), 2),
                         "n": int(np.count_nonzero(m))})
    return {
        "normal": [round(float(x), 4) for x in normal],
        "sign": sign,
        "offset_rad": round(float(offset), 4),
        "sigma_rad": round(sigma, 4),
        "sigma_deg": round(float(np.degrees(sigma)), 2),
        "n_used": int(np.count_nonzero(good)),
        "out_of_plane_bins": bins,
    }


# ------------------------------------------------------------------- range

def range_stats(segments):
    """Per-distance and overall UWB ranging statistics.

    segments: list of (true_range_m, samples) with samples an array of
    measured ranges. Outliers = beyond 4 * robust sigma (MAD) of the
    segment median. Returns per-segment stats plus a linear error model
    err(d) = bias0 + scale * d fitted on segment medians.
    """
    per = []
    meds, trues = [], []
    for true_r, samples in segments:
        s = np.asarray(samples, float)
        med = float(np.median(s))
        mad_sigma = 1.4826 * float(np.median(np.abs(s - med)))
        inlier = np.abs(s - med) <= 4 * max(mad_sigma, 1e-6)
        per.append({
            "true_range_m": true_r,
            "n": int(len(s)),
            "bias_m": round(med - true_r, 4),
            "sigma_m": round(float(np.std(s[inlier])), 4),
            "outlier_frac": round(float(1.0 - np.mean(inlier)), 4),
        })
        meds.append(med)
        trues.append(true_r)
    trues = np.asarray(trues, float)
    errs = np.asarray(meds) - trues
    model = {}
    if len(trues) >= 2:
        scale, bias0 = np.polyfit(trues, errs, 1)
        model = {"bias0_m": round(float(bias0), 4),
                 "scale_ppm": round(float(scale * 1e6), 1)}
    return {"segments": per, "error_model": model}


# --------------------------------------------------------------------- IMU

def allan_deviation(x, fs, n_taus=30):
    """Overlapping Allan deviation of a rate signal sampled at fs Hz.

    Returns (taus_s, adev) arrays. Standard use: gyro rad/s or accel m/s^2
    static captures; the tau~1s value approximates the white-noise density
    N = adev(tau) * sqrt(tau), and the curve minimum tracks bias
    instability.
    """
    x = np.asarray(x, float)
    n = len(x)
    max_m = n // 4
    ms = np.unique(np.logspace(0, np.log10(max(max_m, 2)),
                               n_taus).astype(int))
    ms = ms[ms >= 1]
    csum = np.concatenate([[0.0], np.cumsum(x)])
    taus, adev = [], []
    for m in ms:
        # Overlapping cluster means.
        means = (csum[m:] - csum[:-m]) / m
        d = means[m:] - means[:-m]
        if len(d) < 2:
            continue
        taus.append(m / fs)
        adev.append(np.sqrt(0.5 * np.mean(d * d)))
    return np.asarray(taus), np.asarray(adev)


def imu_static_report(accel, gyro, fs, gravity=9.80665):
    """Characterize a static IMU capture: per-axis noise density (from Allan
    dev at tau=1s), bias instability (curve minimum), sample sigma, plus the
    gravity-magnitude check for accel scale."""
    accel = np.asarray(accel, float)
    gyro = np.asarray(gyro, float)

    def axes_report(data):
        out = []
        for ax in range(3):
            x = data[:, ax] - np.mean(data[:, ax])
            taus, adev = allan_deviation(x, fs)
            i1 = int(np.argmin(np.abs(taus - 1.0)))
            out.append({
                "sample_sigma": round(float(np.std(x)), 6),
                "noise_density": round(float(adev[i1] * np.sqrt(taus[i1])), 6),
                "bias_instability": round(float(np.min(adev)), 6),
            })
        return out

    return {
        "n": int(len(accel)),
        "fs_hz": fs,
        "accel_axes": axes_report(accel),
        "gyro_axes": axes_report(gyro),
        "accel_norm_mean": round(float(np.mean(np.linalg.norm(accel, axis=1))), 4),
        "accel_scale_error": round(float(
            np.mean(np.linalg.norm(accel, axis=1)) / gravity - 1.0), 5),
        "gyro_bias": [round(float(b), 6) for b in np.mean(gyro, axis=0)],
    }


# --------------------------------------------------------------------- mag

def fit_mag_ellipsoid(samples):
    """Hard/soft-iron calibration: fit x'Qx + 2q'x = 1 by linear least
    squares over a rotation capture. Returns the hard-iron offset, the
    soft-iron correction matrix W (unit-determinant symmetric; corrected =
    W @ (raw - offset)), and the residual spread of |corrected| as a
    fraction of the field magnitude."""
    m = np.asarray(samples, float)
    x, y, z = m[:, 0], m[:, 1], m[:, 2]
    d = np.column_stack([x * x, y * y, z * z,
                         2 * x * y, 2 * x * z, 2 * y * z,
                         2 * x, 2 * y, 2 * z])
    p, *_ = np.linalg.lstsq(d, np.ones(len(m)), rcond=None)
    q_mat = np.array([[p[0], p[3], p[4]],
                      [p[3], p[1], p[5]],
                      [p[4], p[5], p[2]]])
    q_vec = p[6:9]
    center = -np.linalg.solve(q_mat, q_vec)
    scale = 1.0 + center @ q_mat @ center
    mm = q_mat / scale                       # (x-c)' M (x-c) = 1
    evals, evecs = np.linalg.eigh(mm)
    if np.any(evals <= 0):
        raise ValueError("degenerate ellipsoid fit — capture needs more "
                         "orientation coverage")
    w = evecs @ np.diag(np.sqrt(evals)) @ evecs.T
    w_unit = w / np.linalg.det(w) ** (1.0 / 3.0)
    corrected = (m - center) @ w_unit.T
    norms = np.linalg.norm(corrected, axis=1)
    field = float(np.mean(norms))
    return {
        "hard_iron": [round(float(c), 4) for c in center],
        "soft_iron": [[round(float(v), 5) for v in row] for row in w_unit],
        "field_magnitude": round(field, 4),
        "residual_frac": round(float(np.std(norms) / field), 5),
        "n": int(len(m)),
    }
