"""Synthetic capture generator: produces capture JSONL + manifest pairs from
known sensor truth, so the characterization pipeline can be validated end to
end before hardware exists (and so tests can assert parameter recovery).
"""

import json

import numpy as np

from swarm.tools import analysis

GRAVITY = 9.80665


def _dump(path, records):
    with open(path, "w") as f:
        for r in records:
            f.write(json.dumps(r) + "\n")


def make_range_capture(capture_path, manifest_path, seed=0,
                       distances=(2.0, 5.0, 10.0, 20.0, 40.0),
                       n_per=400, sigma=0.10, bias0=0.05, scale=200e-6,
                       multipath_prob=0.05, multipath_scale=1.0):
    """Static captures at several known distances; range error model
    err = bias0 + scale*d plus Gaussian noise and Exp multipath outliers."""
    rng = np.random.default_rng(seed)
    records, segments = [], []
    t = 0.0
    for d in distances:
        t0 = t
        for _ in range(n_per):
            r = d + bias0 + scale * d + rng.normal(0, sigma)
            if rng.random() < multipath_prob:
                r += rng.exponential(multipath_scale)
            records.append({"t": round(t, 4), "type": "range", "peer": 1,
                            "range_m": round(float(r), 4)})
            t += 0.05
        segments.append({"t0": round(t0, 4), "t1": round(t, 4),
                         "true_range_m": d})
        t += 5.0
    _dump(capture_path, records)
    with open(manifest_path, "w") as f:
        json.dump({"segments": segments}, f, indent=2)


def make_aoa_capture(capture_path, manifest_path, seed=0,
                     normal=(0.0, 0.0, 1.0), sign=1.0, offset=0.3,
                     sigma=0.12, n_dirs=80, n_per=25, oop_max_deg=60.0):
    """Turntable-style sweep: discrete segments, each holding a known true
    peer direction in the body frame; reported angle follows
    sign * azimuth-in-plane + offset with Gaussian noise that grows with
    out-of-plane angle (mild model of PDoA degradation)."""
    rng = np.random.default_rng(seed)
    n = np.asarray(normal, float)
    n /= np.linalg.norm(n)
    e1, e2 = analysis._plane_basis(n)
    records, segments = [], []
    t = 0.0
    for _ in range(n_dirs):
        az = rng.uniform(-np.pi, np.pi)
        oop = np.radians(rng.uniform(-oop_max_deg, oop_max_deg))
        u = (np.cos(oop) * (np.cos(az) * e1 + np.sin(az) * e2)
             + np.sin(oop) * n)
        t0 = t
        sig = sigma * (1.0 + (abs(oop) / np.radians(60.0)) ** 2)
        for _ in range(n_per):
            ang = analysis.wrap_angle(sign * az + offset
                                      + rng.normal(0, sig))
            records.append({"t": round(t, 4), "type": "aoa", "peer": 1,
                            "angle_rad": round(float(ang), 5)})
            t += 0.05
        segments.append({"t0": round(t0, 4), "t1": round(t, 4),
                         "true_dir_body": [round(float(x), 5) for x in u]})
        t += 2.0
    _dump(capture_path, records)
    with open(manifest_path, "w") as f:
        json.dump({"segments": segments}, f, indent=2)


def make_imu_capture(capture_path, manifest_path, seed=0, seconds=600.0,
                     fs=100.0, accel_density=0.005, gyro_density=2e-4,
                     accel_bias_rw=5e-4, gyro_bias_rw=2e-5,
                     gyro_bias=(0.01, -0.02, 0.005)):
    """Static IMU capture with white noise (given densities) + bias random
    walk, gravity on +z."""
    rng = np.random.default_rng(seed)
    n = int(seconds * fs)
    dt = 1.0 / fs
    records = []
    ba = np.zeros(3)
    bg = np.asarray(gyro_bias, float).copy()
    sq = np.sqrt(fs)
    for i in range(n):
        ba += rng.normal(0, accel_bias_rw * np.sqrt(dt), 3)
        bg += rng.normal(0, gyro_bias_rw * np.sqrt(dt), 3)
        accel = (np.array([0.0, 0.0, GRAVITY]) + ba
                 + rng.normal(0, accel_density * sq, 3))
        gyro = bg + rng.normal(0, gyro_density * sq, 3)
        records.append({"t": round(i * dt, 4), "type": "imu",
                        "accel": [round(float(v), 6) for v in accel],
                        "gyro": [round(float(v), 6) for v in gyro]})
    _dump(capture_path, records)
    with open(manifest_path, "w") as f:
        json.dump({"segments": [{"t0": 0.0, "t1": seconds, "static": True}]},
                  f, indent=2)


def make_mag_capture(capture_path, manifest_path, seed=0, n=1500,
                     hard_iron=(12.0, -5.0, 3.0),
                     soft_diag=(1.1, 0.9, 1.02), field=45.0, sigma=0.4):
    """Rotation capture: field vector seen from uniformly random attitudes,
    distorted by hard-iron offset + diagonal soft-iron scaling + noise."""
    rng = np.random.default_rng(seed)
    records = []
    soft = np.diag(soft_diag)
    hard = np.asarray(hard_iron, float)
    for i in range(n):
        u = rng.normal(size=3)
        u /= np.linalg.norm(u)
        m = soft @ (field * u) + hard + rng.normal(0, sigma, 3)
        records.append({"t": round(i * 0.05, 4), "type": "mag",
                        "mag": [round(float(v), 4) for v in m]})
    _dump(capture_path, records)
    with open(manifest_path, "w") as f:
        json.dump({"segments": []}, f, indent=2)
