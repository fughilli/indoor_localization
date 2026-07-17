"""Closed-loop tests: synthesize captures with known sensor truth, run the
characterization analyses, and assert the parameters are recovered."""

import math
import os
import tempfile
import unittest

import numpy as np

from swarm.tools import analysis, records, synth


class TempFilesMixin:
    def paths(self):
        d = tempfile.mkdtemp()
        return os.path.join(d, "cap.jsonl"), os.path.join(d, "man.json")


class AoaPlaneTest(unittest.TestCase, TempFilesMixin):
    def check(self, normal, sign, offset, sigma=0.10):
        cap, man = self.paths()
        synth.make_aoa_capture(cap, man, seed=3, normal=normal, sign=sign,
                               offset=offset, sigma=sigma)
        caps = records.load_capture(cap)
        manifest = records.load_manifest(man)
        dirs, angles = [], []
        for seg in records.segments_with(manifest, "true_dir_body"):
            for r in records.select(caps, "aoa", seg["t0"], seg["t1"]):
                dirs.append(seg["true_dir_body"])
                angles.append(r["angle_rad"])
        fit = analysis.fit_aoa_plane(np.array(dirs), np.array(angles))
        n_true = np.asarray(normal, float)
        n_true /= np.linalg.norm(n_true)
        n_fit = np.array(fit["normal"])
        # Normal is recovered up to sign (plane, not direction).
        align = abs(float(n_fit @ n_true))
        self.assertGreater(align, math.cos(math.radians(4.0)),
                           "plane normal off: %s vs %s" % (n_fit, n_true))
        self.assertLess(abs(analysis.wrap_angle(fit["offset_rad"] - offset)
                            if n_fit @ n_true > 0 else 0.0), 0.15)
        self.assertLess(abs(fit["sigma_rad"] - sigma), 0.6 * sigma)
        return fit

    def test_z_plane(self):
        fit = self.check(normal=(0, 0, 1), sign=1.0, offset=0.3)
        self.assertGreater(len(fit["out_of_plane_bins"]), 1)

    def test_tilted_plane_negative_sign(self):
        self.check(normal=(0.2, -0.3, 0.93), sign=-1.0, offset=-1.1)

    def test_x_plane(self):
        self.check(normal=(1, 0, 0), sign=1.0, offset=0.0)


class RangeStatsTest(unittest.TestCase, TempFilesMixin):
    def test_recovers_bias_scale_sigma(self):
        cap, man = self.paths()
        synth.make_range_capture(cap, man, seed=5, bias0=0.08, scale=300e-6,
                                 sigma=0.12, multipath_prob=0.06)
        caps = records.load_capture(cap)
        manifest = records.load_manifest(man)
        segments = [
            (seg["true_range_m"],
             [r["range_m"] for r in
              records.select(caps, "range", seg["t0"], seg["t1"])])
            for seg in records.segments_with(manifest, "true_range_m")]
        stats = analysis.range_stats(segments)
        self.assertAlmostEqual(stats["error_model"]["bias0_m"], 0.08, delta=0.05)
        self.assertAlmostEqual(stats["error_model"]["scale_ppm"], 300, delta=200)
        for seg in stats["segments"]:
            self.assertAlmostEqual(seg["sigma_m"], 0.12, delta=0.05)
            self.assertAlmostEqual(seg["outlier_frac"], 0.05, delta=0.05)


class ImuAllanTest(unittest.TestCase, TempFilesMixin):
    def test_recovers_noise_density_and_bias(self):
        cap, man = self.paths()
        synth.make_imu_capture(cap, man, seed=7, seconds=400.0,
                               accel_density=0.005, gyro_density=2e-4,
                               gyro_bias=(0.01, -0.02, 0.005))
        caps = records.load_capture(cap)
        imu = records.select(caps, "imu")
        report = analysis.imu_static_report([r["accel"] for r in imu],
                                            [r["gyro"] for r in imu], 100.0)
        for ax in report["accel_axes"]:
            self.assertAlmostEqual(ax["noise_density"], 0.005, delta=0.003)
        for ax in report["gyro_axes"]:
            self.assertAlmostEqual(ax["noise_density"], 2e-4, delta=1.2e-4)
        self.assertAlmostEqual(report["gyro_bias"][1], -0.02, delta=0.005)
        self.assertLess(abs(report["accel_scale_error"]), 0.01)


class MagEllipsoidTest(unittest.TestCase, TempFilesMixin):
    def test_recovers_hard_and_soft_iron(self):
        cap, man = self.paths()
        synth.make_mag_capture(cap, man, seed=9, hard_iron=(12.0, -5.0, 3.0),
                               soft_diag=(1.1, 0.9, 1.02), field=45.0)
        caps = records.load_capture(cap)
        fit = analysis.fit_mag_ellipsoid(
            [r["mag"] for r in records.select(caps, "mag")])
        for got, want in zip(fit["hard_iron"], (12.0, -5.0, 3.0)):
            self.assertAlmostEqual(got, want, delta=0.5)
        # Corrected field must be far rounder than the raw distortion (10%).
        self.assertLess(fit["residual_frac"], 0.02)


if __name__ == "__main__":
    unittest.main()
