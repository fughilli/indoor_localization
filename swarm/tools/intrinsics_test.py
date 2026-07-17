"""Intrinsics schema tests: blob round-trip, characterize-report merging
(full pipeline from synthetic captures), partial configs, header gen."""

import copy
import os
import tempfile
import unittest

from swarm.tools import analysis, intrinsics, records, synth

EXAMPLE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "examples", "widget_v1_intrinsics.json")


class IntrinsicsTest(unittest.TestCase):
    def setUp(self):
        self.config = intrinsics.load(EXAMPLE)

    def test_blob_round_trip(self):
        decoded = intrinsics.decode_blob(
            intrinsics.encode_blob(copy.deepcopy(self.config)))
        self.assertAlmostEqual(decoded["imu"]["accel_noise_density"], 0.005)
        self.assertAlmostEqual(decoded["uwb_aoa"]["offset_rad"], 0.3, places=6)
        self.assertAlmostEqual(decoded["mag"]["soft_iron"][1][1], 0.875,
                               places=5)
        self.assertAlmostEqual(decoded["alt"]["sigma_m"], 0.3, places=6)

    def test_partial_config_and_missing_fields(self):
        cfg = intrinsics.empty("widget-x")
        cfg["uwb_range"] = {"sigma_m": 0.15}  # only one field known
        decoded = intrinsics.decode_blob(intrinsics.encode_blob(cfg))
        self.assertAlmostEqual(decoded["uwb_range"]["sigma_m"], 0.15,
                               places=6)
        self.assertNotIn("bias0_m", decoded["uwb_range"])  # NaN -> absent
        self.assertNotIn("imu", decoded)                   # section absent

    def test_merge_from_characterization_pipeline(self):
        """Synthetic captures -> characterize analyses -> merged intrinsics."""
        d = tempfile.mkdtemp()
        cap, man = os.path.join(d, "c.jsonl"), os.path.join(d, "m.json")
        cfg = intrinsics.empty("widget-test")

        synth.make_imu_capture(cap, man, seed=1, seconds=200.0)
        imu = records.select(records.load_capture(cap), "imu")
        report = analysis.imu_static_report([r["accel"] for r in imu],
                                            [r["gyro"] for r in imu], 100.0)
        intrinsics.merge_report(cfg, "imu", report)
        self.assertAlmostEqual(cfg["imu"]["accel_noise_density"], 0.005,
                               delta=0.003)

        synth.make_aoa_capture(cap, man, seed=2, normal=(0, 0, 1),
                               sign=-1.0, offset=0.5)
        caps = records.load_capture(cap)
        manifest = records.load_manifest(man)
        dirs, angles = [], []
        for seg in records.segments_with(manifest, "true_dir_body"):
            for r in records.select(caps, "aoa", seg["t0"], seg["t1"]):
                dirs.append(seg["true_dir_body"])
                angles.append(r["angle_rad"])
        intrinsics.merge_report(cfg, "aoa",
                                analysis.fit_aoa_plane(dirs, angles))
        self.assertEqual(cfg["uwb_aoa"]["sign"], -1.0)

        # The merged config survives the firmware round trip.
        decoded = intrinsics.decode_blob(intrinsics.encode_blob(cfg))
        self.assertIn("imu", decoded)
        self.assertIn("uwb_aoa", decoded)

    def test_c_header(self):
        text = intrinsics.c_header(copy.deepcopy(self.config))
        self.assertIn("WIDGET_INTRINSICS_HAS_IMU 1", text)
        self.assertIn("WIDGET_INTRINSICS_UWB_RANGE", text)
        partial = intrinsics.empty()
        text2 = intrinsics.c_header(partial)
        self.assertIn("WIDGET_INTRINSICS_HAS_IMU 0", text2)

    def test_validation(self):
        bad = copy.deepcopy(self.config)
        bad["imu"]["gyro_bias"] = [1.0, 2.0]  # wrong width
        with self.assertRaises(ValueError):
            intrinsics.validate(bad)


if __name__ == "__main__":
    unittest.main()
