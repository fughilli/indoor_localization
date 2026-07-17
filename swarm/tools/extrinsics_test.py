"""Extrinsics schema tests: derivation math, blob round-trip, header gen,
validation errors."""

import copy
import math
import os
import unittest

import numpy as np

from swarm.tools import extrinsics

EXAMPLE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "examples", "widget_v1.json")


class ExtrinsicsTest(unittest.TestCase):
    def setUp(self):
        self.config = extrinsics.load(EXAMPLE)

    def test_example_derives_expected_quats(self):
        cfg = extrinsics.derive(copy.deepcopy(self.config))
        self.assertEqual(cfg["sensors"]["imu"]["rotation_wxyz"],
                         [1.0, 0.0, 0.0, 0.0])
        # mag: +90 deg about z.
        w, x, y, z = cfg["sensors"]["mag"]["rotation_wxyz"]
        self.assertAlmostEqual(w, math.cos(math.pi / 4), places=5)
        self.assertAlmostEqual(z, math.sin(math.pi / 4), places=5)
        self.assertAlmostEqual(x, 0.0, places=6)

    def test_euler_trim_composes(self):
        cfg = {"mapping": {"x": "+x", "z": "+z"}, "euler_deg": [0, 0, 30]}
        r = extrinsics.rotation_matrix(cfg)
        v = r @ np.array([1.0, 0.0, 0.0])
        self.assertAlmostEqual(float(v[0]), math.cos(math.radians(30)), places=6)
        self.assertAlmostEqual(float(v[1]), math.sin(math.radians(30)), places=6)

    def test_blob_round_trip(self):
        blob = extrinsics.encode_blob(copy.deepcopy(self.config))
        decoded = extrinsics.decode_blob(blob)
        self.assertEqual(len(decoded), 5)
        by_id = {d["id"]: d for d in decoded}
        np.testing.assert_allclose(by_id[1]["offset_m"],
                                   [0.012, -0.003, 0.001], atol=1e-6)
        np.testing.assert_allclose(by_id[3]["rotation_wxyz"],
                                   [0.707107, 0, 0, -0.707107], atol=1e-5)
        # Corruption must be detected.
        bad = bytearray(blob)
        bad[10] ^= 0xFF
        with self.assertRaises(ValueError):
            extrinsics.decode_blob(bytes(bad))

    def test_c_header(self):
        text = extrinsics.c_header(copy.deepcopy(self.config))
        self.assertIn("widget_extrinsic_t", text)
        self.assertIn("WIDGET_EXTRINSICS_SENSOR_COUNT 5", text)
        self.assertIn("// uwb", text)

    def test_validation_errors(self):
        bad = copy.deepcopy(self.config)
        bad["sensors"]["imu"]["mapping"] = {"x": "+x", "z": "+x"}
        with self.assertRaises(ValueError):
            extrinsics.validate(bad)
        bad2 = copy.deepcopy(self.config)
        bad2["sensors"]["lidar"] = {"offset_m": [0, 0, 0]}  # no id
        with self.assertRaises(ValueError):
            extrinsics.validate(bad2)
        ok = copy.deepcopy(self.config)
        ok["sensors"]["lidar"] = {"id": 130, "offset_m": [0, 0, 0]}
        extrinsics.derive(ok)  # custom sensor with id is fine


if __name__ == "__main__":
    unittest.main()
