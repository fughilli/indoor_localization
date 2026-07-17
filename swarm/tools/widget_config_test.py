"""widget.v1 config tests: derivation math, textproto/dict/binary round
trips, characterize-report merging, C header gen, validation errors."""

import math
import os
import tempfile
import unittest

import numpy as np

from swarm.tools import analysis, records, synth, widget_config

import widget_config_pb2 as pb

EXAMPLE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "examples", "widget_v1.txtpb")


def sensor(cfg, name):
    return next(s for s in cfg.extrinsics.sensors if s.name == name)


class WidgetConfigTest(unittest.TestCase):
    def setUp(self):
        self.config = widget_config.load(EXAMPLE)

    def test_example_derives_expected_quats(self):
        cfg = widget_config.derive(self.config)
        imu = sensor(cfg, "imu").rotation
        self.assertAlmostEqual(imu.w, 1.0, places=5)
        mag = sensor(cfg, "mag").rotation  # +90 deg about z
        self.assertAlmostEqual(mag.w, math.cos(math.pi / 4), places=5)
        self.assertAlmostEqual(mag.z, math.sin(math.pi / 4), places=5)

    def test_euler_trim_composes(self):
        s = pb.SensorExtrinsic(id=1, name="imu")
        s.mapping.x, s.mapping.z = "+x", "+z"
        s.euler_deg.z = 30.0
        r = widget_config.rotation_matrix(s)
        v = r @ np.array([1.0, 0.0, 0.0])
        self.assertAlmostEqual(float(v[1]), math.sin(math.radians(30)), places=6)

    def test_textproto_and_binary_round_trip(self):
        path = os.path.join(tempfile.mkdtemp(), "cfg.txtpb")
        widget_config.save(widget_config.derive(self.config), path)
        cfg2 = widget_config.load(path)
        self.assertEqual(cfg2.extrinsics.widget_model, "widget-v1")
        blob = widget_config.encode_binary(cfg2)
        cfg3 = pb.WidgetConfig.FromString(blob)
        self.assertAlmostEqual(sensor(cfg3, "uwb").offset_m.y, 0.021, places=6)
        self.assertAlmostEqual(cfg3.intrinsics.mag.soft_iron[4], 0.875,
                               places=5)
        # Optional-field presence survives.
        self.assertTrue(cfg3.intrinsics.imu.HasField("accel_noise_density"))
        self.assertFalse(pb.WidgetConfig().intrinsics.imu.HasField(
            "accel_noise_density"))

    def test_dict_round_trip(self):
        d = widget_config.to_dict(widget_config.derive(self.config))
        self.assertEqual(len(d["extrinsics"]["sensors"]), 5)
        cfg2 = widget_config.from_dict(d)
        self.assertAlmostEqual(sensor(cfg2, "mag").rotation.z, 0.707107,
                               places=5)

    def test_merge_from_characterization_pipeline(self):
        d = tempfile.mkdtemp()
        cap, man = os.path.join(d, "c.jsonl"), os.path.join(d, "m.json")
        cfg = pb.WidgetConfig()

        synth.make_imu_capture(cap, man, seed=1, seconds=200.0)
        imu = records.select(records.load_capture(cap), "imu")
        report = analysis.imu_static_report([r["accel"] for r in imu],
                                            [r["gyro"] for r in imu], 100.0)
        widget_config.merge_report(cfg, "imu", report)
        self.assertAlmostEqual(cfg.intrinsics.imu.accel_noise_density, 0.005,
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
        widget_config.merge_report(cfg, "aoa",
                                   analysis.fit_aoa_plane(dirs, angles))
        self.assertEqual(cfg.intrinsics.uwb_aoa.sign, -1.0)
        # Survives the firmware wire form.
        cfg2 = pb.WidgetConfig.FromString(widget_config.encode_binary(cfg))
        self.assertTrue(cfg2.intrinsics.HasField("uwb_aoa"))

    def test_c_array_header(self):
        text = widget_config.c_array_header(self.config)
        self.assertIn("WIDGET_CONFIG_PB[]", text)
        self.assertIn("WIDGET_CONFIG_PB_LEN", text)

    def test_validation_errors(self):
        cfg = widget_config.load(EXAMPLE)
        sensor(cfg, "imu").mapping.z = "-x"  # not orthogonal to +x
        with self.assertRaises(ValueError):
            widget_config.validate(cfg)
        cfg2 = widget_config.load(EXAMPLE)
        cfg2.extrinsics.sensors.add(id=7, name="lidar")  # bad custom id
        with self.assertRaises(ValueError):
            widget_config.validate(cfg2)
        cfg3 = widget_config.load(EXAMPLE)
        cfg3.extrinsics.sensors.add(id=130, name="lidar")
        widget_config.derive(cfg3)  # custom sensor with valid id is fine


if __name__ == "__main__":
    unittest.main()
