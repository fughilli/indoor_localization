"""Tests for the serial capture tool's line parsing and manifest writing
(the serial loop itself needs hardware)."""

import json
import os
import tempfile
import unittest

from swarm.tools import capture, records


class ParseLineTest(unittest.TestCase):
    def test_schema_record(self):
        rec = capture.parse_line(b'{"t":1.25,"type":"range","peer":1,"range_m":2.0}\r\n')
        self.assertEqual(rec["range_m"], 2.0)

    def test_rejects_non_records(self):
        for raw in [b"[INFO] PLL is now locked.\n", b"+++ DecaWave DWM3000 Test +++",
                    b'{"t":1.0,"type":"ran', b'{"type":"range"}', b"\xff\xfe garbage", b""]:
            self.assertIsNone(capture.parse_line(raw), raw)


class AppendSegmentTest(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.path = os.path.join(self.dir.name, "m.json")

    def tearDown(self):
        self.dir.cleanup()

    def test_creates_and_appends(self):
        capture.append_segment(self.path, {"t0": 0.0, "t1": 10.0, "true_range_m": 1.0})
        capture.append_segment(self.path, {"t0": 20.0, "t1": 30.0, "true_range_m": 2.0})
        manifest = records.load_manifest(self.path)
        self.assertEqual([s["true_range_m"] for s in
                          records.segments_with(manifest, "true_range_m")], [1.0, 2.0])

    def test_rejects_overlap(self):
        capture.append_segment(self.path, {"t0": 5.0, "t1": 10.0, "true_range_m": 1.0})
        with self.assertRaises(ValueError):
            capture.append_segment(self.path, {"t0": 2.0, "t1": 6.0, "true_range_m": 2.0})
        with open(self.path) as f:
            self.assertEqual(len(json.load(f)["segments"]), 1)


class SummaryTest(unittest.TestCase):
    def test_line(self):
        s = capture.Summary()
        for r in (1.0, 1.1, 1.2):
            s.add({"t": 0, "type": "range", "range_m": r})
        s.add({"t": 0, "type": "stats", "ok": 3, "timeout": 2, "rx_error": 0, "protocol": 0})
        self.assertIn("n=  3", s.line())
        self.assertIn("median=1.100", s.line())
        self.assertIn("timeout=2", s.line())
        self.assertNotIn("rx_error", s.line())


if __name__ == "__main__":
    unittest.main()
