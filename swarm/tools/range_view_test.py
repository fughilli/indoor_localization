"""Tests for the range viewer's hub: capture writing, segment recording, and
reboot handling (the serial reader and the page need hardware/a browser)."""

import json
import os
import tempfile
import unittest

from swarm.tools import range_view, records


def rng(t, r):
    return {"t": t, "type": "range", "peer": 1, "range_m": r, "rssi": -55.0, "fp_rssi": -56.0}


class HubTest(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.cap = os.path.join(self.dir.name, "c.jsonl")
        self.man = os.path.join(self.dir.name, "m.json")
        self.hub = range_view.Hub(self.cap, self.man)

    def tearDown(self):
        self.dir.cleanup()

    def test_segment_round_trip(self):
        self.hub.ingest(rng(1.0, 9.9))  # before recording: captured, not in segment
        self.hub.start_segment(2.0)
        for i, r in enumerate([2.01, 2.03, 2.02]):
            self.hub.ingest(rng(2.0 + 0.05 * i, r))
        seg = self.hub.stop_segment()
        self.assertEqual(seg["n"], 3)
        self.assertAlmostEqual(seg["bias_m"], 0.02)
        self.assertEqual(seg["t0"], 2.0)
        manifest = records.load_manifest(self.man)
        self.assertEqual(manifest["segments"][0]["true_range_m"], 2.0)
        caps = records.load_capture(self.cap)
        self.assertEqual(len(records.select(caps, "range", seg["t0"], seg["t1"])), 3)
        self.assertEqual(self.hub.state()["segments"][0]["n"], 3)

    def test_errors(self):
        with self.assertRaises(ValueError):
            self.hub.stop_segment()
        self.hub.start_segment(1.0)
        with self.assertRaises(ValueError):
            self.hub.start_segment(1.0)
        with self.assertRaises(ValueError):  # nothing captured
            self.hub.stop_segment()
        self.assertFalse(os.path.exists(self.man))

    def test_recording_needs_files(self):
        with self.assertRaises(ValueError):
            range_view.Hub().start_segment(1.0)

    def test_reboot_drops_open_segment(self):
        self.hub.ingest(rng(50.0, 1.0))
        self.hub.start_segment(1.0)
        self.hub.ingest(rng(50.1, 1.0))
        self.hub.ingest(rng(0.5, 1.0))  # clock restarted
        self.assertIsNone(self.hub.state()["recording"])

    def test_subscribers_get_records(self):
        q = self.hub.subscribe()
        first = [q.get_nowait()["type"], q.get_nowait()["type"]]
        self.assertEqual(first, ["status", "state"])
        self.hub.ingest(rng(1.0, 1.0))
        types = [q.get_nowait()["type"] for _ in range(q.qsize())]
        self.assertIn("range", types)
        self.hub.ingest({"t": 1.1, "type": "devid", "value": "0xDECA0302"})
        self.assertFalse(any(m["type"] == "devid" for m in
                             [q.get_nowait() for _ in range(q.qsize())]))


if __name__ == "__main__":
    unittest.main()
