"""Tests for the tracker ground-truth tools on synthetic sessions: truth in a
z-up tracker frame with lever arms, the swarm's estimate in its own rotated,
shifted and mirrored frame, ranges with a known per-link excess."""

import json
import math
import unittest

import numpy as np

from swarm.tools import truth_eval, vive_truth

NODES = {0x1CF0: (0.0, 0.0), 0xC248: (3.1, 1.2), 0xD274: (0.8, 3.4)}
WALKER = 0xF694
LEVER = np.array([0.0, 0.0, -0.05])  # antenna 5 cm below the tracker
T0 = 1.79e9


def walker_xy(t):
    return np.array([1.5 + 1.2 * math.cos(0.3 * t), 1.5 + 1.0 * math.sin(0.3 * t)])


def yaw_quat(yaw):
    return [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]


def session(est_noise=0.0, seconds=60.0, est_lag=0.0, truth_late=0.0, seed=1):
    """(capture lines, truth lines). The swarm frame: rotated 40 deg,
    shifted, mirrored in y; its estimate lags truth by `est_lag` s, and the
    tracker clock runs `truth_late` s behind the capture's."""
    rng = np.random.default_rng(seed)
    rot = np.array([[math.cos(0.7), -math.sin(0.7)], [math.sin(0.7), math.cos(0.7)]])

    def to_swarm(xy):
        v = rot @ xy + np.array([-2.0, 0.5])
        return v * np.array([1.0, -1.0])

    cap, tru = [], []
    for k in range(int(seconds * 50)):
        t = k / 50.0
        # Truth at 50 Hz: tracker 5 cm above the antenna, tilted walker.
        for nid, xy in list(NODES.items()) + [(WALKER, tuple(walker_xy(t)))]:
            q = yaw_quat(0.3 * t if nid == WALKER else 0.1)
            if nid == WALKER:  # pitched 30 deg: the lever arm then matters
                c, s = math.cos(0.26), math.sin(0.26)
                qy = [c, 0.0, s, 0.0]
                w1, x1, y1, z1 = q
                w2, x2, y2, z2 = qy
                q = [w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2]
            ant = np.array([xy[0], xy[1], 0.8])
            tracker = ant - truth_eval.quat_rotate(np.array(q), LEVER)
            tru.append(json.dumps({"ht": T0 + t - truth_late, "t": t, "type": "truth", "dev": "T%x" % nid,
                                   "node": nid, "p": list(tracker), "q": q}))
        if k % 5 == 0:  # swarm records at 10 Hz
            nodes = []
            for nid, xy in list(NODES.items()) + [(WALKER, None)]:
                xy = walker_xy(t - est_lag) if nid == WALKER else np.array(xy)
                p = to_swarm(xy) + rng.normal(0, est_noise, 2)
                nodes.append({"id": nid, "p": [float(p[0]), float(p[1]), 0.0]})
            nodes.append({"id": 0xBEEF, "p": [9.0, 9.0, 0.0]})  # untracked node
            cap.append(json.dumps({"ht": T0 + t, "t": 100 + t, "type": "swarm", "id": 0x1CF0,
                                   "nodes": nodes}))
            # The USB node's ranges: clean to the walker, +0.3 m blocked to C248.
            for peer, excess, gap in ((WALKER, 0.0, 2.0), (0xC248, 0.3, 21.0)):
                pxy = walker_xy(t) if peer == WALKER else np.array(NODES[peer])
                d = float(np.linalg.norm(pxy - np.array(NODES[0x1CF0])))
                cap.append(json.dumps({"ht": T0 + t, "t": 100 + t, "type": "range", "peer": peer,
                                       "range_m": d + excess, "rssi": -80.0, "fp_rssi": -80.0 - gap}))
    return cap, tru


def run(cap, tru, levers=None, **kw):
    snaps, ranges, self_id = truth_eval.load_capture(cap)
    truth = truth_eval.Truth(truth_eval.load_truth(tru), levers, **kw)
    return truth_eval.evaluate(snaps, ranges, self_id, truth), snaps, truth


class EvalTest(unittest.TestCase):
    def test_exact_estimate_scores_zero_in_any_frame(self):
        rep, _, _ = run(*session(), levers={WALKER: LEVER})
        self.assertEqual(rep["up_axis"], "z")
        self.assertTrue(rep["global_mirrored"])
        for table in ("global", "snapshot", "pair"):
            for k, s in rep[table].items():
                self.assertLess(s["rms"], 1e-3, (table, k, s))
        self.assertNotIn("beef", rep["global"])

    def test_noise_shows_up_at_its_size(self):
        rep, _, _ = run(*session(est_noise=0.05), levers={WALKER: LEVER})
        for k, s in rep["global"].items():
            self.assertTrue(0.04 < s["rms"] < 0.09, (k, s))

    def test_lever_arm_matters_for_a_tilted_walker(self):
        cap, tru = session()
        without, _, _ = run(cap, tru)
        self.assertGreater(without["global"]["f694"]["rms"], 0.005)

    def test_ranges_binned_by_gap(self):
        rep, _, _ = run(*session(), levers={WALKER: LEVER})
        bins = {b["gap_db"][0]: b for b in rep["ranges"]["by_gap"]}
        # (Linear interpolation of a 50 Hz track on a curve: < 1 mm.)
        self.assertAlmostEqual(bins[0.0]["median"], 0.0, delta=0.002)
        self.assertAlmostEqual(bins[20.0]["median"], 0.3, delta=0.002)
        self.assertAlmostEqual(rep["ranges"]["by_peer"]["c248"]["gap_median"], 21.0)

    def test_fit_dt_recovers_a_clock_offset_not_filter_lag(self):
        cap, tru = session(est_lag=0.3, truth_late=0.2)
        snaps, ranges, self_id = truth_eval.load_capture(cap)
        truth = truth_eval.Truth(truth_eval.load_truth(tru), {WALKER: LEVER})
        dt = truth_eval.fit_dt(snaps, ranges, self_id, truth)
        self.assertAlmostEqual(dt, 0.2, delta=0.02)
        # The lag is still there to see: 0.3 s at ~0.35 m/s.
        truth.dt = dt
        rep = truth_eval.evaluate(snaps, ranges, self_id, truth)
        self.assertGreater(rep["global"]["f694"]["rms"], 0.05)

    def test_report_formats(self):
        rep, _, _ = run(*session(), levers={WALKER: LEVER})
        text = truth_eval.format_report(rep)
        self.assertIn("f694", text)
        self.assertIn("20-25", text)

    def test_unstamped_capture_is_refused(self):
        with self.assertRaises(ValueError):
            truth_eval.load_capture(['{"t":1.0,"type":"swarm","nodes":[]}'])


class ViveTruthTest(unittest.TestCase):
    def test_parse_survive_pose_line(self):
        t, dev, p, q = vive_truth.parse_survive_line(
            "12.500000 T20 POSE 0.100000 -0.200000 1.300000 1.000000 0.000000 0.000000 0.000000\r\n")
        self.assertEqual((t, dev, p, q), (12.5, "T20", [0.1, -0.2, 1.3], [1.0, 0.0, 0.0, 0.0]))
        for line in ["12.5 T20 IMU 1 2 3", "12.5 LH0 LH_POSE 1 2 3 4 5 6 7 0", "garbage", ""]:
            self.assertIsNone(vive_truth.parse_survive_line(line))

    def test_map_accepts_decimal_and_hex(self):
        self.assertEqual(vive_truth.parse_map(["T20=63124", "T21=0x1CF0", "LHR-1=C248"]),
                         {"T20": 63124, "T21": 0x1CF0, "LHR-1": 0xC248})

    def test_recorder_decimates_and_maps(self):
        out = []
        rec = vive_truth.Recorder(node_map={"T20": 63124}, rate=10.0, clock=lambda: 5.0)
        rec.write = out.append
        for k in range(100):  # 100 Hz for 1 s
            rec.pose(k / 100.0, "T20", [0, 0, 0], [1, 0, 0, 0])
        self.assertEqual(len(out), 10)
        self.assertEqual(out[0]["node"], 63124)
        self.assertEqual(out[0]["ht"], 5.0)
        # And the evaluator reads what the recorder writes.
        tracks = truth_eval.load_truth(json.dumps(r) for r in out)
        self.assertEqual(len(tracks[63124][0]), 10)


if __name__ == "__main__":
    unittest.main()
