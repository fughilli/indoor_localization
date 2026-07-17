"""Round-trip test of the ctypes binding against the Rust core."""

import math
import unittest

from swarm.sim import estimator


class EstimatorBindingTest(unittest.TestCase):
    def test_state_round_trip(self):
        e = estimator.Estimator(gate=-1.0)
        e.set_pose([1.0, 2.0, 3.0], pos_sigma=2.0)
        s = e.state()
        self.assertAlmostEqual(s["p"][0], 1.0, places=4)
        self.assertAlmostEqual(s["p"][2], 3.0, places=4)
        self.assertAlmostEqual(s["pos_var"], 4.0, places=3)
        cov = e.pos_cov()
        self.assertAlmostEqual(cov[0][0], 4.0, places=3)
        self.assertAlmostEqual(cov[0][1], 0.0, places=4)

    def test_range_updates_converge(self):
        anchors = [[0, 0, 0], [10, 0, 0], [0, 10, 0], [10, 10, 3]]
        truth = [3.0, 4.0, 1.0]
        e = estimator.Estimator(gate=-1.0)
        e.set_pose([6.0, 7.0, 0.0], pos_sigma=10.0)
        gravity = [0.0, 0.0, 9.80665]
        for _ in range(30):
            e.predict(gravity, [0, 0, 0], 0.01)
            for a in anchors:
                d = math.dist(truth, a)
                self.assertTrue(e.update_range(a, 0.0, d, 0.1))
        err = math.dist(e.state()["p"], truth)
        self.assertLess(err, 0.05)

    def test_gate_rejects_outlier(self):
        e = estimator.Estimator()  # default gate
        e.set_pose([0, 0, 0], pos_sigma=0.5)
        self.assertFalse(e.update_range([5, 0, 0], 0.0, 50.0, 0.1))
        self.assertTrue(e.update_range([5, 0, 0], 0.0, 5.1, 0.1))


if __name__ == "__main__":
    unittest.main()
