"""Round-trip tests for the trilateration solver."""

import math
import unittest

import numpy as np

from host import solver


class TrilaterateTest(unittest.TestCase):

    def ranges(self, pos, anchors):
        return [math.hypot(pos[0] - a[0], pos[1] - a[1]) for a in anchors]

    def test_exact_roundtrip(self):
        anchors = solver.DEFAULT_ANCHORS
        for truth in [(100.0, 200.0), (15.0, 5.0), (240.0, 325.0), (400.0, 600.0)]:
            got = solver.trilaterate(self.ranges(truth, anchors))
            self.assertIsNotNone(got, truth)
            np.testing.assert_allclose(got, truth, atol=1e-6)

    def test_noisy_roundtrip(self):
        anchors = solver.DEFAULT_ANCHORS
        truth = (180.0, 300.0)
        rng = np.random.default_rng(42)
        exact = self.ranges(truth, anchors)
        for _ in range(20):
            noisy = [d + rng.normal(0.0, 2.0) for d in exact]  # 2 cm stddev
            got = solver.trilaterate(noisy)
            self.assertIsNotNone(got)
            self.assertLess(np.hypot(*(got - np.asarray(truth))), 15.0)

    def test_custom_anchors(self):
        anchors = [[0.0, 0.0], [100.0, 0.0], [50.0, 100.0]]
        truth = (50.0, 40.0)
        got = solver.trilaterate(self.ranges(truth, anchors), anchors)
        np.testing.assert_allclose(got, truth, atol=1e-6)

    def test_bad_input(self):
        self.assertIsNone(solver.trilaterate([100.0, 100.0]))  # too few ranges


if __name__ == "__main__":
    unittest.main()
