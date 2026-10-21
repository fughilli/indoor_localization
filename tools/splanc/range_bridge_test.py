"""Filter + mapping tests for range_bridge (no hardware):
  python3 -m unittest tools/splanc/range_bridge_test.py -v

The filter test simulates a walk with the ranging noise measured on the mule
pair (sigma ~2.4 cm at 20 Hz) plus multipath outliers, and compares the
One Euro chain against the 1 s median it replaced."""

import os
import random
import statistics
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import range_bridge  # noqa: E402

HZ, SIGMA, OUTLIER_P = 20.0, 0.024, 0.05


def walk(seed=1):
    """(t, true_m, measured_m): 1 m for 5 s, 1 m/s to 3 m, 3 m for 5 s."""
    rng = random.Random(seed)
    out, t = [], 0.0
    while t < 12.0:
        true = 1.0 if t < 5 else min(3.0, 1.0 + (t - 5))
        meas = true + rng.gauss(0, SIGMA)
        if rng.random() < OUTLIER_P:
            meas += rng.expovariate(1 / 0.3)  # multipath only ever adds path
        out.append((round(t, 3), true, meas))
        t += 1 / HZ
    return out


def run(filter_fn, samples):
    return [(t, true, filter_fn(t, m)) for t, true, m in samples]


def old_filter():
    window = []

    def f(t, r):
        window.append((t, r))
        window[:] = [(tt, rr) for tt, rr in window if tt > t - 1.0]
        return statistics.median(rr for _, rr in window)
    return f


def metrics(trace):
    moving = [abs(est - true) for t, true, est in trace if 5.2 < t < 7.0]
    rest = [est for t, true, est in trace if t > 9.0]
    def settle(band):  # s after arrival until it stays within band of 3 m
        return next(t for t, true, est in trace if t > 7.0 and all(
            abs(e - 3.0) < band for tt, _, e in trace if tt >= t)) - 7.0
    return {"lag_err_m": statistics.mean(moving), "reach10_s": settle(0.10),
            "settle_s": settle(0.03), "jitter_m": statistics.pstdev(rest)}


class FilterTest(unittest.TestCase):
    def test_more_responsive_and_still_steady(self):
        old = statistics.mean([metrics(run(old_filter(), walk(s)))["lag_err_m"] for s in range(10)])
        news = [metrics(run(range_bridge.RangeFilter(), walk(s))) for s in range(10)]
        olds = [metrics(run(old_filter(), walk(s))) for s in range(10)]
        mean = lambda ms, k: statistics.mean(m[k] for m in ms)
        for name, ms in (("1 s median", olds), ("0.3 s med+1euro", news)):
            print("%-14s lag err walking %.3f m  within 10 cm %.2f s  within 3 cm %.2f s  "
                  "jitter at rest %.4f m" % (name, mean(ms, "lag_err_m"), mean(ms, "reach10_s"),
                                             mean(ms, "settle_s"), mean(ms, "jitter_m")))
        self.assertLess(mean(news, "lag_err_m"), old / 2.5)
        # What the lamp shows: the colour is near its final value much sooner;
        # the last few cm settle about as fast as before (not faster).
        self.assertLess(mean(news, "reach10_s"), mean(olds, "reach10_s") / 2)
        self.assertLess(mean(news, "settle_s"), 1.0)
        # Jitter at rest must stay below what the lamp shows: hue moves 0.66
        # over 4.7 m, and the bridge ignores hue steps under 0.004 (~2.8 cm).
        self.assertLess(mean(news, "jitter_m"), 0.012)

    def test_spike_rejected(self):
        f = range_bridge.RangeFilter()
        for i in range(40):
            f(i / HZ, 2.0)
        self.assertLess(abs(f(2.0, 4.0) - 2.0), 0.01)  # one multipath spike


class MappingTest(unittest.TestCase):
    def test_ends_and_clamp(self):
        m = lambda d: range_bridge.map_range(d, 0.3, 5.0, 4.0, 0.4)
        self.assertEqual(m(0.0), m(0.3))
        self.assertEqual(m(0.3)["rate"], 4.0)
        self.assertEqual(m(9.0), m(5.0))
        self.assertAlmostEqual(m(5.0)["hue"], 0.66)


if __name__ == "__main__":
    unittest.main()
