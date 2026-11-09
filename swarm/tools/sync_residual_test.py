"""sync_residual on synthetic logs: nodes with known clocks and positions."""

import unittest

import numpy as np

from swarm.tools import sync_residual as sr


def make_logs(pos, offsets, drifts_ppm, boot, ts_noise_m=0.0, seconds=60.0, rate=33.0, seed=1, drop=0.0):
    """Every node sends a POLL in turn (seq wrapping at 256, as on the air); every other
    node logs it. Device clocks: ts = (t_true * (1 + drift) + offset) in ticks, mod 2^40.
    `boot` is each node's log-clock offset (the `t` field), unrelated to its device time."""
    rng = np.random.default_rng(seed)
    ids = sorted(pos)
    logs = {i: ([], []) for i in ids}
    seq = {i: 0 for i in ids}
    n_frames = int(seconds * rate)
    for k in range(n_frames):
        s = ids[k % len(ids)]
        t_true = k / rate
        seq[s] = (seq[s] + 1) % 256
        dev = lambda n, t: int(round(((t * (1 + drifts_ppm[n] * 1e-6) + offsets[n]) / sr.TICK_S))) % sr.WRAP
        logs[s][0].append({"t": t_true + boot[s], "type": "tx", "to": 65535, "ft": 1, "seq": seq[s], "ts": dev(s, t_true)})
        for r in ids:
            if r == s or rng.random() < drop:
                continue
            tof = np.linalg.norm(np.subtract(pos[s], pos[r])) / sr.C
            noise = rng.normal(0, ts_noise_m) / sr.C
            logs[r][1].append({"t": t_true + tof + boot[r], "type": "rx", "from": s, "to": 65535, "ft": 1,
                               "seq": seq[s], "ts": dev(r, t_true + tof + noise), "clk": int(drifts_ppm[r] * 10), "rssi": -80.0,
                               "fp_rssi": -84.0})
    return logs


POS = {1: (0.0, 0.0, 0.0), 2: (6.0, 0.0, 0.0), 3: (0.0, 5.0, 1.0)}
OFF = {1: 0.0, 2: 3.7, 3: -11.2}
DRIFT = {1: 0.0, 2: 8.0, 3: -15.0}
BOOT = {1: 100.0, 2: 3.0, 3: 5000.0}


class SyncResidualTest(unittest.TestCase):
    def test_join_survives_the_sequence_wrap_and_unrelated_boot_clocks(self):
        logs = make_logs(POS, OFF, DRIFT, BOOT, seconds=30)
        tx, _ = logs[1]
        _, rx = logs[2]
        joined = sr.join(tx, [x for x in rx if x["from"] == 1])
        self.assertEqual(len(joined), len(tx))                   # all matched ...
        self.assertTrue(all(abs((r["t"] - t["t"]) - (BOOT[2] - BOOT[1])) < 0.05 for t, r in joined))   # ... to the right frame

    def test_noise_free_clocks_leave_no_residual(self):
        res = sr.analyze(make_logs(POS, OFF, DRIFT, BOOT), POS, windows=(1.0, 5.0))
        self.assertEqual(len(res), 6)
        for row in res.values():
            self.assertLess(row[1.0], 0.005)
            self.assertLess(row[5.0], 0.005)

    def test_drift_is_recovered(self):
        res = sr.analyze(make_logs(POS, OFF, DRIFT, BOOT), POS, windows=(5.0,))
        self.assertAlmostEqual(res[(1, 2)]["drift_ppm"], 8.0, delta=0.2)      # r - s
        self.assertAlmostEqual(res[(2, 3)]["drift_ppm"], -23.0, delta=0.2)

    def test_timestamp_noise_shows_up_as_the_residual(self):
        res = sr.analyze(make_logs(POS, OFF, DRIFT, BOOT, ts_noise_m=0.10), POS, windows=(2.0,))
        for row in res.values():
            self.assertAlmostEqual(row[2.0], 0.10, delta=0.025)

    def test_dropped_frames_and_the_40_bit_wrap(self):
        # 60 s covers three wraps of the 17.2 s device clock
        res = sr.analyze(make_logs(POS, OFF, DRIFT, BOOT, ts_noise_m=0.05, drop=0.3, seconds=60), POS, windows=(2.0,))
        self.assertEqual(len(res), 6)
        for row in res.values():
            self.assertAlmostEqual(row[2.0], 0.05, delta=0.02)

    def test_wrong_survey_is_a_constant_not_a_residual(self):
        # a 0.3 m survey error is an offset: absorbed by the fitted line, as it must be
        bad = dict(POS)
        bad[2] = (6.3, 0.0, 0.0)
        res = sr.analyze(make_logs(POS, OFF, DRIFT, BOOT), bad, windows=(2.0,))
        self.assertLess(res[(1, 2)][2.0], 0.005)


if __name__ == "__main__":
    unittest.main()
