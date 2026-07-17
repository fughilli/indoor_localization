"""End-to-end swarm convergence tests (fixed seed, thresholds ~3x the
typical results so noise-model tweaks don't cause flakes).

Typical numbers at seed 1234 (90 s runs): surveyed-anchor scenario reaches
0.07-0.10 m tail RMSE on every mobile widget; the GPS scenario is bounded by
GPS noise at 0.7-2.7 m.
"""

import unittest

from swarm.sim import run_sim, world
from swarm.tools import intrinsics


class IntrinsicsMappingTest(unittest.TestCase):
    def test_apply_intrinsics_overrides_defaults(self):
        cfg = intrinsics.empty("t")
        cfg["imu"] = {"accel_noise_density": 0.008}
        cfg["uwb_range"] = {"sigma_m": 0.25, "outlier_frac": 0.11}
        cfg["alt"] = {"sigma_m": 0.7}
        sensors = run_sim.apply_intrinsics(world.SensorConfig(), cfg)
        self.assertAlmostEqual(sensors.accel_sigma, 0.008 * 10.0, places=6)
        self.assertAlmostEqual(sensors.uwb_range_sigma, 0.25)
        self.assertAlmostEqual(sensors.uwb_multipath_prob, 0.11)
        self.assertAlmostEqual(sensors.alt_sigma, 0.7)
        # Untouched fields keep their defaults.
        self.assertAlmostEqual(sensors.gps_sigma_h,
                               world.SensorConfig().gps_sigma_h)


class SwarmConvergenceTest(unittest.TestCase):
    def test_surveyed_anchors_reach_cm_level(self):
        report, _ = run_sim.run(n_widgets=4, n_gps=0, n_surveyed=3,
                                seconds=30.0, seed=1234)
        mobile = [w for w in report["widgets"] if not w["surveyed"]]
        self.assertEqual(len(mobile), 4)
        for w in mobile:
            self.assertIsNotNone(w["converge_s"], "widget %d never converged" % w["id"])
            self.assertLess(w["tail_rmse_m"], 0.3,
                            "widget %d tail RMSE %.3f" % (w["id"], w["tail_rmse_m"]))

    def test_gps_swarm_stays_bounded(self):
        report, _ = run_sim.run(n_widgets=5, n_gps=2, seconds=30.0, seed=1234)
        for w in report["widgets"]:
            self.assertLess(w["tail_rmse_m"], 8.0,
                            "widget %d tail RMSE %.3f" % (w["id"], w["tail_rmse_m"]))
        gps = [w for w in report["widgets"] if w["gps"]]
        for w in gps:
            self.assertLess(w["tail_rmse_m"], 4.0)

    def test_anchorless_swarm_converges_relatively(self):
        # No absolute-position source anywhere: absolute position is
        # unobservable (gauge pinned only by the zero-velocity prior), but
        # the swarm must converge to a tight *relative* mesh whose global
        # orientation is grounded by AoA + mag, and covariance must stay
        # honest about the absolute frame. Typical: ~0.5 m relative RMSE.
        report, world_obj = run_sim.run(n_widgets=6, n_gps=0, seconds=45.0,
                                        seed=1234)
        for w in report["widgets"]:
            self.assertLess(w["tail_rel_rmse_m"], 1.2,
                            "widget %d relative RMSE %.3f"
                            % (w["id"], w["tail_rel_rmse_m"]))
            # Translation-aligned ~ rotation-aligned: orientation is pinned.
            self.assertLess(w["tail_rel_rmse_m"],
                            3.0 * w["tail_rot_rel_rmse_m"] + 0.5)
        for w in world_obj.widgets:
            var = w.est.state()["pos_var"]
            self.assertGreater(var, 1.0)   # still admits absolute ignorance
            self.assertLess(var, 1e8)      # but hasn't blown up


if __name__ == "__main__":
    unittest.main()
