"""End-to-end swarm convergence tests (fixed seed, thresholds ~3x the
typical results so noise-model tweaks don't cause flakes).

Typical numbers at seed 1234 (90 s runs): surveyed-anchor scenario reaches
0.07-0.10 m tail RMSE on every mobile widget; the GPS scenario is bounded by
GPS noise at 0.7-2.7 m.
"""

import unittest

import numpy as np

from swarm.sim import run_sim, world

import widget_config_pb2 as pb


def _rel_errors(w, wids=None):
    """Centroid-aligned position errors for the given widget ids (all if
    None) — the relative metric for anchorless swarms."""
    sel = [x for x in w.widgets if wids is None or x.spec.wid in wids]
    est = np.array([x.est_state(w.t)["p"] for x in sel])
    true = np.array([x.truth.pos(w.t) for x in sel])
    off = (est - true).mean(axis=0)
    return np.linalg.norm((est - off) - true, axis=1)


class IntrinsicsMappingTest(unittest.TestCase):
    def test_apply_intrinsics_overrides_defaults(self):
        cfg = pb.WidgetConfig()
        cfg.intrinsics.imu.accel_noise_density = 0.008
        cfg.intrinsics.uwb_range.sigma_m = 0.25
        cfg.intrinsics.uwb_range.outlier_frac = 0.11
        cfg.intrinsics.alt.sigma_m = 0.7
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


class MotionTest(unittest.TestCase):
    def _orbit(self, **kw):
        base = dict(wid=0, kind="orbit", center=(0.0, 0.0, 1.0),
                    radius=5.0, omega=0.5, z_amp=0.3, yaw_rate=0.0)
        base.update(kw)
        return world.TruthWidget(world.WidgetSpec(**base))

    def test_continuous_motion_unchanged(self):
        # Defaults (rate 1, no pauses) must warp time to identity.
        spec = world.WidgetSpec(wid=0)
        self.assertAlmostEqual(world._motion_time(spec, 3.7), 3.7, places=6)

    def test_motion_rate_scales_speed(self):
        slow = self._orbit(motion_rate=1.0)
        fast = self._orbit(motion_rate=2.0)
        # 2x rate = 2x angle swept = 2x path length over the same window.
        def path_len(tw, t1, dt=0.02):
            steps = int(t1 / dt)
            return sum(float(np.linalg.norm(tw.pos((i + 1) * dt) - tw.pos(i * dt)))
                       for i in range(steps))
        self.assertAlmostEqual(path_len(fast, 5.0) / path_len(slow, 5.0),
                               2.0, delta=0.05)

    def test_pauses_stop_the_device(self):
        # move_period 4 s, 50% duty: moving ~[0,2), paused ~[2,4).
        tw = self._orbit(move_period=4.0, move_duty=0.5)
        plateau_v = np.linalg.norm(tw.vel(1.0))
        pause_v = np.linalg.norm(tw.vel(3.0))
        self.assertGreater(plateau_v, 1.0)          # genuinely moving
        self.assertLess(pause_v, 0.02 * plateau_v)  # genuinely stopped
        # And it holds position across the whole pause.
        self.assertLess(np.linalg.norm(tw.pos(3.5) - tw.pos(2.5)), 0.05)

    def test_pause_transitions_have_no_accel_impulse(self):
        # Linear speed ramps => continuous velocity => bounded numerical
        # acceleration (an instantaneous stop would spike to ~1e3 m/s^2).
        tw = self._orbit(move_period=4.0, move_duty=0.5)
        max_acc = max(np.linalg.norm(tw.acc(t))
                      for t in np.arange(0.0, 8.0, 0.005))
        self.assertLess(max_acc, 50.0, "accel impulse at a transition")

    def test_swarm_converges_with_pauses(self):
        # End to end: a stop-and-go anchorless swarm must still converge
        # relatively (the zero-velocity prior should help, if anything).
        report, _ = run_sim.run(n_widgets=6, n_gps=0, seconds=45.0, seed=1234,
                                move_period=6.0, move_duty=0.5)
        for w in report["widgets"]:
            self.assertLess(w["tail_rel_rmse_m"], 1.2,
                            "widget %d rel RMSE %.3f with pauses"
                            % (w["id"], w["tail_rel_rmse_m"]))


class GainAnnealingTest(unittest.TestCase):
    def test_annealing_cuts_postlock_jitter(self):
        # Static anchorless swarm: without annealing every mean-only range
        # keeps kicking the state at relaxation gain (~2/3) forever and the
        # converged mesh visibly jitters (~0.5 m/frame at seed 1234). With
        # the innovation-annealed gain the hold is handed to the IMU
        # (~0.16 m/frame). Thresholds ~2x typical.
        annealed, _ = run_sim.run(n_widgets=6, n_gps=0, seconds=40.0,
                                  seed=1234, static=True)
        baseline, _ = run_sim.run(n_widgets=6, n_gps=0, seconds=40.0,
                                  seed=1234, static=True,
                                  sensor_overrides={
                                      "mean_only_range_damping": 0.0})
        for w in annealed["widgets"]:
            self.assertLess(w["tail_jitter_m"], 0.35,
                            "widget %d jitter %.3f" % (w["id"],
                                                       w["tail_jitter_m"]))
        jit_a = np.mean([w["tail_jitter_m"] for w in annealed["widgets"]])
        jit_b = np.mean([w["tail_jitter_m"] for w in baseline["widgets"]])
        self.assertLess(jit_a, 0.6 * jit_b,
                        "annealed %.3f vs baseline %.3f" % (jit_a, jit_b))
        # Annealing must not cost relative accuracy.
        rel_a = np.mean([w["tail_rel_rmse_m"] for w in annealed["widgets"]])
        rel_b = np.mean([w["tail_rel_rmse_m"] for w in baseline["widgets"]])
        self.assertLess(rel_a, rel_b + 0.1,
                        "annealed rel %.3f vs baseline %.3f" % (rel_a, rel_b))
        # And the lock signal itself must report convergence.
        for w in annealed["widgets"]:
            self.assertLess(w["lock"], 4.0, "widget %d lock %.1f"
                            % (w["id"], w["lock"]))


class JoinLeaveTest(unittest.TestCase):
    def test_cold_join_pulls_in_without_shaking_mesh(self):
        # A widget that "doesn't know anything" joins a converged anchorless
        # mesh: it must be pulled into the mesh, and the veterans must not
        # be dragged around by its garbage advertisements (the peer-lock
        # rule — absolute variance cannot tell a cold joiner from an
        # anchorless veteran, both are honestly huge).
        specs = world.default_swarm(n=5, n_gps=0)
        veteran_ids = [s.wid for s in specs]
        w = world.World(specs, seed=1234)
        w.run(30.0)
        pre = _rel_errors(w, veteran_ids).max()
        worst = [0.0]

        def watch(frame):
            worst[0] = max(worst[0], _rel_errors(w, veteran_ids).max())

        w.add_widget(world.WidgetSpec(
            wid=99, kind="orbit", center=(15.0, 15.0, 1.5),
            radius=4.0, omega=0.15, phase=1.0, z_amp=0.3, yaw_rate=0.08))
        joiner = w.widgets[-1]
        self.assertGreater(joiner.est.state()["pos_var"], 1e3)  # knows nothing
        w.run(25.0, frame_cb=watch)
        # Veterans: bounded disturbance during pull-in.
        self.assertLess(worst[0], max(3.0 * pre, 1.5),
                        "veteran rel err spiked to %.2f (pre-join %.2f)"
                        % (worst[0], pre))
        # Joiner: converged into the mesh and locked.
        all_ids = veteran_ids + [99]
        joiner_rel = _rel_errors(w, all_ids)[-1]
        self.assertLess(joiner_rel, 1.5,
                        "joiner rel err %.2f" % joiner_rel)
        self.assertLess(joiner.est.range_innov_ratio(), 6.0)

    def test_anchor_loss_grows_variance_honestly(self):
        # The mesh converges on 3 surveyed anchors, then loses all of them:
        # the survivors must keep a tight relative mesh (annealed hold) while
        # their absolute covariance honestly grows back toward "anchorless".
        specs = world.default_swarm(n=4, n_gps=0, n_surveyed=3)
        w = world.World(specs, seed=1234)
        w.run(25.0)
        mobile_ids = [s.wid for s in specs if not s.surveyed]
        var_before = {x.spec.wid: x.est.state()["pos_var"]
                      for x in w.widgets if not x.spec.surveyed}
        for s in specs:
            if s.surveyed:
                w.remove_widget(s.wid)
        self.assertEqual(len(w.widgets), 4)
        w.run(20.0)
        for x in w.widgets:
            wid = x.spec.wid
            self.assertGreater(x.est.state()["pos_var"], var_before[wid],
                               "widget %d variance froze after anchor loss"
                               % wid)
        rel = _rel_errors(w, mobile_ids)
        self.assertLess(rel.max(), 1.0,
                        "relative mesh fell apart: %.2f m" % rel.max())

    def test_stale_advertisement_is_ignored(self):
        # A widget that goes silent (no broadcasts) must age out: peers stop
        # taking updates against its dead-reckoned ghost after
        # max_advert_age.
        specs = [world.WidgetSpec(wid=0, kind="static", center=(10, 15, 1)),
                 world.WidgetSpec(wid=1, kind="static", center=(20, 15, 1))]
        w = world.World(specs, seed=1234)
        w.widgets[1].broadcast = lambda t: None  # silent from the start
        w.run(1.5)
        after_age = (w.widgets[0].accepted_ranges
                     + w.widgets[0].rejected_ranges)
        self.assertGreater(after_age, 0)  # fresh advert was used at first
        w.run(2.0)
        self.assertEqual(after_age,
                         w.widgets[0].accepted_ranges
                         + w.widgets[0].rejected_ranges,
                         "peer kept ranging against a stale advertisement")


if __name__ == "__main__":
    unittest.main()


class RoomScenarioTest(unittest.TestCase):
    """The bench, through the firmware's node runtime (backend "node"):
    three still widgets plus one carried around the room, MPU6050-class IMU
    (scale errors, bias drift, walking sway and bob), per-device range bias.
    Typical at seed 1: still widgets auto-anchor at 2-6 cm aligned error and
    ~4 mm/frame jitter; the walker tracks at ~11 cm with ~1 cm/frame."""

    def _run(self, scenario, seconds, seed=1, **settings):
        report, _ = run_sim.run(4, 0, seconds, seed=seed, scenario=scenario,
                                node_set={k: v for k, v in settings.items()})
        by_kind = {}
        for wg in report["widgets"]:
            by_kind.setdefault(wg["kind"], []).append(wg)
        return by_kind

    def test_still_room_holds_still(self):
        r = self._run("room_static", 90)
        for wg in r["static"]:
            self.assertTrue(wg["anchored"], wg)
            self.assertLess(wg["tail_rot_rel_rmse_m"], 0.2, wg)
            self.assertLess(wg["tail_rot_jitter_m"], 0.01, wg)

    def test_walker_tracks_against_auto_anchors(self):
        r = self._run("room", 120)
        for wg in r["static"]:
            self.assertTrue(wg["anchored"], wg)
            self.assertLess(wg["tail_rot_rel_rmse_m"], 0.2, wg)
        walker = r["orbit"][0]
        self.assertLess(walker["tail_rot_rel_rmse_m"], 0.3, walker)
        self.assertLess(walker["tail_rot_jitter_m"], 0.03, walker)

    def test_auto_anchoring_beats_anchorless_for_the_walker(self):
        on = [self._run("room", 120, seed=s)["orbit"][0]["tail_rot_rel_rmse_m"] for s in (1, 2)]
        off = [self._run("room", 120, seed=s, **{"auto_anchor.on": False})["orbit"][0]["tail_rot_rel_rmse_m"]
               for s in (1, 2)]
        self.assertLess(sum(on), 0.8 * sum(off), (on, off))
