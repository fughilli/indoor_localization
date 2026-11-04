"""Tests for the multipath channel and the landmark solvers (no Rust core needed)."""

import dataclasses
import math
import unittest

import numpy as np

from swarm.sim import landmarks as lm
from swarm.sim import landmarks_run as run
from swarm.sim import multipath as mp
from swarm.sim import world

# A radio with no noise, fading, clutter or calibration error: every number is exact.
CLEAN = dataclasses.replace(
    mp.RadioModel(), fade_db=0.0, sigma_shared=0.0, sigma_floor=0.0, k_len=0.0, sigma_amp_db=0.0,
    sigma_phase0=0.0, k_phase=0.0, cal_phase_sigma=0.0, false_peaks=0.0, detect_slope_db=1e-3,
    snr1_db=90.0, gap_los=(3.5, 0.0), gap_nlos=(18.0, 0.0))
EYE = np.eye(3)


def fast_cfg(**kw):
    base = dict(cell=1.0, vote_sample=500, margin=6.0, min_support=15, min_links=4)
    base.update(kw)
    return lm.SolveConfig(**base)


class GeometryTest(unittest.TestCase):
    def test_chord(self):
        c, r = np.array([5.0, 0, 0]), 0.5
        self.assertAlmostEqual(float(mp.chord([0, 0, 0], [10, 0, 0], c, r)[0]), 1.0)       # through the centre
        self.assertAlmostEqual(float(mp.chord([0, 0.5, 0], [10, 0.5, 0], c, r)[0]), 0.0)    # tangent
        self.assertAlmostEqual(float(mp.chord([0, 0, 0], [5, 0, 0], c, r)[0]), 0.5)         # ends at the centre
        self.assertAlmostEqual(float(mp.chord([0, 0, 0], [3, 0, 0], c, r)[0]), 0.0)         # stops short

    def test_fader_loss_scales_with_chord(self):
        sc = mp.Scene(faders=[mp.Fader((5, 0, 0), 0.5, 20.0)])
        self.assertAlmostEqual(mp.fader_loss_db(sc, np.zeros(3), np.array([10.0, 0, 0])), 20.0)
        self.assertAlmostEqual(mp.fader_loss_db(sc, np.array([0, 0.25, 0]), np.array([10.0, 0.25, 0])),
                               20.0 * math.sqrt(0.75), places=6)

    def test_channels(self):
        self.assertEqual(sorted(mp.CHANNELS), [5, 9])           # what the DW3000 has
        self.assertAlmostEqual(mp.wavelength(5), 0.0462, places=3)


class ChannelTest(unittest.TestCase):
    def setUp(self):
        self.tx, self.rx = np.array([0.0, 0, 1.5]), np.array([10.0, 0, 1.5])
        self.refl = np.array([5.0, 8.0, 1.5])
        self.array = [mp.pair_for(5, 0), mp.pair_for(9, 2)]

    def obs(self, scene, ex, R=EYE):
        ch = mp.Channel(scene, CLEAN, self.array, seed=1)
        return ch.observe(0.0, 0, 1, self.tx, self.rx, R, (self.tx, self.rx, R), ex)

    def test_clear_link_reads_true_range_and_los_gap(self):
        o = self.obs(mp.Scene(), mp.Exchange(5, -1))
        self.assertAlmostEqual(o.rng, 10.0, places=6)
        self.assertEqual(len(o.L), 1)
        self.assertAlmostEqual(o.gap_db, CLEAN.gap_los[0], places=6)

    def test_reflection_path_length_and_phase(self):
        o = self.obs(mp.Scene([mp.Reflector(tuple(self.refl))]), mp.Exchange(5, 0))
        d1, d2 = np.linalg.norm(self.refl - self.tx), np.linalg.norm(self.refl - self.rx)
        np.testing.assert_allclose(o.L, [10.0, d1 + d2], atol=1e-9)
        u = (self.refl - self.rx) / d2
        want = mp.wrap(2 * math.pi / mp.wavelength(5) * self.array[0].baseline[0] * u[0])
        self.assertAlmostEqual(float(o.phase[1]), float(want), places=9)
        # LOS comes from the transmitter, along -x from the receiver: phase +-pi*sin = -pi
        u_los = (self.tx - self.rx) / 10.0
        self.assertAlmostEqual(float(o.phase[0]),
                               float(mp.wrap(2 * math.pi / mp.wavelength(5) * self.array[0].baseline[0] * u_los[0])), places=9)

    def test_phase_follows_receiver_attitude(self):
        scene = mp.Scene([mp.Reflector(tuple(self.refl))])
        a = self.obs(scene, mp.Exchange(5, 0), mp.rot_z(0.0)).phase[1]
        b = self.obs(scene, mp.Exchange(5, 0), mp.rot_z(0.4)).phase[1]
        self.assertNotAlmostEqual(float(a), float(b), places=2)

    def test_fader_hides_the_line_of_sight(self):
        # 70 dB through the centre takes the LOS below the detection threshold: the
        # first arrival is now the reflection, and the power gap says so.
        scene = mp.Scene([mp.Reflector(tuple(self.refl))], [mp.Fader((5, 0, 1.5), 0.4, 70.0)])
        o = self.obs(scene, mp.Exchange(5, -1))
        d1, d2 = np.linalg.norm(self.refl - self.tx), np.linalg.norm(self.refl - self.rx)
        self.assertAlmostEqual(o.rng, d1 + d2, places=6)
        self.assertGreater(o.gap_db, 8.0)          # N(18, 3.5) in the bench fit

    def test_close_paths_merge(self):
        # A reflector 0.2 m beyond the line of sight: one peak, pulled late.
        scene = mp.Scene([mp.Reflector((5.0, 1.0, 1.5))])
        o = self.obs(scene, mp.Exchange(5, -1))
        self.assertEqual(len(o.L), 1)
        self.assertGreater(o.L[0], 10.0)

    def test_dropped_beyond_range(self):
        ch = mp.Channel(mp.Scene(), CLEAN, seed=1)
        far = np.array([100.0, 0, 1.5])
        self.assertIsNone(ch.observe(0, 0, 1, self.tx, far, EYE, (self.tx, far, EYE), mp.Exchange()))


class ObservabilityTest(unittest.TestCase):
    """What a single link pins down: the rank of the residual Jacobian at the truth."""

    def rank(self, pairs):
        tx, rx, m = np.array([0.0, 0, 1.5]), np.array([10.0, 0, 1.5]), np.array([5.0, 8.0, 3.0])
        radio = CLEAN
        ch = mp.Channel(mp.Scene([mp.Reflector(tuple(m))]), radio, pairs, seed=1)
        exchanges = [mp.Exchange(p.channel, i) for i, p in enumerate(pairs)] or [mp.Exchange(5, -1)]
        rows = [ch.observe(0, 0, 1, tx, rx, EYE, (tx, rx, EYE), ex) for ex in exchanges]
        cfg = fast_cfg()
        # one detection per (pair): its length constraint and its phase constraint
        D = lm.build_detections(rows, cfg, pairs)
        f = lambda x: lm.stacked(x, D)
        J = np.stack([(f(m + e) - f(m - e)) / 2e-6 for e in 1e-6 * np.eye(3)], axis=1)
        return np.linalg.matrix_rank(J, tol=1e-3), D

    def test_length_alone_is_an_ellipsoid(self):
        r, D = self.rank([])
        # (no pair: no phase) one length constraint
        self.assertEqual(r, 1)

    def test_one_pair_adds_one_bearing_component(self):
        r, _ = self.rank([mp.pair_for(5, 0)])
        self.assertEqual(r, 2)          # a curve of solutions remains

    def test_two_pairs_fix_a_reflector_from_one_link(self):
        r, _ = self.rank([mp.pair_for(5, 0), mp.pair_for(9, 2)])
        self.assertGreaterEqual(r, 3)

    def test_planar_mesh_has_a_mirror_ambiguity_that_a_vertical_pair_breaks(self):
        # Every node at z = 1.5: the reflector and its mirror image about z = 1.5
        # give identical path lengths; only a baseline with a vertical component
        # can tell them apart.
        m, mirror = np.array([5.0, 8.0, 3.0]), np.array([5.0, 8.0, 0.0])
        pairs = [mp.pair_for(5, 0), mp.pair_for(9, 2)]
        ch = mp.Channel(mp.Scene([mp.Reflector(tuple(m))]), CLEAN, pairs, seed=1)
        tx, rx = np.array([0.0, 0, 1.5]), np.array([10.0, 3.0, 1.5])
        obs = [ch.observe(0, 0, 1, tx, rx, EYE, (tx, rx, EYE), mp.Exchange(p.channel, i)) for i, p in enumerate(pairs)]
        D = lm.build_detections(obs, fast_cfg(), pairs)
        rl_t, rp_t = lm.reflector_residuals(m, D)
        rl_m, rp_m = lm.reflector_residuals(mirror, D)
        np.testing.assert_allclose(rl_t, 0, atol=1e-6)
        np.testing.assert_allclose(rl_m, 0, atol=1e-6)                      # lengths can't tell
        self.assertLess(abs(rp_t[0]), 1e-6)
        self.assertLess(abs(rp_m[0]), 1e-6)                                  # x pair can't tell
        self.assertGreater(abs(rp_m[1]), 3.0)                                # z pair can


class SolverTest(unittest.TestCase):
    def setUp(self):
        self.scene = mp.Scene([mp.Reflector((1.0, 15.0, 2.0)), mp.Reflector((18.5, 2.0, 1.0))])
        self.truth = np.array([r.pos for r in self.scene.reflectors])

    def sample(self, array, schedule, seconds=6, radio=CLEAN, mesh=None, **kw):
        ch = mp.Channel(self.scene, radio, array, seed=3)
        smp = mp.MeshSampler(world.default_swarm(6, 0, 0, 20.0), ch, schedule,
                             mesh=mesh or mp.MeshError(0, 0, 0), seed=3, **kw)
        return smp, smp.run(seconds)

    def test_reflectors_recovered_without_noise(self):
        arr = [mp.pair_for(5, 0), mp.pair_for(9, 2)]
        _, obs = self.sample(arr, [mp.Exchange(5, 0), mp.Exchange(9, 1)])
        found, _ = lm.map_reflectors(obs, fast_cfg(sigma_mesh=0.02, sigma_att=0.01), arr)
        idx, extra = lm.match(np.array([l.pos for l in found]).reshape(-1, 3), self.truth)
        self.assertTrue((idx >= 0).all(), "missed a reflector")
        self.assertEqual(len(extra), 0)
        err = np.linalg.norm(np.array([found[i].pos for i in idx]) - self.truth, axis=1)
        self.assertLess(err.max(), 0.05)

    def test_clutter_alone_makes_no_landmark(self):
        radio = dataclasses.replace(CLEAN, false_peaks=1.0)
        self.scene = mp.Scene()
        _, obs = self.sample([], [mp.Exchange(5, -1)], radio=radio)
        found, _ = lm.map_reflectors(obs, fast_cfg(min_support=30, min_links=6), [])
        self.assertEqual(len(found), 0)

    def test_first_arrival_sees_a_reflector_only_through_a_fader(self):
        # Without a fader the first arrival is the line of sight: no reflector evidence.
        _, obs = self.sample([], [mp.Exchange(5, -1)])
        D = lm.build_detections(obs, fast_cfg(use_cir=False, use_phase=False), [])
        self.assertEqual(len(D), 0)

    def test_fader_located_from_link_attenuation(self):
        self.scene = mp.Scene(faders=[mp.Fader((10.0, 10.0, 1.5), 0.4, 20.0)])
        _, obs = self.sample([], [mp.Exchange(5, -1)], seconds=15)
        found = lm.map_faders(obs, lm.SolveConfig(use_cir=True, use_phase=False, fader_cell=0.5, snr1_db=CLEAN.snr1_db))
        self.assertGreaterEqual(len(found), 1)
        self.assertLess(np.linalg.norm(found[0][0] - [10.0, 10.0, 1.5]), 0.5)
        self.assertAlmostEqual(found[0][1], 20.0, delta=4.0)


class GaugeTest(unittest.TestCase):
    def test_register_recovers_a_yaw_and_translation(self):
        refs = np.array([[1.0, 15.0, 2.0], [18.5, 2.0, 1.0], [12.0, 19.0, 3.0]])
        g = np.array([1.7, -2.3, 0.4, 0.12])
        Rg, tg = lm.gauge_matrix(g)
        mesh = (refs - tg) @ Rg                    # mesh = Rg^T (world - tg)
        got, n = lm.register(mesh[[2, 0, 1]], refs)    # order doesn't matter
        self.assertEqual(n, 3)
        np.testing.assert_allclose(got, g, atol=1e-6)

    def test_register_survives_a_missing_and_a_spurious_landmark(self):
        refs = np.array([[1.0, 15.0, 2.0], [18.5, 2.0, 1.0], [12.0, 19.0, 3.0]])
        g = np.array([-3.0, 2.0, 0.0, -0.4])
        Rg, tg = lm.gauge_matrix(g)
        mesh = np.vstack([(refs[:2] - tg) @ Rg, [7.0, 7.0, 7.0]])
        got, n = lm.register(mesh, refs, tol=0.5)
        self.assertEqual(n, 2)
        np.testing.assert_allclose(got[:2], g[:2], atol=1e-6)

    def test_solve_gauge_recovers_the_offset_without_noise(self):
        scene = mp.Scene([mp.Reflector((1.0, 15.0, 2.0)), mp.Reflector((18.5, 2.0, 1.0)),
                          mp.Reflector((12.0, 19.0, 3.0))])
        arr = [mp.pair_for(5, 0), mp.pair_for(9, 2)]
        truth = np.array([1.7, -2.3, 0.4, 0.12])
        ch = mp.Channel(scene, CLEAN, arr, seed=3)
        smp = mp.MeshSampler(world.default_swarm(6, 0, 0, 20.0), ch, [mp.Exchange(5, 0), mp.Exchange(9, 1)],
                             mesh=mp.MeshError(0, 0, 0), seed=3, gauge0=tuple(truth))
        obs = smp.run(6)
        refs = np.array([r.pos for r in scene.reflectors])
        cfg = fast_cfg(sigma_mesh=0.02, sigma_att=0.01)
        g, cov, n = lm.solve_gauge(obs, refs, cfg, arr, truth + [0.3, -0.2, 0.1, 0.03])
        self.assertGreater(n, 100)
        np.testing.assert_allclose(g, truth, atol=0.03)

    def test_solve_gauge_without_usable_references_stays_put(self):
        # References nowhere near anything the radios saw: nothing to hold the mesh to,
        # so the gauge must stay at its starting value rather than wander.
        scene = mp.Scene([mp.Reflector((1.0, 15.0, 2.0))])
        ch = mp.Channel(scene, CLEAN, [], seed=3)
        smp = mp.MeshSampler(world.default_swarm(6, 0, 0, 20.0), ch, [mp.Exchange(5, -1)],
                             mesh=mp.MeshError(0, 0, 0), seed=3)
        obs = smp.run(3)
        g0 = np.array([0.5, 0.5, 0.0, 0.1])
        g, cov, n = lm.solve_gauge(obs, [[200.0, 200.0, 5.0]], fast_cfg(use_phase=False), [], g0)
        self.assertEqual(n, 0)
        np.testing.assert_allclose(g, g0)


class BroadcastTest(unittest.TestCase):
    def setUp(self):
        self.scene = mp.Scene([mp.Reflector((5.0, 8.0, 1.5))])
        self.arr = [mp.pair_for(5, 0, boresight=(0, 1, 0)), mp.pair_for(9, 2, boresight=(0, 1, 0))]

    def fleet(self, n=5, **kw):
        specs = [world.WidgetSpec(wid=i, kind="static", center=(2.0 * i, 0.5 * (i % 2), 1.5), yaw_rate=0.0)
                 for i in range(n)]
        ch = mp.Channel(self.scene, CLEAN, self.arr, seed=2)
        return mp.MeshSampler(specs, ch, [mp.Exchange(5, 0), mp.Exchange(9, 1)], mesh=mp.MeshError(0, 0, 0, noimu_yaw_sigma=0.0),
                              seed=2, **kw)

    def test_a_broadcast_slot_is_heard_by_everyone_else(self):
        smp = self.fleet(mode="broadcast")
        obs = smp.run(0.05)                                  # 5 slots
        self.assertEqual(len(obs), 5 * 4)
        self.assertEqual({o.tx for o in obs[:4]}, {0})
        self.assertEqual({o.rx for o in obs[:4]}, {1, 2, 3, 4})
        self.assertEqual({o.pair for o in obs[:4]}, {0})     # one channel per slot, all receivers on it

    def test_a_pairwise_slot_is_two_observations(self):
        self.assertEqual(len(self.fleet(mode="pairwise").run(0.05)), 5 * 2)

    def test_clock_error_shifts_every_path_of_a_one_way_frame_equally(self):
        ch = mp.Channel(self.scene, CLEAN, self.arr, seed=1)
        tx, rx = np.array([0.0, 0, 1.5]), np.array([10.0, 0, 1.5])
        a = ch.observe(0, 0, 1, tx, rx, EYE, (tx, rx, EYE), mp.Exchange(5, -1), sync_len=0.0)
        b = ch.observe(0, 0, 1, tx, rx, EYE, (tx, rx, EYE), mp.Exchange(5, -1), sync_len=0.7)
        np.testing.assert_allclose(b.L - a.L, 0.7, atol=1e-9)
        self.assertAlmostEqual(b.rng - a.rng, 0.7, places=9)

    def test_excess_delay_ignores_clock_error_and_absolute_does_not(self):
        ch = mp.Channel(self.scene, CLEAN, [], seed=1)
        tx, rx = np.array([0.0, 0, 1.5]), np.array([10.0, 0, 1.5])
        mk = lambda sync: [ch.observe(0, 0, 1, tx, rx, EYE, (tx, rx, EYE), mp.Exchange(5, -1), sync_len=sync)]
        cfg = fast_cfg(use_phase=False, use_excess=True, sigma_sync=1.0)
        L0 = lm.build_detections(mk(0.0), cfg, []).L
        L1 = lm.build_detections(mk(0.8), cfg, []).L
        self.assertEqual(len(L0), 1)
        np.testing.assert_allclose(L1, L0, atol=1e-9)
        ab = fast_cfg(use_phase=False, use_excess=False, sigma_sync=1.0)
        self.assertAlmostEqual(float(lm.build_detections(mk(0.8), ab, []).L[0] - lm.build_detections(mk(0.0), ab, []).L[0]), 0.8, places=9)

    def test_a_large_sync_error_is_not_taken_for_a_reflection(self):
        # With 1 m of clock error the line of sight reads 1 m long: it must not become a landmark.
        ch = mp.Channel(mp.Scene(), CLEAN, [], seed=1)
        tx, rx = np.array([0.0, 0, 1.5]), np.array([10.0, 0, 1.5])
        o = [ch.observe(0, 0, 1, tx, rx, EYE, (tx, rx, EYE), mp.Exchange(5, -1), sync_len=1.0)]
        self.assertEqual(len(lm.build_detections(o, fast_cfg(use_phase=False, sigma_sync=0.0), [])), 1)   # blind: a ghost
        self.assertEqual(len(lm.build_detections(o, fast_cfg(use_phase=False, sigma_sync=1.0), [])), 0)   # aware: none

    def test_yaw_of_a_node_without_an_imu_is_recovered_from_the_line_of_sight(self):
        smp = self.fleet(mode="broadcast", face=(4.0, 3.0))
        smp.yaw_bias[:] = 0.0
        smp.yaw_bias[2] = 0.3                                # node 2 was placed 0.3 rad off
        obs = smp.run(1.0)
        got = lm.calibrate_yaw(obs, self.arr, {smp.ids[2]}, fast_cfg())
        self.assertAlmostEqual(got[smp.ids[2]], -0.3, delta=0.03)
        fixed = lm.apply_yaw(obs, got)
        mine = [o for o in fixed if o.rx == smp.ids[2]][0]
        truth = [o for o in obs if o.rx == smp.ids[2]][0]
        self.assertFalse(np.allclose(mine.R_rx, truth.R_rx))

    def test_expected_snr_is_corrected_for_the_patch_pattern(self):
        # A line of sight arriving 80 degrees off a patch's boresight reads ~12 dB down;
        # that is the antenna, not a fader.
        arr = [mp.pair_for(5, 0, boresight=(0, 1, 0))]
        ch = mp.Channel(mp.Scene(), CLEAN, arr, seed=1)
        tx, rx = np.array([10.0, math.cos(math.radians(80)) * 10, 1.5]), np.array([0.0, 0, 1.5])
        o = [ch.observe(0, 0, 1, tx, rx, EYE, (tx, rx, EYE), mp.Exchange(5, 0))]
        cfg = lm.SolveConfig(snr1_db=CLEAN.snr1_db)
        naive = lm.link_attenuation(o, cfg, [])
        aware = lm.link_attenuation(o, cfg, arr)
        self.assertGreater(float(naive.y[0]), 5.0)
        self.assertLess(abs(float(aware.y[0])), 0.5)


if __name__ == "__main__":
    unittest.main()
