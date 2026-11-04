"""Tests for hull.py (run: python3 -m unittest tools.kinect.hull_test, or from this
directory: python3 -m unittest hull_test)."""

import struct
import unittest

import numpy as np

import hull
from kinect_bridge import SynthSource

# Camera 1.5 m up, level, at the origin: room (x, y up, z forward) = camera (x, -y, z).
EXT = hull.camera_pose((0.0, 1.5, 0.0))


def scene(centre, radii=(0.25, 0.9, 0.2)):
    return SynthSource().render(centre, radii)


def run(depth, bg=None, tracker=None):
    tracker = tracker or hull.HullTracker(appear=1)
    return hull.process(depth, tracker, EXT, bg), tracker


class HullTest(unittest.TestCase):
    def test_camera_pose_maps_optical_to_room(self):
        p = hull.transform(np.array([[0.0, 0.0, 2.0], [0.0, 1.0, 0.0]]), EXT)
        np.testing.assert_allclose(p, [[0, 1.5, 2.0], [0, 0.5, 0]], atol=1e-9)   # y down is room -y

    def test_pitch_looks_down(self):
        t = hull.camera_pose((0, 1.5, 0), pitch_deg=30)
        fwd = hull.transform(np.array([[0.0, 0.0, 1.0]]), t)[0] - t[:3, 3]
        self.assertAlmostEqual(fwd[1], -0.5, places=6)                            # sin(30) downward

    def test_person_position_recovered(self):
        # Person at 2.5 m, 0.4 m right of the axis, centre at camera height.
        depth = scene((0.4, 0.0, 2.5))
        bg = hull.decimate(np.full_like(depth, 4000.0))[0]
        (present, h, n), _ = run(depth, bg)
        self.assertTrue(present)
        self.assertGreater(n, 15)
        centre_x = (h[0] - h[1]) / 2          # +x and -x supports
        self.assertAlmostEqual(centre_x, 0.4, delta=0.08)
        self.assertAlmostEqual(h[5] * -1, 2.5 - 0.2, delta=0.15)   # nearest surface (-z support)

    def test_hull_encloses_its_points(self):
        rng = np.random.default_rng(1)
        pts = rng.normal(size=(200, 3)) * [0.2, 0.5, 0.15] + [1.0, 1.0, 3.0]
        h = hull.support(pts)
        self.assertTrue((hull.signed_distance(h, pts) <= 1e-9).all())

    def test_grown_hull_reaches_nearby_leds_only(self):
        pts = np.array([[0, 0, 0], [0.2, 0, 0], [0, 0.4, 0], [0, 0, 0.2]], float) + [1.0, 1.0, 2.0]
        h = hull.support(pts)
        near, far = np.array([[1.0, 1.0, 2.4]]), np.array([[1.0, 1.0, 4.0]])
        grow = 0.6
        self.assertLess(hull.signed_distance(h, near)[0], grow)
        self.assertGreater(hull.signed_distance(h, far)[0], grow)

    def test_pack_is_column_major_planes(self):
        h = np.arange(hull.N_PLANES, dtype=float)
        blocks = hull.pack(h)
        self.assertEqual(blocks.shape, (4, 16))
        # GLSL mat4 flat order is column-major: column i = floats [4i, 4i+4).
        for k in range(hull.N_PLANES):
            col = blocks[k // 4].reshape(4, 4)[k % 4]
            np.testing.assert_allclose(col[:3], hull.DIRS[k], atol=1e-6)
            self.assertAlmostEqual(col[3], h[k], places=5)
        # spare columns repeat plane 0, so they never loosen the max
        np.testing.assert_allclose(blocks[3].reshape(4, 4)[3], blocks[0].reshape(4, 4)[0])

    def test_empty_room_is_not_present(self):
        depth = np.full((hull.DEPTH_H, hull.DEPTH_W), 4000.0, np.float32)
        bg = hull.decimate(depth)[0]
        (present, h, n), _ = run(depth, bg)
        self.assertFalse(present)
        self.assertEqual(n, 0)

    def test_presence_hysteresis(self):
        tr = hull.HullTracker(appear=2, vanish=3)
        pts = np.random.default_rng(0).normal(size=(30, 3))
        self.assertFalse(tr.update(pts)[0])      # first sighting isn't enough
        self.assertTrue(tr.update(pts)[0])
        for _ in range(2):                       # two dropped frames: still there
            self.assertTrue(tr.update(np.empty((0, 3)))[0])
        self.assertFalse(tr.update(np.empty((0, 3)))[0])

    def test_learn_background_ignores_a_passer_by(self):
        wall = scene((9, 9, 9))
        grids = [hull.decimate(wall)[0] for _ in range(5)]
        grids[2] = hull.decimate(scene((0, 0, 2.0)))[0]    # one frame with someone in it
        bg = hull.learn_background(grids)
        np.testing.assert_allclose(bg[np.isfinite(bg)], 4.0, atol=1e-3)

    def test_osc_packet(self):
        h = np.linspace(-1, 1, hull.N_PLANES)
        pkt = hull.hull_packet(True, h)
        self.assertTrue(pkt.startswith(b"#bundle\0"))
        body, msgs = pkt[16:], []
        while body:
            n = struct.unpack(">i", body[:4])[0]
            msgs.append(body[4:4 + n])
            body = body[4 + n:]
        self.assertEqual([m.split(b"\0")[0].decode() for m in msgs],
                         ["/present", "/hull0", "/hull1", "/hull2", "/hull3"])
        self.assertEqual(len(msgs[1]), 8 + 20 + 64)            # "/hull0" padded, ",ffff…" (17 chars) padded, 16 floats
        vals = struct.unpack(">16f", msgs[1][-64:])
        self.assertAlmostEqual(vals[0], 1.0)                    # plane 0 normal x
        absent = hull.hull_packet(False, None)
        self.assertEqual(len(absent), 16 + 4 + len(msgs[0]))    # only /present


if __name__ == "__main__":
    unittest.main()
