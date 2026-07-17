"""Headless smoke test of the viewer: serve a small sim, hit /, /state and
/events over HTTP, and check that frames flow."""

import json
import unittest
import urllib.request

from swarm.sim import viewer, world


class ViewerTest(unittest.TestCase):
    def test_serves_page_state_and_events(self):
        specs = world.default_swarm(n=2, n_gps=0, n_surveyed=3)
        w = world.World(specs, seed=1)
        port = 18081
        viewer.serve(w, http_port=port, realtime=False)

        html = urllib.request.urlopen(
            "http://127.0.0.1:%d/" % port, timeout=10).read().decode()
        self.assertIn("Station-free swarm", html)

        with urllib.request.urlopen(
                "http://127.0.0.1:%d/events" % port, timeout=10) as resp:
            line = b""
            while not line.startswith(b"data: "):
                line = resp.readline()
            frame = json.loads(line[len(b"data: "):])
        self.assertEqual(len(frame["widgets"]), 5)
        self.assertIn("est_p", frame["widgets"][0])

        state = json.loads(urllib.request.urlopen(
            "http://127.0.0.1:%d/state" % port, timeout=10).read())
        self.assertIn("widgets", state)


if __name__ == "__main__":
    unittest.main()
