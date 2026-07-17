"""Full-stack headless test: simulator -> TCP ingest -> solver -> HTTP/SSE."""

import http.client
import json
import time
import unittest

from host import server, simulator

TRUTH = (120.0, 250.0)


class IntegrationTest(unittest.TestCase):

    def setUp(self):
        # Port 0 = OS-assigned; read the real ports back from the sockets.
        self.rtls, self.tcp_sock, self.httpd = server.serve(
            simulator.DEFAULT_ANCHORS, (480.0, 650.0), tcp_port=0, http_port=0)
        self.tcp_port = self.tcp_sock.getsockname()[1]
        self.http_port = self.httpd.server_address[1]

    def tearDown(self):
        self.tcp_sock.close()
        self.httpd.shutdown()

    def http_get(self, path):
        conn = http.client.HTTPConnection("127.0.0.1", self.http_port, timeout=10)
        conn.request("GET", path)
        return conn, conn.getresponse()

    def test_simulator_to_sse(self):
        sent = simulator.run("127.0.0.1", self.tcp_port,
                             simulator.DEFAULT_ANCHORS, rate_hz=200.0,
                             steps=20, noise_cm=0.5, fixed=TRUTH)
        self.assertEqual(sent, 20)

        # Ingest runs in a daemon thread; wait for the last packet to land.
        deadline = time.time() + 10
        state = None
        while time.time() < deadline:
            with self.rtls.lock:
                state = self.rtls.latest
            if state is not None:
                break
            time.sleep(0.05)
        self.assertIsNotNone(state, "no packet ingested within 10s")
        self.assertEqual(state["tag_id"], 10)
        self.assertIsNotNone(state["position"])
        x, y = state["position"]
        self.assertLess(abs(x - TRUTH[0]), 5.0)
        self.assertLess(abs(y - TRUTH[1]), 5.0)

        # /state mirrors the latest solution.
        conn, resp = self.http_get("/state")
        self.assertEqual(resp.status, 200)
        snap = json.loads(resp.read())
        conn.close()
        self.assertIsNotNone(snap["position"])
        self.assertEqual(snap["config"]["room"], [480.0, 650.0])

        # /events replays the latest state to a new SSE subscriber.
        conn, resp = self.http_get("/events")
        self.assertEqual(resp.status, 200)
        self.assertIn("text/event-stream", resp.getheader("Content-Type"))
        line = resp.fp.readline().decode()
        self.assertTrue(line.startswith("data: "), line)
        event = json.loads(line[len("data: "):])
        self.assertLess(abs(event["position"][0] - TRUTH[0]), 5.0)
        conn.close()

        # The floor-view page is served.
        conn, resp = self.http_get("/")
        self.assertEqual(resp.status, 200)
        self.assertIn(b"UWB RTLS floor view", resp.read())
        conn.close()


if __name__ == "__main__":
    unittest.main()
