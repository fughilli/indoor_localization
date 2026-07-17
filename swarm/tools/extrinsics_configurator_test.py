"""HTTP smoke test for the extrinsics configurator: page, config round-trip
with derivation, validation error, and both export formats."""

import json
import os
import shutil
import tempfile
import unittest
import urllib.error
import urllib.request

from swarm.tools import extrinsics_configurator

EXAMPLE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "examples", "widget_v1.json")
PORT = 18082


def get(path):
    return urllib.request.urlopen("http://127.0.0.1:%d%s" % (PORT, path),
                                  timeout=10).read()


def post(path, obj):
    req = urllib.request.Request(
        "http://127.0.0.1:%d%s" % (PORT, path),
        data=json.dumps(obj).encode(),
        headers={"Content-Type": "application/json"})
    return urllib.request.urlopen(req, timeout=10).read()


class ConfiguratorTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config_path = os.path.join(tempfile.mkdtemp(), "cfg.json")
        shutil.copy(EXAMPLE, cls.config_path)
        cls.httpd = extrinsics_configurator.serve(cls.config_path, PORT)

    def test_page_and_config(self):
        self.assertIn(b"extrinsics configurator", get("/"))
        cfg = json.loads(get("/config"))
        self.assertIn("uwb", cfg["sensors"])

    def test_post_derives_and_persists(self):
        cfg = json.loads(get("/config"))
        cfg["sensors"]["imu"]["euler_deg"] = [0.0, 0.0, 90.0]
        out = json.loads(post("/config", cfg))
        w, x, y, z = out["sensors"]["imu"]["rotation_wxyz"]
        self.assertAlmostEqual(w, 0.707107, places=4)
        self.assertAlmostEqual(z, 0.707107, places=4)
        with open(self.config_path) as f:
            self.assertEqual(json.load(f)["sensors"]["imu"]["euler_deg"],
                             [0.0, 0.0, 90.0])

    def test_post_invalid_is_rejected(self):
        cfg = json.loads(get("/config"))
        cfg["sensors"]["imu"]["mapping"] = {"x": "+x", "z": "-x"}
        with self.assertRaises(urllib.error.HTTPError) as ctx:
            post("/config", cfg)
        self.assertEqual(ctx.exception.code, 400)
        self.assertIn("orthogonal",
                      json.loads(ctx.exception.read())["error"])

    def test_exports(self):
        header = get("/export/header").decode()
        self.assertIn("widget_extrinsic_t", header)
        blob = get("/export/blob")
        self.assertEqual(blob[:4], b"WEXT")


if __name__ == "__main__":
    unittest.main()
