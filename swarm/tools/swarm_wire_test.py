"""swarm_wire encodings, pinned to the vectors the Rust side asserts
(swarm/net/src/settings.rs: matches_host_tool_encoding)."""

import hashlib
import struct
import unittest

import swarm_wire as w

KEY = bytes([5] * 32)


class WireTest(unittest.TestCase):
    def test_bundle_matches_rust(self):
        b = w.make_bundle({106: w.encode_value("f32", 7.0), 300: w.encode_value("f32", 0.2)}, 3, KEY)
        self.assertEqual(b.hex(), "0300000002006a000000e0402c01cdcc4c3e6e1e97e2")

    def test_override_matches_rust(self):
        o = w.make_override(13, w.CMD_SET, 1, [(300, w.encode_value("f32", 0.2))], KEY)
        self.assertEqual(o.hex(), "0101000000012c01cdcc4c3e402c9832")

    def test_override_tag_binds_destination(self):
        a = w.make_override(13, w.CMD_SET, 1, [], KEY)
        b = w.make_override(14, w.CMD_SET, 1, [], KEY)
        self.assertEqual(a[:-4], b[:-4])
        self.assertNotEqual(a[-4:], b[-4:])

    def test_values_round_trip(self):
        for kind, v in (("f32", 0.25), ("i32", -7), ("bool", True)):
            self.assertEqual(w.decode_value(kind, w.encode_value(kind, v)), v)

    def test_report_parse(self):
        p = struct.pack("<BBII", 0, 1, 9, 4) + struct.pack("<HI", 106 | 1 << 14, 5) + struct.pack("<HI", 300 | 2 << 14, 6)
        self.assertEqual(w.parse_report(p), (0, 1, 9, 4, [(106, 1, 5), (300, 2, 6)]))

    def test_manifest_signed_bytes_and_chunks(self):
        img = bytes(range(256)) * 20
        m = w.manifest_signed_bytes(2, img)
        self.assertEqual(len(m), 44)
        self.assertEqual(m[8:40], hashlib.sha256(img).digest())
        parts = list(w.chunks(img))
        self.assertEqual(b"".join(d for _, _, d in parts), img)
        self.assertEqual(parts[43][:2], (1, 0), "43 chunks per 4 KB page")


if __name__ == "__main__":
    unittest.main()
