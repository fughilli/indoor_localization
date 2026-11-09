"""swarm_wire encodings, pinned to the golden vectors of swarm/proto/testdata/swarm_wire_golden.txt
(produced by swarm/proto/gen_golden.py; the Rust and C++ tests assert the same bytes)."""

import hashlib
import os
import unittest

from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives.asymmetric import ec

import swarm_wire as w
import swarm_wire_pb2 as pb

KEY = bytes([0x42] * 32)
GOLDEN = "swarm/proto/testdata/swarm_wire_golden.txt"


def load_golden():
    here = os.path.dirname(os.path.abspath(__file__))
    for path in (GOLDEN, os.path.join(here, "..", "proto", "testdata", "swarm_wire_golden.txt"),
                 os.path.join(os.environ.get("TEST_SRCDIR", ""), "_main", GOLDEN)):
        if os.path.exists(path):
            with open(path) as f:
                return {k.strip(): bytes.fromhex(v.strip())
                        for k, v in (line.split("=", 1) for line in f if "=" in line)}
    raise FileNotFoundError(GOLDEN)


G = load_golden()
F106 = 0x0000E040
F300 = 0x3E4CCCCD


class GoldenTest(unittest.TestCase):
    def test_bundle(self):
        self.assertEqual(w.make_bundle_body({300: F300, 106: F106}, 3), G["bundle_body"])
        b = w.make_bundle({300: F300, 106: F106}, 3, KEY)
        self.assertEqual(b, G["bundle"])
        self.assertEqual(w.parse_bundle(b, KEY), (3, {106: F106, 300: F300}))

    def test_bundle_tag_checked_over_exact_bytes(self):
        bad = bytearray(G["bundle"])
        bad[-1] ^= 1
        with self.assertRaises(ValueError):
            w.parse_bundle(bytes(bad), KEY)
        # A non-canonical but equivalent body (fields in a different order) verifies against its own
        # tag: the tag covers those bytes, not a re-encoding.
        body = G["bundle_body"][2:] + G["bundle_body"][:2]
        env = pb.Bundle(body=body, tag=w.tag(KEY, w.DOMAIN_BUNDLE, body)).SerializeToString()
        self.assertEqual(w.parse_bundle(env, KEY), (3, {106: F106, 300: F300}))

    def test_override(self):
        o = w.make_override(1, w.CMD_SET, 1, [(300, F300)], KEY)
        self.assertEqual(o, G["override"])
        self.assertEqual(pb.Override.FromString(o).body, G["override_body"])
        self.assertEqual(w.parse_override(o, 1, KEY), (w.CMD_SET, 1, [(300, F300)]))
        with self.assertRaises(ValueError):
            w.parse_override(o, 2, KEY)

    def test_override_tag_binds_destination(self):
        a = pb.Override.FromString(w.make_override(13, w.CMD_SET, 1, [], KEY))
        b = pb.Override.FromString(w.make_override(14, w.CMD_SET, 1, [], KEY))
        self.assertEqual(a.body, b.body)
        self.assertNotEqual(a.tag, b.tag)

    def test_bundle_fragment_and_request_and_ack(self):
        m = pb.BundleFragment.FromString(G["bundle_fragment"])
        self.assertEqual((m.generation, m.total_len, m.offset, m.data), (3, 20, 0, b"\x11\x22\x33\x44"))
        self.assertEqual(w.make_fragment(3, 20, 0, b"\x11\x22\x33\x44"), G["bundle_fragment"])
        self.assertEqual(w.make_bundle_request(3), G["bundle_request"])
        self.assertEqual(pb.Ack(id=7).SerializeToString(), G["ack"])

    def test_settings_report(self):
        p = w.make_report(0, 2, 3, 1, [(106, w.SOURCE_SWARM, F106), (300, w.SOURCE_OVERRIDE, F300)])
        self.assertEqual(p, G["settings_report"])
        self.assertEqual(w.parse_report(p), (0, 2, 3, 1, [(106, 1, F106), (300, 2, F300)]))

    def test_mesh_packet(self):
        p = w.make_mesh_packet(3, 7, w.PORT_OVERRIDE, b"\x01\x02\x03\x04", dest=2, hop_limit=2, hop_start=3,
                               flags=1, attempt=2)
        self.assertEqual(p, G["mesh_packet"])
        m = w.parse_mesh_packet(p)
        self.assertTrue(m.HasField("dest"))
        self.assertEqual((m.origin, m.id, m.dest, m.port, m.payload), (3, 7, 2, 4, b"\x01\x02\x03\x04"))

    def test_mesh_broadcast_omits_dest(self):
        m = w.parse_mesh_packet(w.make_mesh_packet(3, 7, 16, b""))
        self.assertFalse(m.HasField("dest"))
        # An explicit dest 0 is a unicast to node 0, not a broadcast.
        self.assertTrue(w.parse_mesh_packet(w.make_mesh_packet(3, 7, 16, b"", dest=0)).HasField("dest"))

    def test_summary_and_fw(self):
        self.assertEqual(pb.Summary(running=7, staged=8, pages=5, total=12, generation=300).SerializeToString(),
                         G["summary"])
        self.assertEqual(pb.FwRequest(version=5, page=3, missing=0xFF).SerializeToString(), G["fw_request"])
        self.assertEqual(pb.FwData(version=5, page=3, chunk=2, data=bytes(range(10))).SerializeToString(),
                         G["fw_data"])

    def test_manifest_body(self):
        # The golden image is only its digest and size (100000), so build the body from those.
        body = pb.ManifestBody(version=5, size=100000, sha256=hashlib.sha256(b"image").digest())
        self.assertEqual(body.SerializeToString(), G["manifest_body"])
        img = b"image"
        self.assertEqual(w.manifest_signed_bytes(5, img), pb.ManifestBody(
            version=5, size=len(img), sha256=hashlib.sha256(img).digest()).SerializeToString())
        m = pb.Manifest.FromString(G["manifest_dummy_signature"])
        self.assertEqual(m.body, G["manifest_body"])
        self.assertEqual(m.signature, bytes(range(64)))

    def test_link_frames_parse(self):
        m = pb.LinkFrame.FromString(G["link_poll"])
        self.assertEqual((m.ver, m.type, m.src, m.seq, m.slot, m.advert),
                         (1, pb.FRAME_TYPE_POLL, 3, 9, 2, b"\x01\x02\x03"))
        self.assertFalse(m.HasField("dst"))
        self.assertEqual(pb.Summary.FromString(m.summary).generation, 300)
        m = pb.LinkFrame.FromString(G["link_response"])
        self.assertEqual((m.type, m.src, m.dst, m.seq), (pb.FRAME_TYPE_RESPONSE, 20000, 3, 9))
        m = pb.LinkFrame.FromString(G["link_report"])
        self.assertEqual((m.t_round_b, m.t_reply_b), (0x00123456, 0x00654321))
        m = pb.LinkFrame.FromString(G["link_result"])
        self.assertEqual((m.type, m.range_mm), (pb.FRAME_TYPE_RESULT, -1234))
        m = pb.LinkFrame.FromString(G["link_mesh"])
        self.assertEqual((m.type, m.src, m.payload), (pb.FRAME_TYPE_MESH, 3, b"\xaa\xbb"))
        self.assertFalse(m.HasField("dst"))
        for name in ("link_poll", "link_response", "link_report", "link_result", "link_mesh"):
            self.assertEqual(pb.LinkFrame.FromString(G[name]).SerializeToString(), G[name], name)


class ValuesTest(unittest.TestCase):
    def test_values_round_trip(self):
        for kind, v in (("f32", 0.25), ("i32", -7), ("bool", True)):
            self.assertEqual(w.decode_value(kind, w.encode_value(kind, v)), v)
        self.assertEqual(w.encode_value("f32", 0.2), F300)

    def test_fragments_round_trip(self):
        entries = {k: 0x80000000 + k for k in range(1, 97)}
        b = w.make_bundle(entries, 1 << 31, KEY)
        frags = w.split_bundle(b, 1 << 31)
        self.assertEqual(len(frags), (len(b) + w.FRAG_DATA - 1) // w.FRAG_DATA)
        for i, f in enumerate(frags):
            self.assertEqual(pb.BundleFragment.FromString(f).offset, i * w.FRAG_DATA)
        self.assertEqual(w.join_fragments(reversed(frags)), b)
        self.assertEqual(w.parse_bundle(b, KEY)[1], entries)

    def test_chunks(self):
        img = bytes(range(256)) * 20
        self.assertEqual(b"".join(d for _, _, d in w.chunks(img)), img)
        self.assertEqual(list(w.chunks(img))[43][:2], (1, 0), "43 chunks per 4 KB page")


class ManifestTest(unittest.TestCase):
    def setUp(self):
        self.sk = ec.generate_private_key(ec.SECP256R1())
        self.pem = self.sk.private_bytes(serialization.Encoding.PEM, serialization.PrivateFormat.PKCS8,
                                         serialization.NoEncryption())

    def test_sign_verify_round_trip(self):
        img = bytes(range(256)) * 20
        m = w.sign_manifest(2, img, self.pem)
        self.assertLessEqual(len(m), w.MAX_MANIFEST_LEN)
        body = w.verify_manifest(m, self.sk.public_key())
        self.assertEqual((body.version, body.size, body.sha256), (2, len(img), hashlib.sha256(img).digest()))
        self.assertEqual(pb.Manifest.FromString(m).body, w.manifest_signed_bytes(2, img))

    def test_tamper_fails(self):
        m = bytearray(w.sign_manifest(2, b"image", self.pem))
        m[4] ^= 1       # inside the body
        with self.assertRaises(Exception):
            w.verify_manifest(bytes(m), self.sk.public_key())

    def test_worst_case_manifest_budget(self):
        # flags absent (0): the documented 46 + 2 + 66 = 114.
        body = pb.ManifestBody(version=0xFFFFFFFF, size=0xFFFFFFFF, sha256=bytes(32))
        self.assertEqual(len(body.SerializeToString()), 46)
        env = pb.Manifest(body=body.SerializeToString(), signature=bytes(64))
        self.assertEqual(len(env.SerializeToString()), 114)
        # flags = 1 (loader image, reserved) adds two bytes: 116, over the 114 budget (the manifest
        # link frame would be 126 > 125). The docs/swarm-wire.md figure assumes flags absent.
        body.flags = 1
        env = pb.Manifest(body=body.SerializeToString(), signature=bytes(64))
        self.assertEqual(len(env.SerializeToString()), 116)
        # A realistic loader manifest (small version, 3-byte size) fits.
        self.assertLessEqual(len(w.sign_manifest(300, bytes(70000), self.pem, flags=1)), 114)


class BudgetTest(unittest.TestCase):
    """Worst cases of docs/swarm-wire.md: all varints at maximum width (registry keys <= 14 bits)."""

    def entries(self, n):
        return [(w.MAX_KEY, 0xFFFFFFFF)] * n

    def test_override_fits_mesh_payload(self):
        o = w.make_override(0xFFFF, w.CMD_SET, 0xFFFFFFFF, self.entries(w.CMD_MAX_ENTRIES), KEY)
        self.assertLessEqual(len(o), w.MESH_MAX_PAYLOAD)
        with self.assertRaises(ValueError):
            w.make_override(1, w.CMD_SET, 1, self.entries(w.CMD_MAX_ENTRIES + 1), KEY)

    def test_report_fits_mesh_payload(self):
        p = w.make_report(127, 127, 0xFFFFFFFF, 0xFFFFFFFF,
                          [(w.MAX_KEY, 2, 0xFFFFFFFF)] * w.REPORT_ENTRIES_PER_PART)
        self.assertLessEqual(len(p), w.MESH_MAX_PAYLOAD)

    def test_bundle_and_fragments(self):
        entries = {k: 0xFFFFFFFF for k in range(w.MAX_KEY - w.MAX_KEYS + 1, w.MAX_KEY + 1)}
        b = w.make_bundle(entries, 0xFFFFFFFF, KEY)
        self.assertLessEqual(len(b), w.MAX_BUNDLE_LEN)
        frags = w.split_bundle(b, 0xFFFFFFFF)
        self.assertLessEqual(len(frags), w.MAX_FRAGMENTS)
        self.assertTrue(all(len(f) <= w.MESH_MAX_PAYLOAD for f in frags))
        with self.assertRaises(ValueError):
            w.make_bundle({k: 0 for k in range(w.MAX_KEYS + 1)}, 1, KEY)

    def test_mesh_packet_header_and_link_frame(self):
        p = w.make_mesh_packet(0xFFFF, 0xFFFF, 127, bytes(w.MESH_MAX_PAYLOAD), dest=0xFFFF, hop_limit=15,
                               hop_start=15, flags=3, attempt=15)
        self.assertLessEqual(len(p) - w.MESH_MAX_PAYLOAD, 24)
        f = pb.LinkFrame(ver=1, type=pb.FRAME_TYPE_MESH, src=0xFFFF, payload=p).SerializeToString()
        self.assertLessEqual(len(f), 125)
        with self.assertRaises(ValueError):
            w.make_mesh_packet(1, 1, 16, bytes(w.MESH_MAX_PAYLOAD + 1))

    def test_fw_frames(self):
        d = pb.FwData(version=0xFFFFFFFF, page=0xFFFFFFFF, chunk=42, data=bytes(w.CHUNK)).SerializeToString()
        f = pb.LinkFrame(ver=1, type=pb.FRAME_TYPE_FW_DATA, src=0xFFFF, payload=d).SerializeToString()
        self.assertLessEqual(len(f), 125)
        r = pb.FwRequest(version=0xFFFFFFFF, page=0xFFFFFFFF, missing=(1 << 64) - 1).SerializeToString()
        # Real worst case 6 + 6 + 9 = 21 (docs/swarm-wire.md says <= 18, which is only true for
        # versions/pages under 2^21); the link frame is 14 + 21 = 35, nowhere near 125.
        self.assertEqual(len(r), 21)
        f = pb.LinkFrame(ver=1, type=pb.FRAME_TYPE_FW_REQUEST, src=0xFFFF, dst=0xFFFF, seq=0xFFFFFFFF,
                         payload=r).SerializeToString()
        self.assertLessEqual(len(f), 125)


if __name__ == "__main__":
    unittest.main()
