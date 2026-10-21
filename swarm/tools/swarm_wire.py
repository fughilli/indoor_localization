"""Host side of the swarm network's wire formats (swarm/net, Rust): tagged
settings bundles and override commands, firmware manifests, settings
reports. Byte-for-byte what the nodes check; swarm_wire_test.py pins the
encodings against vectors the Rust tests also assert.

Only the standard library here; signing a firmware manifest (P-256) needs
the `cryptography` package and lives in sign_manifest().
"""

import hashlib
import hmac
import struct

TAG_LEN = 4
DOMAIN_BUNDLE = b"B"
DOMAIN_OVERRIDE = b"O"

# Mesh ports (swarm/net/src/settings.rs, net.rs).
PORT_BUNDLE_DATA = 2
PORT_BUNDLE_REQUEST = 3
PORT_OVERRIDE = 4
PORT_SETTINGS_GET = 5
PORT_SETTINGS_REPORT = 6
PORT_FW_MANIFEST = 7

CMD_SET = 1
CMD_CLEAR = 2
CMD_MAX_ENTRIES = 16
MAX_KEYS = 96

PAGE_SIZE = 4096
CHUNK = 96
MANIFEST_LEN = 108

SOURCE_SWARM = 1
SOURCE_OVERRIDE = 2


def tag(key, domain, *parts):
    """First 4 bytes of HMAC-SHA256(key, domain || parts)."""
    return hmac.new(key, domain + b"".join(parts), hashlib.sha256).digest()[:TAG_LEN]


def encode_value(kind, value):
    """Raw u32 for a setting value of registry kind 'f32' / 'i32' / 'bool'."""
    if kind == "f32":
        return struct.unpack("<I", struct.pack("<f", float(value)))[0]
    if kind == "i32":
        return int(value) & 0xFFFFFFFF
    if kind == "bool":
        if isinstance(value, str):
            value = value.lower() in ("1", "true", "yes", "on")
        return 1 if value else 0
    raise ValueError("unknown kind %r" % kind)


def decode_value(kind, raw):
    if kind == "f32":
        return struct.unpack("<f", struct.pack("<I", raw))[0]
    if kind == "i32":
        return struct.unpack("<i", struct.pack("<I", raw))[0]
    return raw != 0


def encode_layer(entries):
    """{key: raw} -> count u16 + (key u16, value u32)*, sorted by key."""
    if len(entries) > MAX_KEYS:
        raise ValueError("too many settings in one layer")
    out = struct.pack("<H", len(entries))
    for k in sorted(entries):
        out += struct.pack("<HI", k, entries[k])
    return out


def make_bundle(entries, generation, key):
    """Tagged swarm-layer bundle: generation u32 + layer + tag."""
    body = struct.pack("<I", generation) + encode_layer(entries)
    return body + tag(key, DOMAIN_BUNDLE, body)


def make_override(dest, kind, seq, entries, key):
    """Tagged override command for node `dest` (mesh port PORT_OVERRIDE).
    entries: [(key, raw)] for CMD_SET, [(key, 0)] or [] for CMD_CLEAR."""
    if len(entries) > CMD_MAX_ENTRIES:
        raise ValueError("at most %d settings per command" % CMD_MAX_ENTRIES)
    body = struct.pack("<BIB", kind, seq, len(entries))
    for k, v in entries:
        body += struct.pack("<HI", k, v)
    return body + tag(key, DOMAIN_OVERRIDE, struct.pack("<H", dest), body)


def parse_report(payload):
    """Settings report part -> (part, parts, generation, override_seq,
    [(key, source, raw)])."""
    part, parts, generation, seq = struct.unpack_from("<BBII", payload)
    out = []
    for at in range(10, len(payload) - 5, 6):
        k, v = struct.unpack_from("<HI", payload, at)
        out.append((k & 0x3FFF, k >> 14, v))
    return part, parts, generation, seq, out


def manifest_signed_bytes(version, image, flags=0):
    return struct.pack("<II", version, len(image)) + hashlib.sha256(image).digest() + struct.pack("<I", flags)


def sign_manifest(version, image, private_key_pem, flags=0):
    """108-byte manifest: signed bytes + ECDSA P-256 r||s over their
    SHA-256. Needs the `cryptography` package."""
    from cryptography.hazmat.primitives import hashes, serialization
    from cryptography.hazmat.primitives.asymmetric import ec, utils

    sk = serialization.load_pem_private_key(private_key_pem, None)
    signed = manifest_signed_bytes(version, image, flags)
    r, s = utils.decode_dss_signature(sk.sign(signed, ec.ECDSA(hashes.SHA256())))
    return signed + r.to_bytes(32, "big") + s.to_bytes(32, "big")


def chunks(image):
    """(page, chunk, bytes) in the order a node stages them."""
    for page in range((len(image) + PAGE_SIZE - 1) // PAGE_SIZE):
        p = image[page * PAGE_SIZE:(page + 1) * PAGE_SIZE]
        for c in range((len(p) + CHUNK - 1) // CHUNK):
            yield page, c, p[c * CHUNK:(c + 1) * CHUNK]
