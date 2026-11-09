"""Host side of the swarm network's wire formats: tagged settings bundles and
override commands, bundle fragments, settings reports, mesh packets and
firmware manifests, all protobuf messages of swarm/proto/swarm_wire.proto
(contract and byte budgets: docs/swarm-wire.md).

Signed and tagged things are `{body, tag}` envelopes: the HMAC (settings) or
ECDSA signature (manifest) covers the exact serialized body bytes. Nothing
here re-encodes a body after it has been signed; parse helpers hand the
original body bytes back for verification.

Signing a firmware manifest (P-256) needs the `cryptography` package and
lives in sign_manifest() / verify_manifest().
"""

import hashlib
import hmac
import struct

import swarm_wire_pb2 as pb

TAG_LEN = 4
DOMAIN_BUNDLE = b"B"
DOMAIN_OVERRIDE = b"O"

# Mesh ports (the Port enum). The manifest is not a mesh port: it is its own
# link frame type, handed to the node over USB with the `fw_manifest` command.
PORT_ACK = pb.PORT_ACK
PORT_BUNDLE_DATA = pb.PORT_BUNDLE_DATA
PORT_BUNDLE_REQUEST = pb.PORT_BUNDLE_REQUEST
PORT_OVERRIDE = pb.PORT_OVERRIDE
PORT_SETTINGS_GET = pb.PORT_SETTINGS_GET
PORT_SETTINGS_REPORT = pb.PORT_SETTINGS_REPORT
PORT_APP = pb.PORT_APP

CMD_SET = pb.OverrideBody.KIND_SET
CMD_CLEAR = pb.OverrideBody.KIND_CLEAR

# Derived constants (docs/swarm-wire.md).
MESH_MAX_PAYLOAD = 88
FRAG_DATA = 72
MAX_FRAGMENTS = 32
CMD_MAX_ENTRIES = 7
REPORT_ENTRIES_PER_PART = 6
MAX_KEYS = 96
# Registry ids fit 14 bits (the old packed report reserved the top two bits for the
# source): two-byte varints, which is what the docs/swarm-wire.md budgets assume.
MAX_KEY = 0x3FFF
MAX_MANIFEST_LEN = 114
MAX_BUNDLE_LEN = 1024

PAGE_SIZE = 4096
CHUNK = 96

SOURCE_SWARM = 1
SOURCE_OVERRIDE = 2


def tag(key, domain, *parts):
    """First 4 bytes of HMAC-SHA256(key, domain || parts)."""
    return hmac.new(key, domain + b"".join(parts), hashlib.sha256).digest()[:TAG_LEN]


def _check_key(k):
    if not 0 <= k <= MAX_KEY:
        raise ValueError("setting key %d outside the registry range 0..%d" % (k, MAX_KEY))


def _ser(msg):
    return msg.SerializeToString()


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


# ---- settings bundles --------------------------------------------------------

def make_bundle_body(entries, generation):
    """Serialized BundleBody from {key: raw}, sorted by key."""
    if len(entries) > MAX_KEYS:
        raise ValueError("too many settings in one layer")
    body = pb.BundleBody(generation=generation)
    for k in sorted(entries):
        _check_key(k)
        body.settings.add(key=k, value=entries[k])
    return _ser(body)


def make_bundle(entries, generation, key):
    """Serialized, tagged swarm-layer Bundle: {body, tag(key, 'B' || body)}."""
    body = make_bundle_body(entries, generation)
    out = _ser(pb.Bundle(body=body, tag=tag(key, DOMAIN_BUNDLE, body)))
    if len(out) > MAX_BUNDLE_LEN:
        raise ValueError("bundle of %d bytes exceeds %d" % (len(out), MAX_BUNDLE_LEN))
    return out


def parse_bundle(data, key=None):
    """Serialized Bundle -> (generation, {key: raw}); if `key` is given the
    tag is verified over the exact body bytes first (ValueError if wrong)."""
    env = pb.Bundle.FromString(data)
    if key is not None and not hmac.compare_digest(env.tag, tag(key, DOMAIN_BUNDLE, env.body)):
        raise ValueError("bad bundle tag")
    body = pb.BundleBody.FromString(env.body)
    return body.generation, {s.key: s.value for s in body.settings}


def make_fragment(generation, total_len, offset, data):
    if offset % FRAG_DATA or len(data) > FRAG_DATA:
        raise ValueError("fragment offset must be a multiple of %d, data at most %d bytes" % (FRAG_DATA, FRAG_DATA))
    return _ser(pb.BundleFragment(generation=generation, total_len=total_len, offset=offset, data=data))


def split_bundle(bundle, generation):
    """Serialized BundleFragments of a serialized Bundle (what a node
    broadcasts; the host sends the whole Bundle over USB and the node does this)."""
    frags = [make_fragment(generation, len(bundle), off, bundle[off:off + FRAG_DATA])
             for off in range(0, len(bundle), FRAG_DATA)]
    if len(frags) > MAX_FRAGMENTS:
        raise ValueError("bundle needs %d fragments, at most %d" % (len(frags), MAX_FRAGMENTS))
    return frags


def join_fragments(frags):
    """Inverse of split_bundle (any order); checks generation, length, offsets."""
    msgs = sorted((pb.BundleFragment.FromString(f) for f in frags), key=lambda m: m.offset)
    if not msgs:
        raise ValueError("no fragments")
    total, gen = msgs[0].total_len, msgs[0].generation
    out = b""
    for m in msgs:
        if m.generation != gen or m.total_len != total or m.offset != len(out) or m.offset % FRAG_DATA:
            raise ValueError("inconsistent fragments")
        out += m.data
    if len(out) != total:
        raise ValueError("incomplete bundle")
    return out


def make_bundle_request(generation):
    return _ser(pb.BundleRequest(generation=generation))


# ---- override commands -------------------------------------------------------

def make_override(dest, kind, seq, entries, key):
    """Serialized, tagged Override for node `dest` (mesh port PORT_OVERRIDE).
    entries: [(key, raw)] for CMD_SET, [(key, 0)] or [] for CMD_CLEAR. The tag
    covers 'O' || dest (u16 LE) || body."""
    if len(entries) > CMD_MAX_ENTRIES:
        raise ValueError("at most %d settings per command" % CMD_MAX_ENTRIES)
    body = pb.OverrideBody(kind=kind, seq=seq)
    for k, v in entries:
        _check_key(k)
        body.settings.add(key=k, value=v)
    body = _ser(body)
    return _ser(pb.Override(body=body, tag=tag(key, DOMAIN_OVERRIDE, struct.pack("<H", dest), body)))


def parse_override(data, dest=None, key=None):
    """Serialized Override -> (kind, seq, [(key, raw)]); verifies the tag for
    `dest` over the exact body bytes when `key` is given."""
    env = pb.Override.FromString(data)
    if key is not None and not hmac.compare_digest(
            env.tag, tag(key, DOMAIN_OVERRIDE, struct.pack("<H", dest), env.body)):
        raise ValueError("bad override tag")
    body = pb.OverrideBody.FromString(env.body)
    return body.kind, body.seq, [(s.key, s.value) for s in body.settings]


# ---- settings reports --------------------------------------------------------

def parse_report(payload):
    """Settings report part -> (part, parts, generation, override_seq,
    [(key, source, raw)])."""
    m = pb.SettingsReport.FromString(payload)
    return m.part, m.parts, m.generation, m.override_seq, [(s.key, s.source, s.value) for s in m.settings]


def make_report(part, parts, generation, override_seq, entries):
    """Serialized SettingsReport part; entries [(key, source, raw)]."""
    if len(entries) > REPORT_ENTRIES_PER_PART:
        raise ValueError("at most %d entries per report part" % REPORT_ENTRIES_PER_PART)
    m = pb.SettingsReport(part=part, parts=parts, generation=generation, override_seq=override_seq)
    for k, src, v in entries:
        _check_key(k)
        m.settings.add(key=k, value=v, source=src)
    return _ser(m)


# ---- mesh packets ------------------------------------------------------------

def make_mesh_packet(origin, id, port, payload, dest=None, hop_limit=0, hop_start=0, flags=0, attempt=0):
    """Serialized MeshPacket. dest=None is a broadcast (the field is omitted;
    an explicit dest, even 0, is encoded)."""
    if len(payload) > MESH_MAX_PAYLOAD:
        raise ValueError("mesh payload of %d bytes exceeds %d" % (len(payload), MESH_MAX_PAYLOAD))
    m = pb.MeshPacket(origin=origin, id=id, hop_limit=hop_limit, hop_start=hop_start,
                      port=port, flags=flags, attempt=attempt, payload=payload)
    if dest is not None:
        m.dest = dest
    return _ser(m)


def parse_mesh_packet(data):
    """Serialized MeshPacket -> the pb message (use HasField('dest'): absent is broadcast)."""
    return pb.MeshPacket.FromString(data)


# ---- firmware manifests ------------------------------------------------------

def manifest_signed_bytes(version, image, flags=0):
    """Serialized ManifestBody: the exact bytes the signature covers."""
    return _ser(pb.ManifestBody(version=version, size=len(image), sha256=hashlib.sha256(image).digest(),
                                flags=flags))


def sign_manifest(version, image, private_key_pem, flags=0):
    """Serialized Manifest {body, signature}: ECDSA P-256 r||s (big-endian)
    over SHA-256 of the body bytes. Needs the `cryptography` package. The
    result is <= 114 bytes; send it with the `fw_manifest` USB command, and
    to resume a push re-send these identical bytes (signatures differ per signing)."""
    from cryptography.hazmat.primitives import hashes, serialization
    from cryptography.hazmat.primitives.asymmetric import ec, utils

    sk = serialization.load_pem_private_key(private_key_pem, None)
    body = manifest_signed_bytes(version, image, flags)
    r, s = utils.decode_dss_signature(sk.sign(body, ec.ECDSA(hashes.SHA256())))
    out = _ser(pb.Manifest(body=body, signature=r.to_bytes(32, "big") + s.to_bytes(32, "big")))
    if len(out) > MAX_MANIFEST_LEN:
        # Only with flags != 0 and every varint wide: 116 bytes (docs/swarm-wire.md budgets 114
        # assuming flags absent). Reported, not papered over.
        raise ValueError("manifest of %d bytes exceeds %d" % (len(out), MAX_MANIFEST_LEN))
    return out


def verify_manifest(manifest, public_key):
    """Verify a serialized Manifest against a `cryptography` P-256 public key
    over the exact body bytes; returns the parsed ManifestBody, or raises."""
    from cryptography.hazmat.primitives import hashes
    from cryptography.hazmat.primitives.asymmetric import ec, utils

    m = pb.Manifest.FromString(manifest)
    if len(m.signature) != 64:
        raise ValueError("signature must be 64 bytes")
    der = utils.encode_dss_signature(int.from_bytes(m.signature[:32], "big"),
                                     int.from_bytes(m.signature[32:], "big"))
    public_key.verify(der, m.body, ec.ECDSA(hashes.SHA256()))
    return pb.ManifestBody.FromString(m.body)


def chunks(image):
    """(page, chunk, bytes) in the order a node stages them."""
    for page in range((len(image) + PAGE_SIZE - 1) // PAGE_SIZE):
        p = image[page * PAGE_SIZE:(page + 1) * PAGE_SIZE]
        for c in range((len(p) + CHUNK - 1) // CHUNK):
            yield page, c, p[c * CHUNK:(c + 1) * CHUNK]
