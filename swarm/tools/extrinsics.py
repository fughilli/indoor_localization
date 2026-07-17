"""Widget sensor extrinsics: standard schema + firmware export formats.

The widget design is meant to be manufactured/customized by third parties,
so sensor placement is configuration, not code. One JSON document (the
source of truth, hand-edited or via //swarm/tools:extrinsics_configurator)
describes every sensor's pose w.r.t. the widget reference frame, and two
derived artifacts feed the firmware:

  - a C header (bake in at build time via a genrule), and
  - a compact CRC-protected binary blob (load dynamically from flash/NVS),
    parsed on-device by swarm/core/src/extrinsics.rs.

Schema (version 1):

  {
    "schema": "widget-extrinsics",
    "version": 1,
    "widget_model": "widget-v1",
    "sensors": {
      "imu": {
        "mapping": {"x": "+x", "z": "+z"},   # sensor axes -> widget axes
        "euler_deg": [0.0, 0.0, 0.0],        # fine trim, roll/pitch/yaw,
                                             #   applied in the sensor frame
        "offset_m": [0.012, -0.003, 0.001],  # sensor origin in widget frame
        "rotation_wxyz": [1, 0, 0, 0]        # DERIVED sensor->widget quat
      },
      ...
    }
  }

`mapping` + `euler_deg` are the human-authorable form; `rotation_wxyz` is
recomputed on every save/export and is what firmware consumes:
v_widget = R(rotation) * v_sensor + offset. Standard sensors: imu, mag,
uwb, gps, alt (ids 1..5); custom sensors may be added with an explicit
"id" >= 128.

Binary blob ("WEXT", little-endian):
  magic 'WEXT' | version u16 | count u16
  per sensor (32 B): id u8 | flags u8 | pad u16 | quat_wxyz 4*f32 | offset 3*f32
  crc32 (zlib/IEEE) u32 over everything before it
"""

import argparse
import json
import struct
import zlib

import numpy as np

SCHEMA_NAME = "widget-extrinsics"
SCHEMA_VERSION = 1
MAGIC = b"WEXT"
SENSOR_IDS = {"imu": 1, "mag": 2, "uwb": 3, "gps": 4, "alt": 5}

_AXES = {
    "+x": np.array([1.0, 0, 0]), "-x": np.array([-1.0, 0, 0]),
    "+y": np.array([0, 1.0, 0]), "-y": np.array([0, -1.0, 0]),
    "+z": np.array([0, 0, 1.0]), "-z": np.array([0, 0, -1.0]),
}


def _euler_matrix(roll, pitch, yaw):
    """R = Rz(yaw) @ Ry(pitch) @ Rx(roll), radians."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def _quat_from_mat(r):
    """Scalar-first quaternion from a rotation matrix (Shepperd's method)."""
    tr = r[0, 0] + r[1, 1] + r[2, 2]
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        q = [0.25 * s, (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s,
             (r[1, 0] - r[0, 1]) / s]
    elif r[0, 0] >= r[1, 1] and r[0, 0] >= r[2, 2]:
        s = np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
        q = [(r[2, 1] - r[1, 2]) / s, 0.25 * s, (r[0, 1] + r[1, 0]) / s,
             (r[0, 2] + r[2, 0]) / s]
    elif r[1, 1] >= r[2, 2]:
        s = np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
        q = [(r[0, 2] - r[2, 0]) / s, (r[0, 1] + r[1, 0]) / s, 0.25 * s,
             (r[1, 2] + r[2, 1]) / s]
    else:
        s = np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
        q = [(r[1, 0] - r[0, 1]) / s, (r[0, 2] + r[2, 0]) / s,
             (r[1, 2] + r[2, 1]) / s, 0.25 * s]
    q = np.array(q)
    q /= np.linalg.norm(q)
    if q[0] < 0:
        q = -q
    return q


def rotation_matrix(sensor_cfg):
    """Sensor->widget rotation from mapping + euler trim."""
    m = sensor_cfg.get("mapping", {"x": "+x", "z": "+z"})
    x = _AXES[m["x"]]
    z = _AXES[m["z"]]
    if abs(float(x @ z)) > 1e-6:
        raise ValueError("mapping axes must be orthogonal: %s" % m)
    y = np.cross(z, x)
    r_map = np.column_stack([x, y, z])
    roll, pitch, yaw = np.radians(sensor_cfg.get("euler_deg", [0, 0, 0]))
    return r_map @ _euler_matrix(roll, pitch, yaw)


def sensor_id(name, cfg):
    if name in SENSOR_IDS:
        return SENSOR_IDS[name]
    sid = cfg.get("id")
    if not isinstance(sid, int) or not 128 <= sid <= 255:
        raise ValueError(
            "custom sensor %r needs an explicit \"id\" in 128..255" % name)
    return sid


def validate(config):
    """Raise ValueError on schema problems; returns the config."""
    if config.get("schema") != SCHEMA_NAME:
        raise ValueError("schema field must be %r" % SCHEMA_NAME)
    if config.get("version") != SCHEMA_VERSION:
        raise ValueError("unsupported version %r" % config.get("version"))
    sensors = config.get("sensors")
    if not isinstance(sensors, dict) or not sensors:
        raise ValueError("sensors must be a non-empty object")
    ids = set()
    for name, cfg in sensors.items():
        sid = sensor_id(name, cfg)
        if sid in ids:
            raise ValueError("duplicate sensor id %d" % sid)
        ids.add(sid)
        off = cfg.get("offset_m", [0, 0, 0])
        if len(off) != 3:
            raise ValueError("%s: offset_m must have 3 entries" % name)
        rotation_matrix(cfg)  # raises on bad mapping
    return config


def derive(config):
    """Recompute every sensor's rotation_wxyz from mapping + euler."""
    validate(config)
    for cfg in config["sensors"].values():
        q = _quat_from_mat(rotation_matrix(cfg))
        cfg["rotation_wxyz"] = [round(float(v), 6) for v in q]
    return config


def encode_blob(config):
    """Config -> WEXT binary blob (see module docstring for layout)."""
    config = derive(config)
    sensors = sorted(config["sensors"].items(),
                     key=lambda kv: sensor_id(*kv))
    out = struct.pack("<4sHH", MAGIC, SCHEMA_VERSION, len(sensors))
    for name, cfg in sensors:
        q = cfg["rotation_wxyz"]
        off = cfg.get("offset_m", [0, 0, 0])
        out += struct.pack("<BBH4f3f", sensor_id(name, cfg), 0, 0,
                           *q, *[float(v) for v in off])
    return out + struct.pack("<I", zlib.crc32(out) & 0xFFFFFFFF)


def decode_blob(blob):
    """WEXT blob -> list of {id, rotation_wxyz, offset_m}. Raises on
    corruption; the reference for the on-device Rust parser."""
    if len(blob) < 12 or blob[:4] != MAGIC:
        raise ValueError("not a WEXT blob")
    (crc,) = struct.unpack("<I", blob[-4:])
    if crc != (zlib.crc32(blob[:-4]) & 0xFFFFFFFF):
        raise ValueError("CRC mismatch")
    _, version, count = struct.unpack("<4sHH", blob[:8])
    if version != SCHEMA_VERSION:
        raise ValueError("unsupported blob version %d" % version)
    if len(blob) != 8 + 32 * count + 4:
        raise ValueError("length mismatch")
    out = []
    for i in range(count):
        rec = blob[8 + 32 * i:8 + 32 * (i + 1)]
        vals = struct.unpack("<BBH4f3f", rec)
        out.append({"id": vals[0],
                    "rotation_wxyz": list(vals[3:7]),
                    "offset_m": list(vals[7:10])})
    return out


def c_header(config):
    """Config -> C header text for build-time baking."""
    config = derive(config)
    sensors = sorted(config["sensors"].items(),
                     key=lambda kv: sensor_id(*kv))
    lines = [
        "// Generated by //swarm/tools:extrinsics from widget model %r."
        % config.get("widget_model", "?"),
        "// Sensor->widget: v_widget = R(quat_wxyz) * v_sensor + offset_m.",
        "// Do not edit; regenerate from the JSON config.",
        "#pragma once",
        "",
        "#define WIDGET_EXTRINSICS_VERSION %d" % SCHEMA_VERSION,
        "#define WIDGET_EXTRINSICS_SENSOR_COUNT %d" % len(sensors),
        "",
        "typedef struct {",
        "    unsigned char sensor_id;  // imu=1 mag=2 uwb=3 gps=4 alt=5",
        "    float quat_wxyz[4];",
        "    float offset_m[3];",
        "} widget_extrinsic_t;",
        "",
        "static const widget_extrinsic_t WIDGET_EXTRINSICS[] = {",
    ]
    for name, cfg in sensors:
        q = cfg["rotation_wxyz"]
        off = cfg.get("offset_m", [0, 0, 0])
        lines.append(
            "    {%d, {%sf, %sf, %sf, %sf}, {%sf, %sf, %sf}},  // %s"
            % (sensor_id(name, cfg), *[repr(float(v)) for v in q],
               *[repr(float(v)) for v in off], name))
    lines += ["};", ""]
    return "\n".join(lines)


def load(path):
    with open(path) as f:
        return validate(json.load(f))


def save(config, path):
    with open(path, "w") as f:
        json.dump(derive(config), f, indent=2)
        f.write("\n")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("command",
                    choices=["validate", "export-blob", "export-header"])
    ap.add_argument("--config", required=True)
    ap.add_argument("--out", help="output path (default: stdout for header)")
    args = ap.parse_args()
    config = load(args.config)
    if args.command == "validate":
        derive(config)
        print("OK: %d sensors" % len(config["sensors"]))
    elif args.command == "export-blob":
        blob = encode_blob(config)
        with open(args.out, "wb") as f:
            f.write(blob)
        print("wrote %d bytes -> %s" % (len(blob), args.out))
    elif args.command == "export-header":
        text = c_header(config)
        if args.out:
            with open(args.out, "w") as f:
                f.write(text)
        else:
            print(text)


if __name__ == "__main__":
    main()
