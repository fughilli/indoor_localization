"""Widget sensor intrinsics: standard schema for per-unit sensor quality
parameters — exactly what //swarm/tools:characterize measures — so a
manufacturer/user can optionally ship them with a widget and improve
algorithm performance (estimator noise model, measurement corrections)
over the conservative built-in defaults.

Schema (version 1) — every section optional; absent sections mean
"use firmware defaults":

  {
    "schema": "widget-intrinsics",
    "version": 1,
    "widget_model": "widget-v1",
    "widget_serial": "0042",            # optional, per-unit calibration
    "imu": {
      "accel_noise_density": 0.005,     # m/s^2/sqrt(Hz)   <- characterize imu
      "gyro_noise_density": 2e-4,       # rad/s/sqrt(Hz)
      "accel_bias_instability": 5e-4,   # m/s^2
      "gyro_bias_instability": 6e-5,    # rad/s
      "gyro_bias": [0.01, -0.02, 0.005] # rad/s, initial estimate
    },
    "uwb_range": {
      "bias0_m": 0.05,                  # err(d) = bias0 + scale*d
      "scale_ppm": 200.0,               #   <- characterize range
      "sigma_m": 0.10,
      "outlier_frac": 0.05
    },
    "uwb_aoa": {
      "plane_normal": [0, 0, 1],        # SENSOR frame <- characterize aoa
      "sign": 1.0,
      "offset_rad": 0.3,
      "sigma_rad": 0.12
    },
    "mag": {
      "hard_iron": [12.0, -5.0, 3.0],   # <- characterize mag
      "soft_iron": [[...3x3...]],       # unit-determinant, symmetric
      "sigma": 0.02                     # direction components after cal
    },
    "gps": {"sigma_h_m": 1.5, "sigma_v_m": 3.0},
    "alt": {"sigma_m": 0.3}
  }

The characterize CLI merges its analysis reports straight into a config
with --intrinsics (see swarm/tools/characterize.py).

Binary blob ("WINT", little-endian), sectioned for forward compatibility —
parsers skip unknown section kinds:

  magic 'WINT' | version u16 | section_count u16
  per section: kind u8 | pad u8 | payload_len u16 | payload (f32 array)
  crc32 (zlib/IEEE) u32 over everything before it

  kind 1 imu:       9 f32  (densities, instabilities, gyro_bias, pad0..)
  kind 2 uwb_range: 4 f32  (bias0, scale_ppm, sigma, outlier_frac)
  kind 3 uwb_aoa:   6 f32  (normal xyz, sign, offset, sigma)
  kind 4 mag:      13 f32  (hard_iron 3, soft_iron row-major 9, sigma)
  kind 5 gps:       2 f32  (sigma_h, sigma_v)
  kind 6 alt:       1 f32  (sigma)
"""

import argparse
import json
import struct
import zlib

SCHEMA_NAME = "widget-intrinsics"
SCHEMA_VERSION = 1
MAGIC = b"WINT"

# kind -> (section name, ordered payload fields); vector fields flatten.
_SECTIONS = {
    1: ("imu", [("accel_noise_density", 1), ("gyro_noise_density", 1),
                ("accel_bias_instability", 1), ("gyro_bias_instability", 1),
                ("gyro_bias", 3)]),
    2: ("uwb_range", [("bias0_m", 1), ("scale_ppm", 1), ("sigma_m", 1),
                      ("outlier_frac", 1)]),
    3: ("uwb_aoa", [("plane_normal", 3), ("sign", 1), ("offset_rad", 1),
                    ("sigma_rad", 1)]),
    4: ("mag", [("hard_iron", 3), ("soft_iron", 9), ("sigma", 1)]),
    5: ("gps", [("sigma_h_m", 1), ("sigma_v_m", 1)]),
    6: ("alt", [("sigma_m", 1)]),
}
_KIND_BY_NAME = {name: kind for kind, (name, _) in _SECTIONS.items()}


def empty(widget_model="unknown"):
    return {"schema": SCHEMA_NAME, "version": SCHEMA_VERSION,
            "widget_model": widget_model}


def validate(config):
    if config.get("schema") != SCHEMA_NAME:
        raise ValueError("schema field must be %r" % SCHEMA_NAME)
    if config.get("version") != SCHEMA_VERSION:
        raise ValueError("unsupported version %r" % config.get("version"))
    for name, kind in _KIND_BY_NAME.items():
        if name not in config:
            continue
        for field, width in _SECTIONS[kind][1]:
            if field not in config[name]:
                continue
            v = config[name][field]
            got = len(_flatten(v)) if isinstance(v, list) else 1
            if got != width:
                raise ValueError("%s.%s: expected %d value(s), got %d"
                                 % (name, field, width, got))
    return config


def _flatten(v):
    if isinstance(v, list):
        out = []
        for e in v:
            out.extend(_flatten(e))
        return out
    return [float(v)]


def _section_payload(config, kind):
    """Flat f32 list for a section, or None if the section is absent.
    Missing fields inside a present section encode as NaN ("use default")."""
    name, fields = _SECTIONS[kind]
    if name not in config:
        return None
    out = []
    for field, width in fields:
        if field in config[name]:
            out.extend(_flatten(config[name][field]))
        else:
            out.extend([float("nan")] * width)
    return out


def encode_blob(config):
    validate(config)
    sections = []
    for kind in sorted(_SECTIONS):
        payload = _section_payload(config, kind)
        if payload is not None:
            sections.append((kind, payload))
    out = struct.pack("<4sHH", MAGIC, SCHEMA_VERSION, len(sections))
    for kind, payload in sections:
        out += struct.pack("<BBH", kind, 0, 4 * len(payload))
        out += struct.pack("<%df" % len(payload), *payload)
    return out + struct.pack("<I", zlib.crc32(out) & 0xFFFFFFFF)


def decode_blob(blob):
    """Reference decoder (the on-device one is
    swarm/core/src/intrinsics.rs). Unknown section kinds are skipped."""
    if len(blob) < 12 or blob[:4] != MAGIC:
        raise ValueError("not a WINT blob")
    (crc,) = struct.unpack("<I", blob[-4:])
    if crc != (zlib.crc32(blob[:-4]) & 0xFFFFFFFF):
        raise ValueError("CRC mismatch")
    _, version, count = struct.unpack("<4sHH", blob[:8])
    if version != SCHEMA_VERSION:
        raise ValueError("unsupported blob version %d" % version)
    config = empty()
    at = 8
    for _ in range(count):
        kind, _, plen = struct.unpack("<BBH", blob[at:at + 4])
        payload = struct.unpack("<%df" % (plen // 4),
                                blob[at + 4:at + 4 + plen])
        at += 4 + plen
        if kind not in _SECTIONS:
            continue
        name, fields = _SECTIONS[kind]
        section, i = {}, 0
        for field, width in fields:
            vals = payload[i:i + width]
            i += width
            if any(v == v for v in vals):  # skip all-NaN (absent) fields
                section[field] = (round(vals[0], 8) if width == 1
                                  else [round(v, 8) for v in vals])
        if name == "mag" and "soft_iron" in section:
            si = section["soft_iron"]
            section["soft_iron"] = [si[0:3], si[3:6], si[6:9]]
        config[name] = section
    return config


def merge_report(config, kind, report):
    """Merge a characterize-CLI analysis report into an intrinsics config.
    kind: 'imu' | 'range' | 'aoa' | 'mag'."""
    if kind == "imu":
        acc = report["accel_axes"]
        gyr = report["gyro_axes"]
        mean = lambda axes, f: sum(a[f] for a in axes) / len(axes)
        config["imu"] = {
            "accel_noise_density": round(mean(acc, "noise_density"), 8),
            "gyro_noise_density": round(mean(gyr, "noise_density"), 8),
            "accel_bias_instability": round(mean(acc, "bias_instability"), 8),
            "gyro_bias_instability": round(mean(gyr, "bias_instability"), 8),
            "gyro_bias": report["gyro_bias"],
        }
    elif kind == "range":
        model = report.get("error_model", {})
        segs = report["segments"]
        config["uwb_range"] = {
            "bias0_m": model.get("bias0_m", 0.0),
            "scale_ppm": model.get("scale_ppm", 0.0),
            "sigma_m": round(sum(s["sigma_m"] for s in segs) / len(segs), 6),
            "outlier_frac": round(sum(s["outlier_frac"] for s in segs)
                                  / len(segs), 6),
        }
    elif kind == "aoa":
        config["uwb_aoa"] = {
            "plane_normal": report["normal"],
            "sign": report["sign"],
            "offset_rad": report["offset_rad"],
            "sigma_rad": report["sigma_rad"],
        }
    elif kind == "mag":
        config["mag"] = {
            "hard_iron": report["hard_iron"],
            "soft_iron": report["soft_iron"],
            "sigma": report["residual_frac"],
        }
    else:
        raise ValueError("no intrinsics mapping for %r" % kind)
    return validate(config)


def c_header(config):
    """Build-time C header: one struct per present section, NaN = default."""
    validate(config)
    lines = [
        "// Generated by //swarm/tools:intrinsics from widget model %r."
        % config.get("widget_model", "?"),
        "// NAN means 'not characterized: use the firmware default'.",
        "#pragma once",
        "#include <math.h>",
        "",
        "#define WIDGET_INTRINSICS_VERSION %d" % SCHEMA_VERSION,
        "",
    ]
    for kind in sorted(_SECTIONS):
        payload = _section_payload(config, kind)
        name = _SECTIONS[kind][0]
        macro = "WIDGET_INTRINSICS_HAS_%s" % name.upper()
        if payload is None:
            lines.append("#define %s 0" % macro)
            continue
        vals = ", ".join("NAN" if v != v else repr(float(v)) + "f"
                         for v in payload)
        lines += ["#define %s 1" % macro,
                  "static const float WIDGET_INTRINSICS_%s[] = {%s};"
                  % (name.upper(), vals)]
    lines.append("")
    return "\n".join(lines)


def load(path):
    with open(path) as f:
        return validate(json.load(f))


def save(config, path):
    with open(path, "w") as f:
        json.dump(validate(config), f, indent=2)
        f.write("\n")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("command",
                    choices=["validate", "export-blob", "export-header"])
    ap.add_argument("--config", required=True)
    ap.add_argument("--out")
    args = ap.parse_args()
    config = load(args.config)
    if args.command == "validate":
        present = [n for n in _KIND_BY_NAME if n in config]
        print("OK: sections %s" % ", ".join(sorted(present)))
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
