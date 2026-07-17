"""Widget configuration (widget.v1 protobuf): load/validate/derive/export.

The schema lives in swarm/proto/widget_config.proto; this module is the
host-side toolkit around it. One WidgetConfig message carries extrinsics
(sensor poses; authoring form = axis mapping + Euler trim, derived form =
canonical sensor->widget quaternion) and optional intrinsics (per-unit
sensor quality from //swarm/tools:characterize).

Forms handled here, all the same schema:
  - .txtpb textproto on disk — the human-authored source of truth,
  - dict/JSON (canonical proto JSON, snake_case) — the configurator HTTP API,
  - binary proto — firmware storage/transport (decoded on-device by the
    micropb bindings, //swarm/config), plus a C-array header wrapping those
    same bytes for build-time baking.
"""

import argparse

import numpy as np
from google.protobuf import json_format, text_format

import widget_config_pb2 as pb

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


def rotation_matrix(sensor):
    """Sensor->widget rotation from a SensorExtrinsic's mapping + trim."""
    mx = sensor.mapping.x or "+x"
    mz = sensor.mapping.z or "+z"
    if mx not in _AXES or mz not in _AXES:
        raise ValueError("%s: bad mapping axis %r/%r" % (sensor.name, mx, mz))
    x, z = _AXES[mx], _AXES[mz]
    if abs(float(x @ z)) > 1e-6:
        raise ValueError("%s: mapping axes must be orthogonal (%s, %s)"
                         % (sensor.name, mx, mz))
    y = np.cross(z, x)
    r_map = np.column_stack([x, y, z])
    e = sensor.euler_deg
    return r_map @ _euler_matrix(*np.radians([e.x, e.y, e.z]))


def validate(config):
    """Raise ValueError on schema problems; returns the config."""
    ids = set()
    for s in config.extrinsics.sensors:
        if not (s.id in SENSOR_IDS.values() or 128 <= s.id <= 255):
            raise ValueError("sensor %r: id %d not a standard kind or in "
                             "128..255" % (s.name, s.id))
        if s.id in ids:
            raise ValueError("duplicate sensor id %d" % s.id)
        ids.add(s.id)
        rotation_matrix(s)  # raises on bad mapping
    mag = config.intrinsics.mag
    if mag.soft_iron and len(mag.soft_iron) != 9:
        raise ValueError("mag.soft_iron must have 9 (row-major) entries")
    return config


def derive(config):
    """Recompute every sensor's derived rotation; normalize authoring fields
    so downstream consumers (UI, exports) always see them populated."""
    validate(config)
    for s in config.extrinsics.sensors:
        if not s.mapping.x:
            s.mapping.x = "+x"
        if not s.mapping.z:
            s.mapping.z = "+z"
        s.euler_deg.SetInParent()
        s.offset_m.SetInParent()
        q = _quat_from_mat(rotation_matrix(s))
        s.rotation.w, s.rotation.x, s.rotation.y, s.rotation.z = (
            round(float(v), 6) for v in q)
    return config


def load(path):
    with open(path) as f:
        return validate(text_format.Parse(f.read(), pb.WidgetConfig()))


def save(config, path):
    with open(path, "w") as f:
        f.write(text_format.MessageToString(config))


def to_dict(config):
    """Canonical proto JSON dict (snake_case) for the configurator UI."""
    return json_format.MessageToDict(
        config, preserving_proto_field_name=True,
        always_print_fields_with_no_presence=True)


def from_dict(d):
    return json_format.ParseDict(d, pb.WidgetConfig())


def encode_binary(config):
    """Firmware wire/storage form: canonical binary proto of the derived
    config. The reference bytes for the micropb conformance test."""
    return derive(config).SerializeToString(deterministic=True)


def c_array_header(config):
    """Build-time baking: the serialized WidgetConfig as a C byte array the
    firmware feeds to its micropb decoder at boot."""
    data = encode_binary(config)
    body = ",\n    ".join(
        ", ".join("0x%02x" % b for b in data[i:i + 12])
        for i in range(0, len(data), 12))
    return "\n".join([
        "// Generated by //swarm/tools:widget_config_tool — serialized",
        "// widget.v1.WidgetConfig (decode with the micropb bindings).",
        "#pragma once",
        "",
        "static const unsigned char WIDGET_CONFIG_PB[] = {",
        "    " + body,
        "};",
        "static const unsigned int WIDGET_CONFIG_PB_LEN = %du;" % len(data),
        "",
    ])


def merge_report(config, kind, report):
    """Merge a characterize-CLI analysis report into config.intrinsics.
    kind: 'imu' | 'range' | 'aoa' | 'mag'."""
    intr = config.intrinsics
    if kind == "imu":
        acc, gyr = report["accel_axes"], report["gyro_axes"]
        mean = lambda axes, f: sum(a[f] for a in axes) / len(axes)
        imu = intr.imu
        imu.accel_noise_density = round(mean(acc, "noise_density"), 8)
        imu.gyro_noise_density = round(mean(gyr, "noise_density"), 8)
        imu.accel_bias_instability = round(mean(acc, "bias_instability"), 8)
        imu.gyro_bias_instability = round(mean(gyr, "bias_instability"), 8)
        (imu.gyro_bias.x, imu.gyro_bias.y, imu.gyro_bias.z) = report["gyro_bias"]
    elif kind == "range":
        model = report.get("error_model", {})
        segs = report["segments"]
        rng = intr.uwb_range
        rng.bias0_m = model.get("bias0_m", 0.0)
        rng.scale_ppm = model.get("scale_ppm", 0.0)
        rng.sigma_m = round(sum(s["sigma_m"] for s in segs) / len(segs), 6)
        rng.outlier_frac = round(
            sum(s["outlier_frac"] for s in segs) / len(segs), 6)
    elif kind == "aoa":
        aoa = intr.uwb_aoa
        (aoa.plane_normal.x, aoa.plane_normal.y,
         aoa.plane_normal.z) = report["normal"]
        aoa.sign = report["sign"]
        aoa.offset_rad = report["offset_rad"]
        aoa.sigma_rad = report["sigma_rad"]
    elif kind == "mag":
        mag = intr.mag
        (mag.hard_iron.x, mag.hard_iron.y, mag.hard_iron.z) = report["hard_iron"]
        del mag.soft_iron[:]
        mag.soft_iron.extend(
            v for row in report["soft_iron"] for v in row)
        mag.sigma = report["residual_frac"]
    else:
        raise ValueError("no intrinsics mapping for %r" % kind)
    return validate(config)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("command",
                    choices=["validate", "export-binpb", "export-header"])
    ap.add_argument("--config", required=True, help="WidgetConfig .txtpb")
    ap.add_argument("--out")
    args = ap.parse_args()
    config = load(args.config)
    if args.command == "validate":
        derive(config)
        print("OK: %d sensors, intrinsics sections: %s" % (
            len(config.extrinsics.sensors),
            ", ".join(f.name for f, _ in config.intrinsics.ListFields()
                      if f.name not in ("widget_model", "widget_serial"))
            or "none"))
    elif args.command == "export-binpb":
        with open(args.out, "wb") as f:
            f.write(encode_binary(config))
        print("wrote %s" % args.out)
    elif args.command == "export-header":
        text = c_array_header(config)
        if args.out:
            with open(args.out, "w") as f:
                f.write(text)
        else:
            print(text)


if __name__ == "__main__":
    main()
