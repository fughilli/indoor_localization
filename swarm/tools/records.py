"""Capture records + session manifests for sensor characterization.

A *capture* is a JSONL file of raw sensor records streamed by a widget in
capture mode (the firmware equivalent of the tag's newline-JSON streaming).
One object per line:

  {"t": 12.345, "type": "imu",   "accel": [ax, ay, az], "gyro": [gx, gy, gz]}
  {"t": 12.351, "type": "mag",   "mag": [mx, my, mz]}
  {"t": 12.360, "type": "range", "peer": 3, "range_m": 4.982,
                                 "rssi": -65.1, "fp_rssi": -68.2}
  {"t": 12.360, "type": "aoa",   "peer": 3, "angle_rad": 0.412,
                                 "pdoa_rad": 1.13}

`t` is the widget's monotonic clock in seconds. Units: m/s^2, rad/s, and
arbitrary-but-consistent magnetometer units. `angle_rad` is the radio's
reported arrival angle, whatever convention the hardware uses — discovering
that convention (plane, sign, offset) is the point of the AoA analysis.

A *manifest* is a JSON file describing the ground truth of the physical
setup, as time segments:

  {"segments": [
      {"t0": 0.0,  "t1": 60.0, "true_range_m": 5.0},
      {"t0": 70.0, "t1": 130.0, "true_range_m": 10.0,
       "true_dir_body": [0.966, 0.259, 0.0]},
      {"t0": 140.0, "t1": 500.0, "static": true}
  ]}

`true_dir_body` is the unit direction from this widget to the ranging peer
in the widget's body frame (from the rig geometry, e.g. a turntable at a
known angle). Analyses pick the segments carrying the fields they need.
"""

import json


def load_capture(path):
    """List of record dicts, sorted by t."""
    records = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line:
                records.append(json.loads(line))
    records.sort(key=lambda r: r["t"])
    return records


def load_manifest(path):
    with open(path) as f:
        return json.load(f)


def select(records, rtype, t0=None, t1=None):
    """Records of one type, optionally windowed to [t0, t1)."""
    return [r for r in records if r["type"] == rtype
            and (t0 is None or r["t"] >= t0)
            and (t1 is None or r["t"] < t1)]


def segments_with(manifest, field):
    """Manifest segments that carry a given ground-truth field."""
    return [s for s in manifest.get("segments", []) if field in s]
