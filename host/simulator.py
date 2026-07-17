"""Synthetic tag: computes anchor ranges for a moving (or fixed) position and
streams them to the server's TCP ingest port in the tag's JSON format.

Full-stack headless demo:  bazel run //host:server  then
bazel run //host:simulator -- --host 127.0.0.1
"""

import argparse
import json
import math
import socket
import time

import numpy as np

DEFAULT_ANCHORS = [[15.0, 5.0], [290.0, 5.0], [165.0, 625.0]]


def true_position(t, room=(480.0, 650.0)):
    """Ellipse around the room centre, one lap per minute."""
    cx, cy = room[0] / 2, room[1] / 2
    phase = 2 * math.pi * t / 60.0
    return (cx + 0.3 * room[0] * math.cos(phase),
            cy + 0.3 * room[1] * math.sin(phase))


def make_packet(pos, anchors, noise_cm=0.0, rng=None, tag_id=10):
    """Tag-format JSON line for a position: per-anchor filtered + raw range."""
    rng = rng or np.random.default_rng()
    entries = {}
    for i, (ax, ay) in enumerate(anchors):
        d = math.hypot(pos[0] - ax, pos[1] - ay)
        noisy = d + (rng.normal(0.0, noise_cm) if noise_cm else 0.0)
        entries["A%d" % (i + 1)] = {
            "distance": round(noisy, 2),
            "raw": round(noisy, 2),
            "rssi": -65.0,
            "fp_rssi": -68.0,
            "round_time": 0,
            "reply_time": 0,
            "clock_offset": 0.0,
        }
    return json.dumps({"tag_id": tag_id, "anchors": entries}) + "\n"


def run(host, port, anchors, rate_hz=5.0, steps=None, noise_cm=1.0, fixed=None):
    rng = np.random.default_rng(1234)
    sent = 0
    with socket.create_connection((host, port)) as sock:
        while steps is None or sent < steps:
            pos = fixed if fixed is not None else true_position(time.time())
            sock.sendall(make_packet(pos, anchors, noise_cm, rng).encode())
            sent += 1
            if steps is None or sent < steps:
                time.sleep(1.0 / rate_hz)
    return sent


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=7007)
    ap.add_argument("--rate", type=float, default=5.0, help="packets per second")
    ap.add_argument("--steps", type=int, default=None,
                    help="stop after N packets (default: run forever)")
    ap.add_argument("--noise", type=float, default=1.0, help="range noise stddev, cm")
    ap.add_argument("--fixed", type=float, nargs=2, metavar=("X", "Y"),
                    help="hold the tag at a fixed position instead of orbiting")
    args = ap.parse_args()
    sent = run(args.host, args.port, DEFAULT_ANCHORS, args.rate, args.steps,
               args.noise, tuple(args.fixed) if args.fixed else None)
    print("sent %d packets" % sent)


if __name__ == "__main__":
    main()
