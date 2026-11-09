#!/usr/bin/env python3
"""The stylus as a flashlight: a cone of light from its tip along its axis, lighting the
LEDs of every fixture localized in the space (effects/flashlight_pose.glsl on each).

The stylus's button cycles five modes:
  - colour:      rolling the stylus about its own axis turns the hue (one turn = the wheel);
  - saturation:  rolling turns the saturation (one turn = white to full colour);
  - width:       rolling widens or narrows the beam (one turn = x ROLL_FACTOR);
  - persistence: rolling lengthens or shortens the trail the beam leaves (one turn =
                 x ROLL_FACTOR on its time constant; at the short end it's off);
  - fixed:       rolling does nothing.
The beam follows the stylus in every mode.

The tip and axis come from the stylus pivot calibration the main demo saves
(mounts/stylus-<serial>.json, `c_S`: the tip in the stylus tracker's frame). The beam
goes out over OSC to every --fixture: /origin, /dir (world, metres), /width (the
half-angle, rad), /color, /decay (the trail's time constant, s; 0 = none). Fixtures need their positions in the same world: a P4 player's
own tracker (adopting lhd's stations), or a room pose (set_fixture_pose) in lhd's world.

    /usr/bin/python3 flashlight.py --stylus LHR-10F97524 --fixture 10.56.103.226 [--fixture ...]
"""

import argparse
import colorsys
import json
import math
import os
import sys
import threading
import time

import numpy as np

import frames as fr
import osc
import poses

HERE = os.path.dirname(os.path.abspath(__file__))
MOUNTS = os.path.join(HERE, "mounts")

MODES = ("color", "saturation", "width", "persistence", "fixed")
WIDTH_MIN, WIDTH_MAX = math.radians(2.0), math.radians(60.0)
DECAY_OFF, DECAY_MAX = 0.05, 20.0  # s; at DECAY_OFF the trail is sent as 0 (none)
ROLL_FACTOR = 3.0  # one full turn multiplies (or divides) the width or the decay by this
PARAM_KEEPALIVE = 1.0


def twist_about(R, axis):
    """The rotation angle of R about `axis` (unit, R's own frame): the twist of its
    swing-twist decomposition, in (-pi, pi]."""
    w, x, y, z = fr.mat_to_quat(R)
    t = x * axis[0] + y * axis[1] + z * axis[2]
    if w < 0.0:  # q and -q are the same rotation; this one keeps the angle in (-pi, pi]
        w, t = -w, -t
    return 2.0 * math.atan2(t, w)


class Flashlight:
    """The beam and its settings from the stylus pose: no I/O, so it's testable."""

    def __init__(self, tip, hue=0.1, sat=1.0, width=math.radians(12.0), decay=DECAY_OFF):
        tip = np.asarray(tip, float)
        self.tip = tip
        self.axis = tip / np.linalg.norm(tip)  # stylus frame: tracker origin -> tip
        self.hue, self.sat, self.width, self.decay = hue, sat, width, decay
        self.mode = 0
        self.prev_R = None

    @property
    def mode_name(self):
        return MODES[self.mode]

    def press(self):
        self.mode = (self.mode + 1) % len(MODES)
        return self.mode_name

    def color(self):
        return colorsys.hsv_to_rgb(self.hue % 1.0, self.sat, 1.0)

    def decay_sent(self):
        """The trail time constant for the effect: 0 (none) at the short end."""
        return 0.0 if self.decay <= DECAY_OFF * 1.0001 else self.decay

    def update(self, M):
        """A new stylus pose (4x4, world <- stylus): applies the roll since the last one
        to the mode's parameter; returns (origin, dir) in the world."""
        R = M[:3, :3]
        if self.prev_R is not None:
            turns = twist_about(self.prev_R.T @ R, self.axis) / (2.0 * math.pi)
            mode = MODES[self.mode]
            if mode == "color":
                self.hue = (self.hue + turns) % 1.0
            elif mode == "saturation":
                self.sat = min(max(self.sat + turns, 0.0), 1.0)
            elif mode == "width":
                self.width = min(max(self.width * ROLL_FACTOR ** turns, WIDTH_MIN), WIDTH_MAX)
            elif mode == "persistence":
                self.decay = min(max(self.decay * ROLL_FACTOR ** turns, DECAY_OFF), DECAY_MAX)
        self.prev_R = R.copy()
        origin = M[:3, :3] @ self.tip + M[:3, 3]
        return origin, R @ self.axis


def load_tip(stylus):
    path = os.path.join(MOUNTS, "stylus-%s.json" % stylus)
    if not os.path.exists(path):
        raise SystemExit("no stylus calibration %s: run the main demo's Pivot step first" % path)
    with open(path) as f:
        return json.load(f)["c_S"]


def parse_target(s):
    host, _, port = s.partition(":")
    return host, int(port or osc.OSC_PORT)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--stylus", required=True, help="tracker serial of the stylus (its pivot calibration is in mounts/)")
    ap.add_argument("--fixture", action="append", default=[], help="fixture host[:port] for OSC; repeat for several")
    ap.add_argument("--source", default="http://127.0.0.1:8765", help="vive_web server URL (SSE)")
    ap.add_argument("--world-up", choices=["z", "y"], default="z")
    ap.add_argument("--no-p4-stations", action="store_true",
                    help="don't send lhd's stations to the fixtures (a P4 adopts them, so it tracks in lhd's world)")
    a = ap.parse_args()
    if not a.fixture:
        ap.error("at least one --fixture")

    light = Flashlight(load_tip(a.stylus))
    transport = osc.Tee(*[osc.UdpTransport(*parse_target(f)) for f in a.fixture])
    streamer = osc.OscStreamer(transport).start()
    lock = threading.Lock()

    def send_params():
        streamer.set("/width", light.width, keepalive=PARAM_KEEPALIVE)
        streamer.set("/color", *light.color(), keepalive=PARAM_KEEPALIVE)
        streamer.set("/decay", light.decay_sent(), keepalive=PARAM_KEEPALIVE)

    def on_pose(ps):
        if ps.dev != a.stylus:
            return
        with lock:
            o, d = light.update(ps.matrix())
            streamer.set("/origin", *o)
            streamer.set("/dir", *d)
            send_params()

    def on_button(dev, n):
        if dev != a.stylus:
            return
        with lock:
            for _ in range(n):
                light.press()
        print("mode: %s" % light.mode_name, flush=True)

    def station_sync():
        # As the main demo's --p4 mode: standalone datagrams (the P4 tracker module claims
        # single messages, never bundles), once a second.
        while True:
            for sid, (p, q) in list(source.lighthouses.items()):
                try:
                    transport.send(osc.message_typed("/lh/station", int(sid), *[float(x) for x in p],
                                                     *[float(x) for x in q]))
                except (ValueError, OSError):
                    pass
            time.sleep(1.0)

    with lock:
        send_params()
    source = poses.SseSource(a.source, world_up=a.world_up)
    source.start(on_pose, on_button)
    if not a.no_p4_stations:
        threading.Thread(target=station_sync, daemon=True, name="p4-stations").start()
    print("flashlight: stylus %s -> %s; mode %s (press the stylus button to change)"
          % (a.stylus, transport.describe(), light.mode_name), flush=True)
    try:
        while True:
            time.sleep(2.0)
            with lock:
                r, g, b = light.color()
                trail = "off" if light.decay_sent() == 0.0 else "%.2f s" % light.decay
                status = "mode %-11s  width %4.1f deg  colour %.2f %.2f %.2f (sat %.2f)  trail %-7s" % (
                    light.mode_name, math.degrees(light.width), r, g, b, light.sat, trail)
            sys.stdout.write("\r" + status)
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        source.stop()
        streamer.stop()


if __name__ == "__main__":
    main()
