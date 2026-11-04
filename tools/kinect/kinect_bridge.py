"""Kinect v2 depth -> hull planes -> a Splanc player, over OSC/UDP.

Runs on a small box with USB 3.0 (the NanoPi R6S) next to the Kinect: grabs depth
frames with libfreenect2, finds what is new in the room against a learned empty
background, boils it down to an approximate hull (hull.py) and streams it to the
player as the `hull0..hull3` / `present` uniforms of hull_glow.fx.

  python3 tools/kinect/kinect_bridge.py --host 192.168.68.70 \\
      --cam-pos 0 1.2 0 --cam-pitch 10 --learn-bg 3          # real Kinect
  python3 tools/kinect/kinect_bridge.py --source synth --dry-run  # no hardware

--cam-pos/--cam-pitch/--cam-yaw place the Kinect in the room frame the fixture is
posed in (y up, metres; the lamp's led.room). Stand out of view while the
background is learned (--learn-bg seconds, or --bg-file to keep it between runs).

Sources: freenect2 (pylibfreenect2), synth (a walking ellipsoid), npy (a saved
(frames, 424, 512) array of depth in mm, for replay).
"""

import argparse
import socket
import sys
import time

import numpy as np

import hull

FRAME_SHAPE = (hull.DEPTH_H, hull.DEPTH_W)


class SynthSource:
    """A person-sized ellipsoid walking back and forth in front of a flat wall."""

    def __init__(self, wall_m=4.0, rate=30.0):
        self.wall, self.dt, self.t = wall_m, 1.0 / rate, 0.0
        v, u = np.mgrid[0:hull.DEPTH_H, 0:hull.DEPTH_W]
        self.rays = np.stack([(u - hull.CX) / hull.FX, (v - hull.CY) / hull.FY, np.ones_like(u, float)], -1)

    def frames(self):
        while True:
            self.t += self.dt
            yield self.render(centre=(1.2 * np.sin(self.t * 0.5), 0.0, 2.2 + 0.8 * np.sin(self.t * 0.3)))
            time.sleep(self.dt)

    def render(self, centre, radii=(0.25, 0.9, 0.2)):
        depth = np.full(FRAME_SHAPE, self.wall * 1000.0, dtype=np.float32)
        c, r = np.asarray(centre), np.asarray(radii)
        d = self.rays / r
        a = (d * d).sum(-1)
        b = -2.0 * (d * (c / r)).sum(-1)
        disc = b * b - 4 * a * (((c / r) ** 2).sum() - 1.0)
        hit = disc > 0
        z = (-b - np.sqrt(np.where(hit, disc, 0))) / (2 * a)       # ray z == 1, so t is the depth
        return np.where(hit & (z > 0), z * 1000.0, depth).astype(np.float32)


class NpySource:
    def __init__(self, path, rate=30.0):
        self.data, self.dt = np.load(path, mmap_mode="r"), 1.0 / rate

    def frames(self):
        for f in self.data:
            yield np.asarray(f, dtype=np.float32)
            time.sleep(self.dt)


class Freenect2Source:
    """Depth only, through libfreenect2's python binding (pylibfreenect2)."""

    def __init__(self, pipeline="auto"):
        from pylibfreenect2 import (CpuPacketPipeline, FrameType, Freenect2,
                                    SyncMultiFrameListener)
        try:
            from pylibfreenect2 import OpenCLPacketPipeline as Cl
        except ImportError:
            Cl = None
        self.FrameType = FrameType
        self.fn = Freenect2()
        if self.fn.enumerateDevices() == 0:
            sys.exit("no Kinect v2 found (USB 3.0 port? udev rules for 045e:02c4?)")
        pipe = Cl() if pipeline in ("auto", "opencl") and Cl else CpuPacketPipeline()
        print(f"[kinect] depth pipeline: {type(pipe).__name__}")
        self.listener = SyncMultiFrameListener(FrameType.Depth)
        self.dev = self.fn.openDevice(self.fn.getDeviceSerialNumber(0), pipeline=pipe)
        self.dev.setIrAndDepthFrameListener(self.listener)
        self.dev.startStreams(rgb=False, depth=True)
        p = self.dev.getIrCameraParams()
        self.intrinsics = (p.fx, p.fy, p.cx, p.cy)

    def frames(self):
        try:
            while True:
                frames = self.listener.waitForNewFrame()
                depth = frames["depth"].asarray().copy()          # float32, mm
                self.listener.release(frames)
                yield depth
        finally:
            self.dev.stop()
            self.dev.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--source", choices=["freenect2", "synth", "npy"], default="freenect2")
    ap.add_argument("--npy", help="depth stack for --source npy")
    ap.add_argument("--pipeline", choices=["auto", "opencl", "cpu"], default="auto")
    ap.add_argument("--host", default="127.0.0.1", help="the player")
    ap.add_argument("--port", type=int, default=9000)
    ap.add_argument("--rate", type=float, default=30.0, help="max updates/s sent")
    ap.add_argument("--cam-pos", type=float, nargs=3, default=(0.0, 1.0, 0.0), metavar=("X", "Y", "Z"))
    ap.add_argument("--cam-pitch", type=float, default=0.0, help="degrees looking down")
    ap.add_argument("--cam-yaw", type=float, default=0.0, help="degrees about the vertical")
    ap.add_argument("--block", type=int, default=16, help="decimation block, pixels")
    ap.add_argument("--margin", type=float, default=0.15, help="metres nearer than background")
    ap.add_argument("--learn-bg", type=float, default=0.0, help="seconds of empty room to learn first")
    ap.add_argument("--bg-file", help="load/save the learned background (.npy)")
    ap.add_argument("--dry-run", action="store_true", help="print instead of sending")
    args = ap.parse_args()

    src = {"freenect2": lambda: Freenect2Source(args.pipeline), "synth": lambda: SynthSource(wall_m=5.0),   # past the 4.5 m range gate, so no background needed
           "npy": lambda: NpySource(args.npy)}[args.source]()
    intr = getattr(src, "intrinsics", None)
    ext = hull.camera_pose(args.cam_pos, args.cam_pitch, args.cam_yaw)
    sock = None if args.dry_run else socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tracker = hull.HullTracker()

    bg, learn = None, []
    if args.bg_file:
        try:
            bg = np.load(args.bg_file)
            print(f"[kinect] background from {args.bg_file}")
        except OSError:
            pass
    t_learn_end = time.time() + args.learn_bg if bg is None and args.learn_bg > 0 else 0

    last_send = last_log = 0.0
    n_frames = 0
    packet = None
    for depth in src.frames():
        now = time.time()
        if now < t_learn_end:
            learn.append(hull.decimate(depth, args.block)[0])
            continue
        if learn:
            bg = hull.learn_background(learn)
            learn = []
            print(f"[kinect] background learned ({np.isfinite(bg).mean():.0%} of cells valid)")
            if args.bg_file:
                np.save(args.bg_file, bg)
        present, h, n_pts = hull.process(depth, tracker, ext, bg, intr, args.block, args.margin)
        n_frames += 1
        packet = hull.hull_packet(present, h)
        if now - last_send >= 1.0 / args.rate:
            last_send = now
            if sock:
                sock.sendto(packet, (args.host, args.port))
        if now - last_log >= 2.0:
            fps = n_frames / (now - last_log) if last_log else 0.0
            last_log, n_frames = now, 0
            extent = "" if h is None else " extent x %.2f..%.2f y %.2f..%.2f z %.2f..%.2f m" % (
                -h[1], h[0], -h[3], h[2], -h[5], h[4])
            print(f"[kinect] {fps:4.1f} fps, {n_pts:3d} fg cells, {'PRESENT' if present else 'empty'}{extent}")


if __name__ == "__main__":
    main()
