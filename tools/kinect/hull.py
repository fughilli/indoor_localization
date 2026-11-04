"""Depth frame -> approximate hull of the person, as plane uniforms for hull_glow.fx.

Pure numpy, so it runs on a small ARM board next to the Kinect. The pipeline per
frame:

  decimate    block-median the 512x424 depth image down to ~32x26 cells (a few
              hundred points instead of 217k pixels; the median kills flying pixels)
  foreground  keep the cells nearer than a learned empty-room background (or
              just inside a range gate when there is none)
  unproject   cells -> 3D points in the camera's optical frame (x right, y down,
              z forward, metres), then to the room frame with a 4x4 extrinsic
  hull        a 14-DOP: the support function h_k = max_p dot(n_k, p) along 14 fixed
              directions (6 axes + 8 cube diagonals). It always encloses the
              points, is cheap, and, unlike a triangulated convex hull, its
              planes never change identity between frames, so it doesn't flicker.

The 14 planes ride to the player as four mat4 uniforms (hull0..hull3, four planes
each; plane i is column i = nx ny nz h, column-major like GLSL) plus `present`.
"""

import math
import warnings

import numpy as np

# Kinect v2 depth camera, nominal intrinsics (pixels). The device's own values
# (libfreenect2 getIrCameraParams) override these when available.
DEPTH_W, DEPTH_H = 512, 424
FX = FY = 365.5
CX, CY = 256.0, 212.0

_diag = 1.0 / math.sqrt(3.0)
DIRS = np.array(
    [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]
    + [(sx * _diag, sy * _diag, sz * _diag) for sx in (1, -1) for sy in (1, -1) for sz in (1, -1)],
    dtype=np.float64,
)
N_PLANES = len(DIRS)          # 14
N_BLOCKS = 4                  # mat4s of four planes each: room for 16


def decimate(depth_mm, block=16, min_mm=500.0, max_mm=4500.0, min_frac=0.25):
    """Block-median of a depth image. Returns (z_m, u, v): per-cell depth in
    metres (NaN where fewer than `min_frac` of the cell's pixels are valid) and
    the cell centres in pixel coordinates (each shaped like z_m)."""
    h = (depth_mm.shape[0] // block) * block
    w = (depth_mm.shape[1] // block) * block
    d = depth_mm[:h, :w].astype(np.float32)
    valid = np.isfinite(d) & (d >= min_mm) & (d <= max_mm)
    d = np.where(valid, d, np.nan)
    cells = d.reshape(h // block, block, w // block, block).transpose(0, 2, 1, 3)
    cells = cells.reshape(h // block, w // block, block * block)
    enough = np.isfinite(cells).mean(axis=2) >= min_frac
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)   # all-NaN cells
        z = np.nanmedian(cells, axis=2)
    z = np.where(enough, z, np.nan) / 1000.0
    v, u = np.mgrid[0:h // block, 0:w // block]
    return z, (u + 0.5) * block, (v + 0.5) * block


def learn_background(grids):
    """Per-cell background from a stack of empty-room decimated depth grids:
    the median over frames (cells that were never valid stay NaN)."""
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        return np.nanmedian(np.stack(grids), axis=0)


def foreground(z, background=None, margin=0.15):
    """Mask of cells that belong to something new in the room: valid, and
    nearer than the background by `margin` metres (or anywhere valid when no
    background is known)."""
    fg = np.isfinite(z)
    if background is not None:
        # A cell that was invalid in the background (a hole) counts as foreground.
        with np.errstate(invalid="ignore"):
            fg &= ~np.isfinite(background) | (z < background - margin)
    return fg


def unproject(z, u, v, mask, fx=FX, fy=FY, cx=CX, cy=CY):
    """Cells -> Nx3 points in the camera optical frame, metres."""
    zz = z[mask]
    return np.stack([(u[mask] - cx) * zz / fx, (v[mask] - cy) * zz / fy, zz], axis=1)


def camera_pose(pos=(0.0, 0.0, 0.0), pitch_deg=0.0, yaw_deg=0.0):
    """world <- camera (4x4) for a camera mounted at `pos` (room frame, y up),
    looking along +z rotated by `yaw_deg` about y, pitched down by `pitch_deg`.
    The optical frame is x right, y down, z forward."""
    p, y = math.radians(pitch_deg), math.radians(yaw_deg)
    # optical -> level camera frame (x right, y up, z forward)
    flip = np.diag([1.0, -1.0, 1.0])
    # pitch down about x (positive pitch tilts the view toward -y)
    rx = np.array([[1, 0, 0], [0, math.cos(p), -math.sin(p)], [0, math.sin(p), math.cos(p)]])
    ry = np.array([[math.cos(y), 0, math.sin(y)], [0, 1, 0], [-math.sin(y), 0, math.cos(y)]])
    t = np.eye(4)
    t[:3, :3] = ry @ rx @ flip
    t[:3, 3] = pos
    return t


def transform(points, t):
    return points @ t[:3, :3].T + t[:3, 3]


def support(points, trim=0.0):
    """h_k = max over points of dot(n_k, p), for the 14 directions. `trim` (0..1)
    takes that fraction off the top (a percentile, not the max) so one stray cell
    can't stretch the hull."""
    proj = points @ DIRS.T                      # (n, 14)
    if trim <= 0.0 or len(points) < 20:
        return proj.max(axis=0)
    return np.percentile(proj, 100.0 * (1.0 - trim), axis=0)


def signed_distance(h, p):
    """Largest plane distance of point(s) p to the polytope (negative inside)."""
    return (np.atleast_2d(p) @ DIRS.T - h).max(axis=1)


def pack(h):
    """14 offsets -> 64 floats = hull0..hull3 for OSC. Plane i is column i of its
    mat4 (nx ny nz h), column-major; the two spare columns repeat plane 0 so they
    never loosen the max."""
    planes = np.zeros((N_BLOCKS * 4, 4), dtype=np.float32)
    planes[:N_PLANES, :3] = DIRS
    planes[:N_PLANES, 3] = h
    planes[N_PLANES:] = planes[0]
    return planes.reshape(N_BLOCKS, 16)


class HullTracker:
    """Frame loop state: smooths the support values, and decides presence with
    hysteresis (a few frames to appear, a longer grace to disappear) so a dropped
    frame doesn't blink the light."""

    def __init__(self, min_points=12, alpha=0.5, appear=2, vanish=15, trim=0.03):
        self.min_points, self.alpha = min_points, alpha
        self.appear, self.vanish, self.trim = appear, vanish, trim
        self.h = None
        self.present = False
        self._seen = self._missed = 0

    def update(self, points):
        """points: Nx3 in the room frame (may be empty). Returns (present, h)."""
        if len(points) >= self.min_points:
            self._seen += 1
            self._missed = 0
            h = support(points, self.trim)
            self.h = h if self.h is None or not self.present else self.alpha * h + (1 - self.alpha) * self.h
            if self._seen >= self.appear:
                self.present = True
        else:
            self._missed += 1
            self._seen = 0
            if self._missed >= self.vanish:
                self.present = False
                self.h = None
        return self.present, self.h


def process(depth_mm, tracker, extrinsic, background=None, intr=None, block=16, margin=0.15):
    """One frame, depth image to (present, h, n_points)."""
    z, u, v = decimate(depth_mm, block)
    mask = foreground(z, background, margin)
    pts = unproject(z, u, v, mask, *(intr or (FX, FY, CX, CY)))
    pts = transform(pts, extrinsic) if len(pts) else pts
    present, h = tracker.update(pts)
    return present, h, len(pts)


# --- OSC ----------------------------------------------------------------------------

def _pad(b):
    return b + b"\0" * (4 - len(b) % 4)


def osc_message(addr, floats):
    f = np.asarray(floats, dtype=">f4")
    return _pad(addr.encode()) + _pad(("," + "f" * len(f)).encode()) + f.tobytes()


def osc_bundle(messages):
    out = b"#bundle\0" + b"\0\0\0\0\0\0\0\1"
    for m in messages:
        out += len(m).to_bytes(4, "big") + m
    return out


def hull_packet(present, h):
    """The datagram for one frame: `present` plus the four blocks."""
    msgs = [osc_message("/present", [1.0 if present else 0.0])]
    if present and h is not None:
        msgs += [osc_message(f"/hull{i}", blk) for i, blk in enumerate(pack(h))]
    return osc_bundle(msgs)
