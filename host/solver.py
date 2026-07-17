"""Least-squares trilateration on the 2D floor plane.

Distances arrive in centimetres from the tag (median-filtered on-device);
anchor positions are floor-plan coordinates in the same unit. Mirrors the
reference implementation's scipy least_squares approach (floor_view.py).
"""

import numpy as np
from scipy.optimize import least_squares

# Reference floor plan: anchor 1 and 2 along the bottom wall, anchor 3 at the
# top. Units are cm; override per deployment via server --anchors.
DEFAULT_ANCHORS = np.array([[15.0, 5.0], [290.0, 5.0], [165.0, 625.0]])
DEFAULT_ROOM = (480.0, 650.0)  # (width, height) cm


def trilaterate(distances, anchors=None):
    """Solve for the (x, y) that best matches the measured ranges.

    Args:
      distances: sequence of N range measurements (cm), one per anchor.
      anchors: Nx2 array of anchor positions (cm); DEFAULT_ANCHORS if None.

    Returns:
      np.array([x, y]) or None when the solver fails.
    """
    anchors = DEFAULT_ANCHORS if anchors is None else np.asarray(anchors, float)
    distances = np.asarray(distances, float)
    if len(distances) != len(anchors) or len(anchors) < 3:
        return None

    def residuals(p):
        return np.hypot(*(anchors - p).T) - distances

    try:
        result = least_squares(residuals, np.mean(anchors, axis=0), method="lm")
    except Exception:
        return None
    return result.x if result.success else None
