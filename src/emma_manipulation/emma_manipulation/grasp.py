"""Top-down grasp poses for a box-shaped block. Pure numpy, no ROS."""

from collections.abc import Sequence
import math

from emma_manipulation.constants import GRASP_TILTS
import numpy as np
from numpy.typing import ArrayLike


def grasp_candidates(
    block_xyz: Sequence[float],
    block_yaw: float,
    arm_base_xy: ArrayLike,
    tilts: Sequence[float] = GRASP_TILTS,
) -> list[np.ndarray]:
    """
    Return 4x4 TCP poses that grasp a block from above, best first.

    The fingers close along the block axis most perpendicular to the line from the arm
    base to the block, so the arm reaches across the block rather than along it. The
    approach axis (TCP z) points down, tilted by each angle in `tilts` away from the arm
    base. Each tilt comes in two variants with the fingers swapped (TCP rotated 180 deg
    about z), which the parallel gripper treats as the same grasp but the wrist does not.
    """
    centre = np.asarray(block_xyz, dtype=float)
    reach = centre[:2] - np.asarray(arm_base_xy, dtype=float)
    norm = np.linalg.norm(reach)
    d = np.array([1.0, 0.0, 0.0]) if norm < 1e-9 else np.array([*(reach / norm), 0.0])

    c, s = math.cos(block_yaw), math.sin(block_yaw)
    axes = [np.array([c, s, 0.0]), np.array([-s, c, 0.0])]
    finger = min(axes, key=lambda a: abs(float(a @ d)))
    # Tilt in the plane across the fingers, so they still close on the block faces. Of the
    # two horizontal directions perpendicular to the fingers, lean toward the reach.
    lean = np.cross(finger, np.array([0.0, 0.0, 1.0]))
    if lean @ d < 0.0:
        lean = -lean

    candidates = []
    for tilt in tilts:
        z = -math.cos(tilt) * np.array([0.0, 0.0, 1.0]) + math.sin(tilt) * lean
        for x in (finger, -finger):
            y = np.cross(z, x)
            tform = np.eye(4)
            tform[:3, 0] = x
            tform[:3, 1] = y
            tform[:3, 2] = z
            tform[:3, 3] = centre
            candidates.append(tform)
    return candidates


def offset(tform: np.ndarray, dist: float) -> np.ndarray:
    """Return `tform` moved back by `dist` along its own -z (away from the approach)."""
    out = np.array(tform, dtype=float, copy=True)
    out[:3, 3] -= dist * out[:3, 2]
    return out
