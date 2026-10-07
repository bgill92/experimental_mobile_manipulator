"""Grasp pose geometry, no robot model needed."""

import math

from emma_manipulation.grasp import grasp_candidates, offset
import numpy as np
import pytest


@pytest.mark.parametrize('yaw', [0.0, 0.3, math.pi / 2, 2.0])
def test_candidates_are_rotations_at_the_block(yaw: float) -> None:
    tilts = [0.0, math.radians(20), math.radians(35)]
    cands = grasp_candidates((0.25, 0.0, 0.155), yaw, (0.1, 0.0), tilts)
    assert len(cands) == 2 * len(tilts)
    for tform in cands:
        rot = tform[:3, :3]
        np.testing.assert_allclose(rot.T @ rot, np.eye(3), atol=1e-12)
        assert np.linalg.det(rot) == pytest.approx(1.0)
        np.testing.assert_allclose(tform[:3, 3], (0.25, 0.0, 0.155))
        # Fingers close along a block axis, horizontally.
        x = tform[:3, 0]
        assert x[2] == pytest.approx(0.0, abs=1e-12)
        block_axis = np.array([math.cos(yaw), math.sin(yaw)])
        dot = abs(x[:2] @ block_axis)
        assert min(dot, abs(dot - 1.0)) < 1e-9


def test_tilt_leans_away_from_the_arm() -> None:
    cands = grasp_candidates((0.25, 0.0, 0.155), 0.0, (0.1, 0.0), [0.0, math.radians(30)])
    np.testing.assert_allclose(cands[0][:3, 2], [0, 0, -1], atol=1e-12)
    np.testing.assert_allclose(cands[0][:3, 0], [0, 1, 0], atol=1e-12)
    z = cands[2][:3, 2]
    assert z[0] == pytest.approx(math.sin(math.radians(30)))
    assert z[2] == pytest.approx(-math.cos(math.radians(30)))


def test_offset_backs_off_along_approach() -> None:
    tform = grasp_candidates((0.25, 0.0, 0.155), 0.0, (0.1, 0.0), [0.0])[0]
    np.testing.assert_allclose(offset(tform, 0.05)[:3, 3], (0.25, 0.0, 0.205))
    np.testing.assert_allclose(offset(tform, 0.05)[:3, :3], tform[:3, :3])
