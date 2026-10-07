"""Block detector image maths on synthetic images, without ROS."""

import math

import cv2
from emma_perception.block_detector import (
    backproject, blob_points, largest_blob, locate, segment, top_face, wrap_quarter_turn)
import numpy as np
import pytest

K = (400.0, 400.0, 320.0, 200.0)
RED = (0, 0, 230)  # BGR.
LOWER, UPPER = [0, 120, 70], [10, 255, 255]


def blank() -> np.ndarray:
    image = np.full((400, 640, 3), 200, dtype=np.uint8)
    image[:, :, 0] = 120  # A dull blue-grey background, nothing red.
    return image


def flat_depth(d: float = 0.25) -> np.ndarray:
    return np.full((400, 640), d, dtype=np.float32)


def square(centre: tuple[float, float], side: float, angle_deg: float) -> np.ndarray:
    corners = cv2.boxPoints((centre, (side, side), angle_deg))
    return np.round(corners).astype(np.int32)


def test_segment_takes_both_ends_of_red_hue() -> None:
    image = blank()
    image[10:20, 10:20] = (0, 0, 230)  # Hue 0.
    image[30:40, 30:40] = (40, 0, 230)  # Hue about 175, past the wrap.
    image[50:60, 50:60] = (0, 230, 0)  # Green.
    mask = segment(image, LOWER, UPPER)
    assert mask[15, 15] == 255
    assert mask[35, 35] == 255
    assert mask[55, 55] == 0
    assert mask[100, 100] == 0


def test_locate_and_backproject_square() -> None:
    image = blank()
    image[160:180, 350:370] = RED  # 20 x 20 px centred on (360, 170) (pixel centres).
    found = locate(segment(image, LOWER, UPPER), flat_depth())
    assert found is not None
    u, v, d, _ = found
    assert abs(u - 360) <= 0.5
    assert abs(v - 170) <= 0.5
    assert d == pytest.approx(0.25)
    np.testing.assert_allclose(backproject(360, 170, d, K), [0.025, -0.01875, 0.25])


def test_rotated_square_yaw() -> None:
    image = blank()
    cv2.fillPoly(image, [square((300, 220), 40, 30.0)], RED)
    found = locate(segment(image, LOWER, UPPER), flat_depth())
    assert found is not None
    *_, angle = found
    assert math.degrees(wrap_quarter_turn(angle)) == pytest.approx(30.0, abs=2.0)


def test_locate_rejects_small_blobs_and_bad_depth() -> None:
    image = blank()
    image[100:104, 100:104] = RED  # 16 px.
    assert locate(segment(image, LOWER, UPPER), flat_depth()) is None
    image[160:180, 350:370] = RED
    assert locate(segment(image, LOWER, UPPER), flat_depth(0.02)) is None
    assert locate(segment(image, LOWER, UPPER), flat_depth(float('nan'))) is None


def test_locate_without_red() -> None:
    assert locate(segment(blank(), LOWER, UPPER), flat_depth()) is None


@pytest.mark.parametrize('angle, expected', [
    (0.0, 0.0), (math.radians(80), math.radians(-10)), (math.radians(-50), math.radians(40)),
    (math.radians(135), math.radians(-45))])
def test_wrap_quarter_turn(angle: float, expected: float) -> None:
    assert wrap_quarter_turn(angle) == pytest.approx(expected)


def test_blob_points_backprojects_every_blob_pixel() -> None:
    image = blank()
    image[160:180, 350:370] = RED
    depth = flat_depth()
    depth[160, 350] = float('nan')  # Dropped from the points.
    blob = largest_blob(segment(image, LOWER, UPPER))
    assert blob is not None
    points = blob_points(blob, depth, K)
    assert points.shape == (399, 3)
    # Pixel centres 350..369 average to 359.5.
    np.testing.assert_allclose(points.mean(axis=0), backproject(359.5, 169.5, 0.25, K), atol=1e-4)


def cube_surface(centre, yaw_deg: float, size: float = 0.01, n: int = 15) -> np.ndarray:
    """Points on a cube's top face and its -x side face (as a camera in -x would see)."""
    s = np.linspace(-size / 2, size / 2, n)
    a, b = np.meshgrid(s, s)
    top = np.column_stack([a.ravel(), b.ravel(), np.full(a.size, size / 2)])
    side = np.column_stack([np.full(a.size, -size / 2), a.ravel(), b.ravel()])
    yaw = math.radians(yaw_deg)
    rot = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0],
                    [0, 0, 1]])
    return np.vstack([top, side]) @ rot.T + np.asarray(centre)


@pytest.mark.parametrize('yaw_deg', [0.0, 20.0, -35.0])
def test_top_face_ignores_the_side_face(yaw_deg: float) -> None:
    result = top_face(cube_surface([0.25, 0.02, 0.155], yaw_deg), 0.01)
    assert result is not None
    centre, yaw = result
    np.testing.assert_allclose(centre, [0.25, 0.02, 0.155], atol=1e-3)
    assert math.degrees(yaw) == pytest.approx(yaw_deg, abs=3.0)
