"""Planner checks against emma's real URDF, with the table and block in the scene."""

import math

from ament_index_python.packages import get_package_share_directory
from emma_manipulation.constants import (
    ARM_JOINTS, BLOCK_SIZE, BLOCK_START, CAMERA_FRAME, HOME_Q, LOOK_Q, PLACE_XYZ, PRE_OFFSET,
    TABLE_CENTER, TABLE_HALF_SIZE)
from emma_manipulation.grasp import grasp_candidates, offset
from emma_manipulation.planner import ArmPlanner, to_joint_trajectory
import numpy as np
import pytest
from roboplan.core import poseError
from sensor_msgs.msg import JointState
import xacro

HOME = np.array(HOME_Q)


def translation(xyz) -> np.ndarray:
    tform = np.eye(4)
    tform[:3, 3] = xyz
    return tform


@pytest.fixture(scope='module')
def planner() -> ArmPlanner:
    urdf = xacro.process_file(
        get_package_share_directory('emma_description') + '/urdf/emma.urdf.xacro').toxml()
    planner = ArmPlanner(urdf, seed=7)
    planner.add_box('table', [2 * h for h in TABLE_HALF_SIZE], translation(TABLE_CENTER))
    planner.add_box('block', [BLOCK_SIZE] * 3, translation(BLOCK_START))
    return planner


@pytest.fixture(scope='module')
def grasps(planner: ArmPlanner) -> list[np.ndarray]:
    arm_base_xy = planner.fk(HOME, 'g_base')[:2, 3]
    return grasp_candidates(BLOCK_START, 0.0, arm_base_xy)


def test_home_collision_free(planner: ArmPlanner) -> None:
    tform = planner.fk(HOME)
    assert np.all(np.isfinite(tform))
    assert not planner.has_collisions(HOME)


def test_tcp_is_45mm_past_gripper_base(planner: ArmPlanner) -> None:
    rel = np.linalg.inv(planner.fk(HOME, 'gripper_base')) @ planner.fk(HOME)
    np.testing.assert_allclose(rel[:3, 3], [0.0, 0.0, 0.045], atol=1e-9)


def test_set_q_orders_by_arm_joints(planner: ArmPlanner) -> None:
    msg = JointState(name=list(reversed(ARM_JOINTS)) + ['gripper_controller'],
                     position=[6.0, 5.0, 4.0, 3.0, 2.0, 1.0, 0.0])
    np.testing.assert_allclose(planner.set_q(msg), [1, 2, 3, 4, 5, 6])


def test_ik_round_trip(planner: ArmPlanner) -> None:
    q_true = np.array([0.3, 0.4, -1.2, -0.6, 0.2, 0.5])
    assert not planner.has_collisions(q_true)
    target = planner.fk(q_true)
    q = planner.ik(target, HOME)
    assert q is not None
    pos_err, rot_err = poseError(planner.fk(q), target)
    assert pos_err < 1e-3
    assert rot_err < math.radians(0.5)


def test_block_reachable_from_home(planner: ArmPlanner, grasps: list[np.ndarray]) -> None:
    # The reachability gate for the table/block constants.
    found = planner.plan_to_any(HOME, [offset(g, PRE_OFFSET) for g in grasps])
    assert found is not None
    path, index = found
    np.testing.assert_allclose(path[0], HOME)
    pos_err, _ = poseError(planner.fk(path[-1]), offset(grasps[index], PRE_OFFSET))
    assert pos_err < 1e-3
    for q in path:
        assert not planner.has_collisions(q)


def test_linear_grasp_and_retreat(planner: ArmPlanner, grasps: list[np.ndarray]) -> None:
    found = planner.plan_to_any(HOME, [offset(g, PRE_OFFSET) for g in grasps])
    assert found is not None
    path, index = found
    down = planner.plan_linear(path[-1], grasps[index])
    assert down is not None
    pos_err, _ = poseError(planner.fk(down[-1]), grasps[index])
    assert pos_err < 1e-3

    planner.attach('block', down[-1])
    try:
        up = planner.plan_linear(down[-1], offset(grasps[index], PRE_OFFSET))
        assert up is not None
        # The held block rises with the TCP.
        lifted = planner.fk(up[-1])[:3, 3] - planner.fk(down[-1])[:3, 3]
        assert lifted[2] > 0.8 * PRE_OFFSET * math.cos(math.radians(35))
    finally:
        planner.detach('block', down[-1])


def test_joint_trajectory_timing() -> None:
    path = [HOME, HOME + 0.4, HOME + 0.41, HOME]
    traj = to_joint_trajectory(path, v_max=0.8)
    assert traj.joint_names == ARM_JOINTS
    assert len(traj.points) == 3
    times = [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in traj.points]
    assert all(b > a for a, b in zip(times, times[1:]))
    np.testing.assert_allclose(times, [0.5, 0.6, 1.1125], atol=1e-9)
    np.testing.assert_allclose(traj.points[-1].positions, HOME)


def camera_looking_at(target, distance: float, down_deg: float) -> np.ndarray:
    """Optical frame pose `distance` from `target`, looking along +x tilted `down_deg` down."""
    down = math.radians(down_deg)
    z = np.array([math.cos(down), 0.0, -math.sin(down)])
    x = np.array([0.0, 1.0, 0.0])  # Image right; image down then points away from the robot.
    tform = np.eye(4)
    tform[:3, :3] = np.column_stack([x, np.cross(z, x), z])
    tform[:3, 3] = np.asarray(target) - distance * z
    return tform


def test_look_pose(planner: ArmPlanner) -> None:
    # How LOOK_Q was found: IK for the optical frame (through its fixed offset from the TCP).
    cam_in_tcp = np.linalg.inv(planner.fk(HOME)) @ planner.fk(HOME, CAMERA_FRAME)
    goal = camera_looking_at(BLOCK_START, 0.2, 65.0)
    q = planner.ik(goal @ np.linalg.inv(cam_in_tcp), LOOK_Q)
    assert q is not None
    print(f'look pose by IK: {np.round(q, 3).tolist()}')

    # The hardcoded LOOK_Q is collision free, reachable from home and sees the work area.
    look = np.array(LOOK_Q)
    assert not planner.has_collisions(look)
    assert planner.plan_joint(HOME, look) is not None
    world_to_cam = np.linalg.inv(planner.fk(look, CAMERA_FRAME))
    f = 200 / math.tan(math.radians(65.0 / 2))  # The MuJoCo camera: fovy 65 deg, 640x400.
    for point, margin in ((BLOCK_START, 20), (PLACE_XYZ, 40),
                          (np.add(BLOCK_START, [0.03, 0.0, 0.0]), 40),
                          (np.add(BLOCK_START, [0.0, -0.03, 0.0]), 40)):
        x, y, z = world_to_cam[:3, :3] @ np.asarray(point) + world_to_cam[:3, 3]
        u, v = f * x / z + 320, f * y / z + 200
        assert 0.15 < z < 0.3
        assert margin < u < 640 - margin and margin < v < 400 - margin, (point, u, v)
    x, y, _ = world_to_cam[:3, :3] @ np.asarray(BLOCK_START) + world_to_cam[:3, 3]
    assert math.hypot(x, y) < 0.01  # Block on the optical axis.


def test_plans_from_just_past_a_joint_limit(planner: ArmPlanner) -> None:
    # A measured joint state can overshoot a limit slightly; planning must still start.
    q = HOME.copy()
    q[5] = math.pi + 1e-4  # The joint 6 limit is 3.14159.
    assert not planner.has_collisions(q)
    np.testing.assert_allclose(planner.clamp(q)[5], 3.14159)
    path = planner.plan_joint(q, HOME)
    assert path is not None
    np.testing.assert_allclose(path[-1], HOME, atol=1e-9)
