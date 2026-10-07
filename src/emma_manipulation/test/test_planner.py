"""Planner checks against emma's real URDF, with the table and block in the scene."""

import math

from ament_index_python.packages import get_package_share_directory
from emma_manipulation.constants import (
    ARM_JOINTS, BLOCK_SIZE, BLOCK_START, HOME_Q, PRE_OFFSET, TABLE_CENTER,
    TABLE_HALF_SIZE)
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
