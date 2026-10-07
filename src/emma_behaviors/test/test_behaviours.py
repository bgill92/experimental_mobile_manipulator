"""Behaviour checks against the real planner, without ROS communication or the sim."""

import math

from ament_index_python.packages import get_package_share_directory
from emma_behaviors import behaviours as bh
from emma_behaviors.pick_and_place import create_tree
from emma_manipulation.constants import (
    ARM_JOINTS, BLOCK_START, HOME_Q, PLACE_XYZ, PRE_OFFSET, TABLE_CENTER, TABLE_HALF_SIZE)
from emma_manipulation.planner import ArmPlanner
import numpy as np
import py_trees
from py_trees.common import Status
import pytest
from sensor_msgs.msg import JointState
import xacro


@pytest.fixture(scope='module')
def planner() -> ArmPlanner:
    urdf = xacro.process_file(
        get_package_share_directory('emma_description') + '/urdf/emma.urdf.xacro').toxml()
    planner = ArmPlanner(urdf, seed=7)
    planner.add_box('table', [2 * h for h in TABLE_HALF_SIZE], bh.translation(TABLE_CENTER))
    return planner


@pytest.fixture
def blackboard():
    py_trees.blackboard.Blackboard.enable_activity_stream()
    client = py_trees.blackboard.Client(name='test')
    for key in (bh.JOINT_STATE, bh.BLOCK_POSE, bh.TRAJECTORY, bh.GRASP_INDEX, bh.GRASPS,
                bh.PREGRASPS, bh.PLACES, bh.PREPLACES):
        client.register_key(key, access=py_trees.common.Access.WRITE)
    client.set(bh.JOINT_STATE, JointState(name=list(ARM_JOINTS), position=list(HOME_Q)))
    yield client
    py_trees.blackboard.Blackboard.clear()


def tick(behaviour: py_trees.behaviour.Behaviour) -> Status:
    behaviour.tick_once()
    return behaviour.status


def test_pose_round_trip() -> None:
    pose = bh.make_pose([0.1, 0.2, 0.3], math.radians(30.0), 'base_link')
    tform = bh.pose_to_tform(pose)
    np.testing.assert_allclose(tform[:3, 3], [0.1, 0.2, 0.3])
    assert bh.yaw_of(pose) == pytest.approx(math.radians(30.0))
    np.testing.assert_allclose(tform[:3, :3] @ tform[:3, :3].T, np.eye(3), atol=1e-12)


def test_compute_grasp_poses(planner: ArmPlanner, blackboard) -> None:
    blackboard.set(bh.BLOCK_POSE, bh.make_pose(BLOCK_START, 0.0, 'base_link'))
    assert tick(bh.ComputeGraspPoses('grasps', planner)) == Status.SUCCESS
    grasps = blackboard.get(bh.GRASPS)
    places = blackboard.get(bh.PLACES)
    assert len(grasps) == len(places) == len(blackboard.get(bh.PREGRASPS)) > 0
    assert blackboard.get(bh.GRASP_INDEX) is None
    assert planner.has_box(bh.BLOCK)
    for grasp, place, preplace in zip(grasps, places, blackboard.get(bh.PREPLACES)):
        np.testing.assert_allclose(grasp[:3, 3], BLOCK_START)
        np.testing.assert_allclose(place[:3, 3], PLACE_XYZ)
        np.testing.assert_allclose(place[:3, :3], grasp[:3, :3])
        assert np.linalg.norm(preplace[:3, 3] - place[:3, 3]) == pytest.approx(PRE_OFFSET)


def test_plan_to_tcp_chooses_then_reuses_grasp(planner: ArmPlanner, blackboard) -> None:
    blackboard.set(bh.BLOCK_POSE, bh.make_pose(BLOCK_START, 0.0, 'base_link'))
    tick(bh.ComputeGraspPoses('grasps', planner))
    linear_first = bh.PlanToTcp('linear first', planner, bh.GRASPS, linear=True)
    assert tick(linear_first) == Status.FAILURE

    assert tick(bh.PlanToTcp('pregrasp', planner, bh.PREGRASPS, linear=False)) == Status.SUCCESS
    index = blackboard.get(bh.GRASP_INDEX)
    assert index is not None
    goal = blackboard.get(bh.TRAJECTORY)
    assert list(goal.trajectory.joint_names) == ARM_JOINTS
    q_pregrasp = goal.trajectory.points[-1].positions
    blackboard.set(bh.JOINT_STATE, JointState(name=list(ARM_JOINTS), position=list(q_pregrasp)))

    assert tick(bh.PlanToTcp('grasp', planner, bh.GRASPS, linear=True)) == Status.SUCCESS
    q_grasp = blackboard.get(bh.TRAJECTORY).trajectory.points[-1].positions
    reached = planner.fk(q_grasp)
    np.testing.assert_allclose(reached[:3, 3], blackboard.get(bh.GRASPS)[index][:3, 3], atol=1e-3)


def test_check_placed(blackboard) -> None:
    near = [PLACE_XYZ[0] + 0.01, PLACE_XYZ[1], PLACE_XYZ[2]]
    blackboard.set(bh.BLOCK_POSE, bh.make_pose(near, 0.0, 'base_link'))
    assert tick(bh.CheckPlaced('check')) == Status.SUCCESS
    blackboard.set(bh.BLOCK_POSE, bh.make_pose(BLOCK_START, 0.0, 'base_link'))
    assert tick(bh.CheckPlaced('check')) == Status.FAILURE


def test_detach_without_block_succeeds(planner: ArmPlanner, blackboard) -> None:
    if planner.has_box(bh.BLOCK):
        planner.remove(bh.BLOCK)
    assert tick(bh.DetachBlock('detach', planner)) == Status.SUCCESS


def test_tree_builds(planner: ArmPlanner) -> None:
    root = create_tree(planner)
    names = [b.name for b in root.iterate()]
    for expected in ('joints2bb', 'Retry', 'Recover', 'Plan look', 'block2bb', 'Close gripper',
                     'check block2bb', 'Check placed', 'Execute home'):
        assert expected in names
