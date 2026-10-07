"""
py_trees behaviours for emma's pick and place.

Planning behaviours run the in-process ArmPlanner synchronously inside a tick and leave a
FollowJointTrajectory goal on the blackboard for the stock action client (`execute`) to
send. They all plan from the latest `/joint_states` sample (`joint_state` on the
blackboard), so each move starts where the arm really is.
"""

from collections.abc import Sequence
import math
from typing import Any

from control_msgs.action import FollowJointTrajectory, ParallelGripperCommand
from emma_manipulation.constants import (
    BLOCK_SIZE, GRIPPER_CLOSED, GRIPPER_OPEN, PLACE_XYZ, PRE_OFFSET)
from emma_manipulation.grasp import grasp_candidates, offset
from emma_manipulation.planner import ArmPlanner, to_joint_trajectory
from geometry_msgs.msg import PoseStamped
import numpy as np
import py_trees
from py_trees.common import Access, Status
import py_trees_ros
import rclpy.qos
from sensor_msgs.msg import JointState

ARM_ACTION = '/arm_controller/follow_joint_trajectory'
GRIPPER_ACTION = '/gripper_action_controller/gripper_cmd'
GRIPPER_JOINT = 'gripper_controller'
BLOCK_POSE_TOPIC = '/block_pose'
BLOCK = 'block'

# Blackboard keys.
JOINT_STATE = 'joint_state'
BLOCK_POSE = 'block_pose'
TRAJECTORY = 'trajectory'
GRASP_INDEX = 'grasp_index'
GRASPS = 'grasp_candidates'
PREGRASPS = 'pregrasp_candidates'
PLACES = 'place_candidates'
PREPLACES = 'preplace_candidates'


def pose_to_tform(pose: PoseStamped) -> np.ndarray:
    """Convert a PoseStamped to a 4x4 transform (the header frame is not checked)."""
    p, q = pose.pose.position, pose.pose.orientation
    x, y, z, w = q.x, q.y, q.z, q.w
    tform = np.eye(4)
    tform[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    tform[:3, 3] = (p.x, p.y, p.z)
    return tform


def yaw_of(pose: PoseStamped) -> float:
    q = pose.pose.orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def make_pose(xyz: Sequence[float], yaw: float, frame: str) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = (float(v) for v in xyz)
    pose.pose.orientation.z = math.sin(yaw / 2)
    pose.pose.orientation.w = math.cos(yaw / 2)
    return pose


def translation(xyz: Sequence[float]) -> np.ndarray:
    tform = np.eye(4)
    tform[:3, 3] = xyz
    return tform


class _LoggingBehaviour(py_trees.behaviour.Behaviour):
    """Base that keeps the tree's ROS node (from `setup`) to log feedback messages."""

    def __init__(self, name: str) -> None:
        super().__init__(name)
        self.node: Any = None
        self.bb = self.attach_blackboard_client(name=name)

    def setup(self, **kwargs: Any) -> None:
        self.node = kwargs.get('node')

    def log(self, message: str) -> None:
        self.feedback_message = message
        if self.node is not None:
            self.node.get_logger().info(f'[{self.name}] {message}')


class _PlannerBehaviour(_LoggingBehaviour):
    """Base for behaviours that use the planner and the arm's current joint state."""

    def __init__(self, name: str, planner: ArmPlanner) -> None:
        super().__init__(name)
        self.planner = planner
        self.bb.register_key(JOINT_STATE, access=Access.READ)

    def current_q(self) -> np.ndarray:
        return self.planner.set_q(self.bb.get(JOINT_STATE))


class _PlanBehaviour(_PlannerBehaviour):
    """Base for planning behaviours: writes the timed trajectory goal on success."""

    def __init__(self, name: str, planner: ArmPlanner) -> None:
        super().__init__(name, planner)
        self.bb.register_key(TRAJECTORY, access=Access.WRITE)

    def write(self, path: list[np.ndarray] | None) -> Status:
        if path is None:
            self.log('no plan found')
            return Status.FAILURE
        self.bb.set(TRAJECTORY, FollowJointTrajectory.Goal(
            trajectory=to_joint_trajectory(path)))
        self.log(f'planned {len(path)} waypoints')
        return Status.SUCCESS


class PlanToJoints(_PlanBehaviour):
    """Plan a collision-free joint move to `q_goal`."""

    def __init__(self, name: str, planner: ArmPlanner, q_goal: Sequence[float]) -> None:
        super().__init__(name, planner)
        self.q_goal = np.asarray(q_goal, dtype=float)

    def update(self) -> Status:
        return self.write(self.planner.plan_joint(self.current_q(), self.q_goal))


class PlanToTcp(_PlanBehaviour):
    """
    Plan the TCP to the pose(s) under blackboard `key`.

    `key` holds one 4x4 or a list of candidates. For a list, the grasp candidate already
    chosen (`grasp_index`) is used if there is one; otherwise an RRT move tries each
    candidate in order and records the one it reached in `grasp_index`, so every later
    move (grasp, lift, place) keeps the same grasp. `linear` moves in a short straight
    line instead of planning around obstacles.
    """

    def __init__(self, name: str, planner: ArmPlanner, key: str, linear: bool) -> None:
        super().__init__(name, planner)
        self.key = key
        self.linear = linear
        self.bb.register_key(key, access=Access.READ)
        self.bb.register_key(GRASP_INDEX, access=Access.WRITE)

    def update(self) -> Status:
        target = self.bb.get(self.key)
        q = self.current_q()
        if isinstance(target, list):
            index = self.bb.get(GRASP_INDEX) if self.bb.exists(GRASP_INDEX) else None
            if index is None:
                if self.linear:
                    self.log('no grasp chosen yet for a linear move')
                    return Status.FAILURE
                found = self.planner.plan_to_any(q, target)
                if found is None:
                    return self.write(None)
                path, index = found
                self.bb.set(GRASP_INDEX, index)
                self.log(f'chose grasp candidate {index}')
                return self.write(path)
            target = target[index]
        if self.linear:
            return self.write(self.planner.plan_linear(q, target))
        found = self.planner.plan_to_any(q, [target])
        return self.write(None if found is None else found[0])


class ComputeGraspPoses(_PlannerBehaviour):
    """
    Turn `block_pose` into grasp, pregrasp, place and preplace candidates.

    Place candidate i is grasp candidate i moved to PLACE_XYZ, so the block lands
    upright whichever grasp is chosen. Also puts the block in the planning scene and
    clears `grasp_index` so the next RRT move chooses afresh.
    """

    def __init__(self, name: str, planner: ArmPlanner,
                 place_xyz: Sequence[float] = PLACE_XYZ) -> None:
        super().__init__(name, planner)
        self.place_xyz = np.asarray(place_xyz, dtype=float)
        self.bb.register_key(BLOCK_POSE, access=Access.READ)
        for key in (GRASPS, PREGRASPS, PLACES, PREPLACES, GRASP_INDEX):
            self.bb.register_key(key, access=Access.WRITE)

    def update(self) -> Status:
        pose = self.bb.get(BLOCK_POSE)
        block = pose_to_tform(pose)
        arm_base_xy = self.planner.fk(self.current_q(), 'g_base')[:2, 3]
        grasps = grasp_candidates(block[:3, 3].tolist(), yaw_of(pose), arm_base_xy)
        shift = self.place_xyz - block[:3, 3]
        places = []
        for grasp in grasps:
            place = grasp.copy()
            place[:3, 3] += shift
            places.append(place)
        self.bb.set(GRASPS, grasps)
        self.bb.set(PREGRASPS, [offset(g, PRE_OFFSET) for g in grasps])
        self.bb.set(PLACES, places)
        self.bb.set(PREPLACES, [offset(p, PRE_OFFSET) for p in places])
        self.bb.set(GRASP_INDEX, None)
        self.planner.add_box(BLOCK, [BLOCK_SIZE] * 3, block, (0.9, 0.1, 0.1, 1.0))
        xyz = ' '.join(f'{v:.3f}' for v in block[:3, 3])
        self.log(f'{len(grasps)} grasp candidates for the block at [{xyz}], '
                 f'yaw {math.degrees(yaw_of(pose)):.0f} deg')
        return Status.SUCCESS


class AttachBlock(_PlannerBehaviour):
    """Attach the block to the gripper in the planning scene."""

    def update(self) -> Status:
        self.planner.attach(BLOCK, self.current_q())
        return Status.SUCCESS


class DetachBlock(_PlannerBehaviour):
    """Leave the block where the gripper holds it. Succeeds if there is no block."""

    def update(self) -> Status:
        if self.planner.has_box(BLOCK):
            self.planner.detach(BLOCK, self.current_q())
        return Status.SUCCESS


class CheckPlaced(_LoggingBehaviour):
    """Check the block's perceived position (`block_pose`) against `place_xyz`."""

    def __init__(self, name: str, place_xyz: Sequence[float] = PLACE_XYZ,
                 tol: float = 0.02) -> None:
        super().__init__(name)
        self.place_xyz = np.asarray(place_xyz, dtype=float)
        self.tol = tol
        self.bb.register_key(BLOCK_POSE, access=Access.READ)

    def update(self) -> Status:
        p = self.bb.get(BLOCK_POSE).pose.position
        error = float(np.linalg.norm(np.array([p.x, p.y, p.z]) - self.place_xyz))
        self.log(f'block at [{p.x:.3f} {p.y:.3f} {p.z:.3f}], {error * 1000:.1f} mm from target')
        return Status.SUCCESS if error <= self.tol else Status.FAILURE


# Stock behaviours.

def execute(name: str) -> py_trees.behaviour.Behaviour:
    """Send the `trajectory` goal to arm_controller and wait for it to finish."""
    return py_trees_ros.action_clients.FromBlackboard(
        name=name, action_type=FollowJointTrajectory, action_name=ARM_ACTION,
        key=TRAJECTORY)


def gripper(name: str, position: float) -> py_trees.behaviour.Behaviour:
    goal = ParallelGripperCommand.Goal()
    goal.command = JointState(name=[GRIPPER_JOINT], position=[position])
    return py_trees_ros.action_clients.FromConstant(
        name=name, action_type=ParallelGripperCommand, action_name=GRIPPER_ACTION,
        action_goal=goal)


def open_gripper(name: str = 'Open gripper') -> py_trees.behaviour.Behaviour:
    return gripper(name, GRIPPER_OPEN)


def close_gripper(name: str = 'Close gripper') -> py_trees.behaviour.Behaviour:
    return gripper(name, GRIPPER_CLOSED)


def joints_to_blackboard() -> py_trees.behaviour.Behaviour:
    """Keep the latest /joint_states message in `joint_state`."""
    return py_trees_ros.subscribers.ToBlackboard(
        name='joints2bb', topic_name='/joint_states', topic_type=JointState,
        qos_profile=rclpy.qos.qos_profile_sensor_data,
        blackboard_variables={JOINT_STATE: None},
        clearing_policy=py_trees.common.ClearingPolicy.NEVER)


def block_to_blackboard(name: str = 'block2bb') -> py_trees.behaviour.Behaviour:
    """
    Wait for a fresh detection on /block_pose and keep it in `block_pose`.

    Cleared on every start, so it stays RUNNING until a message arrives after the behaviour
    starts: one sample per visit, taken after the arm has settled at the look pose.
    """
    return py_trees_ros.subscribers.ToBlackboard(
        name=name, topic_name=BLOCK_POSE_TOPIC, topic_type=PoseStamped,
        qos_profile=rclpy.qos.QoSProfile(depth=1),
        blackboard_variables={BLOCK_POSE: None},
        clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE)
