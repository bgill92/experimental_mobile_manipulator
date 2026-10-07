"""
Plan and run one arm motion from the command line.

    ros2 run emma_manipulation move_arm --home
    ros2 run emma_manipulation move_arm --joints 0 0 0 0 0 0
    ros2 run emma_manipulation move_arm --pose 0.25 0 0.20 0 3.1416 0
    ros2 run emma_manipulation move_arm --linear 0.25 0 0.18 0 3.1416 0

Poses are the TCP in base_link: x y z in metres, then optional roll pitch yaw in radians
(fixed axes, applied x then y then z; default 0 0 0). --pose plans around obstacles with
RRT; --linear moves in a short straight line from the current pose. The table from
emma_manipulation.constants is in the planning scene unless --no-table is given.
"""

import argparse
import math
import sys

from control_msgs.action import FollowJointTrajectory
from emma_manipulation.constants import ARM_JOINTS, HOME_Q, TABLE_CENTER, TABLE_HALF_SIZE
from emma_manipulation.planner import ArmPlanner, to_joint_trajectory
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from roboplan.core import poseError
from sensor_msgs.msg import JointState
from std_msgs.msg import String

ACTION = '/arm_controller/follow_joint_trajectory'


def pose_to_tform(values: list[float]) -> np.ndarray:
    """Build a 4x4 from [x, y, z] or [x, y, z, roll, pitch, yaw]."""
    if len(values) not in (3, 6):
        raise ValueError('a pose is x y z [roll pitch yaw]')
    x, y, z = values[:3]
    roll, pitch, yaw = values[3:] if len(values) == 6 else (0.0, 0.0, 0.0)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    tform = np.eye(4)
    tform[:3, :3] = rz @ ry @ rx
    tform[:3, 3] = (x, y, z)
    return tform


def add_table(planner: ArmPlanner) -> None:
    tform = np.eye(4)
    tform[:3, 3] = TABLE_CENTER
    planner.add_box('table', [2 * h for h in TABLE_HALF_SIZE], tform, (0.55, 0.4, 0.25, 1.0))


class MoveArm(Node):
    """Reads the robot model and joint state, and runs trajectories on arm_controller."""

    def __init__(self) -> None:
        super().__init__('move_arm')
        self._urdf: str | None = None
        self._joint_state: JointState | None = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/robot_description', self._on_urdf, latched)
        self.create_subscription(JointState, '/joint_states', self._on_joints, 10)
        self._client = ActionClient(self, FollowJointTrajectory, ACTION)

    def _on_urdf(self, msg: String) -> None:
        self._urdf = msg.data

    def _on_joints(self, msg: JointState) -> None:
        if all(j in msg.name for j in ARM_JOINTS):
            self._joint_state = msg

    def wait_for_urdf(self, timeout: float = 10.0) -> str:
        end = self.get_clock().now().nanoseconds + timeout * 1e9
        while self._urdf is None and self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._urdf is None:
            raise RuntimeError('no message on /robot_description')
        return self._urdf

    def fresh_joint_state(self, timeout: float = 5.0) -> JointState:
        """Wait for a joint state newer than the call."""
        self._joint_state = None
        end = self.get_clock().now().nanoseconds + timeout * 1e9
        while self._joint_state is None and self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._joint_state is None:
            raise RuntimeError('no arm joints on /joint_states')
        return self._joint_state

    def execute(self, goal: FollowJointTrajectory.Goal, timeout: float = 60.0) -> bool:
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f'{ACTION} not available')
            return False
        send = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=10.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('trajectory goal rejected')
            return False
        future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        response = future.result()
        if response is None:
            self.get_logger().error('timed out waiting for the trajectory to finish')
            return False
        code = response.result.error_code
        if code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(
                f'trajectory failed ({code}): {response.result.error_string}')
            return False
        return True


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    target = parser.add_mutually_exclusive_group(required=True)
    target.add_argument('--home', action='store_true', help='go to HOME_Q')
    target.add_argument('--joints', type=float, nargs=6, metavar='Q',
                        help='arm joint positions in rad')
    target.add_argument('--pose', type=float, nargs='+', metavar='V',
                        help='TCP pose x y z [roll pitch yaw], planned with RRT')
    target.add_argument('--linear', type=float, nargs='+', metavar='V',
                        help='TCP pose x y z [roll pitch yaw], straight-line move')
    parser.add_argument('--no-table', action='store_true',
                        help='leave the table out of the planning scene')
    parser.add_argument('--v-max', type=float, default=0.8,
                        help='max joint speed in rad/s (default 0.8)')
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    rclpy.init(args=argv)
    args = parse_args(rclpy.utilities.remove_ros_args(argv or sys.argv)[1:])
    node = MoveArm()
    try:
        return run(node, args)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def run(node: MoveArm, args: argparse.Namespace) -> int:
    log = node.get_logger()
    planner = ArmPlanner(node.wait_for_urdf())
    if not args.no_table:
        add_table(planner)
    q_start = planner.set_q(node.fresh_joint_state())

    if args.home or args.joints is not None:
        q_goal = np.array(HOME_Q if args.home else args.joints)
        target = planner.fk(q_goal)
        path = planner.plan_joint(q_start, q_goal)
    else:
        target = pose_to_tform(args.pose if args.pose is not None else args.linear)
        if args.pose is not None:
            found = planner.plan_to_any(q_start, [target])
            path = None if found is None else found[0]
        else:
            path = planner.plan_linear(q_start, target)
    if path is None:
        log.error('no collision-free plan found')
        return 1
    log.info(f'planned {len(path)} waypoints')

    goal = FollowJointTrajectory.Goal(trajectory=to_joint_trajectory(path, args.v_max))
    if not node.execute(goal):
        return 1

    reached = planner.fk(planner.set_q(node.fresh_joint_state()))
    pos_err, rot_err = poseError(reached, target)
    xyz = ' '.join(f'{v:.4f}' for v in reached[:3, 3])
    log.info(f'TCP at [{xyz}], error {pos_err * 1000:.1f} mm, {math.degrees(rot_err):.1f} deg')
    return 0


if __name__ == '__main__':
    sys.exit(main())
