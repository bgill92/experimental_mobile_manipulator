"""
Scripted pick and place of the block in the sim, as a py_trees behaviour tree.

    ros2 launch emma_behaviors pick_and_place.launch.py

Needs emma_perception's block_detector publishing /block_pose (the launch file starts it).
Exits 0 when the tree succeeds (block placed near PLACE_XYZ, arm home), 1 otherwise.
"""

import sys
import time

from emma_behaviors.behaviours import (
    AttachBlock, block_to_blackboard, CheckPlaced, close_gripper, ComputeGraspPoses,
    DetachBlock, execute, GRASPS, JOINT_STATE, joints_to_blackboard, open_gripper, PLACES,
    PlanToJoints, PlanToTcp, PREGRASPS, PREPLACES, translation)
from emma_manipulation.constants import HOME_Q, LOOK_Q, TABLE_CENTER, TABLE_HALF_SIZE
from emma_manipulation.planner import ArmPlanner
import py_trees
from py_trees.common import Status
import py_trees_ros
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String


# Seconds the arm settles at the look pose before sampling /block_pose, and the longest wait
# for a detection after that.
SETTLE_TIME = 1.0
DETECT_TIMEOUT = 10.0


def create_tree(planner: ArmPlanner) -> py_trees.behaviour.Behaviour:
    """Build the pick-and-place tree; see the package README for a diagram."""
    def move(name: str, plan: py_trees.behaviour.Behaviour) -> list[py_trees.behaviour.Behaviour]:
        return [plan, execute(f'Execute {name}')]

    def look(prefix: str) -> list[py_trees.behaviour.Behaviour]:
        """Go to LOOK_Q, settle, and take one fresh /block_pose sample."""
        return [
            *move(f'{prefix}look', PlanToJoints(f'Plan {prefix}look', planner, LOOK_Q)),
            py_trees.timers.Timer(f'{prefix.capitalize()}settle', duration=SETTLE_TIME),
            py_trees.decorators.Timeout(
                f'{prefix.capitalize()}detect', block_to_blackboard(f'{prefix}block2bb'),
                duration=DETECT_TIMEOUT),
        ]

    pick_place = py_trees.composites.Sequence(name='Pick and place', memory=True, children=[
        *look(''),
        ComputeGraspPoses('Grasp poses', planner),
        open_gripper(),
        *move('pregrasp', PlanToTcp('Plan pregrasp', planner, PREGRASPS, linear=False)),
        *move('grasp', PlanToTcp('Plan grasp', planner, GRASPS, linear=True)),
        close_gripper(),
        AttachBlock('Attach block', planner),
        *move('lift', PlanToTcp('Plan lift', planner, PREGRASPS, linear=True)),
        *move('preplace', PlanToTcp('Plan preplace', planner, PREPLACES, linear=False)),
        *move('place', PlanToTcp('Plan place', planner, PLACES, linear=True)),
        open_gripper('Release'),
        DetachBlock('Detach block', planner),
        *move('retreat', PlanToTcp('Plan retreat', planner, PREPLACES, linear=True)),
        *look('check '),
        CheckPlaced('Check placed'),
        *move('home', PlanToJoints('Plan home', planner, HOME_Q)),
    ])
    recover = py_trees.composites.Sequence(name='Recover', memory=True, children=[
        open_gripper('Recover: open'),
        DetachBlock('Recover: detach', planner),
        *move('recover home', PlanToJoints('Recover: plan home', planner, HOME_Q)),
        py_trees.behaviours.Failure('Give up'),
    ])
    task = py_trees.composites.Sequence(name='Task', memory=True, children=[
        py_trees.behaviours.WaitForBlackboardVariable('Wait for joints', f'/{JOINT_STATE}'),
        py_trees.composites.Selector(name='Pick or recover', memory=True, children=[
            py_trees.decorators.Retry('Retry', pick_place, num_failures=2),
            recover,
        ]),
    ])
    return py_trees.composites.Parallel(
        name='Pick and place demo',
        policy=py_trees.common.ParallelPolicy.SuccessOnSelected([task], synchronise=False),
        children=[joints_to_blackboard(), task])


def wait_for_urdf(node: Node, timeout: float = 30.0) -> str:
    """Read the latched /robot_description once."""
    urdf: list[str] = []
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    sub = node.create_subscription(String, '/robot_description',
                                   lambda msg: urdf.append(msg.data), latched)
    # Wall time: with use_sim_time the node clock stands still until the sim publishes /clock.
    end = time.monotonic() + timeout
    while not urdf and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    if not urdf:
        raise RuntimeError('no message on /robot_description')
    return urdf[0]


def main(argv: list[str] | None = None) -> int:
    rclpy.init(args=argv)
    node = Node('pick_and_place')
    try:
        planner = ArmPlanner(wait_for_urdf(node))
        planner.add_box('table', [2 * h for h in TABLE_HALF_SIZE], translation(TABLE_CENTER),
                        (0.55, 0.4, 0.25, 1.0))
        tree = py_trees_ros.trees.BehaviourTree(create_tree(planner))
        tree.setup(node=node, timeout=15.0)

        def stop_when_done(tree: py_trees_ros.trees.BehaviourTree) -> None:
            if tree.root.status != Status.RUNNING:
                tree.timer.cancel()

        tree.tick_tock(period_ms=200, post_tick_handler=stop_when_done)
        while rclpy.ok() and tree.root.status in (Status.INVALID, Status.RUNNING):
            rclpy.spin_once(node, timeout_sec=0.1)
        status = tree.root.status
        tip = tree.root.tip()
        node.get_logger().info(
            f'tree finished: {status.value}'
            + ('' if tip is None else f' (last: {tip.name}: {tip.feedback_message})'))
        print(py_trees.display.unicode_tree(tree.root, show_status=True))
        tree.shutdown(destroy_node=False)
        return 0 if status == Status.SUCCESS else 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
