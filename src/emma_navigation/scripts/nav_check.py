#!/usr/bin/env python3
"""Send one Nav2 goal and check that emma really got there.

Usage: nav_check.py x y yaw  (goal in the map frame, metres and radians)

The map's origin is emma's spawn pose, so the expected end pose in the sim world is the spawn pose
from /simulator/floating_base_state composed with the goal. Exits 0 if Nav2 reports success and
the ground-truth pose is within TOLERANCE of that.
"""

import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry

# Nav2's goal checker allows 0.1 m; the rest is AMCL's error against the map.
TOLERANCE = 0.2  # m


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main() -> int:
    gx, gy, gyaw = map(float, sys.argv[1:4])
    rclpy.init()
    nav = BasicNavigator()
    truth: list[Odometry] = []
    nav.create_subscription(Odometry, '/simulator/floating_base_state', truth.append, 10)
    nav.waitUntilNav2Active(localizer='amcl')
    while not truth:
        rclpy.spin_once(nav, timeout_sec=0.1)
    start = truth[-1].pose.pose
    syaw = yaw_of(start.orientation)
    expected = (start.position.x + math.cos(syaw) * gx - math.sin(syaw) * gy,
                start.position.y + math.sin(syaw) * gx + math.cos(syaw) * gy)

    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = nav.get_clock().now().to_msg()
    goal.pose.position.x, goal.pose.position.y = gx, gy
    goal.pose.orientation.z, goal.pose.orientation.w = math.sin(gyaw / 2), math.cos(gyaw / 2)
    nav.goToPose(goal)
    while not nav.isTaskComplete():
        rclpy.spin_once(nav, timeout_sec=0.1)
    result = nav.getResult()

    for _ in range(10):
        rclpy.spin_once(nav, timeout_sec=0.1)
    end = truth[-1].pose.pose
    error = math.hypot(end.position.x - expected[0], end.position.y - expected[1])
    yaw_error = abs(math.remainder(yaw_of(end.orientation) - syaw - gyaw, 2 * math.pi))
    print(f'nav-check: Nav2 result {result.name}, ground-truth error {error:.3f} m, '
          f'{math.degrees(yaw_error):.1f} deg')
    return 0 if result == TaskResult.SUCCEEDED and error < TOLERANCE else 1


if __name__ == '__main__':
    sys.exit(main())
