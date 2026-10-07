"""Shared constants for emma's manipulation, all in metres / radians and `base_link`."""

import math

# Same order as arm_controller's joints in emma_simulation/config/controllers.yaml.
ARM_JOINTS = [
    'joint2_to_joint1',
    'joint3_to_joint2',
    'joint4_to_joint3',
    'joint5_to_joint4',
    'joint6_to_joint5',
    'joint6output_to_joint6',
]

# Arm folded back over the base; matches the ros2_control initial_value in emma.urdf.xacro.
# The wrist is tipped up (joint 4 at 0.3, not -0.495) so the wrist camera clears the arm.
HOME_Q = [0.0, 2.259, -2.505, 0.3, 0.0, 0.0]

# Wrist camera looking at BLOCK_START from 0.2 m, its optical axis 65 deg below horizontal
# (at 45 deg the camera would sit almost over the arm base, out of reach). PLACE_XYZ and a
# block a few cm off BLOCK_START stay in view. Found by IK in test_planner.py::test_look_pose,
# which re-checks it.
LOOK_Q = [0.862, 0.771, -1.533, -0.695, -0.133, 0.855]

BASE_FRAME = 'base_link'
CAMERA_FRAME = 'wrist_camera_color_optical_frame'
TCP_FRAME = 'tcp'

# Table in front of the AGV: box centre and half-size. Top at z = 0.15, x from 0.20 to 0.40.
TABLE_CENTER = (0.30, 0.0, 0.075)
TABLE_HALF_SIZE = (0.10, 0.15, 0.075)

BLOCK_SIZE = 0.010
BLOCK_START = (0.25, 0.0, 0.155)
PLACE_XYZ = (0.25, 0.08, 0.155)

# Distance backed off along the approach axis for pregrasp, retreat and preplace.
PRE_OFFSET = 0.05
# Grasp tilts away from straight down, leaning the approach away from the robot.
GRASP_TILTS = [0.0, math.radians(20.0), math.radians(35.0)]

GRIPPER_OPEN = 0.0
GRIPPER_CLOSED = -0.007
