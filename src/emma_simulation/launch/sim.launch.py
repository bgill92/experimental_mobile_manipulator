"""Emma in MuJoCo through mujoco_ros2_control.

Generate the MuJoCo model first (and again after any URDF change):
    pixi run gen-mjcf

Move the arm:
    ros2 action send_goal /arm_controller/follow_joint_trajectory \\
        control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: \\
        [joint2_to_joint1, joint3_to_joint2, joint4_to_joint3, joint5_to_joint4, \\
        joint6_to_joint5, joint6output_to_joint6], points: \\
        [{positions: [0.5, -0.5, 0.5, 0.0, 0.3, 0.0], time_from_start: {sec: 3}}]}}"

Close the gripper (0 is open, -0.007 is closed):
    ros2 action send_goal /gripper_action_controller/gripper_cmd \\
        control_msgs/action/ParallelGripperCommand \\
        "{command: {name: [gripper_controller], position: [-0.007]}}"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('emma_simulation')
    description_share = FindPackageShare('emma_description')
    controllers_file = PathJoinSubstitution([share, 'config', 'controllers.yaml'])

    robot_description = ParameterValue(
        Command([
            'xacro ', PathJoinSubstitution([description_share, 'urdf', 'emma.urdf.xacro']),
            ' ros2_control:=mujoco',
            ' mujoco_model:=', LaunchConfiguration('mujoco_model'),
            ' headless:=', LaunchConfiguration('headless'),
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run MuJoCo without its viewer window'),
        DeclareLaunchArgument(
            'rviz', default_value='true', description='Start RViz'),
        DeclareLaunchArgument(
            'mujoco_model',
            default_value=PathJoinSubstitution([share, 'mujoco', 'scene.xml']),
            description='MJCF scene'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        ),
        # MuJoCo runs inside this controller manager. It reads robot_description from
        # the topic robot_state_publisher publishes.
        Node(
            package='mujoco_ros2_control',
            executable='ros2_control_node',
            emulate_tty=True,
            output='both',
            parameters=[{'use_sim_time': True}, ParameterFile(controllers_file)],
            on_exit=Shutdown(),
        ),
        # One spawner activates the controllers in order; separate spawners race.
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint_state_broadcaster', 'arm_controller', 'gripper_action_controller',
                '--param-file', controllers_file,
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', PathJoinSubstitution([description_share, 'rviz', 'emma.rviz'])],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
