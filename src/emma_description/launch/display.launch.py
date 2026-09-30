from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gui = LaunchConfiguration('gui')
    model = LaunchConfiguration('model')
    share = FindPackageShare('emma_description')

    robot_description = ParameterValue(
        Command([
            'xacro ', PathJoinSubstitution([share, 'urdf', 'emma.urdf.xacro']),
            ' model:=', model,
        ]),
        value_type=str,
    )
    # The arm-only model has no base_footprint, so root rviz at the arm's base.
    fixed_frame = PythonExpression(
        ["'g_base' if '", model, "' == 'arm' else 'base_footprint'"])

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Start joint_state_publisher_gui to move the arm joints'),
        DeclareLaunchArgument(
            'model', default_value='both', choices=['base', 'arm', 'both'],
            description='Which part of the robot to show'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        # Without the GUI, the plain publisher still publishes zeroed joint states so
        # the arm's TF tree is complete.
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(gui),
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            condition=UnlessCondition(gui),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=[
                '-d', PathJoinSubstitution([share, 'rviz', 'emma.rviz']),
                '--fixed-frame', fixed_frame,
            ],
        ),
    ])
