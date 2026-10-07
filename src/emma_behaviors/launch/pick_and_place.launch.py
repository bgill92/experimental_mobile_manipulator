"""
Sim with the pick scene (table + block) and the pick-and-place behaviour tree.

The launch shuts down when the tree finishes. `ros2 launch` does not pass the tree's exit code
through, so use `pixi run pick-check` for a pass/fail result.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_share = FindPackageShare('emma_simulation')
    return LaunchDescription([
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run MuJoCo without its viewer window'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        IncludeLaunchDescription(
            PathJoinSubstitution([sim_share, 'launch', 'sim.launch.py']),
            launch_arguments={
                'headless': LaunchConfiguration('headless'),
                'rviz': LaunchConfiguration('rviz'),
                'mujoco_model': PathJoinSubstitution([sim_share, 'mujoco', 'pick_scene.xml']),
            }.items(),
        ),
        Node(
            package='emma_behaviors',
            executable='pick_and_place',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': True}],
            on_exit=Shutdown(),
        ),
    ])
