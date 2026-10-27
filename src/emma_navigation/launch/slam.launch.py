"""Map a room with slam_toolbox while driving emma by hand.

Starts the sim in the living room (or `mujoco_model:=...`), slam_toolbox in online async mapping
mode and RViz. Drive with teleop_twist_keyboard and save the map with map_saver_cli; the
commands are in this package's README.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
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
        DeclareLaunchArgument(
            'mujoco_model',
            default_value=PathJoinSubstitution([
                sim_share, 'mujoco', 'molmospaces', 'scenes', 'ithor', 'FloorPlan201_emma.xml']),
            description='MJCF scene'),
        IncludeLaunchDescription(
            PathJoinSubstitution([sim_share, 'launch', 'sim.launch.py']),
            launch_arguments={
                'headless': LaunchConfiguration('headless'),
                'rviz': 'false',
                'mujoco_model': LaunchConfiguration('mujoco_model'),
            }.items(),
        ),
        # slam_toolbox's default mapper params already use base_footprint and /scan.
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py']),
            launch_arguments={'use_sim_time': 'true'}.items(),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', PathJoinSubstitution(
                [FindPackageShare('nav2_bringup'), 'rviz', 'nav2_default_view.rviz'])],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
