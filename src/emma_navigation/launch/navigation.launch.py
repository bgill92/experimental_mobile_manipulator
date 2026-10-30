"""Nav2 on emma in the sim: map_server + AMCL on a saved map, MPPI for the mecanum base.

Starts the sim in the living room (or `mujoco_model:=...` with a matching `map:=...`), the Nav2
stack from nav2_bringup with config/nav2_params.yaml, and RViz. Send goals with RViz's
"Nav2 Goal" tool or scripts/nav_check.py.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('emma_navigation')
    sim_share = FindPackageShare('emma_simulation')
    nav2_share = FindPackageShare('nav2_bringup')
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
        DeclareLaunchArgument(
            'map', default_value=PathJoinSubstitution([share, 'maps', 'FloorPlan201.yaml']),
            description='Map yaml for map_server; must match the scene and spawn pose'),
        IncludeLaunchDescription(
            PathJoinSubstitution([sim_share, 'launch', 'sim.launch.py']),
            launch_arguments={
                'headless': LaunchConfiguration('headless'),
                'rviz': 'false',
                'mujoco_model': LaunchConfiguration('mujoco_model'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([nav2_share, 'launch', 'bringup_launch.py']),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'params_file': PathJoinSubstitution([share, 'config', 'nav2_params.yaml']),
                'use_sim_time': 'true',
                # No keepout or speed masks for this room.
                'use_keepout_zones': 'false',
                'use_speed_zones': 'false',
            }.items(),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', PathJoinSubstitution([nav2_share, 'rviz', 'nav2_default_view.rviz'])],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
