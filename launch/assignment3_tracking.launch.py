#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_pure_pursuit_arg = DeclareLaunchArgument(
        'use_pure_pursuit',
        default_value='true',
        description='Use Pure Pursuit control (true) or Stanley control (false)'
    )
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='levine',
        description='Name of the map to use (levine, Spielberg_map, or berlin)'
    )
    forward_speed_arg = DeclareLaunchArgument(
        'forward_speed',
        default_value='2.0',
        description='Forward speed for tracking (m/s)'
    )
    lookahead_distance_arg = DeclareLaunchArgument(
        'lookahead_distance',
        default_value='1.5',
        description='Lookahead distance for Pure Pursuit (meters)'
    )
    waypoint_file_arg = DeclareLaunchArgument(
        'waypoint_file',
        default_value='',
        description='Path to CSV file with waypoints (empty = use default path)'
    )
    path_type_arg = DeclareLaunchArgument(
        'path_type',
        default_value='oval',
        description='Default path type: oval, figure8, or straight'
    )
    pkg_share = FindPackageShare('f1tenth_gym_ros').find('f1tenth_gym_ros')
    sim_config = PathJoinSubstitution([pkg_share, 'config', 'sim.yaml'])
    tracking_config = PathJoinSubstitution([pkg_share, 'config', 'assignment3_tracking.yaml'])
    simulator_launch = IncludeLaunchDescription(
        PathJoinSubstitution([pkg_share, 'launch', 'gym_bridge_launch.py'])
    )
    tracking_node = Node(
        package='f1tenth_gym_ros',
        executable='assignment3_tracking',
        name='tracking',
        parameters=[
            tracking_config,
            {
                'use_pure_pursuit': LaunchConfiguration('use_pure_pursuit'),
                'forward_speed': LaunchConfiguration('forward_speed'),
                'lookahead_distance': LaunchConfiguration('lookahead_distance'),
                'waypoint_file': LaunchConfiguration('waypoint_file'),
                'default_path_type': LaunchConfiguration('path_type')
            }
        ],
        output='screen',
    )
    static_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_keyboard',
        output='screen',
        prefix='xterm -e',
        parameters=[{'speed': 0.5, 'turn': 1.0}]
    )
    return LaunchDescription([
        use_pure_pursuit_arg,
        map_name_arg,
        forward_speed_arg,
        lookahead_distance_arg,
        waypoint_file_arg,
        path_type_arg,
        simulator_launch,
        tracking_node,
        static_transform_node,
        teleop_node,
    ])