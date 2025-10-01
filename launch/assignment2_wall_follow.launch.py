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
    use_pid_arg = DeclareLaunchArgument(
        'use_pid',
        default_value='true',
        description='Use PID control (true) or bang-bang control (false)'
    )
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='levine',
        description='Name of the map to use (levine, Spielberg_map, or berlin)'
    )
    forward_speed_arg = DeclareLaunchArgument(
        'forward_speed',
        default_value='1.5',
        description='Forward speed for wall following (m/s)'
    )
    desired_distance_arg = DeclareLaunchArgument(
        'desired_distance',
        default_value='1.0',
        description='Desired distance from wall (meters)'
    )
    pkg_share = FindPackageShare('f1tenth_gym_ros').find('f1tenth_gym_ros')
    sim_config = PathJoinSubstitution([pkg_share, 'config', 'sim.yaml'])
    wall_follow_config = PathJoinSubstitution([pkg_share, 'config', 'assignment2_wall_follow.yaml'])
    simulator_launch = IncludeLaunchDescription(
        PathJoinSubstitution([pkg_share, 'launch', 'gym_bridge_launch.py'])
    )
    wall_follow_node = Node(
        package='f1tenth_gym_ros',
        executable='assignment2_wall_follow',
        name='wall_follow',
        parameters=[
            wall_follow_config,
            {
                'use_pid': LaunchConfiguration('use_pid'),
                'forward_speed': LaunchConfiguration('forward_speed'),
                'desired_distance': LaunchConfiguration('desired_distance')
            }
        ],
        output='screen',
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
        use_pid_arg,
        map_name_arg,
        forward_speed_arg,
        desired_distance_arg,
        simulator_launch,
        wall_follow_node,
        teleop_node,
    ])