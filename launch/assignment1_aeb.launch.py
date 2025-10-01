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
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='levine',
        description='Name of the map to use (levine or Spielberg_map)'
    )
    pkg_share = FindPackageShare('f1tenth_gym_ros').find('f1tenth_gym_ros')
    aeb_config = PathJoinSubstitution([pkg_share, 'config', 'assignment1_aeb.yaml'])
    simulator_launch = IncludeLaunchDescription(
        PathJoinSubstitution([pkg_share, 'launch', 'gym_bridge_launch.py'])
    )
    aeb_node = Node(
        package='f1tenth_gym_ros',
        executable='assignment1_aeb',
        name='aeb',
        parameters=[aeb_config],
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
        map_name_arg,
        simulator_launch,
        aeb_node,
        teleop_node,
    ])