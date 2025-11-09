#!/usr/bin/env python3
"""Launch Nav2 smoother server and the DARP path smoother client."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    path_downsample = LaunchConfiguration('path_downsampling_factor', default='4')
    min_turn_radius = LaunchConfiguration('minimum_turning_radius', default='0.0')

    default_config_path = os.path.join(
        get_package_share_directory('darp_ros2'), 'config', 'darp_smoother.yaml')
    config_file = LaunchConfiguration('smoother_config', default=default_config_path)

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[
            config_file,
            {
                'use_sim_time': use_sim_time,
                'ConstrainedSmoother.path_downsampling_factor': path_downsample,
                'ConstrainedSmoother.minimum_turning_radius': min_turn_radius,
            }
        ],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='smoother_lifecycle_manager',
        output='screen',
        parameters=[
            {
                'autostart': True,
                'node_names': ['smoother_server'],
                'bond_timeout': 5.0,
                'bond_timeout_error': True,
                'use_sim_time': use_sim_time,
            }
        ],
    )

    helper_node = Node(
        package='darp_ros2',
        executable='path_smoother_client',
        name='path_smoother_client',
        output='screen',
        parameters=[
            {
                'raw_path_topic': '/coverage_path',
                'smoothed_path_topic': '/coverage_path_smooth',
                'action_name': '/smoother_server/smooth_path',
                'use_sim_time': use_sim_time,
            }
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('path_downsampling_factor', default_value='4'),
        DeclareLaunchArgument('minimum_turning_radius', default_value='0.0'),
        DeclareLaunchArgument('smoother_config', default_value=default_config_path),
        smoother_server,
        lifecycle_manager,
        helper_node,
    ])
