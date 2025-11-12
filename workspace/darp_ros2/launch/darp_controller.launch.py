#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # Path to controller config file
    darp_ros2_dir = get_package_share_directory('darp_ros2')
    controller_yaml = os.path.join(darp_ros2_dir, 'config', 'controller.yaml')

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        parameters=[controller_yaml],
        remappings=[('/cmd_vel', '/cmd_vel'), ('/odom', '/odom')],
        output='screen'
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='controller_lifecycle_manager',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['controller_server'],
            'bond_timeout': 15.0,
            'attempt_respawn_reconnection': True,
            'bond_respawn_max_duration': 30.0
        }],
        output='screen'
    )

    path_follower_client = Node(
        package='darp_ros2',
        executable='path_follower_client',
        name='path_follower_client',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'action_server_name': '/controller_server/follow_path',
            'coverage_path_topic': '/coverage_path',
            'action_timeout_sec': 10.0,
            'watchdog_timeout_sec': 120.0
        }]
    )

    return LaunchDescription([
        declare_use_sim_time,
        LogInfo(msg='=== DARP Controller Launch Sequence ==='),
        static_tf,
        TimerAction(
            period=2.0,
            actions=[LogInfo(msg='Starting controller_server...'), controller_server]
        ),
        TimerAction(
            period=4.0,
            actions=[LogInfo(msg='Starting lifecycle_manager...'), lifecycle_manager]
        ),
        TimerAction(
            period=9.0,
            actions=[LogInfo(msg='Starting path_follower_client...'), path_follower_client]
        ),
        LogInfo(msg='Launch sequence initiated. Total startup time: ~9 seconds.')
    ])
