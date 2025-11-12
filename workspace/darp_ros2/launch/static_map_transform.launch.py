#!/usr/bin/env python3
"""
Static Map Transform Launch File

Publishes a static transform from 'map' to 'odom' frame to provide the missing
root frame required by Nav2. Uses identity transform since we operate in a known
environment without localization (AMCL/SLAM).

This resolves the critical missing frame issue that would cause controller_server
to timeout waiting for transforms.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with static map->odom transform."""

    # Static transform: map -> odom (identity transform)
    # x, y, z, roll, pitch, yaw all = 0
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=[
            '0', '0', '0',  # x, y, z translation
            '0', '0', '0',  # roll, pitch, yaw rotation
            'map',          # parent frame
            'odom'          # child frame
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        static_tf_node
    ])
