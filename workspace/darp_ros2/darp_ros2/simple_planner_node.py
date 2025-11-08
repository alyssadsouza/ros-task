#!/usr/bin/env python3
"""
Simple DARP Planner Node - Minimal test implementation
Runs DARP algorithm once and logs results.

Uses sys.path to import DARP without modifying DARP source code.
"""
import sys
import os

# Add DARP to Python path (Docker path)
darp_path = '/root/workspace/src/darp_python'
if os.path.exists(darp_path):
    sys.path.insert(0, darp_path)
else:
    # Fallback for host testing
    darp_path_host = os.path.expanduser('~/Documents/ros-task/workspace/darp_python')
    if os.path.exists(darp_path_host):
        sys.path.insert(0, darp_path_host)

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

# Import DARP normally - no modifications to DARP code needed!
from multiRobotPathPlanner import MultiRobotPathPlanner

# Import coordinate converter for path publishing
from .coordinate_converter import CoordinateConverter


class SimpleDARPNode(Node):
    """
    Minimal ROS 2 node that runs DARP algorithm.

    Hardcoded configuration aligned with limo.launch.py:
    - 10x10 grid (no obstacles)
    - 1 robot spawned at Gazebo (0, 0)
    - Robot owns entire workspace

    Parameters:
        visualize_darp (bool): Optional. When true, enables DARP's pygame visualization
            window for debugging (default: false).
    """

    def __init__(self):
        super().__init__('simple_darp_planner')

        # Allow visualization toggle via ROS parameter (ros2 run ... --ros-args -p visualize_darp:=true)
        self.declare_parameter('visualize_darp', False)
        self.visualize_darp = bool(self.get_parameter('visualize_darp').value)

        # Hardcoded parameters derived from simulation launch
        self.grid_size = 10
        self.cell_size = 1.0  # meters per grid cell inside 10 m x 10 m arena
        self.origin_x = -5.0  # arena south-west corner
        self.origin_y = -5.0
        self.sim_spawn_xy = (0.0, 0.0)  # limobot spawn from limo.launch.py
        self.num_robots = 1

        # Map Gazebo spawn (0,0) to DARP flattened index (row-major, row 0 = top)
        spawn_col = int((self.sim_spawn_xy[0] - self.origin_x) / self.cell_size)
        spawn_ros_row = int((self.sim_spawn_xy[1] - self.origin_y) / self.cell_size)
        spawn_row = (self.grid_size - 1) - spawn_ros_row
        spawn_row = max(0, min(spawn_row, self.grid_size - 1))
        spawn_col = max(0, min(spawn_col, self.grid_size - 1))
        self.initial_indices = [spawn_row * self.grid_size + spawn_col]

        # Create coordinate converter for path publishing
        self.converter = CoordinateConverter(
            grid_rows=self.grid_size,
            grid_cols=self.grid_size,
            cell_size=self.cell_size,
            origin_x=self.origin_x,
            origin_y=self.origin_y
        )

        # Create path publisher
        self.path_publisher = self.create_publisher(Path, 'coverage_path', 10)

        # Store path for continuous republishing
        self.path_msg = None
        self.republish_timer = None

        self.get_logger().info(
            f'DARP Planner initialized: {self.grid_size}x{self.grid_size} grid, '
            f'{self.num_robots} robot @ cell (row={spawn_row}, col={spawn_col}, index={self.initial_indices[0]}); '
            f'visualization={"on" if self.visualize_darp else "off"}'
        )

        # Run DARP once after 2 second delay
        self.timer = self.create_timer(2.0, self.run_darp)
        self.planned = False

    def create_path_message(self, planner, robot_idx=0):
        """
        Convert DARP path to nav_msgs/Path.

        Args:
            planner: MultiRobotPathPlanner instance with completed planning
            robot_idx: Index of robot (default: 0 for single robot)

        Returns:
            nav_msgs/Path message with all waypoints
        """
        path = Path()
        path.header.frame_id = "map"

        # Get path segments for this robot
        path_segments = planner.best_case.paths[robot_idx]

        if not path_segments:
            self.get_logger().warn(f'No path segments for robot {robot_idx}')
            return path

        # Extract waypoints: start + deduplicated endpoints
        # DARP segments are connected (segment[i].end == segment[i+1].start),
        # so we deduplicate to avoid consecutive identical waypoints
        waypoints = []
        waypoints.append((path_segments[0][0], path_segments[0][1]))  # Start point
        for segment in path_segments:
            to_row, to_col = segment[2], segment[3]
            # Only add if different from last waypoint
            if waypoints[-1] != (to_row, to_col):
                waypoints.append((to_row, to_col))

        self.get_logger().info(
            f'Converting {len(path_segments)} segments to {len(waypoints)} waypoints'
        )

        # Convert waypoints to PoseStamped messages
        # With corrected subcell_to_meters(), each waypoint has unique position
        prev_yaw = 0.0

        for i, (subcell_row, subcell_col) in enumerate(waypoints):
            # Convert subcell coordinates to meters (includes Y-flip)
            x, y = self.converter.subcell_to_meters(subcell_row, subcell_col)

            # Calculate orientation toward next waypoint
            if i < len(waypoints) - 1:
                next_subcell_row, next_subcell_col = waypoints[i + 1]
                next_x, next_y = self.converter.subcell_to_meters(next_subcell_row, next_subcell_col)
                yaw = self.converter.calculate_orientation((x, y), (next_x, next_y))
            else:
                # Last waypoint: use orientation from previous segment
                yaw = prev_yaw

            prev_yaw = yaw

            # Create PoseStamped
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            # Set orientation quaternion
            qx, qy, qz, qw = self.converter.yaw_to_quaternion(yaw)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw

            path.poses.append(pose)

        self.get_logger().info(f'DEBUG: path.poses has {len(path.poses)} items before returning')
        return path

    def publish_path_with_timestamp(self):
        """Publish stored path with current timestamp."""
        if self.path_msg is None:
            return

        # Update timestamps to current time
        now = self.get_clock().now().to_msg()
        self.path_msg.header.stamp = now
        for pose in self.path_msg.poses:
            pose.header.stamp = now

        self.path_publisher.publish(self.path_msg)

    def run_darp(self):
        """
        Execute DARP algorithm and log results.
        Only runs once.
        """
        if self.planned:
            return  # Already executed

        self.planned = True
        self.timer.cancel()

        self.get_logger().info('Running DARP algorithm...')

        try:
            # Single robot starting position aligned with Gazebo spawn pose
            initial_positions = self.initial_indices

            # Single robot owns whole workspace
            portions = [1.0]

            self.get_logger().info(f'Initial positions: {initial_positions}')
            self.get_logger().info(f'Portions: {portions}')

            # Run DARP
            planner = MultiRobotPathPlanner(
                nx=self.grid_size,           # Grid rows
                ny=self.grid_size,           # Grid columns
                notEqualPortions=False,      # Use equal portions
                initial_positions=initial_positions,
                portions=portions,
                obs_pos=[],                  # No obstacles
                visualization=self.visualize_darp
            )

            self.get_logger().info(f'✓ DARP completed successfully')
            self.get_logger().info(f'  Execution time: {planner.execution_time:.2f}s')
            self.get_logger().info(f'  Turns per robot: {planner.best_case.turns}')
            self.get_logger().info(f'  Path lengths: {[len(p) for p in planner.best_case.paths]}')

            # Create and publish path
            self.path_msg = self.create_path_message(planner, robot_idx=0)
            self.publish_path_with_timestamp()

            self.get_logger().info(
                f'✓ Published coverage path: {len(self.path_msg.poses)} waypoints, '
                f'~{len(self.path_msg.poses) * 0.5:.1f}m total length'
            )

            # Set up continuous republishing for RViz (1 Hz)
            self.republish_timer = self.create_timer(1.0, self.publish_path_with_timestamp)

        except Exception as e:
            self.get_logger().error(f'DARP planning failed: {e}')
            import traceback
            traceback.print_exc()


def main(args=None):
    """ROS 2 entry point"""
    rclpy.init(args=args)
    node = SimpleDARPNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
