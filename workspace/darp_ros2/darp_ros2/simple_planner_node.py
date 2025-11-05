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

# Import DARP normally - no modifications to DARP code needed!
from multiRobotPathPlanner import MultiRobotPathPlanner


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

        self.get_logger().info(
            f'DARP Planner initialized: {self.grid_size}x{self.grid_size} grid, '
            f'{self.num_robots} robot @ cell (row={spawn_row}, col={spawn_col}, index={self.initial_indices[0]}); '
            f'visualization={"on" if self.visualize_darp else "off"}'
        )

        # Run DARP once after 2 second delay
        self.timer = self.create_timer(2.0, self.run_darp)
        self.planned = False

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
