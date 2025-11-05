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

    Hardcoded configuration for MVP:
    - 10x10 grid (no obstacles)
    - 2 robots
    - Equal portions (50% each)
    """

    def __init__(self):
        super().__init__('simple_darp_planner')

        # Hardcoded parameters
        self.grid_size = 10
        self.num_robots = 2

        self.get_logger().info(f'DARP Planner initialized: {self.grid_size}x{self.grid_size} grid, {self.num_robots} robots')

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
            # Robot starting positions (grid cell indices)
            # Index 0 = top-left corner (0,0)
            # Index 9 = top-right corner (0,9)
            initial_positions = [0, self.grid_size - 1]

            # Equal territory division
            portions = [0.5, 0.5]

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
                visualization=False          # No pygame window
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
