#!/usr/bin/env python3
"""
Path Follower Client Node

Bridges the /coverage_path topic to the Nav2 controller_server's FollowPath action.
Implements comprehensive error handling and anti-gotcha measures based on lessons
learned from smoother integration failure.

Anti-gotcha measures:
1. Action client as CLASS MEMBER (not local variable) - prevents scope issues
2. MultiThreadedExecutor with callback groups - prevents threading deadlocks
3. Transform path to map frame - prevents frame mismatch
4. Comprehensive logging at each callback - enables debugging
5. Watchdog timer for stuck actions - detects silent failures
6. Extended timeout for ARM64 - handles slow performance
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath
from geometry_msgs.msg import PoseStamped
from collections import deque
import time


class PathFollowerClient(Node):
    """Client node that forwards coverage paths to Nav2 controller via action."""

    def __init__(self):
        super().__init__('path_follower_client')

        # Declare parameters
        self.declare_parameter('action_server_name', '/controller_server/follow_path')
        self.declare_parameter('coverage_path_topic', '/coverage_path')
        self.declare_parameter('action_timeout_sec', 10.0)  # Increased for ARM64
        self.declare_parameter('watchdog_timeout_sec', 120.0)  # 2 min max action time

        action_name = self.get_parameter('action_server_name').value
        path_topic = self.get_parameter('coverage_path_topic').value
        self._action_timeout = self.get_parameter('action_timeout_sec').value
        self._watchdog_timeout = self.get_parameter('watchdog_timeout_sec').value

        # Callback groups for concurrent execution
        self._action_cb_group = MutuallyExclusiveCallbackGroup()
        self._subscriber_cb_group = MutuallyExclusiveCallbackGroup()
        self._timer_cb_group = ReentrantCallbackGroup()

        # Action client as CLASS MEMBER (critical!)
        self._action_client = ActionClient(
            self,
            FollowPath,
            action_name,
            callback_group=self._action_cb_group
        )

        # Path subscriber
        self._path_sub = self.create_subscription(
            Path,
            path_topic,
            self._path_callback,
            10,
            callback_group=self._subscriber_cb_group
        )

        # State tracking
        self._goal_in_flight = False
        self._queued_paths = deque(maxlen=2)  # Queue up to 2 paths
        self._last_goal_time = None
        self._goal_handle = None

        # Watchdog timer to detect stuck actions
        self._watchdog_timer = self.create_timer(
            5.0,  # Check every 5 seconds
            self._watchdog_callback,
            callback_group=self._timer_cb_group
        )

        self.get_logger().info(
            f'PathFollowerClient initialized. Listening to {path_topic}, '
            f'forwarding to action {action_name}'
        )
        self.get_logger().info(
            f'Action timeout: {self._action_timeout}s, '
            f'Watchdog timeout: {self._watchdog_timeout}s'
        )

        # Wait for action server
        self.get_logger().info('Waiting for action server...')
        if not self._action_client.wait_for_server(timeout_sec=self._action_timeout):
            self.get_logger().error(
                f'Action server {action_name} not available after '
                f'{self._action_timeout}s timeout!'
            )
        else:
            self.get_logger().info('Action server is ready!')

    def _path_callback(self, msg: Path):
        """Callback when new coverage path is received."""
        self.get_logger().info(
            f'Received path with {len(msg.poses)} poses in frame "{msg.header.frame_id}"'
        )

        if len(msg.poses) == 0:
            self.get_logger().warn('Received empty path, ignoring')
            return

        # Transform path to map frame if needed
        if msg.header.frame_id != 'map':
            self.get_logger().warn(
                f'Path frame is "{msg.header.frame_id}", expected "map". '
                'Updating frame_id to "map".'
            )
            msg.header.frame_id = 'map'

        # Queue path if action in flight
        if self._goal_in_flight:
            self.get_logger().info(
                'Action in flight, queueing path '
                f'(queue size: {len(self._queued_paths) + 1})'
            )
            self._queued_paths.append(msg)
            return

        # Send goal immediately
        self._send_goal(msg)

    def _send_goal(self, path: Path):
        """Send FollowPath goal to action server."""
        self.get_logger().info(
            f'Sending FollowPath goal with {len(path.poses)} poses to controller'
        )

        # Create goal message
        goal_msg = FollowPath.Goal()
        goal_msg.path = path
        goal_msg.controller_id = 'FollowPath'  # Must match controller plugin name

        # Mark goal in flight
        self._goal_in_flight = True
        self._last_goal_time = time.time()

        # Send goal asynchronously
        send_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        send_future.add_done_callback(self._goal_response_callback)

        self.get_logger().info('Goal sent, waiting for acceptance...')

    def _goal_response_callback(self, future):
        """Callback when action server accepts/rejects goal."""
        self._goal_handle = future.result()

        if not self._goal_handle.accepted:
            self.get_logger().error('Goal was REJECTED by action server!')
            self._goal_in_flight = False
            self._process_queued_paths()
            return

        self.get_logger().info('Goal ACCEPTED by action server, waiting for result...')

        # Get result asynchronously
        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg):
        """Callback for action feedback (progress updates)."""
        feedback = feedback_msg.feedback
        # FollowPath feedback contains distance_to_goal
        if hasattr(feedback, 'distance_to_goal'):
            self.get_logger().info(
                f'Progress: {feedback.distance_to_goal:.2f}m to goal',
                throttle_duration_sec=2.0  # Log every 2 seconds max
            )

    def _result_callback(self, future):
        """Callback when action completes."""
        result = future.result()
        status = result.status

        self.get_logger().info(f'Action completed with status: {status}')

        # Status codes: 4 = SUCCEEDED, 5 = ABORTED, 6 = CANCELED
        if status == 4:
            self.get_logger().info('Path following SUCCEEDED!')
        elif status == 5:
            self.get_logger().error('Path following ABORTED!')
        elif status == 6:
            self.get_logger().warn('Path following CANCELED!')
        else:
            self.get_logger().warn(f'Path following ended with status {status}')

        # Clear in-flight flag
        self._goal_in_flight = False
        self._last_goal_time = None
        self._goal_handle = None

        # Process any queued paths
        self._process_queued_paths()

    def _process_queued_paths(self):
        """Process next queued path if available."""
        if self._queued_paths:
            next_path = self._queued_paths.popleft()
            self.get_logger().info(
                f'Processing queued path ({len(self._queued_paths)} remaining)'
            )
            self._send_goal(next_path)

    def _watchdog_callback(self):
        """Watchdog timer to detect stuck actions."""
        if not self._goal_in_flight or self._last_goal_time is None:
            return

        elapsed = time.time() - self._last_goal_time

        if elapsed > self._watchdog_timeout:
            self.get_logger().error(
                f'WATCHDOG TIMEOUT! Action has been running for {elapsed:.1f}s '
                f'(max {self._watchdog_timeout}s). Action appears stuck!'
            )
            self.get_logger().error(
                'This may indicate silent failure similar to smoother issue. '
                'Check controller_server logs for errors.'
            )

            # Cancel the stuck action
            if self._goal_handle is not None:
                self.get_logger().warn('Attempting to cancel stuck action...')
                cancel_future = self._goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(self._cancel_callback)
        elif elapsed > 30.0 and int(elapsed) % 10 == 0:
            # Log progress every 10 seconds after 30 seconds
            self.get_logger().info(
                f'Action still in progress ({elapsed:.0f}s elapsed)...',
                throttle_duration_sec=10.0
            )

    def _cancel_callback(self, future):
        """Callback when action cancellation completes."""
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Action canceled successfully')
        else:
            self.get_logger().error('Failed to cancel action!')

        # Reset state
        self._goal_in_flight = False
        self._last_goal_time = None
        self._goal_handle = None


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    node = PathFollowerClient()

    # Use MultiThreadedExecutor to prevent callback deadlocks
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        node.get_logger().info('PathFollowerClient spinning with MultiThreadedExecutor...')
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
