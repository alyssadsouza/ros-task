#!/usr/bin/env python3
"""Bridge /coverage_path into Nav2's smoother action and re-publish the result."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from collections import deque

from nav_msgs.msg import Path
from nav2_msgs.action import SmoothPath


class PathSmootherClient(Node):
    """Listens for raw coverage paths, calls the smoother action, republishes results."""

    def __init__(self) -> None:
        super().__init__('path_smoother_client')

        self.declare_parameter('raw_path_topic', '/coverage_path')
        self.declare_parameter('smoothed_path_topic', '/coverage_path_smooth')
        self.declare_parameter('action_name', '/smoother_server/smooth_path')
        self.declare_parameter('wait_for_server_timeout', 5.0)

        raw_topic = self.get_parameter('raw_path_topic').get_parameter_value().string_value
        smoothed_topic = self.get_parameter('smoothed_path_topic').get_parameter_value().string_value
        action_name = self.get_parameter('action_name').get_parameter_value().string_value
        wait_timeout = self.get_parameter('wait_for_server_timeout').get_parameter_value().double_value

        self._client = ActionClient(self, SmoothPath, action_name)
        self.get_logger().info('Waiting for smoother action server...')
        self._client.wait_for_server(timeout_sec=wait_timeout)

        self._smoothed_pub = self.create_publisher(Path, smoothed_topic, 10)
        self._raw_path_sub = self.create_subscription(Path, raw_topic, self._path_callback, 10)

        self._goal_in_flight = False
        self._queued_paths: deque[Path] = deque(maxlen=2)

        self.get_logger().info(
            f'Path smoother client ready: {raw_topic} -> {action_name} -> {smoothed_topic}'
        )

    def _path_callback(self, msg: Path) -> None:
        """Handle incoming raw path."""
        if not msg.poses:
            self.get_logger().warn('Received empty coverage path; ignoring')
            return

        if self._goal_in_flight:
            self._queued_paths.append(msg)
            self.get_logger().debug(
                f'Smoother busy; queued path (queue size={len(self._queued_paths)})')
            return

        self._send_goal(msg)

    def _send_goal(self, path: Path) -> None:
        goal = SmoothPath.Goal()
        goal.path = path

        send_future = self._client.send_goal_async(goal)
        self._goal_in_flight = True
        send_future.add_done_callback(self._goal_response_cb)
        self.get_logger().info(
            f'Submitted path with {len(path.poses)} poses to smoother')

    def _goal_response_cb(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().error(f'Smoother goal failed to send: {exc}')
            self._goal_in_flight = False
            self._maybe_send_queued()
            return

        if not goal_handle.accepted:
            self.get_logger().error('Smoother rejected goal')
            self._goal_in_flight = False
            self._maybe_send_queued()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future) -> None:
        try:
            result = future.result().result
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().error(f'Smoother goal failed: {exc}')
            self._goal_in_flight = False
            self._maybe_send_queued()
            return

        smoothed_path = result.smoothed_path if hasattr(result, 'smoothed_path') else result.path
        self._smoothed_pub.publish(smoothed_path)
        self.get_logger().info(
            f'Published smoothed path with {len(smoothed_path.poses)} poses')
        self._goal_in_flight = False
        self._maybe_send_queued()

    def _maybe_send_queued(self) -> None:
        if self._queued_paths:
            next_path = self._queued_paths.pop()  # keep the newest path
            self._send_goal(next_path)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathSmootherClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
