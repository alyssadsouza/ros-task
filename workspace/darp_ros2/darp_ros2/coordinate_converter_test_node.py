#!/usr/bin/env python3
"""
Coordinate converter smoke tests

Runs basic round-trip checks on the CoordinateConverter against the
Gazebo arena configuration used by limo.launch.py (10 m square arena,
origin at (-5, -5) and 1 m cell resolution).
"""

from __future__ import annotations

import math
from typing import Iterable, Tuple

import rclpy
from rclpy.node import Node

from .coordinate_converter import CoordinateConverter


class CoordinateConverterTestNode(Node):
    """Minimal ROS 2 node that asserts core CoordinateConverter behavior."""

    def __init__(self) -> None:
        super().__init__('coordinate_converter_test')

        # Mirror limo.launch.py world assumptions
        self.grid_rows = 10
        self.grid_cols = 10
        self.cell_size = 1.0
        self.origin_x = -5.0
        self.origin_y = -5.0
        self.spawn_xy = (0.0, 0.0)

        self.converter = CoordinateConverter(
            grid_rows=self.grid_rows,
            grid_cols=self.grid_cols,
            cell_size=self.cell_size,
            origin_x=self.origin_x,
            origin_y=self.origin_y
        )

        # Trigger tests once after node spins up
        self.timer = self.create_timer(0.1, self._run_tests)
        self.get_logger().info('Coordinate converter test node ready; executing checks.')

    # Helper: consistent float comparison
    def _assert_close(
        self,
        observed: Iterable[float],
        expected: Iterable[float],
        tol: float = 1e-6,
        msg: str = ''
    ) -> None:
        observed_tuple = tuple(observed)
        expected_tuple = tuple(expected)
        if len(observed_tuple) != len(expected_tuple):
            raise AssertionError(f'{msg} length mismatch {len(observed_tuple)} vs {len(expected_tuple)}')

        deltas = [abs(o - e) for o, e in zip(observed_tuple, expected_tuple)]
        max_delta = max(deltas) if deltas else 0.0
        if max_delta > tol:
            raise AssertionError(
                f'{msg} expected {expected_tuple}, observed {observed_tuple}, '
                f'max |Δ|={max_delta:.3e}'
            )

    def _assert_equal(self, observed: Iterable[int], expected: Iterable[int], msg: str = '') -> None:
        if tuple(observed) != tuple(expected):
            raise AssertionError(f'{msg} expected {tuple(expected)}, observed {tuple(observed)}')

    def _run_tests(self) -> None:
        self.timer.cancel()
        try:
            self._check_cell_to_meters()
            self._check_meters_to_cell()
            self._check_subcell_to_meters()
            self._check_round_trip_consistency()
            self._check_bounds_clamping()
            self._check_orientation_helpers()
            self._check_grid_metadata()
        except AssertionError as exc:
            self.get_logger().error(f'Coordinate converter test failed: {exc}')
            rclpy.shutdown()
            raise

        self.get_logger().info('All coordinate converter checks passed.')
        rclpy.shutdown()

    def _check_cell_to_meters(self) -> None:
        """Verify canonical cell centers map to expected Gazebo coordinates."""
        cases = [
            ((0, 0), (-4.5, 4.5)),     # top-left
            ((0, 9), (4.5, 4.5)),      # top-right
            ((9, 0), (-4.5, -4.5)),    # bottom-left
            ((9, 9), (4.5, -4.5)),     # bottom-right
            ((4, 5), (0.5, 0.5)),      # Gazebo spawn cell
            ((5, 4), (-0.5, -0.5)),    # cell adjacent to spawn
            ((5, 5), (0.5, -0.5)),     # symmetric across origin
            ((0, 5), (0.5, 4.5)),      # top edge
            ((5, 0), (-4.5, -0.5)),    # left edge
        ]
        for (row, col), expected in cases:
            observed = self.converter.cell_to_meters(row, col)
            self._assert_close(observed, expected, msg=f'cell_to_meters({row}, {col})')

    def _check_meters_to_cell(self) -> None:
        """Ensure Gazebo spawn and corner points map back into DARP indices."""
        cases = [
            ((-4.5, 4.5), (0, 0)),
            ((4.5, 4.5), (0, 9)),
            ((-4.5, -4.5), (9, 0)),
            ((4.5, -4.5), (9, 9)),
            (self.spawn_xy, (4, 5)),
            ((-0.5, -0.5), (5, 4)),
            ((0.5, -0.5), (5, 5)),
        ]
        for (x, y), expected_cell in cases:
            observed = self.converter.meters_to_cell(x, y)
            self._assert_equal(observed, expected_cell, msg=f'meters_to_cell({x}, {y})')

    def _check_subcell_to_meters(self) -> None:
        """Confirm subcell coordinates collapse to matching cell centers."""
        cell_row, cell_col = 2, 3
        subcells = [
            (cell_row * 2, cell_col * 2),
            (cell_row * 2 + 1, cell_col * 2),
            (cell_row * 2, cell_col * 2 + 1),
            (cell_row * 2 + 1, cell_col * 2 + 1),
        ]
        expected = self.converter.cell_to_meters(cell_row, cell_col)
        for subcell in subcells:
            observed = self.converter.subcell_to_meters(*subcell)
            self._assert_close(observed, expected, msg=f'subcell_to_meters{subcell}')

    def _check_round_trip_consistency(self) -> None:
        """Ensure mapToWorld/worldToMap round-trip integrity for representative cells."""
        sample_cells = [
            (0, 0),
            (0, 9),
            (4, 5),
            (5, 5),
            (9, 0),
            (9, 9),
        ]
        for row, col in sample_cells:
            meters = self.converter.cell_to_meters(row, col)
            back_row, back_col = self.converter.meters_to_cell(*meters)
            self._assert_equal(
                (back_row, back_col),
                (row, col),
                msg=f'round-trip cell ({row}, {col})'
            )

    def _check_bounds_clamping(self) -> None:
        """Verify coordinates outside the grid clamp to valid indices."""
        outside_cases = [
            ((6.2, 5.1), (0, 9)),    # beyond +X/+Y limits
            ((-6.4, 5.1), (0, 0)),   # beyond -X/+Y
            ((6.2, -5.9), (9, 9)),   # beyond +X/-Y
            ((-6.4, -5.9), (9, 0)),  # beyond -X/-Y
        ]
        for (x, y), expected_cell in outside_cases:
            observed = self.converter.meters_to_cell(x, y)
            self._assert_equal(observed, expected_cell, msg=f'clamp meters_to_cell({x}, {y})')

    def _check_orientation_helpers(self) -> None:
        """Test yaw and quaternion helpers against REP-103 axis conventions."""
        orientation_cases = [
            ((0.0, 0.0), (1.0, 0.0), 0.0),                  # +X
            ((0.0, 0.0), (0.0, 1.0), math.pi / 2.0),        # +Y
            ((0.0, 0.0), (-1.0, 0.0), math.pi),             # -X
            ((0.0, 0.0), (0.0, -1.0), -math.pi / 2.0),      # -Y
            ((0.0, 0.0), (1.0, 1.0), math.pi / 4.0),        # diagonal
        ]

        for start, end, expected_yaw in orientation_cases:
            yaw = self.converter.calculate_orientation(start, end)
            diff = (yaw - expected_yaw + math.pi) % (2.0 * math.pi) - math.pi
            if abs(diff) > 1e-6:
                raise AssertionError(
                    f'yaw mismatch for {start}->{end}: expected {expected_yaw}, got {yaw}'
                )

            quat = self.converter.yaw_to_quaternion(yaw)
            norm = math.sqrt(sum(component * component for component in quat))
            if abs(norm - 1.0) > 1e-6:
                raise AssertionError(f'Quaternion not normalized for yaw {yaw}: norm={norm}')

    def _check_grid_metadata(self) -> None:
        """Validate helper outputs for grid bounds and spawn indexing."""
        bounds = self.converter.get_grid_bounds()
        expected_bounds = (self.origin_x, self.origin_y, 5.0, 5.0)
        self._assert_close(bounds[:2], expected_bounds[:2], msg='grid_bounds min')
        self._assert_close(bounds[2:], expected_bounds[2:], msg='grid_bounds max')

        spawn_cell = self.converter.meters_to_cell(*self.spawn_xy)
        self._assert_equal(spawn_cell, (4, 5), msg='spawn cell indices')
        expected_flat_index = spawn_cell[0] * self.grid_cols + spawn_cell[1]
        if expected_flat_index != 45:
            raise AssertionError(f'Spawn flattened index expected 45, got {expected_flat_index}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CoordinateConverterTestNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, AssertionError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
