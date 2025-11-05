#!/usr/bin/env python3
"""
Coordinate Converter for DARP ROS 2 Integration

Handles conversion between:
- DARP grid coordinates (integer indices, pygame/matrix convention)
- DARP subcell coordinates (2x resolution for paths)
- ROS map coordinates (meters, ROS REP-103 convention)

CRITICAL COORDINATE SYSTEM NOTES:

1. DARP Grid Convention (pygame/matrix):
   - Origin: TOP-LEFT corner
   - Row 0: TOP of environment
   - Increasing row → moving DOWN
   - Used in: DARP initial_positions, paths, territory assignments

2. ROS Map Convention (REP-103, OccupancyGrid):
   - Origin: BOTTOM-LEFT corner
   - Y=0: BOTTOM of map
   - Increasing Y → moving UP
   - Used in: nav_msgs/Path, geometry_msgs/Pose, map frame

3. Y-AXIS FLIP REQUIRED:
   - ros_y = origin_y + ((grid_rows - 1 - darp_row) + 0.5) * cell_size

4. DARP 2x Resolution:
   - Grid cell (0, 1) → Subcell coordinates (0, 2) and (0, 3)
   - Divide subcell by 2 to get grid cell: grid_row = subcell_row // 2

Verified against:
- ROS costmap_2d source code (mapToWorld implementation)
- DARP Visualization.py (pygame rendering convention)
- REP-103 coordinate frame standard
"""

import math
from typing import Tuple


class CoordinateConverter:
    """
    Convert between DARP grid coordinates and ROS map frame coordinates.

    Coordinate Systems:
    1. DARP Grid: Integer indices (row, col) where row 0 = TOP
    2. DARP Subcells: 2x resolution indices for paths
    3. ROS Map: Floating-point meters (x, y) where Y increases upward

    Example Grid Layout (3x3 grid, cell_size=0.5m):

    DARP Grid (row, col):           ROS Map Coordinates (x, y meters):
    (0,0) (0,1) (0,2)  ← row 0 TOP    (0.25, 1.25) (0.75, 1.25) (1.25, 1.25) ← Y=1.25m
    (1,0) (1,1) (1,2)  ← row 1        (0.25, 0.75) (0.75, 0.75) (1.25, 0.75) ← Y=0.75m
    (2,0) (2,1) (2,2)  ← row 2 BOTTOM (0.25, 0.25) (0.75, 0.25) (1.25, 0.25) ← Y=0.25m
                                       X=0.25m      X=0.75m      X=1.25m
                                       ↑ origin

    Note the Y-axis flip: DARP row 0 maps to highest Y coordinate in ROS.
    """

    def __init__(
        self,
        grid_rows: int,
        grid_cols: int,
        cell_size: float,
        origin_x: float = 0.0,
        origin_y: float = 0.0
    ):
        """
        Initialize coordinate converter.

        Args:
            grid_rows: Number of rows in DARP grid
            grid_cols: Number of columns in DARP grid
            cell_size: Size of each grid cell in meters
            origin_x: X coordinate of grid origin (bottom-left) in map frame
            origin_y: Y coordinate of grid origin (bottom-left) in map frame

        Example:
            converter = CoordinateConverter(10, 10, 0.5)
            # Creates 10x10 grid with 0.5m cells
            # Grid spans from (0,0) to (5.0, 5.0) in map frame
        """
        self.grid_rows = grid_rows
        self.grid_cols = grid_cols
        self.cell_size = cell_size
        self.origin_x = origin_x
        self.origin_y = origin_y

    def cell_to_meters(self, row: int, col: int) -> Tuple[float, float]:
        """
        Convert DARP grid cell indices to ROS map coordinates (cell center).

        CRITICAL: Applies Y-axis flip from DARP (top=0) to ROS (bottom=0) convention.

        Implementation verified against ROS costmap_2d::mapToWorld():
            wx = origin_x_ + (mx + 0.5) * resolution_;
            wy = origin_y_ + (my + 0.5) * resolution_;

        With Y-axis flip for DARP:
            ros_row = (grid_rows - 1) - darp_row

        Args:
            row: DARP grid row index (0=top to grid_rows-1=bottom)
            col: DARP grid column index (0 to grid_cols-1)

        Returns:
            (x, y): Position in meters in ROS map frame (cell center)

        Example:
            >>> converter = CoordinateConverter(10, 10, 0.5)
            >>> # DARP row 0 (top) should map to Y=4.75m (near top of 5m grid)
            >>> x, y = converter.cell_to_meters(0, 0)
            >>> print(f"x={x}, y={y}")
            x=0.25, y=4.75

            >>> # DARP row 9 (bottom) should map to Y=0.25m (near bottom)
            >>> x, y = converter.cell_to_meters(9, 0)
            >>> print(f"x={x}, y={y}")
            x=0.25, y=0.25
        """
        # Y-axis flip: Convert DARP row (0=top) to ROS row (0=bottom)
        ros_row = (self.grid_rows - 1) - row

        # Cell center: origin + (index + 0.5) * cell_size
        # X: Column maps directly (no flip needed)
        # Y: Use flipped row for ROS convention
        x = self.origin_x + (col + 0.5) * self.cell_size
        y = self.origin_y + (ros_row + 0.5) * self.cell_size

        return x, y

    def subcell_to_meters(self, subcell_row: int, subcell_col: int) -> Tuple[float, float]:
        """
        Convert DARP subcell coordinates to ROS map coordinates.

        CRITICAL: DARP uses 2x resolution for path coordinates!
        - A 10x10 grid has 20x20 subcells for paths
        - Subcell (0, 2) corresponds to grid cell (0, 1)
        - Must divide by 2 to get actual grid cell

        Args:
            subcell_row: DARP subcell row index (0 to 2*grid_rows-1)
            subcell_col: DARP subcell column index (0 to 2*grid_cols-1)

        Returns:
            (x, y): Position in meters in ROS map frame

        Example:
            >>> converter = CoordinateConverter(10, 10, 0.5)
            >>> # Subcell (0, 2) should map to cell (0, 1)
            >>> x, y = converter.subcell_to_meters(0, 2)
            >>> expected_x, expected_y = converter.cell_to_meters(0, 1)
            >>> assert abs(x - expected_x) < 1e-9
            >>> assert abs(y - expected_y) < 1e-9
        """
        # Convert from 2x resolution to grid cell
        row = subcell_row // 2
        col = subcell_col // 2

        # Use normal cell-to-meters conversion (includes Y-flip)
        return self.cell_to_meters(row, col)

    def meters_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        """
        Convert ROS map coordinates to DARP grid cell indices.

        CRITICAL: Applies Y-axis flip from ROS (bottom=0) to DARP (top=0) convention.

        Args:
            x: X coordinate in meters in ROS map frame
            y: Y coordinate in meters in ROS map frame

        Returns:
            (row, col): DARP grid cell indices

        Note: Clamps to grid boundaries if coordinates are outside

        Example:
            >>> converter = CoordinateConverter(10, 10, 0.5)
            >>> # Bottom-left in ROS (Y=0.25) should be DARP row 9 (bottom)
            >>> row, col = converter.meters_to_cell(0.25, 0.25)
            >>> print(f"row={row}, col={col}")
            row=9, col=0

            >>> # Top-left in ROS (Y=4.75) should be DARP row 0 (top)
            >>> row, col = converter.meters_to_cell(0.25, 4.75)
            >>> print(f"row={row}, col={col}")
            row=0, col=0
        """
        # Convert to ROS cell indices (floor division)
        col = int((x - self.origin_x) / self.cell_size)
        ros_row = int((y - self.origin_y) / self.cell_size)

        # Clamp ROS row to grid boundaries
        ros_row = max(0, min(ros_row, self.grid_rows - 1))
        col = max(0, min(col, self.grid_cols - 1))

        # Y-axis flip: Convert ROS row to DARP row
        darp_row = (self.grid_rows - 1) - ros_row

        return darp_row, col

    def calculate_orientation(
        self,
        from_point: Tuple[float, float],
        to_point: Tuple[float, float]
    ) -> float:
        """
        Calculate yaw angle (rotation around Z axis) from one point to another.

        Used to set orientation in PoseStamped messages for path waypoints.

        Follows ROS REP-103 convention:
        - X axis points forward/east
        - Y axis points left/north
        - Yaw is rotation around Z (up)

        Args:
            from_point: (x, y) starting position in meters
            to_point: (x, y) target position in meters

        Returns:
            yaw: Angle in radians (-π to π)
                 0 = pointing along +X axis (east)
                 π/2 = pointing along +Y axis (north)
                 π or -π = pointing along -X axis (west)
                 -π/2 = pointing along -Y axis (south)

        Example:
            >>> converter = CoordinateConverter(10, 10, 0.5)
            >>> # Moving from (0, 0) to (1, 0) is along +X axis
            >>> yaw = converter.calculate_orientation((0.0, 0.0), (1.0, 0.0))
            >>> print(f"yaw={yaw:.2f} rad ({math.degrees(yaw):.1f} deg)")
            yaw=0.00 rad (0.0 deg)

            >>> # Moving from (0, 0) to (0, 1) is along +Y axis
            >>> yaw = converter.calculate_orientation((0.0, 0.0), (0.0, 1.0))
            >>> print(f"yaw={yaw:.2f} rad ({math.degrees(yaw):.1f} deg)")
            yaw=1.57 rad (90.0 deg)
        """
        dx = to_point[0] - from_point[0]
        dy = to_point[1] - from_point[1]
        return math.atan2(dy, dx)

    def yaw_to_quaternion(self, yaw: float) -> Tuple[float, float, float, float]:
        """
        Convert yaw angle to normalized quaternion for geometry_msgs/Pose.

        For 2D navigation, we only rotate around Z axis, so x=0, y=0.

        CRITICAL: Returns NORMALIZED quaternion (w²+x²+y²+z² = 1) as required by ROS.
        Research showed this is a common source of errors if not normalized.

        Args:
            yaw: Rotation angle in radians around Z axis

        Returns:
            (x, y, z, w): Normalized quaternion components

        Example:
            >>> converter = CoordinateConverter(10, 10, 0.5)
            >>> x, y, z, w = converter.yaw_to_quaternion(0.0)  # No rotation
            >>> print(f"quat=({x}, {y}, {z}, {w})")
            quat=(0.0, 0.0, 0.0, 1.0)
            >>> # Verify normalization
            >>> norm = math.sqrt(x*x + y*y + z*z + w*w)
            >>> assert abs(norm - 1.0) < 1e-9

            >>> x, y, z, w = converter.yaw_to_quaternion(math.pi/2)  # 90 degrees
            >>> print(f"quat=({x:.6f}, {y:.6f}, {z:.6f}, {w:.6f})")
            quat=(0.000000, 0.000000, 0.707107, 0.707107)
            >>> # Verify normalization
            >>> norm = math.sqrt(x*x + y*y + z*z + w*w)
            >>> assert abs(norm - 1.0) < 1e-9
        """
        # Quaternion for rotation around Z axis only
        # q = [0, 0, sin(yaw/2), cos(yaw/2)]
        # This is already normalized for valid yaw input
        half_yaw = yaw / 2.0
        return (
            0.0,                    # x
            0.0,                    # y
            math.sin(half_yaw),     # z
            math.cos(half_yaw)      # w
        )

    def get_grid_bounds(self) -> Tuple[float, float, float, float]:
        """
        Get the ROS map-frame bounding box of the grid.

        Returns:
            (min_x, min_y, max_x, max_y): Bounds in meters in ROS map frame

        Example:
            >>> converter = CoordinateConverter(10, 10, 0.5)
            >>> bounds = converter.get_grid_bounds()
            >>> print(f"bounds={bounds}")
            bounds=(0.0, 0.0, 5.0, 5.0)
        """
        min_x = self.origin_x
        min_y = self.origin_y
        max_x = self.origin_x + self.grid_cols * self.cell_size
        max_y = self.origin_y + self.grid_rows * self.cell_size
        return min_x, min_y, max_x, max_y
