"""Pure geometry for the body-aligned obstacle detection window.

No ROS, no hardware: costmap grids and robot pose are passed in as plain
numbers/arrays, in the `odom` frame (REP-103: +x forward, +y left, +z up).
"""

import math

import numpy as np


def window_mask(width, height, resolution, origin_x, origin_y,
                 rx, ry, ryaw, forward_m, half_width_m):
    """Boolean (height, width) mask of cells inside the body-aligned window.

    The window is the rectangle directly ahead of the robot: 0..forward_m
    along the robot's +x (forward) axis, and +/-half_width_m across its
    y axis. origin_x/origin_y are used directly -- the robot is never
    assumed to sit at the grid centre.
    """
    cols = np.arange(width)
    rows = np.arange(height)
    col_grid, row_grid = np.meshgrid(cols, rows)  # both (height, width)

    x = origin_x + (col_grid + 0.5) * resolution
    y = origin_y + (row_grid + 0.5) * resolution

    dx = x - rx
    dy = y - ry

    cos_yaw = math.cos(ryaw)
    sin_yaw = math.sin(ryaw)

    bx = cos_yaw * dx + sin_yaw * dy
    by = -sin_yaw * dx + cos_yaw * dy

    return (bx >= 0) & (bx <= forward_m) & (np.abs(by) <= half_width_m)


def area_cm2_to_cells(area_cm2, resolution):
    """Convert a physical area threshold (cm^2) to a cell count at `resolution`.

    Computed from the live resolution so the physical meaning of the
    threshold is preserved if resolution changes.
    """
    cell_area_cm2 = (resolution * 100.0) ** 2
    return area_cm2 / cell_area_cm2
