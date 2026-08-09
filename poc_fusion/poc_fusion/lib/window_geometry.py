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


def window_corners(rx, ry, ryaw, forward_m, half_width_m):
    """The window's four corners as (x, y) in the same frame as rx/ry.

    Returned in draw order -- near-left, near-right, far-right, far-left --
    so a Marker can render them as a closed loop. This applies exactly the
    same body->world transform `window_mask` applies internally (in the
    opposite direction), which is why it lives beside it: if the marker
    drawn in RViz and the mask actually evaluated ever diverge, the picture
    stops being evidence about the real detection region.
    """
    cos_yaw = math.cos(ryaw)
    sin_yaw = math.sin(ryaw)

    body_corners = [
        (0.0, half_width_m),
        (0.0, -half_width_m),
        (forward_m, -half_width_m),
        (forward_m, half_width_m),
    ]
    return [
        (rx + cos_yaw * bx - sin_yaw * by,
         ry + sin_yaw * bx + cos_yaw * by)
        for bx, by in body_corners
    ]


def yaw_from_quaternion(x, y, z, w):
    """Rotation about +z, in radians, from a quaternion.

    Only the yaw component is needed: the costmap is a 2-D grid and the
    window is defined in the ground plane. Extracted here rather than
    pulling in tf_transformations so the module stays dependency-free and
    the sign convention is pinned by a test (a flipped sign would mirror
    the detection window about the robot's forward axis with no other
    visible symptom).
    """
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def area_cm2_to_cells(area_cm2, resolution):
    """Convert a physical area threshold (cm^2) to a cell count at `resolution`.

    Computed from the live resolution so the physical meaning of the
    threshold is preserved if resolution changes.
    """
    cell_area_cm2 = (resolution * 100.0) ** 2
    return area_cm2 / cell_area_cm2
