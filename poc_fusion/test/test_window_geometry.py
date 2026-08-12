import math

import numpy as np

from poc_fusion.lib.window_geometry import window_mask, area_cm2_to_cells


# Shared grid: origin is deliberately NOT at (0, 0) and the robot is
# deliberately NOT at the grid centre, per the brief's ban on assuming
# the robot sits at grid centre.
WIDTH = 20
HEIGHT = 20
RES = 0.1
ORIGIN_X = -1.0
ORIGIN_Y = -1.0


def _cell_center(row, col):
    x = ORIGIN_X + (col + 0.5) * RES
    y = ORIGIN_Y + (row + 0.5) * RES
    return x, y


def test_cell_directly_ahead_is_in_mask():
    # Robot at (0, 0), facing +x (ryaw=0). Cell at col=14 -> x=0.45, ahead.
    row, col = 10, 14
    mask = window_mask(WIDTH, HEIGHT, RES, ORIGIN_X, ORIGIN_Y,
                        rx=0.0, ry=0.0, ryaw=0.0,
                        forward_m=1.0, half_width_m=0.3)
    assert mask[row, col] == True  # noqa: E712


def test_cell_behind_is_not_in_mask():
    # Robot at (0, 0), facing +x. Cell at col=4 -> x=-0.55, behind the robot.
    row, col = 10, 4
    mask = window_mask(WIDTH, HEIGHT, RES, ORIGIN_X, ORIGIN_Y,
                        rx=0.0, ry=0.0, ryaw=0.0,
                        forward_m=1.0, half_width_m=0.3)
    assert mask[row, col] == False  # noqa: E712


def test_cell_beyond_forward_m_is_not_in_mask():
    # Robot at (0, 0), facing +x, forward_m=0.5. Cell at col=17 -> x=0.75,
    # which is ahead but past the forward_m cutoff.
    row, col = 10, 17
    x, _ = _cell_center(row, col)
    assert x > 0.5  # sanity: cell really is beyond forward_m
    mask = window_mask(WIDTH, HEIGHT, RES, ORIGIN_X, ORIGIN_Y,
                        rx=0.0, ry=0.0, ryaw=0.0,
                        forward_m=0.5, half_width_m=0.3)
    assert mask[row, col] == False  # noqa: E712


def test_cell_outside_half_width_m_is_not_in_mask():
    # Robot at (0, 0), facing +x, half_width_m=0.3. Cell at row=15 -> y=0.55,
    # ahead in x but too far to the side.
    row, col = 15, 14
    _, y = _cell_center(row, col)
    assert y > 0.3  # sanity: cell really is outside half_width_m
    mask = window_mask(WIDTH, HEIGHT, RES, ORIGIN_X, ORIGIN_Y,
                        rx=0.0, ry=0.0, ryaw=0.0,
                        forward_m=1.0, half_width_m=0.3)
    assert mask[row, col] == False  # noqa: E712


def test_noncentred_origin_is_honoured():
    # Same grid, but the robot pose is placed near the corner of the grid
    # (far from any "grid centre" assumption) and origin_x/origin_y are
    # both non-zero. A cell 0.3m ahead of the robot must be in the mask,
    # and it must line up with origin_x/origin_y directly rather than any
    # assumed grid-centre offset.
    origin_x, origin_y = 5.0, -3.0
    rx, ry = 5.4, -2.7  # near the (origin_x, origin_y) corner of the grid
    # Cell ahead: x = rx + 0.3 = 5.7, y = ry = -2.7
    col = round((5.7 - origin_x) / RES - 0.5)
    row = round((-2.7 - origin_y) / RES - 0.5)
    mask = window_mask(WIDTH, HEIGHT, RES, origin_x, origin_y,
                        rx=rx, ry=ry, ryaw=0.0,
                        forward_m=1.0, half_width_m=0.3)
    assert mask[row, col] == True  # noqa: E712

    # A cell at the true grid centre (which is NOT near the robot for this
    # origin) must NOT be in the mask -- pins that the implementation uses
    # origin_x/origin_y, not an assumed grid-centre robot position.
    centre_row, centre_col = HEIGHT // 2, WIDTH // 2
    assert mask[centre_row, centre_col] == False  # noqa: E712


def test_rotation_sweep_cell_stays_ahead_when_re_derived_in_odom():
    # Regression test for the bug this design replaced: a fixed detection
    # window that silently swings to the robot's rear as it turns.
    #
    # For each yaw, place a synthetic lethal cell 1m directly ahead of the
    # robot IN ODOM (re-deriving its (x, y) from the robot's current pose
    # and yaw each time), and assert it is always inside the mask.
    rx, ry = 0.0, 0.0
    forward_m = 1.2
    half_width_m = 0.3
    origin_x, origin_y = -3.0, -3.0
    width = height = 60
    res = 0.1

    for ryaw_deg in (0, 45, -45, 90, -90, 180):
        ryaw = math.radians(ryaw_deg)
        # Cell 1m directly ahead of the robot in the robot's current heading.
        cell_x = rx + math.cos(ryaw) * 1.0
        cell_y = ry + math.sin(ryaw) * 1.0
        col = round((cell_x - origin_x) / res - 0.5)
        row = round((cell_y - origin_y) / res - 0.5)

        mask = window_mask(width, height, res, origin_x, origin_y,
                            rx=rx, ry=ry, ryaw=ryaw,
                            forward_m=forward_m, half_width_m=half_width_m)
        assert mask[row, col] == True, f"failed at ryaw_deg={ryaw_deg}"  # noqa: E712


def test_rotation_away_from_fixed_odom_cell_leaves_mask():
    # Hold a lethal cell fixed in odom (1m ahead of the robot's initial
    # heading) and rotate the robot's yaw away from it: the cell must leave
    # the mask once the robot is no longer facing it.
    rx, ry = 0.0, 0.0
    origin_x, origin_y = -3.0, -3.0
    width = height = 60
    res = 0.1
    forward_m = 1.2
    half_width_m = 0.3

    # Cell fixed in odom, 1m ahead of yaw=0.
    cell_x, cell_y = 1.0, 0.0
    col = round((cell_x - origin_x) / res - 0.5)
    row = round((cell_y - origin_y) / res - 0.5)

    mask_facing = window_mask(width, height, res, origin_x, origin_y,
                               rx=rx, ry=ry, ryaw=0.0,
                               forward_m=forward_m, half_width_m=half_width_m)
    assert mask_facing[row, col] == True  # noqa: E712

    # Robot turns 180 degrees away; the same odom-fixed cell is now behind it.
    mask_away = window_mask(width, height, res, origin_x, origin_y,
                             rx=rx, ry=ry, ryaw=math.pi,
                             forward_m=forward_m, half_width_m=half_width_m)
    assert mask_away[row, col] == False  # noqa: E712


def test_area_cm2_to_cells_differs_by_resolution():
    # Same physical area (100 cm^2) must yield different cell counts at
    # different resolutions, since the threshold is defined in physical
    # units and must be re-derived from the live resolution.
    cells_fine = area_cm2_to_cells(100.0, 0.05)  # cell area = 25 cm^2 -> 4 cells
    cells_coarse = area_cm2_to_cells(100.0, 0.10)  # cell area = 100 cm^2 -> 1 cell
    assert cells_fine != cells_coarse
    assert cells_fine == 4
    assert cells_coarse == 1


# --- Task 7 additions: marker corners and yaw extraction -------------------
#
# Step 6 publishes the detection window as a visualization_msgs/Marker whose
# four corners are expressed in the costmap frame. Placing those corners is
# the same body->world transform window_mask() applies internally, so it
# lives here rather than in the node: if the marker and the mask ever
# disagree, the RViz rectangle stops being evidence about the real mask.

import math as _math

import pytest as _pytest

from poc_fusion.lib.window_geometry import window_corners, yaw_from_quaternion


def test_corners_at_origin_with_zero_yaw():
    corners = window_corners(rx=0.0, ry=0.0, ryaw=0.0,
                             forward_m=1.0, half_width_m=0.3)
    assert corners == [
        (0.0, 0.3), (0.0, -0.3), (1.0, -0.3), (1.0, 0.3),
    ]


def test_corners_are_translated_by_robot_position():
    corners = window_corners(rx=2.0, ry=-1.0, ryaw=0.0,
                             forward_m=1.0, half_width_m=0.3)
    assert corners[0] == _pytest.approx((2.0, -0.7))
    assert corners[2] == _pytest.approx((3.0, -1.3))


def test_corners_rotate_with_robot_yaw():
    # At +90 degrees the robot's +x (forward) points along world +y, so the
    # far edge of the window must sit at y = +forward_m, not x.
    corners = window_corners(rx=0.0, ry=0.0, ryaw=_math.pi / 2,
                             forward_m=1.0, half_width_m=0.3)
    far_left, far_right = corners[3], corners[2]
    assert far_left == _pytest.approx((-0.3, 1.0))
    assert far_right == _pytest.approx((0.3, 1.0))


def test_corners_agree_with_window_mask_under_rotation():
    # The property that actually matters: every corner midpoint the marker
    # draws must fall inside the mask the detector evaluates. Catches a
    # marker that silently uses a different sign convention from window_mask.
    from poc_fusion.lib.window_geometry import window_mask
    rx, ry, ryaw = 1.5, 1.5, 0.7
    forward_m, half_width_m = 1.0, 0.3
    resolution, width, height = 0.05, 60, 60
    origin_x, origin_y = 0.0, 0.0
    mask = window_mask(width, height, resolution, origin_x, origin_y,
                       rx, ry, ryaw, forward_m, half_width_m)
    corners = window_corners(rx, ry, ryaw, forward_m, half_width_m)
    centre_x = sum(c[0] for c in corners) / 4.0
    centre_y = sum(c[1] for c in corners) / 4.0
    col = int((centre_x - origin_x) / resolution)
    row = int((centre_y - origin_y) / resolution)
    assert mask[row, col]


def test_yaw_from_identity_quaternion_is_zero():
    assert yaw_from_quaternion(0.0, 0.0, 0.0, 1.0) == _pytest.approx(0.0)


def test_yaw_from_quarter_turn_quaternion():
    # 90 degrees about +z: (0, 0, sin(45deg), cos(45deg)).
    half = _math.sqrt(0.5)
    assert yaw_from_quaternion(0.0, 0.0, half, half) == _pytest.approx(_math.pi / 2)


def test_yaw_is_negative_for_a_clockwise_rotation():
    # Catches a sign flip, which would mirror the whole detection window
    # about the robot's forward axis without any other symptom.
    half = _math.sqrt(0.5)
    assert yaw_from_quaternion(0.0, 0.0, -half, half) == _pytest.approx(-_math.pi / 2)
