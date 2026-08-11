"""Feasible bounds for the stop-detection window.

No ROS, no hardware: every input is a measured length in metres.

WHY THIS MODULE EXISTS
----------------------
The stop window and the frozen `proximity_alert` avoidance controller were
designed against different distance regimes and never reconciled. Measured on
2026-08-11:

    fused stop fires    lethal cell anywhere 0..1.0 m ahead of base_link
    avoidance fires     LiDAR front range <= 0.20 m  (= 0.273 m at base_link)

The gate zeroed all motion 3.7x further out than the avoidance controller
needs to start a maneuver, so a maneuver could never begin. Task 9 Part 2
recorded the consequence: one `False -> True` transition, then 186 consecutive
`True` readings over 30.9 s, with the gate deleting 46 live forward commands.

The stop was never latched -- `evaluate_stop_action` and `evaluate_gate` are
both stateless. It persisted because the obstacle stayed in the window, and
the robot could not leave: `GATE_ZERO` publishes a bare `Twist()`, so the
lateral and reverse components that would have cleared the window were zeroed
along with the forward one.

So the window must sit strictly BETWEEN two fixed points:

    bumper + stopping distance  <  window  <  avoidance trigger - overshoot

Too far out and it pre-empts avoidance (the bug above). Too far in and it
fires after contact is already unavoidable. This module computes both edges
so the choice is derived rather than asserted, and so a test can hold the
deployed config to it.

WHY HALF A CELL IS ADDED EVERYWHERE
-----------------------------------
`window_mask` tests a cell's CENTRE against the window edge, and the costmap
is a rolling window in `odom` -- the grid is not aligned to the robot and
slides under it as it drives. The effective edge therefore jitters by up to
half a cell in either direction. Every bound here absorbs that as
`cell_size_m / 2` rather than assuming the nominal edge is the real one.
"""


def _half_cell(cell_size_m):
    return cell_size_m / 2.0


def max_window_forward_m(safety_distance_m, lidar_x_offset_m,
                          stopping_distance_m, cell_size_m):
    """Largest window depth that still lets avoidance take over.

    `safety_distance_m` is a LiDAR range; the window is measured from
    `base_link`, so the LiDAR's forward offset converts between them. The
    robot coasts `stopping_distance_m` past the trigger before it halts, which
    brings the obstacle closer than `safety_distance_m` and tightens the bound.
    """
    obstacle_at_rest = (safety_distance_m + lidar_x_offset_m
                        - stopping_distance_m)
    return obstacle_at_rest - _half_cell(cell_size_m)


def min_window_forward_m(front_extent_m, stopping_distance_m, cell_size_m):
    """Smallest window depth that still prevents contact.

    `front_extent_m` is the frontmost physical point of the robot measured
    from `base_link` -- not a wheel axle and not a TF frame, because neither
    is what actually hits the obstacle.
    """
    return front_extent_m + stopping_distance_m + _half_cell(cell_size_m)


def feasible_window_forward_m(safety_distance_m, lidar_x_offset_m,
                               front_extent_m, stopping_distance_m,
                               cell_size_m):
    """The `(low, high)` band. `low > high` means NO valid window exists.

    That is a real possible outcome, not an error to be smoothed over: it
    means the robot's bumper reaches past its own avoidance trigger once
    stopping distance is accounted for, and no window value can satisfy both
    constraints. The caller must widen the corridor (finer costmap
    resolution, or a different avoidance trigger) rather than pick a number
    that violates one bound quietly.
    """
    low = min_window_forward_m(front_extent_m, stopping_distance_m,
                                cell_size_m)
    high = max_window_forward_m(safety_distance_m, lidar_x_offset_m,
                                 stopping_distance_m, cell_size_m)
    return low, high


def max_window_half_width_m(robot_half_width_m, clearance_m, cell_size_m):
    """Widest half-width that keeps the backstop on the robot's swept path.

    The avoidance controller's `front` is the MINIMUM range over a 180 degree
    arc (`front_arc_deg: 180.0`), so an obstacle well off the robot's axis
    still reads as "front". An obstacle at 60 degrees bearing and 0.20 m range
    sits at roughly x=0.17, y=0.17 in base_link.

    If the window is wider than the robot's own swept path, that obstacle is
    inside it -- and stays inside it during the very strafe that is clearing
    it, holding the gate closed for the whole maneuver. Confining the window
    to the swept path plus a clearance margin means the backstop only answers
    the question it exists to answer: "is the robot about to hit this?"
    """
    return robot_half_width_m + clearance_m + _half_cell(cell_size_m)
