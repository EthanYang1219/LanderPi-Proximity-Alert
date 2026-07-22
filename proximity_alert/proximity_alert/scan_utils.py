"""Pure, ROS-free helpers for reasoning about LiDAR scans.

Deliberately imports nothing from rclpy/sensor_msgs so the forward-arc logic
can be unit-tested on a plain machine with a lightweight duck-typed scan.
"""
import math


def min_range_in_forward_arc(msg, scan_arc_deg):
    """Closest valid range in the forward arc, and the angle it was seen at.

    Returns ``(closest_range, closest_angle)``:
      * ``closest_range`` -- nearest valid reading (m) within
        +/- ``scan_arc_deg`` / 2 of straight-ahead, or ``None`` if there are
        no valid readings.
      * ``closest_angle`` -- that reading's beam angle wrapped into
        ``[-pi, pi]`` (``0.0`` when there is no valid reading), so callers can
        tell which side the obstacle is on.

    This LiDAR (LD19) reports angles as 0..2pi, so "forward" (0 rad) sits at
    both ends of the sweep; each beam is wrapped via ``atan2(sin, cos)`` before
    the arc test, otherwise only the 0..+half_arc (front-right) side would be
    measured and front-left obstacles would be missed.
    """
    half_arc = math.radians(scan_arc_deg) / 2.0

    closest = None
    closest_angle = 0.0
    angle = msg.angle_min
    for r in msg.ranges:
        rel = math.atan2(math.sin(angle), math.cos(angle))
        if -half_arc <= rel <= half_arc:
            # NaN/inf fail this comparison, so they're excluded here too.
            if msg.range_min <= r <= msg.range_max:
                if closest is None or r < closest:
                    closest = r
                    closest_angle = rel
        angle += msg.angle_increment

    return closest, closest_angle


def _in_window(rel, center, half_width):
    # rel, center in [-pi, pi]; returns True if rel within +/- half_width of
    # center, measured as a wrapped angular distance (handles the rear wrap).
    d = math.atan2(math.sin(rel - center), math.cos(rel - center))
    return -half_width <= d <= half_width


def reduce_to_sectors(msg, front_arc_deg, front_subsector_deg,
                      side_window_deg, rear_window_deg):
    half_front = math.radians(front_arc_deg) / 2.0
    half_sub = math.radians(front_subsector_deg) / 2.0
    half_side = math.radians(side_window_deg) / 2.0
    half_rear = math.radians(rear_window_deg) / 2.0
    left_c = math.pi / 2.0
    right_c = -math.pi / 2.0

    out = {k: float("inf") for k in
           ("front", "front_left", "front_center", "front_right",
            "left", "right", "rear")}

    for i, r in enumerate(msg.ranges):
        if msg.range_min <= r <= msg.range_max and math.isfinite(r):
            angle = msg.angle_min + i * msg.angle_increment
            rel = math.atan2(math.sin(angle), math.cos(angle))
            if -half_front <= rel <= half_front:
                out["front"] = min(out["front"], r)
                if _in_window(rel, half_sub * 2, half_sub):      # front-left bin
                    out["front_left"] = min(out["front_left"], r)
                elif _in_window(rel, 0.0, half_sub):             # front-center bin
                    out["front_center"] = min(out["front_center"], r)
                elif _in_window(rel, -half_sub * 2, half_sub):   # front-right bin
                    out["front_right"] = min(out["front_right"], r)
            if _in_window(rel, left_c, half_side):
                out["left"] = min(out["left"], r)
            if _in_window(rel, right_c, half_side):
                out["right"] = min(out["right"], r)
            if _in_window(rel, math.pi, half_rear):
                out["rear"] = min(out["rear"], r)
    return out
