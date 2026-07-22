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
