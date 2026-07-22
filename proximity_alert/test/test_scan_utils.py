import math
from types import SimpleNamespace

from proximity_alert.scan_utils import min_range_in_forward_arc


def _make_scan(ranges, angle_min, angle_increment, range_min=0.05, range_max=5.0):
    # Duck-typed stand-in for sensor_msgs/LaserScan so this stays a pure,
    # ROS-free unit test (no rclpy/sensor_msgs needed to run it).
    return SimpleNamespace(
        angle_min=angle_min,
        angle_increment=angle_increment,
        range_min=range_min,
        range_max=range_max,
        ranges=ranges,
    )


def test_returns_closest_reading_within_arc():
    # angles: -90, 0, 90, 180 deg; 180 deg arc keeps only the first three.
    scan = _make_scan([1.0, 0.5, 2.0, 3.0], angle_min=-math.pi / 2, angle_increment=math.pi / 2)
    closest, _ = min_range_in_forward_arc(scan, scan_arc_deg=180.0)
    assert closest == 0.5


def test_wraps_ld19_0_to_2pi_angles():
    # The real LD19 reports angles 0..2pi, so a front-left beam sits near 2pi,
    # not at a negative angle. angles (45 deg steps): 0,45,90,135,180,225,270,
    # 315 -> wrapped: 0,45,90,135,180,-135,-90,-45. With a 180 deg arc, index 7
    # (315 deg == -45 deg, front-left) MUST be included -- it holds the closest
    # reading (0.5). A non-wrapping implementation filters on the raw 315 deg,
    # misses it, and wrongly returns 1.0 -- the bug this test guards against.
    # (-45 deg is chosen well inside the arc, not on the +/-90 deg boundary
    # where floating-point rounding of the wrap makes inclusion ambiguous.)
    ranges = [1.0, 2.0, 1.5, 3.0, 0.3, 3.0, 2.5, 0.5]
    scan = _make_scan(ranges, angle_min=0.0, angle_increment=math.pi / 4)
    closest, closest_angle = min_range_in_forward_arc(scan, scan_arc_deg=180.0)
    assert closest == 0.5
    # front-left -> wrapped angle is negative, so callers turn/drift right.
    assert closest_angle < 0.0


def test_index_at_180deg_is_excluded_from_forward_arc():
    # 180 deg (directly behind) must never count as a forward obstacle even
    # though it is the numerically closest reading.
    scan = _make_scan([1.0, 0.2], angle_min=0.0, angle_increment=math.pi)
    closest, _ = min_range_in_forward_arc(scan, scan_arc_deg=180.0)
    assert closest == 1.0


def test_ignores_readings_outside_range_min_max():
    scan = _make_scan([0.5], angle_min=0.0, angle_increment=0.1, range_min=0.6)
    closest, _ = min_range_in_forward_arc(scan, scan_arc_deg=180.0)
    assert closest is None


def test_ignores_nan_and_inf_readings():
    scan = _make_scan([float("nan"), float("inf"), 1.2], angle_min=0.0, angle_increment=0.1)
    closest, _ = min_range_in_forward_arc(scan, scan_arc_deg=180.0)
    assert closest == 1.2


def test_returns_none_and_zero_angle_when_no_valid_readings():
    scan = _make_scan([float("nan"), float("nan")], angle_min=0.0, angle_increment=0.1)
    closest, closest_angle = min_range_in_forward_arc(scan, scan_arc_deg=180.0)
    assert closest is None
    assert closest_angle == 0.0
