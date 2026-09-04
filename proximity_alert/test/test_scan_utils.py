"""Tests for `valid_beams`, the single place that decides what counts as a
usable LiDAR reading and how its angle is wrapped.

These previously targeted `min_range_in_forward_arc`, which was superseded by
`reduce_to_sectors`'s "front" value and has been removed. The behaviour worth
protecting is the same, and it now sits one level lower: every sector
reduction, obstacle measurement and gap search in this package is built on
`valid_beams`, so a regression here is a regression everywhere.
"""
import math
from types import SimpleNamespace

from proximity_alert.scan_utils import valid_beams


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


def test_wraps_ld19_0_to_2pi_angles_into_plus_minus_pi():
    # The real LD19 reports angles 0..2pi, so a front-left beam sits near 2pi
    # rather than at a negative angle. angles (45 deg steps): 0,45,90,135,180,
    # 225,270,315 -> wrapped: 0,45,90,135,180,-135,-90,-45.
    #
    # This is the bug the whole convention exists to prevent: without the
    # wrap, a forward-arc test filters on the raw 315 deg, decides it is
    # outside +/-90 deg, and the front-left obstacle is never seen at all.
    ranges = [1.0, 2.0, 1.5, 3.0, 0.3, 3.0, 2.5, 0.5]
    scan = _make_scan(ranges, angle_min=0.0, angle_increment=math.pi / 4)

    bearings = [b for b, _ in valid_beams(scan)]

    assert all(-math.pi <= b <= math.pi for b in bearings)
    # index 7 (raw 315 deg) must land at -45 deg -- front-LEFT, inside a
    # 180 deg forward arc, not behind the robot.
    assert math.isclose(bearings[7], -math.pi / 4, abs_tol=1e-9)
    # and it must still carry its own range.
    assert valid_beams(scan)[7][1] == 0.5


def test_pairs_each_bearing_with_its_own_range_in_scan_order():
    scan = _make_scan([1.0, 2.0, 3.0], angle_min=0.0, angle_increment=math.pi / 4)
    beams = valid_beams(scan)
    assert [r for _, r in beams] == [1.0, 2.0, 3.0]
    assert math.isclose(beams[1][0], math.pi / 4, abs_tol=1e-9)


def test_angles_are_index_derived_so_error_does_not_accumulate():
    # Derived as angle_min + i * angle_increment rather than accumulated, so a
    # full-length scan's last beam is exact rather than drifting.
    n = 450
    inc = 2 * math.pi / n
    scan = _make_scan([1.0] * n, angle_min=0.0, angle_increment=inc)
    beams = valid_beams(scan)
    last_raw = (n - 1) * inc
    expected = math.atan2(math.sin(last_raw), math.cos(last_raw))
    assert math.isclose(beams[-1][0], expected, abs_tol=1e-12)


def test_drops_readings_outside_the_sensors_range_limits():
    scan = _make_scan([0.5, 1.0], angle_min=0.0, angle_increment=0.1, range_min=0.6)
    assert [r for _, r in valid_beams(scan)] == [1.0]

    scan = _make_scan([1.0, 9.0], angle_min=0.0, angle_increment=0.1, range_max=5.0)
    assert [r for _, r in valid_beams(scan)] == [1.0]


def test_drops_nan_and_inf_readings():
    scan = _make_scan([float("nan"), float("inf"), 1.2], angle_min=0.0, angle_increment=0.1)
    assert [r for _, r in valid_beams(scan)] == [1.2]


def test_returns_empty_when_nothing_is_valid():
    scan = _make_scan([float("nan"), float("nan")], angle_min=0.0, angle_increment=0.1)
    assert valid_beams(scan) == []
