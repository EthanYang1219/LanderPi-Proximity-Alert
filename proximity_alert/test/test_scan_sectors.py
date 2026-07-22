import math
from types import SimpleNamespace
from proximity_alert.scan_utils import reduce_to_sectors


def _scan(pairs, range_min=0.05, range_max=8.0):
    # pairs: list of (angle_rad, range_m); builds a scan with matching arrays.
    pairs = sorted(pairs, key=lambda p: p[0])
    angles = [a for a, _ in pairs]
    ranges = [r for _, r in pairs]
    inc = (angles[1] - angles[0]) if len(angles) > 1 else 0.1
    return SimpleNamespace(angle_min=angles[0], angle_increment=inc,
                           range_min=range_min, range_max=range_max, ranges=ranges)


def test_front_center_picks_nearest_ahead():
    s = _scan([(0.0, 0.5), (math.radians(80), 2.0), (math.radians(-80), 2.0)])
    out = reduce_to_sectors(s, 180.0, 60.0, 60.0, 60.0)
    assert out["front_center"] == 0.5
    assert out["front"] == 0.5


def test_left_and_right_windows_separate_sides():
    s = _scan([(math.radians(90), 1.0), (math.radians(-90), 3.0)])
    out = reduce_to_sectors(s, 180.0, 60.0, 60.0, 60.0)
    assert out["left"] == 1.0   # +90 deg is left
    assert out["right"] == 3.0


def test_rear_window_wraps_around_pi():
    s = _scan([(math.radians(175), 0.7), (math.radians(-175), 0.9), (0.0, 4.0)])
    out = reduce_to_sectors(s, 180.0, 60.0, 60.0, 60.0)
    assert out["rear"] == 0.7


def test_empty_sector_is_inf():
    s = _scan([(0.0, 1.0)])
    out = reduce_to_sectors(s, 180.0, 60.0, 60.0, 60.0)
    assert out["rear"] == float("inf")
