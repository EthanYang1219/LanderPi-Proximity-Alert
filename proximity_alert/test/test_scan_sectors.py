import math
from types import SimpleNamespace
from proximity_alert.scan_utils import reduce_to_sectors


def _scan(pairs, range_min=0.05, range_max=8.0, inc_deg=1.0):
    # Build a uniform-increment scan over [-pi, pi) like a real LiDAR: each
    # listed (angle_rad, range_m) is placed in its nearest uniform bin; empty
    # bins are inf (excluded by the range guard). This keeps the beams evenly
    # spaced so angle = angle_min + i*increment reconstructs each angle exactly.
    inc = math.radians(inc_deg)
    n = int(round(2 * math.pi / inc))
    angle_min = -math.pi
    ranges = [float("inf")] * n
    for a, r in pairs:
        rel = math.atan2(math.sin(a), math.cos(a))
        idx = int(round((rel - angle_min) / inc)) % n
        ranges[idx] = r
    return SimpleNamespace(angle_min=angle_min, angle_increment=inc,
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
