import math
from types import SimpleNamespace
from proximity_alert.scan_utils import size_obstacle


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


def test_none_when_nothing_close():
    s = _scan([(0.0, 2.0), (math.radians(10), 2.0)])
    assert size_obstacle(s, 180.0, 0.40) is None


def test_narrow_obstacle_small_span():
    # a ~10 deg wide near cluster around straight ahead
    s = _scan([(math.radians(-5), 0.30), (0.0, 0.29), (math.radians(5), 0.30),
               (math.radians(60), 3.0), (math.radians(-60), 3.0)])
    info = size_obstacle(s, 180.0, 0.40)
    assert info is not None
    assert info["span_deg"] < 20.0


def test_preferred_side_points_to_open_room():
    # obstacle slightly right of center, more open space on the left
    s = _scan([(math.radians(-10), 0.30), (math.radians(-5), 0.30),
               (math.radians(70), 4.0), (math.radians(-70), 1.0)])
    info = size_obstacle(s, 180.0, 0.40)
    assert info["preferred_side"] == 1.0  # left is more open
