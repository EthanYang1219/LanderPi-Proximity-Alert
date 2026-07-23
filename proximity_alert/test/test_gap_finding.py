import math
from types import SimpleNamespace
from proximity_alert.scan_utils import select_gap


def _scan(pairs, range_min=0.05, range_max=8.0, inc_deg=1.0):
    # Uniform-increment scan over [-pi, pi); each listed (angle, range) lands
    # in its nearest bin, empty bins are inf (excluded by the range guard).
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


def test_none_when_all_blocked():
    s = _scan([(math.radians(a), 0.3) for a in range(-90, 91, 5)])
    assert select_gap(s, 0.6, 40.0, 0.0) is None


def test_picks_the_only_wide_gap():
    pairs = [(math.radians(a), 0.3) for a in range(-90, 91, 5)]
    for a in range(-20, 25, 5):  # open a ~40 deg window near center
        pairs = [(ang, 5.0 if abs(ang - math.radians(a)) < 1e-6 else r) for ang, r in pairs]
    bearing = select_gap(_scan(pairs), 0.6, 30.0, 0.0)
    assert bearing is not None
    assert abs(bearing) < math.radians(15)  # near center


def test_tiebreak_prefers_gap_closest_to_goal():
    # two equally wide gaps; goal_heading nudges selection toward the right one
    pairs = [(math.radians(a), 0.3) for a in range(-90, 91, 5)]
    def openw(pairs, lo, hi):
        return [(ang, 5.0 if math.radians(lo) <= ang <= math.radians(hi) else r)
                for ang, r in pairs]
    pairs = openw(pairs, -70, -40)
    pairs = openw(pairs, 40, 70)
    bearing = select_gap(_scan(pairs), 0.6, 25.0, math.radians(55))
    assert bearing > 0  # goal is to the right, so pick the right gap


def test_wraps_gap_straddling_pi_behind_robot():
    # Everything blocked except a ~40 deg window centered on 180 deg (behind).
    # Split at the +/-180 array boundary each half is only 20 deg; without the
    # wrap-merge, both are rejected against a 30 deg min width -> None (the bug).
    pairs = [(math.radians(a), (5.0 if abs(a) >= 160 else 0.3))
             for a in range(-180, 180, 5)]
    bearing = select_gap(_scan(pairs), 0.6, 30.0, math.radians(180))
    assert bearing is not None
    assert abs(abs(bearing) - math.pi) < math.radians(20)  # points behind
