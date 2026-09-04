"""Pure, ROS-free helpers for reasoning about LiDAR scans.

Deliberately imports nothing from rclpy/sensor_msgs so the forward-arc logic
can be unit-tested on a plain machine with a lightweight duck-typed scan.

Every function here starts from `valid_beams`, which owns the two conventions
that used to be restated in each of them: what counts as a usable reading, and
how a beam's angle is wrapped into [-pi, pi].
"""
import math


def valid_beams(msg):
    """Every usable beam in the scan as ``(bearing, range)``, in scan order.

    ``bearing`` is the beam's angle wrapped into ``[-pi, pi]`` via
    ``atan2(sin, cos)``. The wrap is not optional bookkeeping: this LiDAR
    (LD19) reports angles as ``0..2pi``, so "forward" (0 rad) sits at BOTH
    ends of the sweep. Comparing raw angles against a forward arc would
    measure only the front-right half of it and miss front-left obstacles
    entirely.

    A reading is usable when it is finite and inside the sensor's own
    ``[range_min, range_max]``. NaN and inf fail that comparison anyway; the
    explicit ``isfinite`` check is kept so the intent is readable rather than
    incidental.

    Returned as a list, not a generator: all three callers below need more
    than one pass over it, and one of them sorts it.

    The beam angle is derived by index (`angle_min + i * angle_increment`)
    rather than accumulated across the sweep, so no float error builds up
    along a 450-beam scan.
    """
    beams = []
    for i, r in enumerate(msg.ranges):
        if msg.range_min <= r <= msg.range_max and math.isfinite(r):
            angle = msg.angle_min + i * msg.angle_increment
            beams.append((math.atan2(math.sin(angle), math.cos(angle)), r))
    return beams


def _in_window(rel, center, half_width):
    # rel, center in [-pi, pi]; returns True if rel within +/- half_width of
    # center, measured as a wrapped angular distance (handles the rear wrap).
    d = math.atan2(math.sin(rel - center), math.cos(rel - center))
    return -half_width <= d <= half_width


def reduce_to_sectors(msg, front_arc_deg, front_subsector_deg,
                      side_window_deg, rear_window_deg):
    """Nearest range in each named sector, or inf where the sector is empty."""
    half_front = math.radians(front_arc_deg) / 2.0
    half_sub = math.radians(front_subsector_deg) / 2.0
    half_side = math.radians(side_window_deg) / 2.0
    half_rear = math.radians(rear_window_deg) / 2.0
    left_c = math.pi / 2.0
    right_c = -math.pi / 2.0

    out = {k: float("inf") for k in
           ("front", "front_left", "front_center", "front_right",
            "left", "right", "rear")}

    for rel, r in valid_beams(msg):
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


def size_obstacle(msg, front_arc_deg, obstacle_detect_range):
    """Measure the near obstacle in the forward arc, or None if there isn't one.

    Reports lateral extent in METRES (`y_lo`/`y_hi`), not angular span, because
    angular span grows as an obstacle nears and so cannot answer "can I strafe
    past this". `preferred_side` is chosen purely from far-field openness --
    which side has more room BEYOND the obstacle.
    """
    half_front = math.radians(front_arc_deg) / 2.0
    near = []  # (rel, r)
    left_open = 0.0
    right_open = 0.0
    left_n = right_n = 0
    for rel, r in valid_beams(msg):
        if -half_front <= rel <= half_front:
            if r <= obstacle_detect_range:
                near.append((rel, r))
            else:
                if rel > 0:
                    left_open += r; left_n += 1
                elif rel < 0:
                    right_open += r; right_n += 1
    if not near:
        return None
    rels = [a for a, _ in near]
    ys = [r * math.sin(a) for a, r in near]
    span_deg = math.degrees(max(rels) - min(rels))
    left_avg = left_open / left_n if left_n else 0.0
    right_avg = right_open / right_n if right_n else 0.0
    preferred_side = 1.0 if left_avg >= right_avg else -1.0
    return {"span_deg": span_deg, "y_lo": min(ys), "y_hi": max(ys),
            "preferred_side": preferred_side}


def select_gap(msg, min_gap_clearance, min_gap_width_deg, goal_heading):
    """Bearing of the widest usable opening, tie-broken toward `goal_heading`.

    Returns None when no contiguous clear run is at least `min_gap_width_deg`
    wide.
    """
    beams = valid_beams(msg)
    if not beams:
        return None
    beams.sort(key=lambda b: b[0])
    n = len(beams)
    clear = [r >= min_gap_clearance for _, r in beams]

    # maximal contiguous clear runs as (start_idx, end_idx) inclusive
    runs = []
    i = 0
    while i < n:
        if clear[i]:
            j = i
            while j + 1 < n and clear[j + 1]:
                j += 1
            runs.append((i, j))
            i = j + 1
        else:
            i += 1
    if not runs:
        return None

    candidates = []  # (width_rad, bearing_rad)
    # Wrap-merge: if the first AND last beams are clear, the last run and the
    # first run are ONE circular run straddling +/-pi (e.g. a gap behind the
    # robot). Combine them so it isn't split into two rejected slivers.
    if len(runs) >= 2 and clear[0] and clear[-1]:
        ls, _le = runs[-1]
        _fs, fe = runs[0]
        start_ang, end_ang = beams[ls][0], beams[fe][0]
        width = (end_ang - start_ang) + 2 * math.pi
        bearing = math.atan2(math.sin(start_ang + width / 2),
                             math.cos(start_ang + width / 2))
        candidates.append((width, bearing))
        runs = runs[1:-1]  # consumed into the wrapped run

    for s, e in runs:
        candidates.append((beams[e][0] - beams[s][0],
                           (beams[s][0] + beams[e][0]) / 2.0))

    min_width = math.radians(min_gap_width_deg)
    best = None  # ((width, -|bearing-goal|), bearing)
    for width, bearing in candidates:
        if width >= min_width:
            key = (width, -abs(math.atan2(math.sin(bearing - goal_heading),
                                          math.cos(bearing - goal_heading))))
            if best is None or key > best[0]:
                best = (key, bearing)
    return None if best is None else best[1]
