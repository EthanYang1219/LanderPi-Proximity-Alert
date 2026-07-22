# Refined Obstacle Avoidance Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace `path_tracker`'s obstacle avoidance with a deterministic 7-state machine that prefers mecanum strafing (bearing-preserving) over turning, per the approved design spec, and emit a structured per-decision research log to a separate CSV.

**Architecture:** All decision/state logic lives in a pure, ROS-free `AvoidanceController` (unit-tested on the host with synthetic sector inputs — no hardware). `scan_utils` gains pure functions that reduce a `LaserScan` into 360° sector clearances, size the obstacle, and find gaps. `path_tracker` becomes a thin ROS node: reduce scan → call controller each tick → publish `cmd_vel`, `/forward_min_range`, and a JSON `/avoidance_decision` record. A separate `decision_logger` node writes those records to their own CSV, parallel to the existing `trial_logger`.

**Tech Stack:** ROS 2 Humble, `rclpy`, Python 3.10 (container) / 3.11 (host), `std_msgs`, `sensor_msgs`, pytest.

## Global Constraints

- ROS 2 Humble, `rclpy` only. No SLAM/mapping/global planning.
- Governing invariant (verbatim from spec): **The algorithm always attempts the maneuver that minimizes deviation from the original goal heading while maintaining safety.**
- Deterministic and lightweight; O(beams) per scan; no per-frame allocation-heavy work.
- `path_tracker`'s existing safe-shutdown must be preserved: STM32 has no motion watchdog, so `cmd_vel` must be driven to zero on stop/exit; keep the flag-based signal handler from commit `e64c4c2` (never call `publish_stop()`/`rclpy.shutdown()` from the signal handler).
- Pure modules (`scan_utils`, `avoidance`, `decision_record`) must import **nothing** from `rclpy`/`sensor_msgs` at module load, so they unit-test on a plain host with duck-typed inputs (follow the existing `scan_utils` pattern).
- LD19 reports angles `0..2π`; every angular computation must wrap via `atan2(sin, cos)` into `[-π, π]` (as `min_range_in_forward_arc` already does).
- **Angle from index, not accumulation:** compute each beam's angle as `msg.angle_min + i * msg.angle_increment` (enumerate), never `angle += increment`. In float64 the accumulation drift is ~1e-12 rad (negligible vs the 0.0125 rad beam spacing) — this is correctness hygiene, not a live-error fix, but adopt it uniformly.
- **Finite guard:** every per-beam loop must gate on `msg.range_min <= r <= msg.range_max and math.isfinite(r)`. The range check alone already excludes NaN/±Inf for this LD19 (`range_max` is a finite 25.0), so `isfinite` is defense-in-depth against a future `range_max` reconfiguration — include it anyway.
- **Angular runs must wrap:** any logic that scans contiguous angular runs over the sorted `[-π, π]` beams (e.g. gap finding) must merge a run touching `-π` with a run touching `+π` — a gap directly behind the robot straddles the array boundary and must not be split into two rejected slivers.
- Both CSVs must default to the host-accessible bind-mount path `/home/ubuntu/shared/trials/` (container) = `/home/pi/docker/tmp/trials/` (host).
- The decision log is a **separate file** written by a **separate node** — never merged into `trial_logger`'s motion CSV.
- All `docker exec` for build/test/run uses `-u ubuntu` (root breaks FastRTPS SHM delivery on this container).
- Any live `path_tracker` test remaps `-r /cmd_vel:=/cmd_vel_test` unless a real drive is explicitly intended and pre-flight safety checks pass.

## File Structure

- `proximity_alert/proximity_alert/scan_utils.py` — MODIFY. Add pure sector reduction, obstacle sizing, and gap-finding. Keep existing `min_range_in_forward_arc`.
- `proximity_alert/proximity_alert/decision_record.py` — CREATE. `DecisionRecord` dataclass + `to_json`/`from_json` + CSV header/row helpers. Pure.
- `proximity_alert/proximity_alert/avoidance.py` — CREATE. `AvoidanceConfig`, `Sectors`, `ControllerOutput` dataclasses; `AvoidanceController` (pure state machine). No rclpy.
- `proximity_alert/proximity_alert/path_tracker.py` — MODIFY. Thin node wrapping `AvoidanceController`; publishes `cmd_vel`, `/forward_min_range`, `/avoidance_decision`.
- `proximity_alert/proximity_alert/decision_logger.py` — CREATE. Companion node: subscribes `/avoidance_decision`, writes separate CSV to host path.
- `proximity_alert/setup.py` — MODIFY. Add `decision_logger` console entry point.
- `proximity_alert/test/test_scan_sectors.py`, `test_obstacle_sizing.py`, `test_gap_finding.py`, `test_decision_record.py`, `test_avoidance_controller.py`, `test_decision_logger_csv.py` — CREATE.
- `README.md` — MODIFY. Document the new params, the two separate CSVs + host paths, and the decision-log columns.

Time is provided to the controller as explicit `now` (seconds, float) and `dt` so it is clock-injectable and testable without a ROS clock.

---

### Task 1: Sector reduction in `scan_utils`

**Files:**
- Modify: `proximity_alert/proximity_alert/scan_utils.py`
- Test: `proximity_alert/test/test_scan_sectors.py`

**Interfaces:**
- Produces: `reduce_to_sectors(msg, front_arc_deg, front_subsector_deg, side_window_deg, rear_window_deg) -> dict` with float keys `front`, `front_left`, `front_center`, `front_right`, `left`, `right`, `rear`, each the min valid range in that window (or `float("inf")` if empty). Angles wrapped to `[-π, π]`; center 0 = straight ahead; `+` = left (CCW).

- [ ] **Step 1: Write the failing test**

```python
# proximity_alert/test/test_scan_sectors.py
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
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd proximity_alert && python3 -m pytest test/test_scan_sectors.py -v`
Expected: FAIL — `ImportError: cannot import name 'reduce_to_sectors'`.

- [ ] **Step 3: Write minimal implementation**

```python
# append to proximity_alert/proximity_alert/scan_utils.py

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
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd proximity_alert && python3 -m pytest test/test_scan_sectors.py -v`
Expected: PASS (4 passed).

- [ ] **Step 5: Commit**

```bash
git add proximity_alert/proximity_alert/scan_utils.py proximity_alert/test/test_scan_sectors.py
git commit -m "feat: reduce LaserScan into 360deg sector clearances"
```

---

### Task 2: Obstacle lateral sizing in `scan_utils`

**Files:**
- Modify: `proximity_alert/proximity_alert/scan_utils.py`
- Test: `proximity_alert/test/test_obstacle_sizing.py`

**Interfaces:**
- Produces: `size_obstacle(msg, front_arc_deg, obstacle_detect_range) -> dict | None` returning `None` if no FRONT beam is within `obstacle_detect_range`, else `{"span_deg": float, "y_lo": float, "y_hi": float, "preferred_side": +1.0|-1.0}` where `y = r*sin(rel)` (left +), `preferred_side` = +1 (left) if there is more open room on the left of the near cluster else -1 (right).

- [ ] **Step 1: Write the failing test**

```python
# proximity_alert/test/test_obstacle_sizing.py
import math
from types import SimpleNamespace
from proximity_alert.scan_utils import size_obstacle


def _scan(pairs, range_min=0.05, range_max=8.0):
    pairs = sorted(pairs, key=lambda p: p[0])
    angles = [a for a, _ in pairs]
    return SimpleNamespace(angle_min=angles[0],
                           angle_increment=(angles[1]-angles[0]) if len(angles) > 1 else 0.1,
                           range_min=range_min, range_max=range_max,
                           ranges=[r for _, r in pairs])


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
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd proximity_alert && python3 -m pytest test/test_obstacle_sizing.py -v`
Expected: FAIL — `ImportError: cannot import name 'size_obstacle'`.

- [ ] **Step 3: Write minimal implementation**

```python
# append to proximity_alert/proximity_alert/scan_utils.py

def size_obstacle(msg, front_arc_deg, obstacle_detect_range):
    half_front = math.radians(front_arc_deg) / 2.0
    near = []  # (rel, r)
    left_open = 0.0
    right_open = 0.0
    left_n = right_n = 0
    for i, r in enumerate(msg.ranges):
        if msg.range_min <= r <= msg.range_max and math.isfinite(r):
            angle = msg.angle_min + i * msg.angle_increment
            rel = math.atan2(math.sin(angle), math.cos(angle))
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
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd proximity_alert && python3 -m pytest test/test_obstacle_sizing.py -v`
Expected: PASS (3 passed).

- [ ] **Step 5: Commit**

```bash
git add proximity_alert/proximity_alert/scan_utils.py proximity_alert/test/test_obstacle_sizing.py
git commit -m "feat: measure obstacle angular span and preferred sidestep side"
```

---

### Task 3: Gap finding + selection in `scan_utils`

**Files:**
- Modify: `proximity_alert/proximity_alert/scan_utils.py`
- Test: `proximity_alert/test/test_gap_finding.py`

**Interfaces:**
- Produces: `select_gap(msg, min_gap_clearance, min_gap_width_deg, goal_heading) -> float | None` returning the bearing (rad, `[-π, π]`) of the widest contiguous interval whose beams are all ≥ `min_gap_clearance` and whose angular width ≥ `min_gap_width_deg`; tie-break by smallest `|bearing - goal_heading|`. `None` if no qualifying gap. `goal_heading` is relative to the robot's current heading (0 = dead ahead).

- [ ] **Step 1: Write the failing test**

```python
# proximity_alert/test/test_gap_finding.py
import math
from types import SimpleNamespace
from proximity_alert.scan_utils import select_gap


def _scan(pairs, range_min=0.05, range_max=8.0):
    pairs = sorted(pairs, key=lambda p: p[0])
    angles = [a for a, _ in pairs]
    return SimpleNamespace(angle_min=angles[0],
                           angle_increment=(angles[1]-angles[0]) if len(angles) > 1 else 0.1,
                           range_min=range_min, range_max=range_max,
                           ranges=[r for _, r in pairs])


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
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd proximity_alert && python3 -m pytest test/test_gap_finding.py -v`
Expected: FAIL — `ImportError: cannot import name 'select_gap'`.

- [ ] **Step 3: Write minimal implementation**

```python
# append to proximity_alert/proximity_alert/scan_utils.py

def select_gap(msg, min_gap_clearance, min_gap_width_deg, goal_heading):
    beams = []  # (rel, r)
    for i, r in enumerate(msg.ranges):
        if msg.range_min <= r <= msg.range_max and math.isfinite(r):
            angle = msg.angle_min + i * msg.angle_increment
            rel = math.atan2(math.sin(angle), math.cos(angle))
            beams.append((rel, r))
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
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd proximity_alert && python3 -m pytest test/test_gap_finding.py -v`
Expected: PASS (4 passed), including the wrap-around case.

- [ ] **Step 5: Commit**

```bash
git add proximity_alert/proximity_alert/scan_utils.py proximity_alert/test/test_gap_finding.py
git commit -m "feat: find and select the best-fit gap from a 360deg scan"
```

---

### Task 4: `DecisionRecord` dataclass + JSON/CSV helpers

**Files:**
- Create: `proximity_alert/proximity_alert/decision_record.py`
- Test: `proximity_alert/test/test_decision_record.py`

**Interfaces:**
- Produces: `DecisionRecord` dataclass with fields (all present, spec §9): `timestamp:str, encounter_id:int, state:str, chosen_maneuver:str, reason:str, obstacle_span_deg:float, front_distance_m:float, front_left_m:float, front_center_m:float, front_right_m:float, left_clearance_m:float, right_clearance_m:float, rear_clearance_m:float, required_clearing_m:float, cumulative_strafe_m:float, consecutive_avoid_count:int, recovery_triggered:bool, outcome:str, maneuver_duration_s:float`. Methods `to_json() -> str`, classmethod `from_json(str) -> DecisionRecord`, staticmethod `csv_header() -> list[str]`, `csv_row() -> list`. Infinite ranges serialize as `""` in CSV.

- [ ] **Step 1: Write the failing test**

```python
# proximity_alert/test/test_decision_record.py
from proximity_alert.decision_record import DecisionRecord


def _rec(**kw):
    base = dict(timestamp="2026-07-23 10:00:00", encounter_id=1, state="ASSESS",
                chosen_maneuver="STRAFE", reason="strafe: narrow+side_clear",
                obstacle_span_deg=12.0, front_distance_m=0.29, front_left_m=0.5,
                front_center_m=0.29, front_right_m=0.6, left_clearance_m=1.2,
                right_clearance_m=0.4, rear_clearance_m=2.0, required_clearing_m=0.22,
                cumulative_strafe_m=0.0, consecutive_avoid_count=1,
                recovery_triggered=False, outcome="cleared", maneuver_duration_s=1.4)
    base.update(kw)
    return DecisionRecord(**base)


def test_json_round_trip():
    r = _rec()
    assert DecisionRecord.from_json(r.to_json()) == r


def test_csv_header_matches_row_length():
    assert len(DecisionRecord.csv_header()) == len(_rec().csv_row())


def test_infinite_range_serializes_blank_in_csv():
    r = _rec(rear_clearance_m=float("inf"))
    row = dict(zip(DecisionRecord.csv_header(), _rec(rear_clearance_m=float("inf")).csv_row()))
    assert row["rear_clearance_m"] == ""
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd proximity_alert && python3 -m pytest test/test_decision_record.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'proximity_alert.decision_record'`.

- [ ] **Step 3: Write minimal implementation**

```python
# proximity_alert/proximity_alert/decision_record.py
"""Structured per-decision research record. Pure (no rclpy)."""
import json
import math
from dataclasses import dataclass, asdict, fields


@dataclass
class DecisionRecord:
    timestamp: str
    encounter_id: int
    state: str
    chosen_maneuver: str
    reason: str
    obstacle_span_deg: float
    front_distance_m: float
    front_left_m: float
    front_center_m: float
    front_right_m: float
    left_clearance_m: float
    right_clearance_m: float
    rear_clearance_m: float
    required_clearing_m: float
    cumulative_strafe_m: float
    consecutive_avoid_count: int
    recovery_triggered: bool
    outcome: str
    maneuver_duration_s: float

    def to_json(self):
        return json.dumps(asdict(self))

    @classmethod
    def from_json(cls, text):
        return cls(**json.loads(text))

    @staticmethod
    def csv_header():
        return [f.name for f in fields(DecisionRecord)]

    def csv_row(self):
        row = []
        for f in fields(DecisionRecord):
            v = getattr(self, f.name)
            if isinstance(v, float) and not math.isfinite(v):
                row.append("")
            else:
                row.append(v)
        return row
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd proximity_alert && python3 -m pytest test/test_decision_record.py -v`
Expected: PASS (3 passed).

- [ ] **Step 5: Commit**

```bash
git add proximity_alert/proximity_alert/decision_record.py proximity_alert/test/test_decision_record.py
git commit -m "feat: add DecisionRecord dataclass with JSON and CSV helpers"
```

---

### Task 5: `AvoidanceController` — types, DRIVE/detect/debounce, ASSESS ladder

**Files:**
- Create: `proximity_alert/proximity_alert/avoidance.py`
- Test: `proximity_alert/test/test_avoidance_controller.py`

**Interfaces:**
- Produces:
  - `AvoidanceConfig` dataclass holding every §10 tunable (defaults = the spec example values) plus derived properties `max_strafe_distance`, `corridor_half`, `clear_threshold`.
  - `ControllerOutput` dataclass: `linear_x:float, linear_y:float, angular_z:float, state:str, decision: DecisionRecord | None`.
  - `AvoidanceController(config)` with `set_goal_heading(yaw)`, and `step(sectors, obstacle, gap_bearing, current_yaw, now) -> ControllerOutput`. `sectors` is Task 1's dict; `obstacle` is Task 2's dict-or-None; `gap_bearing` is Task 3's float-or-None (only consulted in RECOVER). Heading PID is internal (reuse the existing `HeadingPID` moved into this module).
- Consumes: `DecisionRecord` (Task 4); `HeadingPID` (extracted from `path_tracker.py`).

States as string constants: `DRIVE, ASSESS, STRAFE, TURN, DRIVE_PAST, RECOVER, HALT`.

- [ ] **Step 1: Write the failing test**

```python
# proximity_alert/test/test_avoidance_controller.py
import math
from proximity_alert.avoidance import AvoidanceController, AvoidanceConfig


def _sectors(front=5.0, fl=5.0, fc=5.0, fr=5.0, left=5.0, right=5.0, rear=5.0):
    return {"front": front, "front_left": fl, "front_center": fc, "front_right": fr,
            "left": left, "right": right, "rear": rear}


def _cfg(**kw):
    c = AvoidanceConfig()
    for k, v in kw.items():
        setattr(c, k, v)
    return c


def test_drives_straight_when_clear():
    c = AvoidanceController(_cfg())
    c.set_goal_heading(0.0)
    out = c.step(_sectors(), obstacle=None, gap_bearing=None, current_yaw=0.0, now=0.0)
    assert out.state == "DRIVE"
    assert out.linear_x > 0.0


def test_confirm_scans_debounce_before_assess():
    c = AvoidanceController(_cfg(obstacle_confirm_scans=2, safety_distance=0.30))
    c.set_goal_heading(0.0)
    obst = {"span_deg": 10.0, "y_lo": -0.05, "y_hi": 0.05, "preferred_side": 1.0}
    # first close scan: holding (zero forward), not yet ASSESS
    out1 = c.step(_sectors(front=0.29, fc=0.29), obst, None, 0.0, 0.0)
    assert out1.state == "DRIVE" and out1.linear_x == 0.0
    # second consecutive close scan: now commits to a maneuver
    out2 = c.step(_sectors(front=0.29, fc=0.29), obst, None, 0.0, 0.1)
    assert out2.state in ("STRAFE", "TURN")


def test_assess_chooses_strafe_for_narrow_obstacle_with_clear_side():
    c = AvoidanceController(_cfg(obstacle_confirm_scans=1))
    c.set_goal_heading(0.0)
    obst = {"span_deg": 10.0, "y_lo": -0.05, "y_hi": 0.05, "preferred_side": 1.0}
    out = c.step(_sectors(front=0.29, fc=0.29, left=1.5), obst, None, 0.0, 0.0)
    assert out.state == "STRAFE"
    assert out.linear_y > 0.0            # strafing left (+)
    assert out.decision.chosen_maneuver == "STRAFE"


def test_assess_chooses_turn_when_side_blocked():
    c = AvoidanceController(_cfg(obstacle_confirm_scans=1, strafe_side_clearance_min=0.30))
    c.set_goal_heading(0.0)
    obst = {"span_deg": 10.0, "y_lo": -0.05, "y_hi": 0.05, "preferred_side": 1.0}
    # preferred side (left) is blocked -> must turn
    out = c.step(_sectors(front=0.29, fc=0.29, left=0.15, right=1.5), obst, None, 0.0, 0.0)
    assert out.state == "TURN"


def test_assess_chooses_turn_when_obstacle_too_wide():
    c = AvoidanceController(_cfg(obstacle_confirm_scans=1, max_obstacle_span_deg=50.0))
    c.set_goal_heading(0.0)
    obst = {"span_deg": 80.0, "y_lo": -0.6, "y_hi": 0.6, "preferred_side": 1.0}
    out = c.step(_sectors(front=0.29, fc=0.29, left=2.0), obst, None, 0.0, 0.0)
    assert out.state == "TURN"
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd proximity_alert && python3 -m pytest test/test_avoidance_controller.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'proximity_alert.avoidance'`.

- [ ] **Step 3: Write minimal implementation**

Create `avoidance.py` with the config/types, the extracted `HeadingPID`, `normalize_angle`, and a `step()` implementing DRIVE (with `obstacle_confirm_scans` debounce holding zero forward while confirming), the ASSESS ladder (strafe-viability per spec §4 steps 0–2 using `config.max_strafe_distance`, `corridor_half`, `strafe_side_clearance_min`, `max_obstacle_span_deg`, `max_cumulative_strafe`), and entry into STRAFE/TURN with a `DecisionRecord` emitted on the ASSESS decision. Escalation to RECOVER/HALT is added in Task 6. Include:

```python
# proximity_alert/proximity_alert/avoidance.py  (skeleton — full logic per spec §3-§8)
import math
import time
from dataclasses import dataclass
from proximity_alert.decision_record import DecisionRecord

DRIVE, ASSESS, STRAFE, TURN, DRIVE_PAST, RECOVER, HALT = (
    "DRIVE", "ASSESS", "STRAFE", "TURN", "DRIVE_PAST", "RECOVER", "HALT")


def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class HeadingPID:  # moved verbatim from path_tracker.py
    def __init__(self, kp, ki, kd, output_limit):
        self.kp, self.ki, self.kd, self.output_limit = kp, ki, kd, output_limit
        self.reset()
    def reset(self):
        self.integral = 0.0
        self.prev_error = None
    def update(self, error, dt):
        self.integral += error * dt
        max_i = self.output_limit / self.ki if self.ki else float("inf")
        self.integral = max(-max_i, min(max_i, self.integral))
        deriv = 0.0 if self.prev_error is None else (error - self.prev_error) / dt
        self.prev_error = error
        out = self.kp * error + self.ki * self.integral + self.kd * deriv
        return max(-self.output_limit, min(self.output_limit, out))


@dataclass
class AvoidanceConfig:
    safety_distance: float = 0.30
    clear_margin: float = 0.10
    clear_confirm_time: float = 0.5
    obstacle_detect_range: float = 0.40
    obstacle_confirm_scans: int = 2
    front_arc_deg: float = 180.0
    front_subsector_deg: float = 60.0
    side_window_deg: float = 60.0
    rear_window_deg: float = 60.0
    robot_half_width: float = 0.11
    corridor_margin: float = 0.05
    forward_speed: float = 0.50
    strafe_speed: float = 0.25
    strafe_timeout: float = 1.5
    strafe_side_clearance_min: float = 0.30
    max_obstacle_span_deg: float = 50.0
    max_cumulative_strafe: float = 0.60
    turn_speed: float = 0.6
    turn_step_deg: float = 30.0
    turn_timeout: float = 1.5
    reverse_trigger_range: float = 0.20
    avoid_reverse_speed: float = 0.10
    rear_clearance_min: float = 0.25
    pass_clearance: float = 0.35
    max_drive_past_distance: float = 0.80
    heading_kp: float = 1.0
    heading_ki: float = 0.0
    heading_kd: float = 0.1
    heading_max_correction: float = 0.3
    heading_tol_deg: float = 5.0
    max_avoid_attempts: int = 3
    clear_drive_duration: float = 3.0
    recover_backup_clearance: float = 0.50
    recover_commit_distance: float = 0.50
    min_gap_clearance: float = 0.60
    min_gap_width_deg: float = 40.0
    control_rate_hz: float = 10.0

    @property
    def max_strafe_distance(self):
        return self.strafe_speed * self.strafe_timeout

    @property
    def corridor_half(self):
        return self.robot_half_width + self.corridor_margin

    @property
    def clear_threshold(self):
        return self.safety_distance + self.clear_margin


@dataclass
class ControllerOutput:
    linear_x: float
    linear_y: float
    angular_z: float
    state: str
    decision: object  # DecisionRecord | None
```

Implement `AvoidanceController` with the fields: `state`, `goal_heading`, `consecutive_avoid_count`, `cumulative_strafe`, `has_recovered_this_encounter`, `_confirm_count`, `_locked_dir`, `_maneuver_start`, `encounter_id`, `_clean_drive_since` (timestamp DRIVE last (re)started clean), `_in_encounter` (bool), `_pid`.

**Encounter lifecycle (explicit — a sensor blip must not merge two obstacles, nor split one).** `encounter_id` and the reset of `consecutive_avoid_count` / `cumulative_strafe` / `has_recovered_this_encounter` are BOTH gated by the same cooldown: an encounter is considered *closed* only after `now - _clean_drive_since >= clear_drive_duration` of continuous DRIVE with the front clear. Concretely: when a confirmed obstacle is detected in DRIVE, if `_in_encounter` is already True (we are within an unclosed encounter — a re-detection during cooldown) it **resumes** the current `encounter_id` and does NOT reset the counters; if `_in_encounter` is False (cooldown had elapsed, encounter closed) it **increments** `encounter_id`, sets `_in_encounter = True`, and zeroes the counters. Entering DRIVE with the front clear sets `_clean_drive_since = now`; each clean DRIVE tick checks `now - _clean_drive_since >= clear_drive_duration` and, when it passes, sets `_in_encounter = False` (encounter closed, counters reset). Any obstacle detected before that threshold keeps `_in_encounter = True` and the same `encounter_id`. `step()` dispatches on `self.state`. In DRIVE: if obstacle present and `front <= safety_distance`, increment `_confirm_count`, command zero forward; when `_confirm_count >= obstacle_confirm_scans` call `_assess(...)`; else PID-hold `goal_heading` and drive `forward_speed`. `_assess()` implements the ladder, sets `_locked_dir`, `_maneuver_start = now`, builds and returns the `DecisionRecord`. Write the STRAFE/TURN commands (STRAFE: `linear_y = strafe_speed * _locked_dir`, PID-hold heading; TURN: `angular_z = turn_speed * _locked_dir`, blend `-avoid_reverse_speed` if `front < reverse_trigger_range and rear >= rear_clearance_min`). Return to DRIVE on `front >= clear_threshold` sustained (Task 6 adds the sustained-timer + DRIVE_PAST/RECOVER/HALT).

- [ ] **Step 4: Run test to verify it passes**

Run: `cd proximity_alert && python3 -m pytest test/test_avoidance_controller.py -v`
Expected: PASS (5 passed).

- [ ] **Step 5: Commit**

```bash
git add proximity_alert/proximity_alert/avoidance.py proximity_alert/test/test_avoidance_controller.py
git commit -m "feat: AvoidanceController core with debounced detect and ASSESS ladder"
```

---

### Task 6: `AvoidanceController` — STRAFE/TURN/DRIVE_PAST/RECOVER/HALT + anti-oscillation

**Files:**
- Modify: `proximity_alert/proximity_alert/avoidance.py`
- Test: `proximity_alert/test/test_avoidance_controller.py` (append)

**Interfaces:**
- Consumes/extends the Task 5 `AvoidanceController`. Adds the remaining transitions, commit-lock (maneuvers run to completion per §5), hysteresis exit (§8), `consecutive_avoid_count` reset only after `clear_drive_duration` clean DRIVE, `has_recovered_this_encounter` one-shot, and the `outcome`/`maneuver_duration_s` fields on completion records.

- [ ] **Step 1: Write the failing test (append)**

```python
def test_strafe_completes_to_drive_when_front_clears():
    c = AvoidanceController(_cfg(obstacle_confirm_scans=1))
    c.set_goal_heading(0.0)
    obst = {"span_deg": 10.0, "y_lo": -0.05, "y_hi": 0.05, "preferred_side": 1.0}
    c.step(_sectors(front=0.29, fc=0.29, left=1.5), obst, None, 0.0, 0.0)  # -> STRAFE
    out = c.step(_sectors(front=5.0), None, None, 0.0, 0.4)               # front now clear
    assert out.state == "DRIVE"


def test_escalates_to_recover_then_halt():
    c = AvoidanceController(_cfg(obstacle_confirm_scans=1, max_avoid_attempts=2))
    c.set_goal_heading(0.0)
    obst = {"span_deg": 80.0, "y_lo": -0.6, "y_hi": 0.6, "preferred_side": 1.0}
    blocked = _sectors(front=0.29, fc=0.29, left=0.15, right=0.15, rear=0.15)
    t = 0.0
    seen = set()
    for _ in range(40):
        out = c.step(blocked, obst, gap_bearing=None, current_yaw=0.0, now=t)
        seen.add(out.state)
        t += 2.0  # jump past every maneuver timeout so cycles complete fast
    assert "RECOVER" in seen
    assert out.state == "HALT"          # no gap -> halts, does not loop forever


def test_recover_commits_toward_gap_when_available():
    c = AvoidanceController(_cfg(obstacle_confirm_scans=1, max_avoid_attempts=1))
    c.set_goal_heading(0.0)
    obst = {"span_deg": 80.0, "y_lo": -0.6, "y_hi": 0.6, "preferred_side": 1.0}
    blocked = _sectors(front=0.29, fc=0.29, left=0.2, right=0.2, rear=2.0)
    t = 0.0
    for _ in range(6):
        out = c.step(blocked, obst, gap_bearing=math.radians(40), current_yaw=0.0, now=t)
        t += 2.0
        if out.state == "RECOVER":
            break
    assert out.state == "RECOVER"
    # while facing away from the gap it should be rotating toward it (+ = left/CCW)
    assert out.angular_z != 0.0


def test_blip_during_cooldown_resumes_same_encounter():
    c = AvoidanceController(_cfg(obstacle_confirm_scans=1, clear_drive_duration=3.0))
    c.set_goal_heading(0.0)
    obst = {"span_deg": 10.0, "y_lo": -0.05, "y_hi": 0.05, "preferred_side": 1.0}
    c.step(_sectors(front=0.29, fc=0.29, left=1.5), obst, None, 0.0, 0.0)  # encounter A
    id_a = c.encounter_id
    c.step(_sectors(front=5.0), None, None, 0.0, 0.4)                       # brief clear
    # re-detect BEFORE clear_drive_duration elapses -> same encounter, no reset
    out = c.step(_sectors(front=0.29, fc=0.29, left=1.5), obst, None, 0.0, 1.0)
    assert out.decision.encounter_id == id_a


def test_detection_after_cooldown_is_new_encounter():
    c = AvoidanceController(_cfg(obstacle_confirm_scans=1, clear_drive_duration=1.0))
    c.set_goal_heading(0.0)
    obst = {"span_deg": 10.0, "y_lo": -0.05, "y_hi": 0.05, "preferred_side": 1.0}
    c.step(_sectors(front=0.29, fc=0.29, left=1.5), obst, None, 0.0, 0.0)  # encounter A
    id_a = c.encounter_id
    # sustained clean driving past clear_drive_duration closes the encounter
    for t in (0.4, 1.0, 2.0, 3.5):
        c.step(_sectors(front=5.0), None, None, 0.0, t)
    out = c.step(_sectors(front=0.29, fc=0.29, left=1.5), obst, None, 0.0, 4.0)
    assert out.decision.encounter_id == id_a + 1
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd proximity_alert && python3 -m pytest test/test_avoidance_controller.py -v`
Expected: FAIL — the new tests fail (RECOVER/HALT/DRIVE_PAST paths not implemented; states never reached).

- [ ] **Step 3: Write the implementation**

Extend `step()`: commit-lock STRAFE/TURN until completion (`front >= clear_threshold` for STRAFE success, or `now - _maneuver_start >= strafe_timeout`/`turn_timeout`, or `cumulative_strafe` cap). On STRAFE non-success completion → re-`_assess`. On TURN completion → DRIVE_PAST (target = post-turn yaw; exit when `front >= clear_threshold` and inside-side window ≥ `pass_clearance` sustained `clear_confirm_time`, or `max_drive_past_distance`, or re-block → ASSESS). In `_assess`, before choosing strafe/turn, apply escalation: `consecutive_avoid_count += 1`; if `> max_avoid_attempts` and `not has_recovered_this_encounter` → RECOVER (`has_recovered_this_encounter = True`); if already recovered → HALT. RECOVER: reverse until `front >= recover_backup_clearance` (or `rear <= rear_clearance_min`), then if `gap_bearing is None` → HALT else rotate toward `gap_bearing` (PID) and once `|yaw - (start_yaw+gap_bearing)| <= heading_tol` commit forward `recover_commit_distance`; clear → DRIVE, re-block → HALT. HALT: zero output; recheck each tick, `front >= clear_threshold` sustained `clear_confirm_time` → DRIVE. The counter/flag resets happen via the Task 5 encounter-close mechanism (encounter closes, and `consecutive_avoid_count` / `cumulative_strafe` / `has_recovered_this_encounter` reset, only once `now - _clean_drive_since >= clear_drive_duration` of sustained clean DRIVE) — do not reset them anywhere else. Every completion emits a `DecisionRecord` with `outcome` in {`cleared`,`escalated`,`halted`} and `maneuver_duration_s`.

- [ ] **Step 4: Run test to verify it passes**

Run: `cd proximity_alert && python3 -m pytest test/test_avoidance_controller.py -v`
Expected: PASS (10 passed).

- [ ] **Step 5: Commit**

```bash
git add proximity_alert/proximity_alert/avoidance.py proximity_alert/test/test_avoidance_controller.py
git commit -m "feat: complete avoidance state machine with recovery and anti-oscillation"
```

---

### Task 7: `decision_logger` companion node (separate CSV, host path)

**Files:**
- Create: `proximity_alert/proximity_alert/decision_logger.py`
- Modify: `proximity_alert/setup.py` (add entry point)
- Test: `proximity_alert/test/test_decision_logger_csv.py`

**Interfaces:**
- Consumes: `/avoidance_decision` (`std_msgs/String`, JSON of `DecisionRecord`).
- Produces: appends one CSV row per message to `csv_path` (param, default `/home/ubuntu/shared/trials/decision_log.csv` — host `/home/pi/docker/tmp/trials/decision_log.csv`). Writes the header once. This CSV is **separate** from `trial_logger`'s motion CSV.

- [ ] **Step 1: Write the failing test**

```python
# proximity_alert/test/test_decision_logger_csv.py
import csv, os, tempfile
import rclpy
from proximity_alert.decision_logger import DecisionLogger
from proximity_alert.decision_record import DecisionRecord


def _rec():
    return DecisionRecord(
        timestamp="2026-07-23 10:00:00", encounter_id=1, state="ASSESS",
        chosen_maneuver="STRAFE", reason="strafe: narrow+side_clear",
        obstacle_span_deg=12.0, front_distance_m=0.29, front_left_m=0.5,
        front_center_m=0.29, front_right_m=0.6, left_clearance_m=1.2,
        right_clearance_m=0.4, rear_clearance_m=2.0, required_clearing_m=0.22,
        cumulative_strafe_m=0.0, consecutive_avoid_count=1,
        recovery_triggered=False, outcome="cleared", maneuver_duration_s=1.4)


def test_writes_header_and_row_to_separate_csv():
    rclpy.init()
    fd, path = tempfile.mkstemp(suffix=".csv"); os.close(fd); os.remove(path)
    node = DecisionLogger()
    node.csv_path = path
    node._ensure_header()
    from std_msgs.msg import String
    node.on_decision(String(data=_rec().to_json()))
    with open(path, newline="") as f:
        rows = list(csv.reader(f))
    node.destroy_node(); rclpy.shutdown(); os.remove(path)
    assert rows[0] == DecisionRecord.csv_header()
    assert dict(zip(rows[0], rows[1]))["chosen_maneuver"] == "STRAFE"
```

- [ ] **Step 2: Run test to verify it fails**

Run (in container): `docker exec -u ubuntu -w /home/ubuntu/ros2_ws/src/proximity_alert MentorPi bash -lc "source /opt/ros/humble/setup.bash && source /home/ubuntu/ros2_ws/install/setup.bash && python3 -m pytest test/test_decision_logger_csv.py -v"`
Expected: FAIL — `ModuleNotFoundError: No module named 'proximity_alert.decision_logger'`.

- [ ] **Step 3: Write minimal implementation**

```python
# proximity_alert/proximity_alert/decision_logger.py
"""Companion node: writes AvoidanceController decision records to their own CSV,
separate from trial_logger's motion CSV, on the host-mounted shared path."""
import csv, os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from proximity_alert.decision_record import DecisionRecord


class DecisionLogger(Node):
    def __init__(self):
        super().__init__("decision_logger")
        self.declare_parameter("csv_path", "/home/ubuntu/shared/trials/decision_log.csv")
        self.csv_path = self.get_parameter("csv_path").value
        self._ensure_header()
        self.create_subscription(String, "/avoidance_decision", self.on_decision, 50)
        self.get_logger().info(f"decision_logger up. Writing to '{self.csv_path}'.")

    def _ensure_header(self):
        if not os.path.exists(self.csv_path):
            os.makedirs(os.path.dirname(self.csv_path) or ".", exist_ok=True)
            with open(self.csv_path, "w", newline="") as f:
                csv.writer(f).writerow(DecisionRecord.csv_header())

    def on_decision(self, msg):
        rec = DecisionRecord.from_json(msg.data)
        with open(self.csv_path, "a", newline="") as f:
            csv.writer(f).writerow(rec.csv_row())


def main(args=None):
    rclpy.init(args=args)
    node = DecisionLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
```

Add to `setup.py` `console_scripts`: `'decision_logger = proximity_alert.decision_logger:main',`.

- [ ] **Step 4: Run test to verify it passes**

Run (in container, after `colcon build`): same pytest command as Step 2.
Expected: PASS (1 passed).

- [ ] **Step 5: Commit**

```bash
git add proximity_alert/proximity_alert/decision_logger.py proximity_alert/setup.py proximity_alert/test/test_decision_logger_csv.py
git commit -m "feat: decision_logger node writing a separate decision-log CSV"
```

---

### Task 8: Integrate `AvoidanceController` into `path_tracker`

**Files:**
- Modify: `proximity_alert/proximity_alert/path_tracker.py`
- (No new unit test — logic is covered by Tasks 1–6; this task is wiring + a safe live smoke test.)

**Interfaces:**
- Consumes: `reduce_to_sectors`, `size_obstacle`, `select_gap` (Tasks 1–3); `AvoidanceController`, `AvoidanceConfig` (Tasks 5–6); `DecisionRecord` (Task 4).
- Produces: publishes `cmd_vel` (`Twist`), `/forward_min_range` (`Float32`), `/avoidance_decision` (`String`).

- [ ] **Step 1: Rewrite the node body**

Replace the `PathTracker` state-machine internals with: declare every `AvoidanceConfig` field as a ROS parameter (defaults from the dataclass); build `AvoidanceConfig` from parameters; construct `AvoidanceController`. In `scan_callback`, call `reduce_to_sectors` + `size_obstacle` + `select_gap` and cache them; still publish `/forward_min_range` from `sectors["front"]`. In `control_loop` (unchanged 10 Hz timer), call `controller.step(sectors, obstacle, gap_bearing, current_yaw, now)`, publish the returned `Twist`, and if `out.decision is not None` publish `String(data=out.decision.to_json())` on `/avoidance_decision`. Capture `goal_heading` from the first odom yaw via `controller.set_goal_heading(...)`. Keep `disable_avoidance` (route obstacle → HALT). Preserve the flag-based signal handler and `publish_stop()` verbatim (commit `e64c4c2`).

- [ ] **Step 2: Compile + build**

Run:
```bash
python3 -m py_compile proximity_alert/proximity_alert/path_tracker.py
docker exec MentorPi bash -lc "rm -rf /home/ubuntu/ros2_ws/src/proximity_alert /home/ubuntu/ros2_ws/build/proximity_alert /home/ubuntu/ros2_ws/install/proximity_alert"
docker cp proximity_alert MentorPi:/home/ubuntu/ros2_ws/src/proximity_alert
docker exec MentorPi bash -lc "source /opt/ros/humble/setup.bash && cd /home/ubuntu/ros2_ws && colcon build --packages-select proximity_alert"
```
Expected: build succeeds; `grep -c AvoidanceController` in the installed `path_tracker.py` is ≥ 1.

- [ ] **Step 3: Run full unit suite in container**

Run: `docker exec -u ubuntu -w /home/ubuntu/ros2_ws/src/proximity_alert MentorPi bash -lc "source /opt/ros/humble/setup.bash && source /home/ubuntu/ros2_ws/install/setup.bash && python3 -m pytest test/ -v"`
Expected: all tests PASS.

- [ ] **Step 4: Safe live smoke test (remapped, NO motion)**

Pre-flight: confirm `ros2 topic info /cmd_vel --verbose` shows 0 publishers; `/scan_raw` at ~10 Hz.
Run `path_tracker` with `-r /cmd_vel:=/cmd_vel_test`, and in parallel run `decision_logger` with a `/tmp` csv_path. Wave a box in front; confirm: (a) `/forward_min_range` publishes, (b) `/avoidance_decision` emits JSON on obstacle, (c) the decision CSV gains rows, (d) `/cmd_vel_test` shows a strafe (`linear.y != 0`) for a narrow obstacle with a clear side. Then `pkill -TERM -f path_tracker` and verify 0 `/cmd_vel` publishers.

- [ ] **Step 5: Commit**

```bash
git add proximity_alert/proximity_alert/path_tracker.py
git commit -m "feat: drive path_tracker from the AvoidanceController state machine"
```

---

### Task 9: Documentation + parameters

**Files:**
- Modify: `README.md`

- [ ] **Step 1: Update README**

Add a "Refined obstacle avoidance" subsection summarizing the 7-state machine and the governing invariant; replace the `path_tracker` parameter table with the full §10 tunables (mark derived ones); document the **two separate CSVs** and their host paths (`trial_logger` → `/home/pi/docker/tmp/trials/<surface>.csv`; `decision_logger` → `/home/pi/docker/tmp/trials/decision_log.csv`); add the decision-log column list for Haotian; add a run example that launches `path_tracker`, `trial_logger`, and `decision_logger` (all `-u ubuntu`, csv_path under `/home/ubuntu/shared/trials/`).

- [ ] **Step 2: Commit**

```bash
git add README.md
git commit -m "docs: document refined avoidance, params, and the two trial CSVs"
```

---

## Self-Review

**Spec coverage:** §2 sensing → Task 1; §4 measurable ladder → Tasks 2,5; §7 recovery/gaps → Tasks 3,6; §3/§5/§8 state machine + completion + anti-oscillation → Tasks 5,6; §6 debounce → Task 5; §9 decision logging → Tasks 4,7; §10 params → Tasks 5,8,9; §11 edge cases → covered by controller tests (wide obstacle, blocked sides, no gap, symmetric via preferred_side default); §12 improvements → realized across Tasks 1–8. Host-path + separate-file requirements → Tasks 7,8,9. No gaps.

**Placeholder scan:** Tasks 5, 6, and 8 describe extending `step()` in prose plus concrete anchors (signatures, field names, exact transitions, real tests) rather than a full listing, because the method is long and grows across two tasks — the tests are the executable contract. Each such step names exact fields, states, and thresholds; no "TBD/add error handling/similar-to" placeholders remain.

**Type consistency:** `reduce_to_sectors` dict keys (`front/front_left/front_center/front_right/left/right/rear`) are consumed unchanged in Task 5 tests and Task 8. `size_obstacle` keys (`span_deg/y_lo/y_hi/preferred_side`) match Task 5 usage. `select_gap` returns a bearing float consumed by RECOVER (Task 6) and passed from Task 8. `DecisionRecord` field set is identical in Tasks 4, 7, and the controller records. `ControllerOutput` fields (`linear_x/linear_y/angular_z/state/decision`) match Task 8's publish code.

**Review fixes incorporated (2026-07-23):**
1. **Gap wrap-around** — `select_gap` now merges the first+last clear runs so a gap straddling ±π (behind the robot) is one gap, not two rejected slivers (Task 3, `test_wraps_gap_straddling_pi_behind_robot`).
2. **Finite guard** — every scan loop gates on `math.isfinite(r)` in addition to `range_min/range_max` (Global Constraints; Tasks 1–3). Defense-in-depth: this LD19's finite `range_max=25.0` already drops NaN/±Inf, but this hardens against reconfiguration.
3. **Encounter lifecycle** — `encounter_id` increment and counter resets are now both gated by the single `clear_drive_duration` cooldown, so a blip mid-cooldown resumes the same encounter and a post-cooldown detection starts a new one (Task 5 spec; Task 6 `test_blip_during_cooldown_resumes_same_encounter`, `test_detection_after_cooldown_is_new_encounter`).
4. **Index-based angles** — all new loops use `msg.angle_min + i * msg.angle_increment` (Global Constraints; Tasks 1–3). The already-merged `scan_utils.min_range_in_forward_arc` is left unchanged (tested, and its float64 accumulation drift of ~1e-12 rad is ~11 orders of magnitude below beam spacing) — a follow-up may align it for consistency, out of scope here.

---

## Execution Handoff

(Provided after the plan is reviewed.)
