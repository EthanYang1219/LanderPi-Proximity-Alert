# Fixed-Distance Point A-to-B Stop Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give `path_tracker.py` a `target_distance` parameter that stops the robot after a set straight-line distance from its start position, layered on top of the existing `AvoidanceController` without modifying it.

**Architecture:** A new pure module `nav_utils.py` owns all arrival rules as unit-testable functions. `path_tracker.py` tracks its distance from start by extending the `/odom` callback it already has, and gates `control_loop()` on the arrival decision — the `AvoidanceController` state machine is stepped and its output is either forwarded or replaced with a stop, but the machine itself is never bypassed or altered. A new `/path_tracker/status` topic reports which mechanism is currently governing the robot.

**Tech Stack:** ROS 2 Humble, rclpy, Python 3, pytest.

**Priority context:** The odometry-slippage dataset Haotian needs is already collected. This change is about **functionality**, not data gathering. Tasks 1–3 make the feature work and are the deliverable. Task 4 (the `arrival_reason` CSV column) is optional plumbing and can be skipped entirely without affecting the feature.

## Global Constraints

- ROS 2 Humble compatible, `rclpy` only.
- Do not invent topics or message types. Confirmed-real only: `/scan_raw`, `/odom`, `/cmd_vel`, `/forward_min_range`, `/avoidance_decision`.
- **`avoidance.py` must not be modified by this plan.** All new logic lives in the `path_tracker` ROS wrapper.
- With `target_distance` at its default `0.0`, behavior must be identical to today: no distance tracking, no odom watchdog, no new stop condition.
- Use the ROS clock (`self.get_clock().now()`), never `time.time()` — the node already does this and mixing the two breaks `use_sim_time`.
- Modules go in `proximity_alert/proximity_alert/`, tests in `proximity_alert/test/`, imports use the `proximity_alert.` prefix.
- Every task ends with the **full suite green** (74 tests as of `a76ab2b`, plus whatever the task adds), not just the task's own tests.

## Test Environment

`rclpy` is **not importable on the host** — `.ros2_stubs/` is a Python-only copy missing the `_rclpy_pybind11` C extension. Task 1 is pure Python and runs on the host:

```bash
cd proximity_alert && python3 -m pytest test/test_nav_utils.py -v
```

Tasks 2–4 need the container. `docker exec` **must** use `-u ubuntu` — running as root breaks FastRTPS shared memory silently:

```bash
docker exec -u ubuntu MentorPi bash -c 'rm -rf /tmp/pa_test && mkdir -p /tmp/pa_test'
docker cp proximity_alert MentorPi:/tmp/pa_test/
docker exec -u root MentorPi chown -R ubuntu:ubuntu /tmp/pa_test
docker exec -u ubuntu MentorPi bash -c \
  'source /opt/ros/humble/setup.bash && cd /tmp/pa_test/proximity_alert && \
   PYTHONPATH=/tmp/pa_test/proximity_alert:$PYTHONPATH python3 -m pytest test/ -q'
```

Re-run the `docker cp` after every edit — the container has a copy, not a mount.

## Current-State Facts

Verified against `path_tracker.py` and `avoidance.py` at `a76ab2b`. An engineer new to this codebase will guess wrong on all of these:

- `path_tracker` is a thin ROS wrapper. `scan_callback` **only** reduces the scan (`reduce_to_sectors` / `size_obstacle` / `select_gap`) and stores the result. **`control_loop()`, on a 20 Hz timer, is what publishes `/cmd_vel`.** Do not put driving decisions in `scan_callback`.
- `/odom` is **already subscribed** (`path_tracker.py:105`, via the `odom_topic` parameter). `odom_callback` (`:162`) sets `current_yaw`, latches `goal_heading_abs`, and calls `controller.set_goal_heading()`. **Extend it; never replace it** — deleting those lines silently kills heading hold.
- Tuning lives on `self.config` (an `AvoidanceConfig`), not as node attributes. There is no `self.safety_distance`. The publisher is `self.cmd_vel_pub`, not `self.cmd_pub`.
- `AvoidanceController.state` is a public attribute. `DRIVE`, `STRAFE`, `TURN`, `HALT` are importable module constants from `proximity_alert.avoidance`.
- **`HALT` is recoverable, not terminal.** `_halt()` (`avoidance.py:350`) transitions back to `DRIVE` once the front stays clear for `clear_confirm_time`. The controller **must keep being stepped while in `HALT`** or the robot freezes forever.
- **Do not force `controller.state` in tests.** `_maneuver_start` is `None` until `_assess()` runs, so a forced `STRAFE`/`TURN` crashes with `TypeError`. Task 3 uses a fake controller instead.
- `scan_is_stale()` (`:169`) is the established watchdog pattern — copy its shape for odom.

## Design Decisions

| Question | Decision | Rationale |
|---|---|---|
| What counts as "arrived by obstacle" | `controller.state == HALT` | The controller routinely detects obstacles and drives around them. Only `HALT` means it ran out of options. |
| Arrival hit mid-STRAFE/TURN | Defer until `state == DRIVE` | Stopping mid-maneuver leaves the robot sideways at a point that isn't B. Accepts slight overshoot. |
| Stale-odom stop | Fourth status value `stopped_odom_fault` | A sensor fault must not be indistinguishable from a normal drive. |
| Multiple trials per process | One process per trial; `arrived` latches | No reset plumbing. Documented in README. |

`/path_tracker/status` carries four values: `driving`, `arrived_target_distance`, `arrived_obstacle`, `stopped_odom_fault`.

## File Structure

| File | Responsibility |
|---|---|
| `proximity_alert/proximity_alert/nav_utils.py` (create) | Pure arrival rules: distance math, the priority ladder, and whether a tick can skip the controller. No ROS imports. Mirrors the `scan_utils.py` pattern. |
| `proximity_alert/proximity_alert/path_tracker.py` (modify) | Adds two parameters, distance/freshness tracking in the existing `odom_callback`, an odom watchdog, and arrival gating + status publishing in `control_loop`. |
| `proximity_alert/test/test_nav_utils.py` (create) | Task 1 — pure rules, host-runnable. |
| `proximity_alert/test/test_path_tracker_distance.py` (create) | Task 2 — odom tracking, plus a regression guard on heading hold. |
| `proximity_alert/test/test_path_tracker_arrival.py` (create) | Task 3 — gating behavior, via a fake controller. |
| `README.md` (modify) | Task 3 — new parameters, status values, one-process-per-trial rule. |

Tasks 2 and 3 both modify `path_tracker.py` but are split because Task 2 is pure observation (provably zero behavior change) and Task 3 is where behavior changes. A reviewer can accept the tracking and reject the gating.

---

### Task 1: Pure arrival-decision logic

**Files:**
- Create: `proximity_alert/proximity_alert/nav_utils.py`
- Test: `proximity_alert/test/test_nav_utils.py`

**Interfaces:**
- Consumes: `DRIVE`, `HALT` constants from `proximity_alert.avoidance` (already exist).
- Produces:
  - `DRIVING = "driving"`, `ARRIVED_TARGET = "arrived_target_distance"`, `ARRIVED_OBSTACLE = "arrived_obstacle"`, `STOPPED_ODOM_FAULT = "stopped_odom_fault"` — module constants.
  - `distance_from_start(start_x: float, start_y: float, current_x: float, current_y: float) -> float`
  - `decide_arrival(*, controller_state: str, target_distance: float, distance_traveled: float, odom_stale: bool, already_arrived: bool) -> str` — returns one of the four constants. Keyword-only.
  - `should_skip_controller(*, target_distance: float, odom_stale: bool, already_arrived: bool) -> bool` — keyword-only.

- [ ] **Step 1: Write the failing tests**

```python
# proximity_alert/test/test_nav_utils.py
from proximity_alert.avoidance import DRIVE, HALT, STRAFE, TURN
from proximity_alert.nav_utils import (
    ARRIVED_OBSTACLE, ARRIVED_TARGET, DRIVING, STOPPED_ODOM_FAULT,
    decide_arrival, distance_from_start, should_skip_controller,
)


def _decide(**kw):
    base = dict(controller_state=DRIVE, target_distance=0.0,
                distance_traveled=0.0, odom_stale=False, already_arrived=False)
    base.update(kw)
    return decide_arrival(**base)


# --- distance math ---

def test_distance_from_start_straight_line():
    assert distance_from_start(0.0, 0.0, 3.0, 4.0) == 5.0


def test_distance_from_start_zero_when_unmoved():
    assert distance_from_start(1.0, 1.0, 1.0, 1.0) == 0.0


def test_distance_from_start_is_unsigned_displacement():
    # Driving backwards still increases distance travelled -- this is
    # displacement magnitude, not signed progress along the goal axis.
    assert distance_from_start(0.0, 0.0, -2.0, 0.0) == 2.0


# --- backward compatibility: target_distance disabled ---

def test_disabled_target_never_arrives_however_far_it_drives():
    assert _decide(target_distance=0.0, distance_traveled=100.0) == DRIVING


def test_disabled_target_ignores_stale_odom():
    # No watchdog existed before this feature; leaving target_distance at
    # its default must not introduce one.
    assert _decide(target_distance=0.0, odom_stale=True) == DRIVING


def test_disabled_target_still_reports_obstacle_halt():
    assert _decide(target_distance=0.0, controller_state=HALT) == ARRIVED_OBSTACLE


# --- target-distance arrival ---

def test_arrives_when_target_reached_while_driving():
    assert _decide(target_distance=2.0, distance_traveled=2.0) == ARRIVED_TARGET


def test_arrives_when_target_overshot():
    assert _decide(target_distance=2.0, distance_traveled=2.4) == ARRIVED_TARGET


def test_drives_when_below_target():
    assert _decide(target_distance=2.0, distance_traveled=1.9) == DRIVING


def test_arrival_deferred_mid_strafe():
    assert _decide(controller_state=STRAFE, target_distance=2.0,
                   distance_traveled=5.0) == DRIVING


def test_arrival_deferred_mid_turn():
    assert _decide(controller_state=TURN, target_distance=2.0,
                   distance_traveled=5.0) == DRIVING


def test_arrival_latch_survives_later_ticks():
    assert _decide(already_arrived=True, target_distance=2.0,
                   distance_traveled=0.0) == ARRIVED_TARGET


# --- obstacle halt ---

def test_halt_reports_arrived_obstacle_not_target():
    # Controller gave up well before the target was reached.
    assert _decide(controller_state=HALT, target_distance=5.0,
                   distance_traveled=1.0) == ARRIVED_OBSTACLE


def test_target_reached_wins_over_halt_only_from_drive():
    # Reaching the target during a HALT does not count as arriving at B:
    # the robot is stopped by an obstacle, not by the odometer.
    assert _decide(controller_state=HALT, target_distance=2.0,
                   distance_traveled=2.5) == ARRIVED_OBSTACLE


# --- stale-odom watchdog ---

def test_stale_odom_stops_when_target_set():
    assert _decide(target_distance=2.0, odom_stale=True,
                   distance_traveled=0.5) == STOPPED_ODOM_FAULT


def test_stale_odom_wins_over_untrustworthy_distance():
    # distance_traveled is derived from odom, so stale odom means the
    # distance is stale too and must not be trusted to declare arrival.
    assert _decide(target_distance=2.0, odom_stale=True,
                   distance_traveled=99.0) == STOPPED_ODOM_FAULT


def test_latched_arrival_outranks_stale_odom():
    assert _decide(already_arrived=True, target_distance=2.0,
                   odom_stale=True) == ARRIVED_TARGET


# --- controller-skip predicate ---

def test_skip_controller_when_arrived():
    assert should_skip_controller(target_distance=2.0, odom_stale=False,
                                  already_arrived=True) is True


def test_skip_controller_when_stale_and_target_set():
    assert should_skip_controller(target_distance=2.0, odom_stale=True,
                                  already_arrived=False) is True


def test_no_skip_when_stale_but_target_disabled():
    assert should_skip_controller(target_distance=0.0, odom_stale=True,
                                  already_arrived=False) is False


def test_no_skip_when_driving_normally():
    assert should_skip_controller(target_distance=2.0, odom_stale=False,
                                  already_arrived=False) is False
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd proximity_alert && python3 -m pytest test/test_nav_utils.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'proximity_alert.nav_utils'`

- [ ] **Step 3: Write the implementation**

```python
# proximity_alert/proximity_alert/nav_utils.py
"""Pure, ROS-free arrival logic for the fixed-distance A-to-B stop.

Kept separate from path_tracker (and from the AvoidanceController it wraps)
so the priority rules are unit-testable without a live node, following the
same pattern as scan_utils.py.
"""
import math

from proximity_alert.avoidance import DRIVE, HALT

DRIVING = "driving"
ARRIVED_TARGET = "arrived_target_distance"
ARRIVED_OBSTACLE = "arrived_obstacle"
STOPPED_ODOM_FAULT = "stopped_odom_fault"


def distance_from_start(start_x, start_y, current_x, current_y):
    """Straight-line displacement from the start pose.

    Deliberately the same math.hypot(dx, dy) trial_logger uses for
    odom_distance_m, so the two nodes agree on "distance travelled".

    Note this is DISPLACEMENT, not path length -- a sideways strafe counts,
    and driving backwards increases it. With heading hold engaged the two
    are close, but they diverge after an avoidance maneuver.
    """
    return math.hypot(current_x - start_x, current_y - start_y)


def decide_arrival(*, controller_state, target_distance, distance_traveled,
                   odom_stale, already_arrived):
    """Which stop condition (if any) governs this control tick.

    Priority, highest first:

    1. Already arrived -- latched, so the robot stays stopped. Arrival can
       only latch from DRIVE (rule 3), so this can never mask an obstacle
       halt that was already in progress.
    2. Stale odom while target_distance is set -- distance_traveled is
       derived from odom, so stale odom means the distance is untrustworthy.
       Stop rather than drive blind or declare a bogus arrival.
    3. Target reached AND controller is in DRIVE -- deferring until DRIVE
       lets a strafe/turn finish, so the robot ends square and on-heading
       instead of frozen sideways mid-maneuver. Costs a little overshoot.
    4. Controller in HALT -- it exhausted its avoidance options, so an
       obstacle is what stopped this run.
    5. Otherwise, driving.

    target_distance <= 0.0 disables rules 2 and 3 entirely, preserving
    today's behavior exactly.
    """
    if already_arrived:
        return ARRIVED_TARGET

    if target_distance > 0.0:
        if odom_stale:
            return STOPPED_ODOM_FAULT
        if controller_state == DRIVE and distance_traveled >= target_distance:
            return ARRIVED_TARGET

    if controller_state == HALT:
        return ARRIVED_OBSTACLE

    return DRIVING


def should_skip_controller(*, target_distance, odom_stale, already_arrived):
    """True when this tick is decided without consulting the state machine.

    Both cases are terminal stops, and ticking the controller through them
    would keep advancing its maneuver timers against motion that is not
    happening. Every other case -- including HALT, which is recoverable and
    needs stepping to notice the path has cleared -- must still step.

    Mirrors rules 1 and 2 of decide_arrival; keep the two in sync.
    """
    return already_arrived or (target_distance > 0.0 and odom_stale)
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd proximity_alert && python3 -m pytest test/test_nav_utils.py -v`
Expected: 21 passed

- [ ] **Step 5: Commit**

```bash
git add proximity_alert/proximity_alert/nav_utils.py proximity_alert/test/test_nav_utils.py
git commit -m "feat: add pure arrival-decision logic for fixed-distance A-to-B stop"
```

---

### Task 2: Track distance from start in `path_tracker.py`

Observation only — no behavior change. Split from Task 3 so the tracking is proven correct before anything gates on it.

**Files:**
- Modify: `proximity_alert/proximity_alert/path_tracker.py` (`__init__`, `odom_callback:162`, new `odom_is_stale`)
- Test: `proximity_alert/test/test_path_tracker_distance.py`

**Interfaces:**
- Consumes: `distance_from_start` from Task 1.
- Produces: node attributes `self.target_distance: float`, `self.odom_timeout_sec: float`, `self.start_pos: tuple[float, float] | None`, `self.distance_traveled: float`, `self.last_odom_time`, `self.arrived: bool`; method `odom_is_stale() -> bool`.

- [ ] **Step 1: Write the failing tests**

```python
# proximity_alert/test/test_path_tracker_distance.py
import rclpy
from nav_msgs.msg import Odometry

from proximity_alert.path_tracker import PathTracker


def _odom_at(x, y):
    msg = Odometry()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.w = 1.0
    return msg


def test_target_distance_defaults_to_disabled():
    rclpy.init()
    node = PathTracker()
    target, timeout = node.target_distance, node.odom_timeout_sec
    node.destroy_node()
    rclpy.shutdown()
    assert target == 0.0
    assert timeout == 1.0


def test_distance_starts_at_zero():
    rclpy.init()
    node = PathTracker()
    d, arrived = node.distance_traveled, node.arrived
    node.destroy_node()
    rclpy.shutdown()
    assert d == 0.0
    assert arrived is False


def test_tracks_distance_from_first_odom():
    rclpy.init()
    node = PathTracker()
    node.odom_callback(_odom_at(10.0, 10.0))   # start is wherever it boots
    node.odom_callback(_odom_at(13.0, 14.0))
    d = node.distance_traveled
    node.destroy_node()
    rclpy.shutdown()
    assert d == 5.0


def test_start_position_does_not_drift():
    # start_pos must be a snapshot, not a reference into the last message.
    rclpy.init()
    node = PathTracker()
    node.odom_callback(_odom_at(1.0, 0.0))
    node.odom_callback(_odom_at(4.0, 0.0))
    node.odom_callback(_odom_at(7.0, 0.0))
    d = node.distance_traveled
    node.destroy_node()
    rclpy.shutdown()
    assert d == 6.0     # 7 - 1, not 7 - 4


def test_odom_callback_still_sets_goal_heading():
    # Regression guard: distance tracking must not displace the existing
    # heading-hold latch, which is what keeps the robot driving straight.
    rclpy.init()
    node = PathTracker()
    node.odom_callback(_odom_at(0.0, 0.0))
    goal_set, yaw = node._goal_set, node.current_yaw
    controller_goal = node.controller.goal_heading
    node.destroy_node()
    rclpy.shutdown()
    assert goal_set is True
    assert yaw is not None
    assert controller_goal is not None


def test_odom_freshness_tracked():
    rclpy.init()
    node = PathTracker()
    stale_before = node.odom_is_stale()
    node.odom_callback(_odom_at(0.0, 0.0))
    stale_after = node.odom_is_stale()
    node.destroy_node()
    rclpy.shutdown()
    assert stale_before is True     # nothing received yet
    assert stale_after is False
```

- [ ] **Step 2: Run the tests to verify they fail**

Copy to the container (see Test Environment), then run:
`python3 -m pytest test/test_path_tracker_distance.py -v`
Expected: FAIL with `AttributeError: 'PathTracker' object has no attribute 'target_distance'`

- [ ] **Step 3: Write the implementation**

Add the import beside the existing `from proximity_alert.avoidance import ...` line:

```python
from proximity_alert.nav_utils import distance_from_start
```

In `__init__`, immediately after the existing `self.scan_timeout = ...` line:

```python
        # Fixed-distance A-to-B stop. 0.0 disables it entirely, so a node
        # launched without this parameter behaves exactly as it did before.
        self.declare_parameter("target_distance", 0.0)
        self.declare_parameter("odom_timeout_sec", 1.0)
        self.target_distance = self.get_parameter("target_distance").value
        self.odom_timeout_sec = self.get_parameter("odom_timeout_sec").value
```

With the other internal state, beside `self.current_yaw = None`:

```python
        self.start_pos = None
        self.distance_traveled = 0.0
        self.last_odom_time = None
        self.arrived = False
```

**Extend** `odom_callback` — keep every existing line exactly as-is and append:

```python
    def odom_callback(self, msg: Odometry):
        self.current_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        if not self._goal_set:
            self.goal_heading_abs = self.current_yaw
            self.controller.set_goal_heading(self.current_yaw)
            self._goal_set = True

        # Straight-line distance from wherever the node started. Stored as
        # plain floats rather than the message's position object, which is
        # reused/overwritten by the middleware.
        pos = msg.pose.pose.position
        if self.start_pos is None:
            self.start_pos = (pos.x, pos.y)
        self.distance_traveled = distance_from_start(
            self.start_pos[0], self.start_pos[1], pos.x, pos.y
        )
        self.last_odom_time = self.get_clock().now()
```

Add a watchdog mirroring `scan_is_stale()`, directly beneath it:

```python
    def odom_is_stale(self):
        if self.last_odom_time is None:
            return True
        age = (self.get_clock().now() - self.last_odom_time).nanoseconds / 1e9
        return age > self.odom_timeout_sec
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `python3 -m pytest test/test_path_tracker_distance.py -v` → Expected: 6 passed
Then the full suite: `python3 -m pytest test/ -q` → Expected: 80 passed

- [ ] **Step 5: Commit**

```bash
git add proximity_alert/proximity_alert/path_tracker.py \
        proximity_alert/test/test_path_tracker_distance.py
git commit -m "feat: track straight-line distance from start in path_tracker"
```

---

### Task 3: Gate `control_loop` on arrival and publish `/path_tracker/status`

**Files:**
- Modify: `proximity_alert/proximity_alert/path_tracker.py` (`__init__`, `control_loop:175`, new `_publish`)
- Modify: `README.md` (parameter table + a status-topic subsection)
- Test: `proximity_alert/test/test_path_tracker_arrival.py`

**Interfaces:**
- Consumes: `decide_arrival`, `should_skip_controller`, and the four status constants from Task 1; `self.distance_traveled` / `odom_is_stale()` / `self.arrived` from Task 2.
- Produces: topic `/path_tracker/status` (`std_msgs/String`); node attributes `self._last_cmd: Twist` and `self._last_status: str` recording the most recent publish so tests can assert without spinning an executor.

- [ ] **Step 1: Write the failing tests**

```python
# proximity_alert/test/test_path_tracker_arrival.py
import math

import rclpy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from proximity_alert.avoidance import DRIVE, HALT, STRAFE, ControllerOutput
from proximity_alert.path_tracker import PathTracker


class _FakeController:
    """Stands in for AvoidanceController so these tests exercise the gating
    logic, not the state machine.

    Forcing the real controller's `state` attribute directly raises
    TypeError -- `_maneuver_start` stays None until `_assess()` runs -- and
    driving it into STRAFE/HALT legitimately would take a fixture far
    larger than the behavior under test. The real controller is covered by
    test_avoidance_controller.py; what needs proving here is that
    path_tracker forwards or replaces its output correctly.
    """

    def __init__(self, state=DRIVE, linear_x=0.5):
        self.state = state
        self.goal_heading = None
        self.step_calls = 0
        self._out = ControllerOutput(linear_x, 0.0, 0.0, state, None)

    def set_goal_heading(self, yaw):
        self.goal_heading = yaw

    def step(self, sectors, obstacle, gap, yaw, now):
        self.step_calls += 1
        return self._out


def _clear_scan():
    scan = LaserScan()
    scan.angle_min = -math.pi
    scan.angle_max = math.pi
    scan.angle_increment = math.pi / 90
    scan.range_min = 0.05
    scan.range_max = 12.0
    scan.ranges = [5.0] * 180
    return scan


def _odom_at(x, y):
    msg = Odometry()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.w = 1.0
    return msg


def _primed(node, target_distance, controller=None):
    """Node with a fresh scan and a start odom fix, ready to drive."""
    node.target_distance = target_distance
    if controller is not None:
        node.controller = controller
    node.odom_callback(_odom_at(0.0, 0.0))
    node.scan_callback(_clear_scan())
    return node


# --- backward compatibility, real controller ---

def test_disabled_target_drives_forward_unchanged():
    rclpy.init()
    node = _primed(PathTracker(), 0.0)
    node.odom_callback(_odom_at(99.0, 0.0))
    node.control_loop()
    cmd, status = node._last_cmd, node._last_status
    node.destroy_node()
    rclpy.shutdown()
    assert cmd.linear.x > 0.0          # still driving after 99 m
    assert status == "driving"


def test_stale_odom_ignored_when_target_disabled():
    # No odom watchdog existed before this feature.
    rclpy.init()
    node = PathTracker()
    node.target_distance = 0.0
    node.scan_callback(_clear_scan())   # fresh scan, but no odom ever
    node.control_loop()
    cmd = node._last_cmd
    node.destroy_node()
    rclpy.shutdown()
    assert cmd.linear.x > 0.0


# --- target arrival, fake controller ---

def test_stops_and_reports_arrival_at_target():
    rclpy.init()
    node = _primed(PathTracker(), 2.0, _FakeController(state=DRIVE))
    node.odom_callback(_odom_at(2.5, 0.0))
    node.control_loop()
    cmd, status = node._last_cmd, node._last_status
    node.destroy_node()
    rclpy.shutdown()
    assert cmd.linear.x == 0.0
    assert cmd.angular.z == 0.0
    assert status == "arrived_target_distance"


def test_arrival_deferred_until_controller_returns_to_drive():
    rclpy.init()
    fake = _FakeController(state=STRAFE)
    node = _primed(PathTracker(), 2.0, fake)
    node.odom_callback(_odom_at(2.5, 0.0))
    node.control_loop()
    mid_status, mid_cmd = node._last_status, node._last_cmd
    fake.state = DRIVE                      # maneuver finished
    node.control_loop()
    after = node._last_status
    node.destroy_node()
    rclpy.shutdown()
    assert mid_status == "driving"          # did not abort the strafe
    assert mid_cmd.linear.x > 0.0           # controller output forwarded
    assert after == "arrived_target_distance"


def test_arrival_latches_and_stops_stepping_controller():
    rclpy.init()
    fake = _FakeController(state=DRIVE)
    node = _primed(PathTracker(), 2.0, fake)
    node.odom_callback(_odom_at(2.5, 0.0))
    node.control_loop()
    calls_at_arrival = fake.step_calls
    node.control_loop()
    node.control_loop()
    cmd, status, calls_after = node._last_cmd, node._last_status, fake.step_calls
    node.destroy_node()
    rclpy.shutdown()
    assert cmd.linear.x == 0.0
    assert status == "arrived_target_distance"
    # Latched: the state machine is not ticked against motion that is not
    # happening, so its maneuver timers cannot drift.
    assert calls_after == calls_at_arrival


# --- obstacle halt ---

def test_controller_halt_reports_arrived_obstacle():
    rclpy.init()
    node = _primed(PathTracker(), 5.0, _FakeController(state=HALT, linear_x=0.0))
    node.control_loop()
    status = node._last_status
    node.destroy_node()
    rclpy.shutdown()
    assert status == "arrived_obstacle"


def test_halt_is_still_stepped_so_it_can_recover():
    # HALT is recoverable -- _halt() returns to DRIVE once the front stays
    # clear. Skipping step() while halted would freeze the robot forever.
    rclpy.init()
    fake = _FakeController(state=HALT, linear_x=0.0)
    node = _primed(PathTracker(), 5.0, fake)
    node.control_loop()
    node.control_loop()
    calls = fake.step_calls
    node.destroy_node()
    rclpy.shutdown()
    assert calls == 2


# --- stale-odom watchdog ---

def test_stale_odom_stops_when_target_set():
    rclpy.init()
    fake = _FakeController(state=DRIVE)
    node = PathTracker()
    node.target_distance = 5.0
    node.controller = fake
    node.scan_callback(_clear_scan())   # fresh scan, but no odom ever
    node.control_loop()
    cmd, status = node._last_cmd, node._last_status
    node.destroy_node()
    rclpy.shutdown()
    assert cmd.linear.x == 0.0
    assert status == "stopped_odom_fault"
    assert fake.step_calls == 0         # not stepped on a fault stop


# --- stale scan still wins, unchanged ---

def test_stale_scan_still_holds():
    rclpy.init()
    node = PathTracker()
    node.target_distance = 2.0
    node.odom_callback(_odom_at(0.0, 0.0))   # odom fresh, scan never arrived
    node.control_loop()
    cmd = node._last_cmd
    node.destroy_node()
    rclpy.shutdown()
    assert cmd.linear.x == 0.0
```

- [ ] **Step 2: Run the tests to verify they fail**

Copy to the container, then run:
`python3 -m pytest test/test_path_tracker_arrival.py -v`
Expected: FAIL with `AttributeError: 'PathTracker' object has no attribute '_last_cmd'`

- [ ] **Step 3: Write the implementation**

Extend the `nav_utils` import added in Task 2:

```python
from proximity_alert.nav_utils import (
    ARRIVED_TARGET, DRIVING, STOPPED_ODOM_FAULT,
    decide_arrival, distance_from_start, should_skip_controller,
)
```

In `__init__`, beside the existing `self.decision_pub = ...` line:

```python
        self.status_pub = self.create_publisher(String, "/path_tracker/status", 10)
```

With the other internal state:

```python
        self._last_cmd = Twist()
        self._last_status = DRIVING
```

Replace `control_loop` entirely and add `_publish` beneath it. The stale-scan guard and the `controller.step()` call are unchanged from today:

```python
    def control_loop(self):
        if self.scan_is_stale() or self._sectors is None:
            # No fresh obstacle data -> hold still rather than drive blind.
            self.get_logger().warn(
                "No fresh scan within scan_timeout; holding.",
                throttle_duration_sec=1.0,
            )
            self._publish(Twist(), DRIVING)
            return

        # Two stops are decided without the state machine. Ticking it
        # through them would keep advancing its maneuver timers against
        # motion that is not happening. Note HALT is NOT one of them -- it
        # is recoverable, and needs stepping to notice the path has cleared.
        if should_skip_controller(
            target_distance=self.target_distance,
            odom_stale=self.odom_is_stale(),
            already_arrived=self.arrived,
        ):
            if self.arrived:
                self._publish(Twist(), ARRIVED_TARGET)
            else:
                self.get_logger().warn(
                    "No fresh odom within odom_timeout_sec but target_distance "
                    "is set; stopping rather than driving on a stale distance.",
                    throttle_duration_sec=1.0,
                )
                self._publish(Twist(), STOPPED_ODOM_FAULT)
            return

        now = self.get_clock().now().nanoseconds / 1e9
        yaw = self.current_yaw if self.current_yaw is not None else 0.0
        out = self.controller.step(self._sectors, self._obstacle, self._gap, yaw, now)

        # Arrival is decided AFTER the controller runs and only gates whether
        # its output is forwarded. The state machine is never bypassed or
        # altered, so avoidance behaves exactly as it did before.
        status = decide_arrival(
            controller_state=self.controller.state,
            target_distance=self.target_distance,
            distance_traveled=self.distance_traveled,
            odom_stale=self.odom_is_stale(),
            already_arrived=self.arrived,
        )

        if status == ARRIVED_TARGET:
            self.arrived = True
            self.get_logger().info(
                f"Target distance {self.target_distance:.2f} m reached "
                f"({self.distance_traveled:.2f} m travelled) -- stopping."
            )
            self._publish(Twist(), status)
            return

        cmd = Twist()
        cmd.linear.x = float(out.linear_x)
        cmd.linear.y = float(out.linear_y)
        cmd.angular.z = float(out.angular_z)
        self._publish(cmd, status)

        if out.decision is not None:
            self.decision_pub.publish(String(data=out.decision.to_json()))
            if self.audio_alert_enabled:
                self._maybe_play_audio_alert(out.decision)

    def _publish(self, cmd, status):
        """Single exit point for every tick, so /cmd_vel and the status topic
        can never disagree about what the robot is doing."""
        self._last_cmd, self._last_status = cmd, status
        self.cmd_vel_pub.publish(cmd)
        self.status_pub.publish(String(data=status))
```

Extend the startup log line with `target_distance={self.target_distance}`.

- [ ] **Step 4: Run the tests to verify they pass**

Run: `python3 -m pytest test/test_path_tracker_arrival.py -v` → Expected: 9 passed
Then the full suite: `python3 -m pytest test/ -q` → Expected: 89 passed

- [ ] **Step 5: Update the README**

Add to the `path_tracker` parameter table, after `disable_avoidance`:

```markdown
| `target_distance` | `0.0` | Straight-line distance from the start position, in meters, after which the robot stops and reports `arrived_target_distance`. `0.0` disables the whole feature — no distance tracking and no odom watchdog, exactly as before this parameter existed. Arrival is deferred until the avoidance state machine is back in `DRIVE`, so a strafe or turn finishes before the stop (costs a little overshoot) |
| `odom_timeout_sec` | `1.0` | Odometry watchdog, seconds. If `target_distance` is set and no `/odom` message arrives within this window, the robot stops and reports `stopped_odom_fault` rather than driving on a stale distance estimate. Ignored entirely when `target_distance` is `0.0` |
```

Add a subsection after the parameter table:

```markdown
### Arrival status (`/path_tracker/status`)

`path_tracker` publishes a `std_msgs/String` on `/path_tracker/status` every
control tick, reporting what is currently governing the robot:

| Value | Meaning |
|---|---|
| `driving` | Driving forward, or mid-avoidance-maneuver |
| `arrived_target_distance` | `target_distance` reached while in `DRIVE`; stopped and latched |
| `arrived_obstacle` | The avoidance state machine reached `HALT` — it ran out of options. Note this is recoverable: if the obstacle is removed and the path stays clear, it returns to `driving` |
| `stopped_odom_fault` | `/odom` went stale while `target_distance` was set; stopped as a precaution |

**One `path_tracker` process per trial.** Arrival latches permanently — once
`arrived_target_distance` is reached the node stays stopped and will not
drive again. Ctrl-C and relaunch between runs. This is deliberate: an
auto-reset could be tripped by nudging the robot between trials.
```

- [ ] **Step 6: Commit**

```bash
git add proximity_alert/proximity_alert/path_tracker.py \
        proximity_alert/test/test_path_tracker_arrival.py README.md
git commit -m "feat: stop at target_distance and publish /path_tracker/status"
```

---

### Task 4 (OPTIONAL — defer): `arrival_reason` CSV column

**Skip unless a trial run actually needs it.** The slippage dataset is already collected; this is descriptive metadata for future runs and delivers nothing for the functionality goal.

**Files:**
- Modify: `proximity_alert/proximity_alert/trial_logger.py`
- Modify: `proximity_alert/proximity_alert/floor_test_reconcile.py`
- Test: `proximity_alert/test/test_trial_logger_arrival_reason.py`

**Constraints an implementer will get wrong without reading this:**

- `CSV_HEADER` is a **module-level 12-column constant** (`trial_logger.py:78`): `timestamp, surface, trial_num, transit_time_s, odom_distance_m, ground_truth_distance_m, slippage_error_m, slippage_pct, avoidance_events, lidar_stop_range_m, notes, battery_level`. Do not drop `avoidance_events`, `notes`, or `battery_level`.
- `arrival_reason` goes **at the very end**, after `battery_level` — *not* after `lidar_stop_range_m`. The backward-compat logic at `trial_logger.py:209` assumes newer columns are last; inserting mid-list misaligns every already-collected trial CSV.
- `_append_row` takes 8 parameters today. Add a 9th; do not rewrite the signature.
- Preserve the blank-not-`inf` handling for non-finite ranges (`trial_logger.py:248`).
- `floor_test_reconcile.py` parses this CSV too — update its record parsing and `test_floor_test_reconcile.py` in the same commit, or its tests break.

Subscribe to `/path_tracker/status` (`std_msgs/String` — extend the existing `from std_msgs.msg import Float32, UInt16` line), store `self.last_status` defaulting to `"driving"`, and write it at the existing stop-detection point.

---

## Self-Review

**Spec coverage:**

| Spec requirement | Task |
|---|---|
| `target_distance` parameter, default `0.0`, backward compatible | 1 (`decide_arrival` short-circuit), 2 (`test_target_distance_defaults_to_disabled`), 3 (`test_disabled_target_drives_forward_unchanged`, `test_stale_odom_ignored_when_target_disabled`) |
| Track distance from start via `/odom`, same `math.hypot` as `trial_logger` | 1 (`distance_from_start`), 2 |
| Obstacle avoidance unchanged | 3 — arrival gates the controller's *output*; `avoidance.py` untouched. Regression guards: `test_odom_callback_still_sets_goal_heading` (2), `test_halt_is_still_stepped_so_it_can_recover` (3) |
| Stop-condition priority | 1 (`decide_arrival`, tested directly) |
| `/path_tracker/status` topic | 3 |
| `odom_timeout_sec` watchdog | 2 (`odom_is_stale`), 3 (`test_stale_odom_stops_when_target_set`) |
| `arrival_reason` CSV column | 4, deferred |
| Calibration step | Manual pre-trial process, no code. Not needed for this functionality work; raise it when numeric trials resume. |

**Deviations from the spec** (all deliberate — the spec predates the `AvoidanceController` refactor):

1. **Four status values, not three.** `stopped_odom_fault` added; folding a sensor fault into `driving` makes a watchdog stop invisible.
2. **Arrival is evaluated in `control_loop()`, not `scan_callback`.** `scan_callback` no longer makes driving decisions.
3. **Arrival requires `controller.state == DRIVE`.** The spec's flat priority ladder would stop the robot mid-strafe.
4. **"Arrived by obstacle" means `HALT`, not "obstacle inside `safety_distance`".** The controller routinely detects and drives around obstacles.
5. **Task 4 deferred.** Data collection is complete; this change is for functionality.

**Placeholder scan:** No TBDs, no "add error handling", no "similar to Task N". Every code step has literal code.

**Type consistency:** `decide_arrival` and `should_skip_controller` are keyword-only in Task 1's implementation and called keyword-only in Task 3. The four status constants are defined once in Task 1 and imported by name in Task 3; test assertions use the literal strings those constants hold. `distance_from_start` is positional in both its definition and its Task 2 call site. `self.arrived` is created in Task 2 and consumed in Task 3.

**Known limitation, accepted:** `distance_from_start` measures displacement, not path length — a sideways strafe counts toward `target_distance`. It matches `trial_logger`'s existing `odom_distance_m` metric, which is why it was kept; documented in the docstring rather than silently assumed. Switching to goal-axis projection is a one-line change now and an awkward migration later.
