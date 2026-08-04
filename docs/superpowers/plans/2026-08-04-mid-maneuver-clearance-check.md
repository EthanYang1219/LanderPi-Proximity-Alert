# Mid-Maneuver Clearance Check Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give `STRAFE` and `TURN` a live, debounced clearance check that aborts back to `_assess()` before contact, closing the gap that caused three real obstacle contacts on 2026-08-03/04.

**Architecture:** Each maneuver gains one additional per-tick check — a debounced OR of "front too close" / "the side being approached too close" — sharing one counter (`self._maneuver_abort_count`) since STRAFE and TURN never run concurrently. On the debounced condition firing, the maneuver returns `self._assess()`, the same reaction `_drive_past()` already uses for its own front-reblock check, so escalation through the existing `consecutive_avoid_count` → RECOVER → HALT ladder is automatic.

**Tech Stack:** Python 3, pytest, pure `avoidance.py` (no ROS dependencies).

## Global Constraints

Per the approved spec (`docs/superpowers/specs/2026-08-04-mid-maneuver-clearance-check-design.md`):

- `avoidance.py` remains pure — no `rclpy`/`sensor_msgs` imports.
- ASSESS's strafe-vs-turn decision logic, `DRIVE_PAST`, `RECOVER`, `HALT`, and encounter open/close are unchanged. This touches only `_strafe()`, `_turn()`, and the two STRAFE/TURN commit points inside `_assess()`.
- No new `AvoidanceConfig` fields — reuses `safety_distance`, `strafe_side_clearance_min`, `obstacle_confirm_scans`.
- The reaction to a detected danger is always `self._assess()` — never a new state, never immediate HALT.
- `self._maneuver_abort_count` is one shared counter (STRAFE/TURN are mutually exclusive states): reset to 0 on any tick the danger condition is false, and reset to 0 fresh at each new STRAFE/TURN commit inside `_assess()` (so an aborted maneuver's counter never carries into the next one).
- TURN's danger check runs *before* its heading-reached/timeout check, so a tick that is both "heading reached" and "dangerous" aborts to `_assess()` rather than handing off to `DRIVE_PAST`.
- The separate scan-vs-tick debounce weakness (found during this investigation: `control_rate_hz` exceeding the real ~10Hz LiDAR rate means "N consecutive ticks" isn't "N independent scans") is explicitly out of scope for this plan.

---

## File Structure

- `proximity_alert/proximity_alert/avoidance.py` — MODIFY. Add `self._maneuver_abort_count` to `__init__`; add the danger check to `_strafe()`; add the danger check to `_turn()`; add the counter reset at both STRAFE/TURN commit points inside `_assess()`.
- `proximity_alert/test/test_avoidance_controller.py` — MODIFY. Add STRAFE's abort tests (Task 1) and TURN's abort tests plus the cross-maneuver reset test (Task 2).

No new files. This is a two-function change sharing one piece of state, appropriately scoped as a single small plan.

---

## Task 1: STRAFE mid-maneuver abort

**Files:**
- Modify: `proximity_alert/proximity_alert/avoidance.py` (`__init__`, `_strafe()`, `_assess()`'s STRAFE commit branch)
- Test: `proximity_alert/test/test_avoidance_controller.py`

**Interfaces:**
- Consumes: existing `self._s` (sectors dict with `front`/`left`/`right`), `self._locked_dir`, `cfg.safety_distance`, `cfg.strafe_side_clearance_min`, `cfg.obstacle_confirm_scans`, `self._assess()`.
- Produces: `self._maneuver_abort_count` (int, also used by Task 2's `_turn()` changes).

- [ ] **Step 1: Write the failing tests**

Append to `proximity_alert/test/test_avoidance_controller.py`:

```python
def test_strafe_aborts_when_flank_closing_sustained():
    cfg = _cfg(obstacle_confirm_scans=2, strafe_speed=0.25, strafe_timeout=1.5)
    c = AvoidanceController(cfg)
    c.set_goal_heading(0.0)
    obst = {"span_deg": 10.0, "y_lo": -0.05, "y_hi": 0.05, "preferred_side": 1.0}
    front = _blocked_front(cfg)
    out = c.step(_sectors(front=front, fc=front, left=1.5), obst, None, 0.0, (0.0, 0.0), 0.0)
    assert out.state == "STRAFE"
    assert c.consecutive_avoid_count == 1
    # Flank closes below strafe_side_clearance_min (0.20) for obstacle_confirm_scans=2
    # consecutive ticks -- front stays above safety_distance so only the flank check
    # can be what fires.
    c.step(_sectors(front=0.25, left=0.15), obst, None, 0.0, (0.0, 0.0), 0.1)
    out = c.step(_sectors(front=0.25, left=0.15), obst, None, 0.0, (0.0, 0.0), 0.2)
    assert c.consecutive_avoid_count == 2  # _assess() ran again
    assert out.state == "TURN"  # side_clear=0.15 < strafe_side_clearance_min -> can't strafe


def test_strafe_aborts_when_front_closing_sustained():
    cfg = _cfg(obstacle_confirm_scans=2, strafe_speed=0.25, strafe_timeout=1.5)
    c = AvoidanceController(cfg)
    c.set_goal_heading(0.0)
    obst = {"span_deg": 10.0, "y_lo": -0.05, "y_hi": 0.05, "preferred_side": 1.0}
    front = _blocked_front(cfg)
    out = c.step(_sectors(front=front, fc=front, left=1.5), obst, None, 0.0, (0.0, 0.0), 0.0)
    assert out.state == "STRAFE"
    # Flank stays wide open so only the front check can fire.
    c.step(_sectors(front=0.15, left=1.5), obst, None, 0.0, (0.0, 0.0), 0.1)
    out = c.step(_sectors(front=0.15, left=1.5), obst, None, 0.0, (0.0, 0.0), 0.2)
    assert c.consecutive_avoid_count == 2
    assert out.state == "STRAFE"  # left is still wide open -> re-assess picks STRAFE again


def test_strafe_does_not_abort_on_single_close_reading():
    cfg = _cfg(obstacle_confirm_scans=2, strafe_speed=0.25, strafe_timeout=1.5)
    c = AvoidanceController(cfg)
    c.set_goal_heading(0.0)
    obst = {"span_deg": 10.0, "y_lo": -0.05, "y_hi": 0.05, "preferred_side": 1.0}
    front = _blocked_front(cfg)
    c.step(_sectors(front=front, fc=front, left=1.5), obst, None, 0.0, (0.0, 0.0), 0.0)
    out = c.step(_sectors(front=0.25, left=0.15), obst, None, 0.0, (0.0, 0.0), 0.1)
    assert out.state == "STRAFE"
    assert c.consecutive_avoid_count == 1  # no re-assess yet -- only one close tick


def test_strafe_abort_counter_resets_on_clear_tick():
    cfg = _cfg(obstacle_confirm_scans=2, strafe_speed=0.25, strafe_timeout=1.5)
    c = AvoidanceController(cfg)
    c.set_goal_heading(0.0)
    obst = {"span_deg": 10.0, "y_lo": -0.05, "y_hi": 0.05, "preferred_side": 1.0}
    front = _blocked_front(cfg)
    c.step(_sectors(front=front, fc=front, left=1.5), obst, None, 0.0, (0.0, 0.0), 0.0)
    c.step(_sectors(front=0.25, left=0.15), obst, None, 0.0, (0.0, 0.0), 0.1)    # close tick 1
    c.step(_sectors(front=0.25, left=1.5), obst, None, 0.0, (0.0, 0.0), 0.2)     # clear tick -- resets counter
    out = c.step(_sectors(front=0.25, left=0.15), obst, None, 0.0, (0.0, 0.0), 0.3)  # close tick (only 1 since reset)
    assert out.state == "STRAFE"
    assert c.consecutive_avoid_count == 1  # never reached 2 consecutive -- reset worked
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd proximity_alert && python3 -m pytest test/test_avoidance_controller.py -k "strafe_aborts or strafe_does_not_abort or strafe_abort_counter" -v`

Expected: FAIL. `test_strafe_aborts_when_flank_closing_sustained` and `test_strafe_aborts_when_front_closing_sustained` fail on `assert c.consecutive_avoid_count == 2` (stays at 1 — `_strafe()` currently has no abort path, so it just keeps strafing). `test_strafe_does_not_abort_on_single_close_reading` and `test_strafe_abort_counter_resets_on_clear_tick` currently pass vacuously (nothing aborts at all yet) — that's expected; they become meaningful once Step 3 adds real abort behavior for them to guard against.

- [ ] **Step 3: Implement**

In `AvoidanceController.__init__`, add the new counter next to the existing debounce counter:

```python
        self._confirm_count = 0
        self._maneuver_abort_count = 0
```

In `_assess()`, add a reset at the STRAFE commit point:

```python
        if strafe_ok:
            self._locked_dir = 1.0 if ps > 0 else -1.0
            self._maneuver_start = self._now
            self._maneuver_abort_count = 0
            self.state = STRAFE
```

Replace `_strafe()`:

```python
    def _strafe(self):
        s, cfg = self._s, self.config
        elapsed = self._now - self._maneuver_start
        if s["front"] >= cfg.clear_threshold:
            return self._complete_to_drive("cleared", elapsed)
        if elapsed >= cfg.strafe_timeout or self.cumulative_strafe >= cfg.max_cumulative_strafe:
            return self._assess()
        flank = s["left"] if self._locked_dir > 0 else s["right"]
        danger = s["front"] <= cfg.safety_distance or flank <= cfg.strafe_side_clearance_min
        if danger:
            self._maneuver_abort_count += 1
            if self._maneuver_abort_count >= cfg.obstacle_confirm_scans:
                return self._assess()
        else:
            self._maneuver_abort_count = 0
        self.cumulative_strafe += cfg.strafe_speed * self._dt
        ly = cfg.strafe_speed * self._locked_dir
        return ControllerOutput(0.0, ly, self._hold(self.goal_heading), STRAFE, None)
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd proximity_alert && python3 -m pytest test/test_avoidance_controller.py -k "strafe_aborts or strafe_does_not_abort or strafe_abort_counter" -v`

Expected: 4 passed.

- [ ] **Step 5: Run the full test suite to verify no regression**

Run: `cd proximity_alert && python3 -m pytest test/test_avoidance_controller.py -v`

Expected: all tests pass, including every pre-existing STRAFE test (`test_assess_chooses_strafe_for_narrow_obstacle_with_clear_side`, `test_strafe_completes_to_drive_when_front_clears`, the crab-correction tests in the `_strafe_then_clear`/`_strafe_cfg` block, etc.) — none of those touch the new danger condition's inputs in a way that should trip it, since they never leave `left`/`right` below their respective thresholds for `obstacle_confirm_scans` consecutive ticks.

- [ ] **Step 6: Commit**

```bash
git add proximity_alert/proximity_alert/avoidance.py proximity_alert/test/test_avoidance_controller.py
git commit -m "feat: abort STRAFE when flank or front clearance closes mid-maneuver"
```

---

## Task 2: TURN mid-maneuver abort

**Files:**
- Modify: `proximity_alert/proximity_alert/avoidance.py` (`_turn()`, `_assess()`'s TURN commit branch)
- Test: `proximity_alert/test/test_avoidance_controller.py`

**Interfaces:**
- Consumes: `self._maneuver_abort_count` (from Task 1), `self._s`, `self._locked_dir`, `cfg.safety_distance`, `cfg.obstacle_confirm_scans`, `self._assess()`.
- Produces: nothing new consumed elsewhere — this is the last task.

- [ ] **Step 1: Write the failing tests**

Append to `proximity_alert/test/test_avoidance_controller.py`:

```python
def test_turn_aborts_when_into_side_closing_sustained():
    cfg = _cfg(obstacle_confirm_scans=2)
    c = AvoidanceController(cfg)
    c.set_goal_heading(0.0)
    obst = {"span_deg": 80.0, "y_lo": -0.6, "y_hi": 0.6, "preferred_side": 1.0}
    front = _blocked_front(cfg)
    out = c.step(_sectors(front=front, fc=front, left=0.15, right=0.15), obst, None, 0.0, (0.0, 0.0), 0.0)
    assert out.state == "TURN"
    assert c.consecutive_avoid_count == 1
    # locked_dir chosen by left>=right with left==right -> locked_dir=+1 (turns toward
    # left), so the "into" side is left. front stays wide open so only the into-side
    # check can fire. yaw held at 0.0 across every tick (fake-clock test) so heading-
    # reached never fires and can't race with the danger check.
    c.step(_sectors(front=5.0, left=0.15), obst, None, 0.0, (0.0, 0.0), 0.1)
    out = c.step(_sectors(front=5.0, left=0.15), obst, None, 0.0, (0.0, 0.0), 0.2)
    assert c.consecutive_avoid_count == 2
    assert out.state == "TURN"  # still too wide (width=1.2 > max_obstacle_width) -> TURN again


def test_turn_aborts_when_front_closing_sustained():
    cfg = _cfg(obstacle_confirm_scans=2)
    c = AvoidanceController(cfg)
    c.set_goal_heading(0.0)
    obst = {"span_deg": 80.0, "y_lo": -0.6, "y_hi": 0.6, "preferred_side": 1.0}
    front = _blocked_front(cfg)
    out = c.step(_sectors(front=front, fc=front, left=0.15, right=0.15), obst, None, 0.0, (0.0, 0.0), 0.0)
    assert out.state == "TURN"
    # Sides stay wide open so only the front check can fire.
    c.step(_sectors(front=0.15, left=5.0, right=5.0), obst, None, 0.0, (0.0, 0.0), 0.1)
    out = c.step(_sectors(front=0.15, left=5.0, right=5.0), obst, None, 0.0, (0.0, 0.0), 0.2)
    assert c.consecutive_avoid_count == 2
    assert out.state == "TURN"  # still too wide -> TURN again


def test_turn_does_not_abort_on_single_close_reading():
    cfg = _cfg(obstacle_confirm_scans=2)
    c = AvoidanceController(cfg)
    c.set_goal_heading(0.0)
    obst = {"span_deg": 80.0, "y_lo": -0.6, "y_hi": 0.6, "preferred_side": 1.0}
    front = _blocked_front(cfg)
    c.step(_sectors(front=front, fc=front, left=0.15, right=0.15), obst, None, 0.0, (0.0, 0.0), 0.0)
    out = c.step(_sectors(front=5.0, left=0.15), obst, None, 0.0, (0.0, 0.0), 0.1)
    assert out.state == "TURN"
    assert c.consecutive_avoid_count == 1  # no re-assess yet -- only one close tick


def test_turn_abort_counter_resets_on_clear_tick():
    cfg = _cfg(obstacle_confirm_scans=2)
    c = AvoidanceController(cfg)
    c.set_goal_heading(0.0)
    obst = {"span_deg": 80.0, "y_lo": -0.6, "y_hi": 0.6, "preferred_side": 1.0}
    front = _blocked_front(cfg)
    c.step(_sectors(front=front, fc=front, left=0.15, right=0.15), obst, None, 0.0, (0.0, 0.0), 0.0)
    c.step(_sectors(front=5.0, left=0.15), obst, None, 0.0, (0.0, 0.0), 0.1)   # close tick 1
    c.step(_sectors(front=5.0, left=5.0), obst, None, 0.0, (0.0, 0.0), 0.2)    # clear tick -- resets counter
    out = c.step(_sectors(front=5.0, left=0.15), obst, None, 0.0, (0.0, 0.0), 0.3)  # close tick (only 1 since reset)
    assert out.state == "TURN"
    assert c.consecutive_avoid_count == 1  # never reached 2 consecutive -- reset worked


def test_turn_danger_takes_priority_over_heading_reached():
    cfg = _cfg(obstacle_confirm_scans=1)
    c = AvoidanceController(cfg)
    c.set_goal_heading(0.0)
    obst = {"span_deg": 80.0, "y_lo": -0.6, "y_hi": 0.6, "preferred_side": 1.0}
    front = _blocked_front(cfg)
    c.step(_sectors(front=front, fc=front, left=0.15, right=0.15), obst, None, 0.0, (0.0, 0.0), 0.0)
    # turn_target = yaw_at_entry(0.0) + radians(turn_step_deg=30) * locked_dir(+1).
    # Passing that exact yaw makes `reached` true on this same tick, alongside a
    # danger-condition front reading -- obstacle_confirm_scans=1 means the danger
    # check aborts immediately, so this proves it pre-empts the DRIVE_PAST hand-off
    # rather than racing with it.
    turn_target = math.radians(30.0)
    out = c.step(_sectors(front=0.15, left=5.0, right=5.0), obst, None, turn_target, (0.0, 0.0), 0.1)
    assert out.state == "TURN"  # still too wide -> re-assess picks TURN again, not DRIVE_PAST
    assert c.consecutive_avoid_count == 2


def test_maneuver_abort_counter_resets_between_strafe_and_turn():
    cfg = _cfg(obstacle_confirm_scans=2, strafe_speed=0.25, strafe_timeout=1.5)
    c = AvoidanceController(cfg)
    c.set_goal_heading(0.0)
    obst = {"span_deg": 10.0, "y_lo": -0.05, "y_hi": 0.05, "preferred_side": 1.0}
    front = _blocked_front(cfg)
    c.step(_sectors(front=front, fc=front, left=1.5), obst, None, 0.0, (0.0, 0.0), 0.0)  # -> STRAFE
    # Two consecutive close-flank ticks abort STRAFE. The fresh re-assess sees
    # left=0.15 (too tight to strafe) and right at its default 5.0, so
    # locked_dir = 1.0 if left(0.15) >= right(5.0) else -1.0 -> -1.0 (turns right;
    # the "into" side for this new TURN is s["right"]).
    c.step(_sectors(front=0.25, left=0.15), obst, None, 0.0, (0.0, 0.0), 0.1)
    out = c.step(_sectors(front=0.25, left=0.15), obst, None, 0.0, (0.0, 0.0), 0.2)
    assert out.state == "TURN"
    # If the abort counter had carried over from STRAFE's abort (already at the
    # threshold when it fired), a single further close-right tick would wrongly
    # abort TURN immediately. It must take the full debounce depth again, fresh.
    out = c.step(_sectors(front=5.0, right=0.15), obst, None, 0.0, (0.0, 0.0), 0.3)
    assert out.state == "TURN"
    assert c.consecutive_avoid_count == 2  # not yet 3 -- this tick alone didn't abort
    out = c.step(_sectors(front=5.0, right=0.15), obst, None, 0.0, (0.0, 0.0), 0.4)
    assert c.consecutive_avoid_count == 3  # second consecutive close tick -- now it aborts
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd proximity_alert && python3 -m pytest test/test_avoidance_controller.py -k "turn_aborts or turn_does_not_abort or turn_abort_counter or turn_danger or maneuver_abort_counter_resets" -v`

Expected: FAIL. `test_turn_aborts_when_into_side_closing_sustained`, `test_turn_aborts_when_front_closing_sustained`, and `test_turn_danger_takes_priority_over_heading_reached` fail on `consecutive_avoid_count` staying at 1 (`_turn()` has no abort path yet). `test_maneuver_abort_counter_resets_between_strafe_and_turn` fails at its first `assert out.state == "TURN"` (STRAFE's abort from Task 1 already works, but nothing yet makes the resulting TURN detect the right-side danger). The two "does not abort"/"resets" tests currently pass vacuously, same as in Task 1.

- [ ] **Step 3: Implement**

In `_assess()`, add a reset at the TURN commit point:

```python
        self._locked_dir = 1.0 if s["left"] >= s["right"] else -1.0
        self._maneuver_start = self._now
        self._maneuver_abort_count = 0
        self._turn_target = self._yaw + math.radians(cfg.turn_step_deg) * self._locked_dir
        self.state = TURN
```

Replace `_turn()`:

```python
    def _turn(self):
        s, cfg = self._s, self.config
        into_side = s["left"] if self._locked_dir > 0 else s["right"]
        danger = s["front"] <= cfg.safety_distance or into_side <= cfg.safety_distance
        if danger:
            self._maneuver_abort_count += 1
            if self._maneuver_abort_count >= cfg.obstacle_confirm_scans:
                return self._assess()
        else:
            self._maneuver_abort_count = 0
        elapsed = self._now - self._maneuver_start
        reached = abs(normalize_angle(self._turn_target - self._yaw)) <= self._tol()
        if reached or elapsed >= cfg.turn_timeout:
            self.state = DRIVE_PAST
            self.target_heading = self._yaw
            self._drive_past_dist = 0.0
            self._clear_since = None
            self._maneuver_start = self._now
            return self._drive_past()
        return ControllerOutput(self._reverse_component(), 0.0,
                                cfg.turn_speed * self._locked_dir, TURN, None)
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd proximity_alert && python3 -m pytest test/test_avoidance_controller.py -k "turn_aborts or turn_does_not_abort or turn_abort_counter or turn_danger or maneuver_abort_counter_resets" -v`

Expected: 6 passed.

- [ ] **Step 5: Run the full test suite to verify no regression**

Run: `cd proximity_alert && python3 -m pytest test/test_avoidance_controller.py -v`

Expected: all tests pass, including every pre-existing TURN test (`test_assess_chooses_turn_when_side_blocked`, `test_assess_chooses_turn_when_obstacle_too_wide`, the whole reverse-arc geometry block — `test_turn_arcs_backward_while_rotating_by_default`, `test_legacy_reverse_speed_used_when_turn_radius_unset`, `test_turn_radius_derives_reverse_speed_from_turn_speed`, `test_reverse_still_blocked_by_rear_clearance_regardless_of_radius`, `test_taper_scales_reverse_down_near_rear_limit`, `test_taper_does_not_exceed_full_speed_with_open_rear`) and `test_escalates_to_recover_then_halt`/`test_recover_commits_toward_gap_when_available`, which drive TURN repeatedly under a fully-boxed-in scenario — confirm those still escalate through RECOVER/HALT correctly with the new check active.

- [ ] **Step 6: Run the ROS-dependent suite for a full regression check**

`avoidance.py` is pure and this task doesn't touch `path_tracker.py`, but confirm nothing downstream broke:

Run (inside the `MentorPi` container, `-u ubuntu`): `cd /home/ubuntu/ros2_ws/src/proximity_alert && python3 -m pytest test/ -v`

Expected: all tests pass (this exercises `test_path_tracker_arrival.py`'s `_FakeController`, which is unaffected since it doesn't call the real `_strafe()`/`_turn()`).

- [ ] **Step 7: Commit**

```bash
git add proximity_alert/proximity_alert/avoidance.py proximity_alert/test/test_avoidance_controller.py
git commit -m "feat: abort TURN when into-side or front clearance closes mid-pivot"
```

---

## Self-Review

**Spec coverage:** Every numbered test in the spec's Testing section (1-10) maps to a task test: 1→`test_strafe_aborts_when_flank_closing_sustained`, 2→`test_strafe_aborts_when_front_closing_sustained`, 3→`test_strafe_does_not_abort_on_single_close_reading`, 4→`test_strafe_abort_counter_resets_on_clear_tick`, 5→`test_turn_aborts_when_into_side_closing_sustained`, 6→`test_turn_aborts_when_front_closing_sustained`, 7→`test_turn_does_not_abort_on_single_close_reading`/`test_turn_abort_counter_resets_on_clear_tick`, 8→`test_turn_danger_takes_priority_over_heading_reached`, 9→Task 1/2 Step 5 full-suite regression runs, 10→`test_maneuver_abort_counter_resets_between_strafe_and_turn`. The threshold-reuse rationale, `_assess()` reset points, and check-ordering requirements from the spec's Design section are all reflected in the Step 3 implementation blocks. No gaps.

**Placeholder scan:** No TBD/TODO; every step has complete, runnable code and exact commands.

**Type consistency:** `self._maneuver_abort_count` is introduced once in Task 1 (`__init__`, `_strafe()`, `_assess()`'s STRAFE branch) and consumed with the identical name and semantics in Task 2 (`_turn()`, `_assess()`'s TURN branch) — no renaming across tasks.
