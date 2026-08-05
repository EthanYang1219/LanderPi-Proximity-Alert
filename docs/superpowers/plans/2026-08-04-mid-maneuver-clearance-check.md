# Mid-Maneuver Clearance Check Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give `STRAFE` and `TURN` a live, debounced maneuver-invalidation check that aborts back to `_assess()` before contact, closing the gap behind three real obstacle contacts on 2026-08-03/04.

**Architecture:** `DRIVE` asks *"is there an obstacle?"*; `STRAFE`/`TURN` ask *"is the maneuver I chose still valid?"* — different questions. During a maneuver the robot already knows the front is blocked (that's the premise), so the only new information is whether the escape route it committed to has closed off. Each maneuver gains one per-tick check on the side it is moving toward, sharing one counter (`self._maneuver_abort_count`) since the two states are mutually exclusive. On the debounced condition firing, the maneuver returns `self._assess()` — the same reaction `_drive_past()` already uses — so escalation through `consecutive_avoid_count` → RECOVER → HALT is automatic.

**Tech Stack:** Python 3, pytest, pure `avoidance.py` (no ROS dependencies).

## Global Constraints

Per the approved spec (`docs/superpowers/specs/2026-08-04-mid-maneuver-clearance-check-design.md`):

- `avoidance.py` remains pure — no `rclpy`/`sensor_msgs` imports.
- **No absolute `front` term in either danger condition.** `front <= safety_distance` is the expected operating state throughout both maneuvers (they enter *because* of it, and `_assess()` calls `_strafe()`/`_turn()` on that same blocked snapshot). Including it would abort every maneuver ~1 tick after starting. Only the into-side clearance is checked.
- ASSESS's strafe-vs-turn decision logic, `DRIVE_PAST`, `RECOVER`, `HALT`, and encounter open/close are unchanged. This touches only `_strafe()`, `_turn()`, and `_assess()`'s two commit points.
- No new `AvoidanceConfig` fields — reuses `strafe_side_clearance_min` (STRAFE), `safety_distance` (TURN), `obstacle_confirm_scans` (debounce depth).
- The reaction to detected danger is always `self._assess()` — never a new state, never immediate HALT.
- `self._maneuver_abort_count` is one shared counter: reset to 0 on any tick the danger condition is false, and reset to 0 at each new STRAFE/TURN commit in `_assess()`.
- TURN's danger check runs *before* its heading-reached/timeout check, so a tick that is both aborts rather than handing off to `DRIVE_PAST`.
- The scan-vs-tick debounce weakness (`control_rate_hz=20` vs. the LiDAR's ~10Hz) is out of scope, but **must** appear as an in-code comment.

**Test-authoring note (applies to every test below):** with `obstacle_confirm_scans=N`, entering a maneuver takes **N blocked `_drive()` ticks** — the first N-1 return `DRIVE` with `linear_x == 0.0` (confirming), and only the Nth reaches `_assess()`. Every test here uses `obstacle_confirm_scans=2`, so each one spends two ticks entering the maneuver before exercising anything.

---

## File Structure

- `proximity_alert/proximity_alert/avoidance.py` — MODIFY. Add `self._maneuver_abort_count` to `__init__`; add the danger check to `_strafe()` and `_turn()`; add the counter reset at both commit points in `_assess()`.
- `proximity_alert/test/test_avoidance_controller.py` — MODIFY. STRAFE tests (Task 1), TURN tests (Task 2), end-to-end re-plan test (Task 3).

No new files.

---

## Task 1: STRAFE maneuver-invalidation check

**Files:**
- Modify: `proximity_alert/proximity_alert/avoidance.py` (`__init__`, `_strafe()`, `_assess()`'s STRAFE commit branch)
- Test: `proximity_alert/test/test_avoidance_controller.py`

**Interfaces:**
- Consumes: existing `self._s` (sectors dict), `self._locked_dir`, `cfg.strafe_side_clearance_min`, `cfg.obstacle_confirm_scans`, `self._assess()`.
- Produces: `self._maneuver_abort_count` (int) — also used by Task 2.

- [ ] **Step 1: Write the failing tests**

Append to `proximity_alert/test/test_avoidance_controller.py`:

```python
# ---------- mid-maneuver invalidation: STRAFE ----------

def _strafe_entry_cfg():
    # strafe_speed/strafe_timeout pinned so STRAFE is actually reachable --
    # see the comment in test_assess_chooses_strafe_for_narrow_obstacle_with_clear_side.
    return _cfg(obstacle_confirm_scans=2, strafe_speed=0.25, strafe_timeout=1.5)


def _enter_strafe(c, cfg):
    """Two blocked DRIVE ticks (obstacle_confirm_scans=2) -> _assess() -> STRAFE."""
    obst = {"span_deg": 10.0, "y_lo": -0.05, "y_hi": 0.05, "preferred_side": 1.0}
    front = _blocked_front(cfg)
    blocked = _sectors(front=front, fc=front, left=1.5)
    c.step(blocked, obst, None, 0.0, (0.0, 0.0), 0.0)          # confirming
    out = c.step(blocked, obst, None, 0.0, (0.0, 0.0), 0.1)    # -> _assess -> STRAFE
    assert out.state == "STRAFE", f"setup failed, got {out.state}"
    assert c.consecutive_avoid_count == 1
    return obst


def test_strafe_aborts_when_flank_closing_sustained():
    cfg = _strafe_entry_cfg()
    c = AvoidanceController(cfg)
    c.set_goal_heading(0.0)
    obst = _enter_strafe(c, cfg)
    # Flank drops below strafe_side_clearance_min (0.20) for 2 consecutive ticks.
    # front held at 0.25: above safety_distance so it is not "blocked", below
    # clear_threshold (0.30) so the strafe does not complete -- isolating the flank.
    closing = _sectors(front=0.25, left=0.15)
    c.step(closing, obst, None, 0.0, (0.0, 0.0), 0.2)          # count=1
    out = c.step(closing, obst, None, 0.0, (0.0, 0.0), 0.3)    # count=2 -> abort
    assert c.consecutive_avoid_count == 2                       # _assess() ran again
    assert out.state == "TURN"   # left=0.15 < strafe_side_clearance_min -> can't re-strafe


def test_strafe_does_not_abort_on_single_close_reading():
    cfg = _strafe_entry_cfg()
    c = AvoidanceController(cfg)
    c.set_goal_heading(0.0)
    obst = _enter_strafe(c, cfg)
    out = c.step(_sectors(front=0.25, left=0.15), obst, None, 0.0, (0.0, 0.0), 0.2)
    assert out.state == "STRAFE"
    assert c.consecutive_avoid_count == 1      # no re-assess -- only one close tick


def test_strafe_abort_counter_resets_on_clear_tick():
    cfg = _strafe_entry_cfg()
    c = AvoidanceController(cfg)
    c.set_goal_heading(0.0)
    obst = _enter_strafe(c, cfg)
    c.step(_sectors(front=0.25, left=0.15), obst, None, 0.0, (0.0, 0.0), 0.2)   # count=1
    c.step(_sectors(front=0.25, left=1.5), obst, None, 0.0, (0.0, 0.0), 0.3)    # count=0
    out = c.step(_sectors(front=0.25, left=0.15), obst, None, 0.0, (0.0, 0.0), 0.4)  # count=1
    assert out.state == "STRAFE"
    assert c.consecutive_avoid_count == 1      # never reached 2 consecutive


def test_strafe_does_not_abort_on_blocked_front_alone():
    # Regression guard for the removed absolute-front term. front <= safety_distance
    # is the NORMAL condition throughout a strafe (it is why the strafe started), so
    # a wide-open flank must keep the maneuver running no matter how close front is.
    cfg = _strafe_entry_cfg()
    c = AvoidanceController(cfg)
    c.set_goal_heading(0.0)
    obst = _enter_strafe(c, cfg)
    blocked_front_open_flank = _sectors(front=0.10, left=1.5)
    for i, t in enumerate((0.2, 0.3, 0.4, 0.5)):
        out = c.step(blocked_front_open_flank, obst, None, 0.0, (0.0, 0.0), t)
        assert out.state == "STRAFE", f"aborted on tick {i} with an open flank"
    assert c.consecutive_avoid_count == 1      # never re-assessed
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd proximity_alert && python3 -m pytest test/test_avoidance_controller.py -k "strafe_aborts or strafe_does_not_abort or strafe_abort_counter" -v`

Expected: `test_strafe_aborts_when_flank_closing_sustained` FAILS on `assert c.consecutive_avoid_count == 2` (stays 1 — `_strafe()` has no abort path yet, so it just keeps strafing). The other three PASS vacuously (nothing aborts at all yet); they become meaningful guards once Step 3 lands.

- [ ] **Step 3: Implement**

In `AvoidanceController.__init__`, add the counter beside the existing debounce counter:

```python
        self._confirm_count = 0
        self._maneuver_abort_count = 0
```

In `_assess()`, reset it at the STRAFE commit point:

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
        # DRIVE asks "is there an obstacle?"; a maneuver asks "is the maneuver I
        # chose still valid?" -- different questions. We already KNOW the front is
        # blocked; that is why we are strafing, and front <= safety_distance stays
        # true for most of a healthy strafe. So front is deliberately NOT checked
        # here: doing so would abort every strafe ~1 tick after it started. What is
        # genuinely new information is whether the gap we committed to using has
        # closed off -- three obstacle contacts on 2026-08-03/04 came from exactly
        # that going unnoticed (flank 0.369m -> 0.150m mid-strafe, unwatched).
        #
        # Known limitation, out of scope here: obstacle_confirm_scans debounces
        # CONTROL TICKS, not distinct LiDAR scans. control_rate_hz (20) exceeds the
        # LD19's ~10Hz publish rate, so two consecutive "ticks" may be the same scan
        # counted twice. Affects every debounce in this controller; a future change
        # should key debounce off new-scan arrival instead of tick count.
        flank = s["left"] if self._locked_dir > 0 else s["right"]
        if flank <= cfg.strafe_side_clearance_min:
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

- [ ] **Step 5: Run the full controller suite to verify no regression**

Run: `cd proximity_alert && python3 -m pytest test/test_avoidance_controller.py -v`

Expected: all pass, including `test_assess_chooses_strafe_for_narrow_obstacle_with_clear_side`, `test_strafe_completes_to_drive_when_front_clears`, and the crab-correction block (`_strafe_then_clear`/`_strafe_cfg`) — none of those drive the flank below `strafe_side_clearance_min` for consecutive ticks.

- [ ] **Step 6: Commit**

```bash
git add proximity_alert/proximity_alert/avoidance.py proximity_alert/test/test_avoidance_controller.py
git commit -m "feat: abort STRAFE when the flank it committed to closes mid-maneuver"
```

---

## Task 2: TURN maneuver-invalidation check — WITHDRAWN, DO NOT IMPLEMENT

> **Status: cancelled 2026-08-05.** This task was implemented, its six targeted
> tests passed, and it was then reverted in full. It broke six pre-existing
> reverse-arc geometry tests by driving the controller to `RECOVER` on the TURN
> commit tick (`consecutive_avoid_count` 0 → 4 within one `step()` call via
> re-entrant `_assess()`), and re-reading the hardware logs showed every logged
> `TURN` pivoted into 2.3–4.9m of open space — the check would not have
> prevented any observed collision.
>
> The failure is semantic, not an implementation defect: `TURN` is entered
> *because* the environment is constrained, so aborting it for tight clearance
> disables it in the case it exists to escape. **Do not patch and retry this
> task.** See "Why TURN carries no check" and Known Limitation 1 in
> `docs/superpowers/specs/2026-08-04-mid-maneuver-clearance-check-design.md`,
> which also records the relative-clearance approach to revisit if hardware
> evidence of TURN-side degradation ever appears.
>
> The steps below are retained only as a record of what was tried.

**Files:**
- Modify: `proximity_alert/proximity_alert/avoidance.py` (`_turn()`, `_assess()`'s TURN commit branch)
- Test: `proximity_alert/test/test_avoidance_controller.py`

**Interfaces:**
- Consumes: `self._maneuver_abort_count` (Task 1), `self._s`, `self._locked_dir`, `cfg.safety_distance`, `cfg.obstacle_confirm_scans`, `self._assess()`.

- [ ] **Step 1: Write the failing tests**

Append to `proximity_alert/test/test_avoidance_controller.py`:

```python
# ---------- mid-maneuver invalidation: TURN ----------

def _enter_turn(c, cfg):
    """Two blocked DRIVE ticks -> _assess() -> TURN.

    Obstacle is physically wide (1.2m > max_obstacle_width) so strafe_ok fails on
    width and TURN is chosen. left(2.0) >= right(0.15) makes _locked_dir = +1, so
    the side being turned INTO is s["left"].
    """
    obst = {"span_deg": 80.0, "y_lo": -0.6, "y_hi": 0.6, "preferred_side": 1.0}
    front = _blocked_front(cfg)
    blocked = _sectors(front=front, fc=front, left=2.0, right=0.15)
    c.step(blocked, obst, None, 0.0, (0.0, 0.0), 0.0)          # confirming
    out = c.step(blocked, obst, None, 0.0, (0.0, 0.0), 0.1)    # -> _assess -> TURN
    assert out.state == "TURN", f"setup failed, got {out.state}"
    assert c.consecutive_avoid_count == 1
    return obst


def test_turn_aborts_when_into_side_closing_sustained():
    cfg = _cfg(obstacle_confirm_scans=2)
    c = AvoidanceController(cfg)
    c.set_goal_heading(0.0)
    obst = _enter_turn(c, cfg)
    # The side being turned into (left) drops below safety_distance for 2 ticks.
    # yaw stays 0.0 so heading-reached never fires and cannot race the danger check.
    closing = _sectors(front=5.0, left=0.15)
    c.step(closing, obst, None, 0.0, (0.0, 0.0), 0.2)          # count=1
    out = c.step(closing, obst, None, 0.0, (0.0, 0.0), 0.3)    # count=2 -> abort
    assert c.consecutive_avoid_count == 2                       # _assess() ran again
    assert out.state == "TURN"    # still too wide to strafe -> TURN again, other way


def test_turn_does_not_abort_on_single_close_reading():
    cfg = _cfg(obstacle_confirm_scans=2)
    c = AvoidanceController(cfg)
    c.set_goal_heading(0.0)
    obst = _enter_turn(c, cfg)
    out = c.step(_sectors(front=5.0, left=0.15), obst, None, 0.0, (0.0, 0.0), 0.2)
    assert out.state == "TURN"
    assert c.consecutive_avoid_count == 1


def test_turn_abort_counter_resets_on_clear_tick():
    cfg = _cfg(obstacle_confirm_scans=2)
    c = AvoidanceController(cfg)
    c.set_goal_heading(0.0)
    obst = _enter_turn(c, cfg)
    c.step(_sectors(front=5.0, left=0.15), obst, None, 0.0, (0.0, 0.0), 0.2)   # count=1
    c.step(_sectors(front=5.0, left=5.0), obst, None, 0.0, (0.0, 0.0), 0.3)    # count=0
    out = c.step(_sectors(front=5.0, left=0.15), obst, None, 0.0, (0.0, 0.0), 0.4)
    assert out.state == "TURN"
    assert c.consecutive_avoid_count == 1


def test_turn_does_not_abort_on_blocked_front_alone():
    # Regression guard for the removed absolute-front term, TURN side.
    cfg = _cfg(obstacle_confirm_scans=2)
    c = AvoidanceController(cfg)
    c.set_goal_heading(0.0)
    obst = _enter_turn(c, cfg)
    blocked_front_open_side = _sectors(front=0.10, left=2.0)
    for i, t in enumerate((0.2, 0.3, 0.4, 0.5)):
        out = c.step(blocked_front_open_side, obst, None, 0.0, (0.0, 0.0), t)
        assert out.state == "TURN", f"aborted on tick {i} with an open into-side"
    assert c.consecutive_avoid_count == 1


def test_turn_danger_takes_priority_over_heading_reached():
    cfg = _cfg(obstacle_confirm_scans=2)
    c = AvoidanceController(cfg)
    c.set_goal_heading(0.0)
    obst = _enter_turn(c, cfg)
    closing = _sectors(front=5.0, left=0.15)
    c.step(closing, obst, None, 0.0, (0.0, 0.0), 0.2)          # count=1
    # _locked_dir=+1 and yaw at entry was 0.0, so turn_target = +30 deg. Feeding
    # exactly that yaw makes `reached` true on the SAME tick the danger count hits
    # its threshold -- the danger check must win and re-assess instead of handing
    # off to DRIVE_PAST on a collision course.
    out = c.step(closing, obst, None, math.radians(30.0), (0.0, 0.0), 0.3)
    assert out.state != "DRIVE_PAST"
    assert out.state == "TURN"
    assert c.consecutive_avoid_count == 2


def test_maneuver_abort_counter_resets_between_strafe_and_turn():
    cfg = _strafe_entry_cfg()
    c = AvoidanceController(cfg)
    c.set_goal_heading(0.0)
    obst = _enter_strafe(c, cfg)
    # Abort the STRAFE (flank closes). The re-assess sees left=0.15 vs right=5.0,
    # so _locked_dir = -1 and the new TURN's into-side is s["right"].
    closing = _sectors(front=0.25, left=0.15)
    c.step(closing, obst, None, 0.0, (0.0, 0.0), 0.2)
    out = c.step(closing, obst, None, 0.0, (0.0, 0.0), 0.3)
    assert out.state == "TURN"
    assert c.consecutive_avoid_count == 2
    # Had the counter carried over (already at threshold when STRAFE aborted), one
    # close-right tick would wrongly abort TURN immediately. It must take the full
    # debounce depth again from zero.
    out = c.step(_sectors(front=5.0, right=0.15), obst, None, 0.0, (0.0, 0.0), 0.4)
    assert out.state == "TURN"
    assert c.consecutive_avoid_count == 2      # this tick alone did NOT abort
    c.step(_sectors(front=5.0, right=0.15), obst, None, 0.0, (0.0, 0.0), 0.5)
    assert c.consecutive_avoid_count == 3      # second consecutive tick aborts
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd proximity_alert && python3 -m pytest test/test_avoidance_controller.py -k "turn_aborts or turn_does_not_abort or turn_abort_counter or turn_danger or maneuver_abort_counter_resets" -v`

Expected: `test_turn_aborts_when_into_side_closing_sustained` and `test_turn_danger_takes_priority_over_heading_reached` FAIL (`consecutive_avoid_count` stays 1; the priority test additionally reports `DRIVE_PAST`, since without the check the heading-reached branch wins). `test_maneuver_abort_counter_resets_between_strafe_and_turn` FAILS at its final assertion (STRAFE's abort works from Task 1, but nothing makes the resulting TURN react to the right side). The two "does not abort" tests pass vacuously.

- [ ] **Step 3: Implement**

In `_assess()`, reset the counter at the TURN commit point:

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
        # Same maneuver-invalidation question as _strafe(), and the same reason
        # front is not consulted -- see the comment block there.
        #
        # Runs BEFORE the heading-reached/timeout check below on purpose: a tick
        # that is both "heading reached" and "turning into something" must abort,
        # not hand off to DRIVE_PAST on a collision course.
        #
        # _assess() picks _locked_dir as the MORE OPEN side, so this fires only
        # when both sides are tight -- correctly meaning "do not pivot into a wall,
        # escalate instead" rather than second-guessing a viable turn.
        #
        # Coverage note: no sector reads the 120..150 deg rear-diagonal bands that
        # the rear corners sweep through during this state's reverse arc. That is a
        # pre-existing sectorization gap (it affects _reverse_component's rear gate
        # too), documented in the spec and out of scope here.
        into_side = s["left"] if self._locked_dir > 0 else s["right"]
        if into_side <= cfg.safety_distance:
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

- [ ] **Step 5: Run the full controller suite to verify no regression**

Run: `cd proximity_alert && python3 -m pytest test/test_avoidance_controller.py -v`

Expected: all pass. Pay particular attention to the reverse-arc geometry block (`_turning()` helper — `test_turn_arcs_backward_while_rotating_by_default`, `test_legacy_reverse_speed_used_when_turn_radius_unset`, `test_turn_radius_derives_reverse_speed_from_turn_speed`, `test_reverse_still_blocked_by_rear_clearance_regardless_of_radius`, `test_taper_scales_reverse_down_near_rear_limit`, `test_taper_does_not_exceed_full_speed_with_open_rear`), which uses `left=0.15, right=0.15` — both sides below `safety_distance`. Those call `_turning()` for exactly one tick with `obstacle_confirm_scans=1`, so the danger count reaches 1 and aborts on the commit tick, re-entering `_assess()`. **If any of these now report a state other than `TURN`, stop and report it rather than adjusting the test** — it means the commit-tick re-entrancy documented in the spec changes observable behavior at `obstacle_confirm_scans=1`, which is a design question, not a test-fixture question.

Also confirm `test_escalates_to_recover_then_halt` and `test_recover_commits_toward_gap_when_available` still reach RECOVER/HALT (they use a fully boxed-in scenario, so faster escalation is expected and acceptable — the assertions are on reaching those states, not on timing).

- [ ] **Step 6: Commit**

```bash
git add proximity_alert/proximity_alert/avoidance.py proximity_alert/test/test_avoidance_controller.py
git commit -m "feat: abort TURN when the side it pivots into closes mid-maneuver"
```

---

## Task 3: End-to-end re-plan regression test

**Files:**
- Test: `proximity_alert/test/test_avoidance_controller.py`

**Interfaces:**
- Consumes: everything from Tasks 1 and 2. No production changes — this task is a regression test only. If it fails, the bug is in Task 1 or 2, not here.

- [ ] **Step 1: Write the test**

This mirrors the real hardware failure end to end. Append to `proximity_alert/test/test_avoidance_controller.py`:

```python
def test_aborted_maneuver_replans_and_resumes_driving():
    """The observed hardware scenario, start to finish.

    STRAFE begins -> a second obstacle closes the flank mid-maneuver -> the
    maneuver aborts and re-plans -> the obstacle clears -> the controller drives
    on. Tests 1-10 each pin one isolated behavior; this proves the abort is
    RECOVERABLE rather than merely detected -- that the ladder returns to normal
    driving instead of stalling or escalating once the hazard passes.
    """
    cfg = _strafe_entry_cfg()
    c = AvoidanceController(cfg)
    c.set_goal_heading(0.0)
    obst = _enter_strafe(c, cfg)
    encounter = c.encounter_id

    # Second obstacle appears alongside: the flank we committed to closes.
    closing = _sectors(front=0.25, left=0.15)
    c.step(closing, obst, None, 0.0, (0.0, 0.0), 0.2)
    out = c.step(closing, obst, None, 0.0, (0.0, 0.0), 0.3)
    assert out.state == "TURN"          # aborted the strafe, re-planned as a turn

    # Both obstacles clear. TURN runs out its turn_timeout (1.5s) and hands off to
    # DRIVE_PAST, which needs front >= clear_threshold and inside flank >=
    # pass_clearance sustained for clear_confirm_time (0.5s) before completing.
    clear = _sectors(front=5.0, left=5.0, right=5.0)
    c.step(clear, None, None, 0.0, (0.0, 0.0), 0.4)     # still turning
    c.step(clear, None, None, 0.0, (0.0, 0.0), 2.0)     # timeout -> DRIVE_PAST
    out = c.step(clear, None, None, 0.0, (0.0, 0.0), 2.7)   # clear_confirm_time met

    assert out.state == "DRIVE"
    assert out.linear_x > 0.0                       # actually driving again
    assert c.encounter_id == encounter              # same encounter throughout
    assert c.has_recovered_this_encounter is False  # never escalated to RECOVER
    assert c.consecutive_avoid_count <= cfg.max_avoid_attempts
```

- [ ] **Step 2: Run it**

Run: `cd proximity_alert && python3 -m pytest test/test_avoidance_controller.py::test_aborted_maneuver_replans_and_resumes_driving -v`

Expected: PASS. Unlike Tasks 1-2 this is written against already-implemented behavior, so it should pass immediately — it is a regression lock, not a red-green cycle. **If it fails, do not adjust the test to match:** it encodes the scenario the whole change exists to fix, so a failure means Task 1 or Task 2's implementation is wrong. Stop and report which assertion failed and the actual state sequence.

- [ ] **Step 3: Run the full controller suite**

Run: `cd proximity_alert && python3 -m pytest test/test_avoidance_controller.py -v`

Expected: all pass.

- [ ] **Step 4: Run the ROS-dependent suite for a full regression check**

`avoidance.py` is pure and no task here touches `path_tracker.py`, but confirm nothing downstream broke. From a terminal on the Pi:

```bash
docker exec -u ubuntu MentorPi bash -lc 'source /opt/ros/humble/setup.bash && cd /home/ubuntu/ros2_ws/src/proximity_alert && python3 -m pytest test/ -q'
```

Expected: all pass. (The host has no `rclpy`, so `path_tracker`/logger tests can only run inside the container.)

- [ ] **Step 5: Commit**

```bash
git add proximity_alert/test/test_avoidance_controller.py
git commit -m "test: end-to-end regression for abort-and-replan after a mid-maneuver hazard"
```

---

## Self-Review

**Spec coverage:** Spec tests 1-11 map to plan tests: 1→`test_strafe_aborts_when_flank_closing_sustained`, 2→`test_strafe_does_not_abort_on_single_close_reading`, 3→`test_strafe_abort_counter_resets_on_clear_tick`, 4→`test_strafe_does_not_abort_on_blocked_front_alone`, 5→`test_turn_aborts_when_into_side_closing_sustained`, 6→`test_turn_does_not_abort_on_single_close_reading`/`test_turn_abort_counter_resets_on_clear_tick`, 7→`test_turn_does_not_abort_on_blocked_front_alone`, 8→`test_turn_danger_takes_priority_over_heading_reached`, 9→Task 1/2 Step 5 regression runs, 10→`test_maneuver_abort_counter_resets_between_strafe_and_turn`, 11→Task 3. The spec's required in-code documentation (front-term removal rationale, tick-vs-scan limitation, rear-diagonal coverage gap, more-open-side rationale) all appear in the Step 3 comment blocks.

**Placeholder scan:** No TBD/TODO; every step has complete code and exact commands.

**Type consistency:** `self._maneuver_abort_count` is introduced once in Task 1 and consumed with identical name and semantics in Task 2. Test helpers `_strafe_entry_cfg()`/`_enter_strafe()` are defined in Task 1 and reused by name in Tasks 2 and 3; `_enter_turn()` is defined in Task 2. All build on the file's existing `_cfg`/`_sectors`/`_blocked_front` helpers.

**Known risk flagged, not hidden:** Task 2 Step 5 calls out that the existing reverse-arc geometry tests use `obstacle_confirm_scans=1` with both sides at 0.15m, which can trip the new check on the commit tick. The plan instructs stopping and reporting rather than editing those tests, since that outcome is a design signal.
