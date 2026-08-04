# Mid-Maneuver Clearance Check Design

## Problem

`STRAFE` and `TURN` each commit to a maneuver based on a single pre-maneuver
LiDAR snapshot taken in `_assess()`, then run with no further clearance
monitoring until they finish on their own terms:

- `_strafe()` (avoidance.py:365) only checks `s["front"] >= cfg.clear_threshold`
  (did we clear it?), `elapsed >= cfg.strafe_timeout`, and the cumulative-strafe
  cap. It never re-reads `left`/`right` after the initial decision.
- `_turn()` (avoidance.py:404) only checks whether the target heading was
  reached or `elapsed >= cfg.turn_timeout`. It checks no clearance at all,
  in any direction, during the pivot.

This is a real gap, not a theoretical one. On 2026-08-03/04, three separate
contacts were made with the same box obstacle across one test session
(`trials/lateral_offset_trials.csv`, trials 2, 7, 10 — "ran over the tape
measurement", "two obstacles, a box and chair in order", "touched the box
slightly"), and `decision_log.csv` shows the mechanism directly:

- 18:56:01 — entered `TURN` with `left` already at 0.175m, then pivoted with
  zero monitoring of that side.
- 19:10:00→19:10:02 — `STRAFE` ran for ~2s (`cumulative_strafe_m` went from
  0.0 to 0.41), during which `left_clearance_m` collapsed from 0.369m to
  0.150m — below `strafe_side_clearance_min` (0.20m), the very threshold
  that justified starting the strafe — with nothing watching it happen.

A real box has corners; the single pre-maneuver snapshot doesn't capture
that the true clearance along the strafe/turn path is tighter than what was
measured from the original angle. Nothing currently catches that discovery
before contact.

Separately, live sampling of `/scan_raw` during this investigation showed
real scan-to-scan variance even in a static scene (the front-sector minimum
jumping 0.5–1.2m every 4–8 scans against a stable ~5.38m baseline) — this is
raw sensor jitter, not a code defect, and it's why any new check here must be
debounced rather than reacting to a single reading.

## Governing principle: detection vs. invalidation

`DRIVE` and the maneuver states are asking fundamentally different questions,
and conflating them is what made the first draft of this design wrong:

| State | Question | Signal |
|---|---|---|
| `DRIVE` | *Is there an obstacle?* | `front <= safety_distance` |
| `STRAFE` / `TURN` | *Is the maneuver I chose still valid?* | the side I committed to using has closed off |

During `STRAFE` or `TURN` the robot **already knows** there is an obstacle in
front — that is why it is maneuvering. `front <= safety_distance` is the
expected operating condition for the entire duration of both states, not
evidence of danger. What is genuinely new information mid-maneuver is whether
the escape route the maneuver depends on is still there.

Every design decision below follows from that distinction.

## Goal

Give `STRAFE` and `TURN` a live, debounced **maneuver-invalidation** check
that aborts back to `_assess()` — the same reaction `_drive_past()`
(avoidance.py:418-421) already uses — before contact, without introducing a
new state, new escalation logic, new config fields, or new controller state
beyond a single counter.

## Non-Goals

- **No absolute `front` term in the danger condition.** See "Why the front
  term was removed" below — this is the correction that came out of design
  review, and re-adding it would break both maneuvers outright.
- **Not fixing the raw LiDAR jitter itself.** Sensor/driver behavior, not
  tunable in this codebase.
- **Not changing `control_rate_hz`, or making debounce counters count
  distinct scans instead of control ticks.** At the default
  `control_rate_hz=20` against the LiDAR's real ~10Hz publish rate, `step()`
  runs roughly 2x per real scan, so "N consecutive confirming ticks" can be
  satisfied by the same single scan read twice. This affects
  `obstacle_confirm_scans`, `encounter_close_confirm_scans`, and the new
  check equally. Real and verified, but fixing it means redefining what a
  tick means for every debounce counter in the controller. Deferred, and
  recorded as an in-code comment at the new check — not silently dropped.
- **Not touching ASSESS's strafe-vs-turn decision logic, DRIVE_PAST,
  RECOVER, HALT, or encounter open/close.**
- **Not adding new `AvoidanceConfig` fields.** Reuses
  `strafe_side_clearance_min` and `safety_distance`.

## Design

### STRAFE

Each tick, in addition to the existing checks, evaluate one danger condition:

```
(flank being strafed toward) <= cfg.strafe_side_clearance_min
```

where "flank being strafed toward" is `s["left"]` if `self._locked_dir > 0`
else `s["right"]` — the same `locked_dir`-based side selection
`_reverse_component()` and `_drive_past()`'s `inside` already use.

This must hold for `cfg.obstacle_confirm_scans` **consecutive** ticks before
it triggers an abort. The counter resets to 0 the instant the condition isn't
met — same reset-on-miss semantics as `self._confirm_count` in `_drive()`
(avoidance.py:261, 266). Once fired, call `self._assess()` (a failure exit,
identical in kind to `_strafe()`'s existing `strafe_timeout` exit).

Order of checks per tick: cleared-check first (unchanged), then
timeout/cumulative-cap (unchanged), then the new debounced danger check, then
continue strafing.

### TURN

Same shape:

```
(side being turned into) <= cfg.safety_distance
```

where "side being turned into" is `s["left"]` if `self._locked_dir > 0` else
`s["right"]`. Debounced identically, reset-on-miss, aborts to `self._assess()`.

Order of checks: the new debounced danger check **first**, then the existing
heading-reached/timeout check — a tick that is both "heading reached" and
"dangerous" must abort, not hand off to `DRIVE_PAST` on a collision course.

### Why the front term was removed

The first draft of this design also included `front <= cfg.safety_distance`
in both danger conditions. Tracing the actual tick sequence showed this is
invalid and would have broken avoidance entirely:

- `_assess()` commits to a maneuver and calls `self._strafe()` / `self._turn()`
  immediately, on the **same blocked snapshot** that triggered avoidance. So
  `front <= safety_distance` is true on the maneuver's very first tick, by
  construction.
- `STRAFE` continues while `front < clear_threshold` (0.30) and entered
  because `front <= safety_distance` (0.20), so that condition typically
  stays true for most of a normal, healthy strafe.

With `obstacle_confirm_scans=2`, every STRAFE would therefore abort one tick
after starting; at `obstacle_confirm_scans=1`, `_assess()` → `_strafe()` →
abort → `_assess()` would re-enter until `max_avoid_attempts` broke the chain.
Every encounter would escalate to RECOVER → HALT within a few ticks, with
nothing actually wrong. Removing the term is not a scope reduction — the
absolute front check was simply the wrong signal for these states, per the
detection-vs-invalidation principle above.

**Deferred alternative:** a *relative* front check — comparing live `front`
against its value at maneuver commit and aborting on significant degradation —
would capture the real signal in the logs (`front` closing 0.185m → 0.125m
during the 19:10 strafe) without the false-fire problem. It is intentionally
deferred: it requires storing per-maneuver entry state plus a new degradation
threshold to tune, and this change should not introduce both under the same
review cycle that caught the original error. The flank check alone addresses
the confirmed collision mechanism.

### Debounce counter

One new piece of controller state: `self._maneuver_abort_count` (integer,
starts at 0). `STRAFE` and `TURN` never run concurrently, so this is a single
shared counter.

- **Reset to 0** whenever the danger condition is not met on a tick (either
  function), and in `_assess()` at the moment it commits to a new `STRAFE` or
  `TURN` (avoidance.py:346-353 and 355-363), so a freshly-entered maneuver
  always starts clean.
- **Increment by 1** on each tick the danger condition is met, before checking
  whether it has reached `cfg.obstacle_confirm_scans`.

### Why repeated re-assessment cannot oscillate indefinitely

Returning to `_assess()` under persistent danger is bounded, not a loop:

- Every abort routes through `_assess()`, which increments
  `consecutive_avoid_count` unconditionally while inside an encounter — it
  only zeroes on `not self._in_encounter` (avoidance.py:310-316), false for a
  maneuver aborting mid-encounter.
- Once `consecutive_avoid_count > cfg.max_avoid_attempts`, `_assess()`
  escalates to `RECOVER` (one-shot, guarded by `has_recovered_this_encounter`),
  then `HALT`.
- `RECOVER` and `HALT` carry no mid-maneuver check, so escalation is terminal
  with respect to this mechanism.
- Each abort→re-assess→re-commit cycle costs at least
  `cfg.obstacle_confirm_scans` ticks, since the counter resets on every commit.

Worst case under sustained danger: `max_avoid_attempts` abort cycles, one
RECOVER, then HALT — the same resolution path a repeatedly-failing
`_drive_past()` already takes today.

**One re-entrancy note:** at `obstacle_confirm_scans=1` (not the default; used
in some tests), the check can fire on the commit tick itself, so `_assess()`
can be re-entered within a single `step()` call. Depth is bounded by
`max_avoid_attempts` and terminates in RECOVER/HALT — it is not unbounded
recursion, but it does mean a boxed-in robot escalates within one tick rather
than over several. Accepted; the default `obstacle_confirm_scans=2` avoids it
entirely, since the commit tick only reaches a count of 1.

### Swept footprint and sector coverage

Design review asked whether the `into_side` check actually represents the
robot's swept footprint during a pivot. Verified against `reduce_to_sectors()`
(scan_utils.py:50-81), the actual angular coverage at current defaults:

| Sector | Covers | From |
|---|---|---|
| `front` | -90°…+90° | `front_arc_deg=180`, min over the whole forward hemisphere |
| `left` | 60°…120° | `side_window_deg=60` centered at +90° |
| `right` | -120°…-60° | `side_window_deg=60` centered at -90° |
| `rear` | 150°…210° | `rear_window_deg=60` centered at 180° |

Two findings:

**Front corners are covered by `front`, but `front` is no longer consulted
mid-maneuver.** `front` is the minimum over the entire ±90° forward
hemisphere, not a narrow dead-ahead beam, so it *would* capture front-corner
closure. Since the front term is removed (above), forward-corner degradation
during a maneuver is not detected by this design — it is precisely what the
deferred relative-front check would cover. `left`/`right` still cover 60°…120°
of the forward-diagonal region, so the flank check catches the corner case
that matters most for a strafe.

**A pre-existing coverage hole exists on the rear diagonals.** The bands
**120°…150°** and **-150°…-120°** are read by no sector: `left`/`right` stop
at ±120°, `rear` starts at 150°. `_turn()` commands reverse and rotation on
the same tick (`_reverse_component()`), so the chassis translates backward
while pivoting and its rear corners sweep through exactly those blind bands.
`_reverse_component()`'s `rear_clearance_min` gate reads only `rear`, so it
misses them too. Out of scope here — closing it means widening
`side_window_deg`/`rear_window_deg` or adding diagonal sectors, changing every
consumer of `reduce_to_sectors()` including `scan_trace_logger`'s logged
sector defaults. Warrants its own spec.

### Threshold choice rationale

- **STRAFE reuses `strafe_side_clearance_min`.** Self-consistent with the
  value that justified starting the strafe in `_assess()`'s `strafe_ok` gate
  (avoidance.py:341): if the flank drops back below the same bar that
  permitted this strafe, the maneuver's own precondition has been invalidated.
- **TURN reuses `safety_distance`.** TURN has no dedicated side-clearance
  parameter, and `safety_distance` already defines "too close" for `DRIVE`
  and `_drive_past()`. Note `_assess()` picks TURN's direction as the *more
  open* side (`left >= right`), so this fires only when both sides are tight —
  which correctly means "don't pivot into a wall, escalate instead."

### Purity constraint

`avoidance.py` remains pure — no `rclpy`/`sensor_msgs` imports. This design
reads only existing `sectors` keys (`left`, `right`) already passed into
`step()`, plus existing config fields.

## Testing

Following this project's TDD convention (`test_avoidance_controller.py`):

1. **STRAFE aborts** when the flank being strafed toward closes below
   `strafe_side_clearance_min`, sustained for `obstacle_confirm_scans` ticks —
   asserts `_assess()` ran again (`consecutive_avoid_count` incremented, state
   re-chosen) rather than continuing to strafe.
2. **STRAFE does NOT abort on a single close reading** — regression guard
   against the measured sensor jitter this was designed to tolerate.
3. **STRAFE's abort counter resets on a clear tick** — close, clear, close
   must not abort (proves reset-on-miss, not a sliding window).
4. **STRAFE does NOT abort merely because `front` is inside
   `safety_distance`** — the regression guard for the removed front term.
   A strafe with a wide-open flank and a blocked front must keep strafing
   indefinitely (until its own timeout), never abort.
5. **TURN aborts** when the side being turned into closes below
   `safety_distance`, sustained for `obstacle_confirm_scans` ticks.
6. **TURN does NOT abort on a single close reading**, and its counter resets
   on a miss.
7. **TURN does NOT abort merely because `front` is inside `safety_distance`**
   — same regression guard as test 4, for TURN.
8. **Danger takes priority over heading-reached** — a TURN tick that both
   reaches its target heading and meets the debounced danger condition aborts
   to `_assess()`, not `DRIVE_PAST`.
9. **Existing STRAFE/TURN tests pass unmodified** — this adds a check; it does
   not change existing cleared/timeout/heading-reached behavior when nothing
   is closing in.
10. **The abort counter resets fresh** when `_assess()` commits to a new
    STRAFE or TURN — an aborted maneuver's counter must not carry into the
    next attempt.
11. **End-to-end re-plan after a mid-maneuver abort** — mirrors the observed
    hardware failure: STRAFE begins, a second obstacle appears mid-maneuver
    (flank closes), the maneuver aborts, the obstacle clears, and the
    controller re-plans and drives on. Asserts the run ends in `DRIVE` with
    forward motion, within the same `encounter_id`, without escalating to
    RECOVER/HALT. Where 1-10 pin isolated behaviors, this proves the abort is
    *recoverable* rather than merely detected.

## Known Limitations

Carried deliberately, each with a reason:

1. **No forward-degradation detection mid-maneuver.** The absolute front term
   was removed as invalid; the relative replacement is deferred (see "Why the
   front term was removed"). A maneuver whose *front* clearance worsens while
   its flank stays open will not abort.
2. **Debounce counts control ticks, not distinct LiDAR scans.** Weakens every
   debounce in the controller, not just this check. Must appear as an in-code
   comment at the new check.
3. **Rear-diagonal sector blind bands (120°…150°, -150°…-120°).** Pre-existing;
   affects `_reverse_component()` equally.
4. **RECOVER carries no mid-maneuver check.** It is this mechanism's
   escalation target, so adding one risks a loop between the two.

## Self-Review

**Placeholder scan:** No TBD/TODO; every condition, threshold, and reset is
fully specified.

**Internal consistency:** STRAFE and TURN share one shape — into-side only,
debounced, reset-on-miss, abort via `_assess()` — differing only in which
existing threshold each reuses, per the rationale given. No section still
references a front term in the danger condition.

**Scope check:** Two functions gain one check each, sharing one counter. No
new states, no new config fields, no changes outside `_strafe()`/`_turn()`/
`_assess()`'s two commit lines.

**Ambiguity check:** Check ordering is explicit in both functions, including
the TURN heading-vs-danger priority case. "Being strafed/turned toward" reuses
the existing `_locked_dir` convention. The commit-tick re-entrancy case at
`obstacle_confirm_scans=1` is stated rather than left to be discovered.
