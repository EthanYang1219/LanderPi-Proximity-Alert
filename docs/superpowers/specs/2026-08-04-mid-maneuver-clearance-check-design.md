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

- 18:56:01 — entered `TURN` already inside `safety_distance` (front=0.175m,
  left=0.175m against a 0.20m `safety_distance`), then pivoted with zero
  monitoring.
- 19:10:00→19:10:02 — `STRAFE` ran for ~2s (`cumulative_strafe_m` went from
  0.0 to 0.41), during which `left_clearance_m` collapsed from 0.369m to
  0.150m — below `strafe_side_clearance_min` (0.20m), the very threshold
  that justified starting the strafe — with nothing watching it happen.
  `front_distance_m` also closed from 0.185m to 0.125m in the same window.

A real box has corners; the single pre-maneuver snapshot doesn't capture
that the true clearance along the strafe/turn path is tighter than what was
measured from the original angle. Nothing currently catches that discovery
before contact.

Separately, live sampling of `/scan_raw` during this investigation showed
real scan-to-scan variance even in a static scene (the front-sector minimum
jumping 0.5–1.2m every 4–8 scans against a stable ~5.38m baseline) — this is
raw sensor jitter, not a code defect, and it's why any new check here must be
debounced rather than reacting to a single reading.

## Goal

Give `STRAFE` and `TURN` a live, debounced clearance check that aborts back
to `_assess()` — the same reaction `_drive_past()` (avoidance.py:418-421)
already uses on `front <= cfg.safety_distance` — before contact, without
introducing a new state or new escalation logic.

## Non-Goals

- **Not fixing the raw LiDAR jitter itself.** That's sensor/driver behavior,
  not something tunable in this codebase, and out of scope for this change.
- **Not changing `control_rate_hz`, or making debounce counters count
  distinct scans instead of control ticks.** A related weakness was found
  during this investigation — at the default `control_rate_hz=20` against
  the LiDAR's real ~10Hz publish rate, `step()` runs roughly 2x per real
  scan, so "N consecutive confirming ticks" can be satisfied by the same
  single scan being read twice rather than two independent readings. This
  affects `obstacle_confirm_scans`, `encounter_close_confirm_scans`, and
  the new check this spec adds equally. It is a real, verified issue but is
  explicitly **deferred** — fixing it means changing what "a tick" means for
  every debounce counter in the controller, which is a larger, separate
  change from adding the missing checks themselves. Track it as a known
  limitation, not silently drop it.
- **Not touching ASSESS's strafe-vs-turn decision logic, DRIVE_PAST,
  RECOVER, HALT, or anything about how encounters open/close.** Those are
  unchanged.
- **Not adding new `AvoidanceConfig` fields.** The design reuses
  `safety_distance`, `strafe_side_clearance_min`, and `obstacle_confirm_scans`,
  all of which already mean almost exactly what's needed here.

## Design

### STRAFE

Each tick, in addition to the existing `front >= clear_threshold` (cleared)
check, also evaluate a **danger condition**:

```
front <= cfg.safety_distance
  OR (flank being strafed toward) <= cfg.strafe_side_clearance_min
```

where "flank being strafed toward" is `s["left"]` if `self._locked_dir > 0`
else `s["right"]` — the same `locked_dir`-based side selection `_reverse_component()`
and `_drive_past()`'s `inside` already use elsewhere in this file.

This combined condition (front OR flank, evaluated as one boolean per tick)
must hold for `cfg.obstacle_confirm_scans` **consecutive** ticks before it
triggers an abort. The counter resets to 0 the instant the condition isn't
met on a given tick — same reset-on-miss semantics as `self._confirm_count`
in `_drive()` (avoidance.py:261, 266). Once the debounced condition fires,
call `self._assess()` (not `_complete_to_drive` — this is a failure exit,
identical in kind to `_strafe()`'s existing `elapsed >= cfg.strafe_timeout`
exit, which already calls `_assess()`).

Order of checks within `_strafe()` per tick: cleared-check first (unchanged),
then timeout/cumulative-cap check (unchanged), then the new debounced danger
check, then (if none fired) continue strafing.

### TURN

Same shape. Each tick, evaluate:

```
front <= cfg.safety_distance
  OR (side being turned into) <= cfg.safety_distance
```

where "side being turned into" is `s["left"]` if `self._locked_dir > 0` else
`s["right"]`. Debounced the same way, over `cfg.obstacle_confirm_scans`
consecutive ticks, reset-on-miss. Once it fires, call `self._assess()`.

Order of checks within `_turn()` per tick: the new debounced danger check
first, then the existing heading-reached/timeout check (unchanged) — the
danger check must run before the maneuver would otherwise hand off to
`DRIVE_PAST`, since a heading-reached tick that is *also* a danger tick
should abort, not proceed into `DRIVE_PAST` on a collision course.

### Debounce counter

One new piece of controller state: `self._maneuver_abort_count` (integer,
starts at 0). `STRAFE` and `TURN` never run concurrently (they're mutually
exclusive states in `self.state`), so this is a single shared counter, not
one per maneuver.

Reset points:
- To `0` whenever the danger condition is not met on a given tick (in either
  `_strafe()` or `_turn()`).
- To `0` in `_assess()` at the moment it commits to a new `STRAFE` or `TURN`
  (avoidance.py:346-353 and 355-363) — so a freshly-entered maneuver always
  starts with a clean slate, the same way `_maneuver_start` is freshly set
  there.

Increment: by `1` on each tick the danger condition is met, in either
`_strafe()` or `_turn()`, before checking whether it has reached
`cfg.obstacle_confirm_scans`.

### Why `_assess()` and not a new state or immediate HALT

`_drive_past()` already establishes the precedent: `front <= cfg.safety_distance`
mid-maneuver calls `_assess()` again, which can choose a new `STRAFE`/`TURN`,
and which already tracks `consecutive_avoid_count` and escalates to
`RECOVER` once `max_avoid_attempts` is exceeded, then `HALT` once `RECOVER`
also fails. Routing the new abort through `_assess()` means a maneuver that
keeps re-triggering this check escalates through the existing ladder
automatically — no new escalation logic needed, and no new outcome states
for `_record()`/`DecisionRecord` to represent.

### Threshold choice rationale

- STRAFE's flank check reuses `strafe_side_clearance_min` rather than
  `safety_distance`: it's self-consistent with the value that justified
  starting the strafe in `_assess()`'s `strafe_ok` gate (avoidance.py:341) —
  if the flank drops back below the same bar that permitted this strafe,
  abort.
- TURN's checks reuse `safety_distance`: TURN has no dedicated side-clearance
  parameter of its own, and `safety_distance` is already what gates `DRIVE`'s
  stop condition and `_drive_past()`'s abort condition, so reusing it keeps
  the "what counts as too close" definition consistent everywhere except the
  strafe-specific case above.

### Purity constraint

`avoidance.py` remains pure — no `rclpy`/`sensor_msgs` imports. This design
only reads existing `sectors` dict keys (`front`, `left`, `right`) already
passed into `step()`, and existing config fields. No new inputs are needed.

## Testing

Following this project's established TDD convention
(`test_avoidance_controller.py`), new tests are needed for:

1. **STRAFE aborts when the flank being strafed toward closes below
   `strafe_side_clearance_min`, sustained for `obstacle_confirm_scans` ticks**
   — assert it calls `_assess()` again (state becomes `STRAFE` or `TURN`
   again depending on the new snapshot, `consecutive_avoid_count` increments)
   rather than continuing to strafe.
2. **STRAFE aborts when `front` closes below `safety_distance` mid-strafe**,
   sustained for `obstacle_confirm_scans` ticks, same assertion shape.
3. **STRAFE does NOT abort on a single close reading** (fewer than
   `obstacle_confirm_scans` consecutive ticks) — regression guard against
   the raw sensor jitter this was designed to tolerate.
4. **STRAFE's abort counter resets on a single clear tick** — N-1 close
   ticks, one clear tick, N-1 more close ticks must NOT abort (proves
   reset-on-miss, not a sliding window).
5. **TURN aborts when the side being turned into closes below
   `safety_distance`**, sustained for `obstacle_confirm_scans` ticks.
6. **TURN aborts when `front` closes below `safety_distance` mid-turn**,
   sustained for `obstacle_confirm_scans` ticks.
7. **TURN does NOT abort on a single close reading**, and its counter resets
   on a miss — same shape as tests 3-4, for TURN.
8. **A danger condition on the same tick heading is reached takes priority**
   — TURN reaching its target heading while also meeting the debounced
   danger condition aborts to `_assess()`, not `DRIVE_PAST`.
9. **Existing STRAFE/TURN tests continue to pass unmodified** — this design
   adds a check, it does not change `_strafe()`/`_turn()`'s existing
   cleared/timeout/heading-reached behavior when nothing is closing in.
10. **The abort counter resets fresh when `_assess()` commits to a new
    STRAFE or TURN** — a maneuver that aborts and immediately re-enters
    STRAFE/TURN must not carry over stale counter state from the previous
    attempt.

## Self-Review

**Placeholder scan:** No TBD/TODO; every check's exact condition, threshold,
and reset semantics are specified.

**Internal consistency:** STRAFE and TURN's checks follow the identical
shape (front OR into-side, debounced, reset-on-miss, abort via `_assess()`)
differing only in which threshold each reuses, per the rationale given.
Matches the precedent already in `_drive_past()`.

**Scope check:** Single, focused change — two functions gain one additional
per-tick check each, sharing one counter. No new states, no new config,
no changes outside `_strafe()`/`_turn()`/`_assess()`'s STRAFE/TURN-entry
lines. Appropriately scoped for one implementation plan.

**Ambiguity check:** Order of checks within each function is specified
explicitly (including the TURN heading-reached-vs-danger priority case,
which could otherwise be read two ways). Which side counts as "being
strafed/turned toward" is defined in terms of the existing `_locked_dir`
convention already used elsewhere in the file, not a new convention.
