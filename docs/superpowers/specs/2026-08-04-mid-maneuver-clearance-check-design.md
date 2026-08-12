# Mid-Maneuver Clearance Check Design

## Problem

`STRAFE` commits to a maneuver based on a single pre-maneuver LiDAR snapshot
taken in `_assess()`, then runs with no further clearance monitoring until it
finishes on its own terms: `_strafe()` (avoidance.py:365) only checks
`s["front"] >= cfg.clear_threshold` (did we clear it?),
`elapsed >= cfg.strafe_timeout`, and the cumulative-strafe cap. It never
re-reads `left`/`right` after the initial decision.

This is a real gap, not a theoretical one. On 2026-08-03/04, three separate
contacts were made with the same box obstacle across one test session
(`trials/lateral_offset_trials.csv`, trials 2, 7, 10 — "ran over the tape
measurement", "two obstacles, a box and chair in order", "touched the box
slightly"), and `decision_log.csv` shows the mechanism directly:

- 19:10:00→19:10:02 — `STRAFE` ran for ~2s (`cumulative_strafe_m` went from
  0.0 to 0.41), during which `left_clearance_m` collapsed from 0.369m to
  0.150m — below `strafe_side_clearance_min` (0.20m), the very threshold
  that justified starting the strafe — with nothing watching it happen.

A real box has corners; the single pre-maneuver snapshot doesn't capture
that the true clearance along the strafe path is tighter than what was
measured from the original angle. Nothing currently catches that discovery
before contact.

**`TURN` is deliberately excluded**, on the evidence. An earlier draft of this
design gave `TURN` a symmetric check on the side being pivoted into. Pulling
`right_clearance_m` alongside `left_clearance_m` for every logged `TURN` row
showed that check would never have fired:

| front | left | right |
|---|---|---|
| 0.148 | 0.150 | 2.822 |
| 0.175 | 0.175 | 4.852 |
| 0.145 | 0.145 | 4.875 |

`_assess()` sets `_locked_dir = 1.0 if s["left"] >= s["right"] else -1.0`, so
every one of these turned **right**, into 2.3–4.9m of open space. The into-side
was never close in any observed collision. See "Why TURN carries no check".

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
| `STRAFE` | *Is the maneuver I chose still valid?* | the flank I committed to using has closed off |
| `TURN` | *Can I escape at all, when strafing isn't viable?* | — (see below) |

During `STRAFE` the robot **already knows** there is an obstacle in front —
that is why it is maneuvering. `front <= safety_distance` is the expected
operating condition for the entire duration of the state, not evidence of
danger. What is genuinely new information mid-maneuver is whether the escape
route the maneuver depends on is still there.

`TURN` asks a third question, and it is the reason `TURN` gets no check.
`TURN` is entered precisely when the environment is *already* constrained —
when strafing has been ruled out. Aborting it because clearance is tight is
aborting it for the condition that caused it to be selected.

Every design decision below follows from these distinctions.

## Goal

Give `STRAFE` a live, debounced **maneuver-invalidation** check that aborts
back to `_assess()` — the same reaction `_drive_past()` (avoidance.py:418-421)
already uses — before contact, without introducing a new state, new escalation
logic, new config fields, or new controller state beyond a single counter.

## Non-Goals

- **No mid-maneuver check in `TURN`.** See "Why TURN carries no check" — the
  collision evidence does not support one, and the symmetric version actively
  broke `TURN`.
- **No absolute `front` term in the danger condition.** See "Why the front
  term was removed" below — this is the correction that came out of design
  review, and re-adding it would break the maneuver outright.
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
- **Not touching ASSESS's strafe-vs-turn decision logic, TURN, DRIVE_PAST,
  RECOVER, HALT, or encounter open/close.**
- **Not adding new `AvoidanceConfig` fields.** Reuses
  `strafe_side_clearance_min`.

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

### Why TURN carries no check

`_turn()` is unchanged by this design. An earlier draft gave it the symmetric
condition `(side being turned into) <= cfg.safety_distance`, debounced the same
way. It was implemented, its own targeted tests passed, and it was then
withdrawn on two independent grounds.

**1. The evidence does not support it.** As shown in Problem, every logged
`TURN` pivoted toward 2.3–4.9m of open space. The into-side was never close.
This check would not have prevented any observed collision — it was
speculative, unlike the flank check, which is traceable to a specific logged
collapse (0.369m → 0.150m).

**2. It disables TURN in the situation TURN exists for.** `_assess()` selects
the *more open* side and then calls `_turn()` immediately, on the same snapshot.
When both sides are tight — the boxed-in case — the more-open side is still
below `safety_distance`, so the check fires on the commit tick. Measured
directly, `consecutive_avoid_count` went **0 → 4 in a single `step()` call**
via re-entrant `_assess()` → `_turn()` → abort → `_assess()`, exhausting
`max_avoid_attempts=3` and landing in `RECOVER` before the robot moved once.

Six pre-existing reverse-arc geometry tests (`turn_radius` derivation, rear
clearance gating, taper scaling) broke, all reporting `RECOVER` where they
assert `TURN`. Those tests were not incidental casualties: they exercise the
reverse-arc geometry that only runs in the constrained scenario, which is
exactly the scenario the check was cancelling. `TURN` commands reverse *and*
rotation together specifically to back out of tight spots; aborting it for
tightness removes the controller's escape hatch.

The failure was semantic, not an implementation defect, so no patch to the
check was attempted — it was removed.

### Why the front term was removed

The first draft of this design also included `front <= cfg.safety_distance`
in the danger condition. Tracing the actual tick sequence showed this is
invalid and would have broken avoidance entirely:

- `_assess()` commits to a maneuver and calls `self._strafe()` immediately, on
  the **same blocked snapshot** that triggered avoidance. So
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
starts at 0). `STRAFE` is currently its only user; the generic name is kept
so a future maneuver-invalidation check can share it without renaming.

- **Reset to 0** whenever the danger condition is not met on a tick, and in
  `_assess()` at the moment it commits to a new `STRAFE` (avoidance.py:347-355),
  so a freshly-entered strafe always starts clean.
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

**Re-entrancy is structurally near-impossible for STRAFE**, and this is the
key asymmetry with the withdrawn TURN check. `_assess()` commits to STRAFE only
when `side_clear >= cfg.strafe_side_clearance_min` (avoidance.py:342), and sets
`_locked_dir` to that same side — so the commit tick's flank is above the abort
threshold by construction. The check cannot fire on the tick it was committed
on, at any `obstacle_confirm_scans` value.

The single exception is exact equality (`side_clear == strafe_side_clearance_min`,
where `>=` and `<=` are both true) combined with `obstacle_confirm_scans=1`.
There, `_assess()` can be re-entered within one `step()` call. Depth is bounded
by `max_avoid_attempts`, terminating in RECOVER/HALT — not unbounded recursion.
Accepted; the default `obstacle_confirm_scans=2` avoids even that, since the
commit tick can only reach a count of 1.

### Swept footprint and sector coverage

Design review asked whether the flank check actually represents the robot's
swept footprint during a maneuver. Verified against `reduce_to_sectors()`
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
misses them too. This design does not touch `TURN` at all, so the hole is
untouched either way. Out of scope here — closing it means widening
`side_window_deg`/`rear_window_deg` or adding diagonal sectors, changing every
consumer of `reduce_to_sectors()` including `scan_trace_logger`'s logged
sector defaults. Warrants its own spec.

### Threshold choice rationale

**STRAFE reuses `strafe_side_clearance_min`.** Self-consistent with the value
that justified starting the strafe in `_assess()`'s `strafe_ok` gate
(avoidance.py:342): if the flank drops back below the same bar that permitted
this strafe, the maneuver's own precondition has been invalidated. No new
config field, and no threshold to tune independently — the check and the
commit gate cannot drift apart.

This self-consistency is itself an argument against the withdrawn TURN check,
which had to borrow `safety_distance` from `DRIVE`/`_drive_past()` because
`TURN` has no precondition of its own to re-test. A check with no matching
entry condition is not an invalidation check.

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
5. **Existing STRAFE and TURN tests pass unmodified** — this adds one check to
   one function; it does not change existing cleared/timeout behavior when
   nothing is closing in, and does not touch `TURN` at all. The six reverse-arc
   geometry tests (`turn_radius` derivation, rear clearance gating, taper
   scaling) are the specific regression guard here: they broke under the
   withdrawn TURN check and must pass without fixture edits.

   Tests 1-5 are implemented and passing (40 tests, no fixture modifications).

6. **End-to-end re-plan after a mid-maneuver abort** — mirrors the observed
    hardware failure: STRAFE begins, a second obstacle appears mid-maneuver
    (flank closes), the maneuver aborts, the obstacle clears, and the
    controller re-plans and drives on. Asserts the run ends in `DRIVE` with
    forward motion, within the same `encounter_id`, without escalating to
    RECOVER/HALT. Where 1-10 pin isolated behaviors, this proves the abort is
    *recoverable* rather than merely detected.

## Known Limitations

Carried deliberately, each with a reason:

1. **`TURN` has no mid-maneuver clearance monitoring at all.** Only `STRAFE`
   performs mid-maneuver invalidation. A `TURN` that begins in a viable
   direction and has that direction close off during the pivot will not abort;
   it runs to its target heading or `turn_timeout` regardless. This is
   deliberate: no logged collision shows `TURN`'s into-side degrading (every
   observed `TURN` pivoted into 2.3–4.9m of open space), and the absolute
   into-side check that would cover it disables `TURN` in the boxed-in case
   that `TURN` exists to escape. See "Why TURN carries no check".

   **Future direction if evidence appears:** a *relative* clearance check —
   storing the into-side clearance at maneuver commit and aborting on
   significant degradation from that baseline, rather than on an absolute
   threshold. That would fire on genuine closure while staying silent when the
   robot is simply boxed in but stable, which is the failure mode of the
   absolute version. It needs new per-maneuver entry state and a degradation
   threshold to tune, so it should not be added speculatively — only once
   hardware logs actually show a `TURN` into-side collapsing mid-pivot. This is
   the same deferred mechanism proposed for front degradation in limitation 2;
   one design would likely cover both.
2. **No forward-degradation detection mid-maneuver.** The absolute front term
   was removed as invalid; the relative replacement is deferred (see "Why the
   front term was removed"). A maneuver whose *front* clearance worsens while
   its flank stays open will not abort.
3. **Debounce counts control ticks, not distinct LiDAR scans.** Weakens every
   debounce in the controller, not just this check. Must appear as an in-code
   comment at the new check.
4. **Rear-diagonal sector blind bands (120°…150°, -150°…-120°).** Pre-existing;
   affects `_reverse_component()` equally.
5. **RECOVER carries no mid-maneuver check.** It is this mechanism's
   escalation target, so adding one risks a loop between the two.

## Self-Review

**Placeholder scan:** No TBD/TODO; every condition, threshold, and reset is
fully specified.

**Internal consistency:** One check, in one state, on one signal — flank only,
debounced, reset-on-miss, abort via `_assess()`, reusing the threshold that
gates entry to that same state. No section still describes a front term or an
into-side term in the danger condition.

**Evidence discipline:** Every element of the shipped design traces to a
logged event. The two elements that did not — the absolute front term and the
TURN into-side check — were both removed during review rather than kept as
"probably useful". Both failed for the same underlying reason: they applied
`DRIVE`'s detection question to a state whose entry condition already implies
the answer.

**Scope check:** One function gains one check, using one counter. No new
states, no new config fields, no changes outside `_strafe()` and `_assess()`'s
STRAFE commit branch. `_turn()` is byte-for-byte unchanged.

**Ambiguity check:** Check ordering within `_strafe()` is explicit. "Flank
being strafed toward" reuses the existing `_locked_dir` convention. The
exact-equality re-entrancy edge case at `obstacle_confirm_scans=1` is stated
rather than left to be discovered.
