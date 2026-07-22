# Refined Obstacle Avoidance — Design Spec

**Project:** LanderPi Proximity Alert (UPEI ADAS research)
**Component:** `path_tracker.py` obstacle-avoidance behavior
**Date:** 2026-07-23
**Status:** Approved design, ready for implementation planning
**Scope:** Design only. No implementation code in this document.

---

## Governing invariant (design philosophy)

> **The algorithm always attempts the maneuver that minimizes deviation from the original goal heading while maintaining safety.**

Every design decision below derives from this single rule. It also fixes the
maneuver preference order, because each maneuver costs a known amount of goal-heading
deviation:

| Maneuver | Goal-heading deviation | Preference |
|---|---|---|
| **Strafe** (mecanum lateral) | ~0 (bearing held throughout) | **1st — preferred** |
| **Turn-and-drive** | bounded, must be re-earned | 2nd — fallback |
| **Bounded recovery** | largest (commit toward a gap) | 3rd — escalation |
| **Halt + signal** | motion stops | last resort |

Safety (never command motion toward unconfirmed-clear space; always able to stop)
is the hard constraint that gates every option; deviation-minimization is the
objective optimized *within* that constraint.

---

## 1. Scope & context

### Goal
Refine `path_tracker`'s reactive obstacle avoidance to intelligently combine the
existing turn-and-drive strategy with mecanum strafing, on a deterministic,
lightweight, embedded ROS 2 (Humble, `rclpy`) state machine. No SLAM, no mapping,
no global planning.

### Established context (from the platform, confirmed this project)
- **Chassis:** mecanum. Pure-rotation commands hit a vendor kinematics quirk (all 4
  wheels identical rps), so turns blend a small reverse `linear.x`. **Pure lateral
  strafe (`linear.y`) is unaffected and works cleanly** — confirmed on hardware.
- **Sensor:** LD19 2D LiDAR, angles reported `0..2π` (must wrap via `atan2(sin,cos)`;
  already handled in `scan_utils.min_range_in_forward_arc`). **Fixed-height 2D scan
  plane** — cannot see thin/overhanging obstacles (documented blind-spot limitation).
- **Localization:** no wheel encoders; `/odom` is open-loop but yaw is EKF-fused
  (IMU + odom), usable for bearing-hold over short tracks.
- **Control:** 10 Hz loop; PID heading-hold (`Kp=1.0, Ki=0.0, Kd=0.1`) already works well.
- **Safety constraint:** STM32 has no motion watchdog — it holds the last commanded
  velocity indefinitely, so the node must always drive `cmd_vel` to zero on stop/exit.

### Goal model (decided)
Bearing-hold. `goal_heading` is captured once at start (the A→B bearing) and is the
PID target during all normal driving. After any avoidance the robot re-converges onto
`goal_heading` — it does **not** re-capture "straight" from wherever it happens to be
facing (the current code's behavior, which lets it wander off the A→B line).

### Environment (decided)
Mostly discrete objects on open floor (boxes, cones), usually narrow enough that a
bounded strafe sidesteps them. True deadlock is rare but handled.

### Non-goals
SLAM / mapping / global path planning; multi-obstacle route optimization; recovering
from the fixed-height blind spot in software; moving-obstacle prediction.

---

## 2. Sensing model (sectors)

The LiDAR is reduced **once per scan** (O(beams), in the existing scan callback) into
sector clearances — each is just the minimum valid range over its angular window, with
beams wrapped into `[-π, π]` and filtered by `range_min/range_max`:

- **FRONT** — forward arc `±front_arc_deg/2`, sub-divided into **FRONT_LEFT /
  FRONT_CENTER / FRONT_RIGHT** (`front_subsector_deg` each). Drives the stop trigger
  and locates the obstacle laterally.
- **LEFT / RIGHT** — lateral windows (`side_window_deg` each, centered near ±90°).
  Confirm a strafe target side is clear.
- **REAR** — window around 180° (`rear_window_deg`). Gates reversing.

For lateral sizing, each near FRONT return contributes a lateral offset `y = r·sin(θ)`;
the obstacle occupies `[y_lo, y_hi]` relative to the robot centerline. This is the
measurable basis for "narrow vs wide" (§4).

A single 360° scan already contains all sectors simultaneously (it's a 360° sensor),
so recovery gap-selection (§7) uses **current** scan data, never a swept history.

---

## 3. State machine

Seven single-purpose states.

| State | Behavior | Exit conditions |
|---|---|---|
| **DRIVE** | Forward at `forward_speed`; PID holds `target_heading = goal_heading`. Publishes `/forward_min_range`. | FRONT ≤ `safety_distance` (confirmed, §6) → **ASSESS** |
| **ASSESS** | Transient (one evaluation on entry): localize obstacle, run the ladder (§4), increment `consecutive_avoid_count`, emit a decision-log record (§9). | → STRAFE / TURN / RECOVER / HALT |
| **STRAFE** | Lateral move toward locked clear side; PID still holds `goal_heading` (corrects yaw drift). | FRONT clear+margin → **DRIVE**; else completion (§5) → **ASSESS** |
| **TURN** | Locked-direction rotate toward more-open side (+ reverse if FRONT very close & REAR clear). | Completion (§5) → **DRIVE_PAST** |
| **DRIVE_PAST** | Forward holding the *post-turn* heading until the obstacle has passed on the flank (§ exit below). | passed → **DRIVE**; FRONT re-blocks → **ASSESS**; bounded cap → **ASSESS** |
| **RECOVER** | One bounded routine (§7): back off → pick current-scan gap → rotate to face it → commit-drive. | gap cleared → **DRIVE**; no gap / re-block → **HALT** |
| **HALT** | Stop (zero `cmd_vel`); flag operator; re-check each tick. | FRONT clear+margin sustained → **DRIVE** |

**Heading semantics.** `goal_heading` is fixed (captured at A). `target_heading` (the
PID setpoint) equals `goal_heading` in DRIVE / STRAFE / HALT-resume; equals the
post-turn heading during DRIVE_PAST; equals the chosen gap bearing during a RECOVER
commit. After STRAFE the bearing was never lost, so DRIVE resumes straight. After TURN,
DRIVE_PAST deliberately holds the *turned* heading long enough to clear the obstacle
laterally **before** DRIVE's PID curves back toward `goal_heading` — preventing the
"re-aim straight back into the obstacle I just avoided" trap.

**DRIVE_PAST exit (precise).** Exit to DRIVE when
`FRONT ≥ safety_distance + clear_margin` **AND** the *inside* side-window (the side the
obstacle was on) has risen back above `pass_clearance` (obstacle now behind the flank),
both sustained for `clear_confirm_time`. Bounded fallback: after
`max_drive_past_distance` traveled, re-enter ASSESS. If FRONT re-blocks mid-DRIVE_PAST,
re-enter ASSESS.

---

## 4. Decision logic — the ASSESS ladder (measurable)

Evaluated once per obstacle event, top to bottom, first match wins. All thresholds are
concrete (§10); no qualitative terms.

**Step 0 — Localize & size.** From FRONT beams with `range ≤ obstacle_detect_range`:
- `preferred_side` = toward the more-open front sub-sector (generalized
  `turn_away_direction`).
- `obstacle_span_deg` = angular span of those near returns.
- `required_clearing` = (near-edge lateral offset on `preferred_side`) + `corridor_half`,
  where `corridor_half = robot_half_width + corridor_margin`.

**Step 1 — STRAFE if all hold:**
- (a) `preferred_side` window clearance ≥ `strafe_side_clearance_min` (side confirmed clear); **and**
- (b) `obstacle_span_deg ≤ max_obstacle_span_deg` (it's narrow, not a wall); **and**
- (c) `required_clearing ≤ max_strafe_distance` (a bounded step actually clears it); **and**
- (d) `cumulative_strafe_this_encounter + max_strafe_distance ≤ max_cumulative_strafe`
  (long-wall guard — haven't already strafed too far).
→ **STRAFE**, lock direction = `preferred_side`.

**Step 2 — TURN otherwise.** Turn toward the side with greater LEFT-vs-RIGHT window
clearance (tie → fixed default). Add a reverse component if
`FRONT < reverse_trigger_range` **and** `REAR ≥ rear_clearance_min`. → **TURN**, lock direction.

**Step 3 — Escalate.** `consecutive_avoid_count > max_avoid_attempts` →
**RECOVER** (once per encounter, guarded by `has_recovered_this_encounter`). Already
recovered and still here → **HALT**.

---

## 5. Maneuver completion — heading/clearance-based, with timeouts

Prefer completing on the **achieved state**, not a blind fixed duration; keep a timeout
purely as a bounded-safety cap (deterministic worst case).

- **STRAFE** completes when: FRONT clear+margin (success) **OR** `max_strafe_distance`
  laterally traveled (measured from odom-y delta) **OR** `strafe_timeout`. On non-success
  completion → ASSESS.
- **TURN** completes when: `|yaw − turn_target| ≤ heading_tol` (the turn increment is a
  target heading, `turn_step_deg` off current) **OR** `turn_timeout`. → DRIVE_PAST.
- **DRIVE_PAST** completes per its precise exit (§3), with `max_drive_past_distance` cap.
- **RECOVER rotate-to-gap** completes when `|yaw − gap_bearing| ≤ heading_tol` **OR**
  `rotate_timeout`.

`max_strafe_distance = strafe_speed × strafe_timeout` and similar are **derived**, not
independently configured (§10) — one physical quantity, one source of truth, no
conflicting knobs.

---

## 6. Obstacle-trigger debounce

To avoid reacting to a single spurious near-return, DRIVE triggers ASSESS only after
FRONT ≤ `safety_distance` on `obstacle_confirm_scans` consecutive scans (e.g. 2–3). A
single-scan dropout below threshold does not trigger a maneuver. The stale-scan
watchdog (`scan_timeout`) is independent and still forces HALT if scans stop arriving.
(The stop itself remains immediate at the control level — debounce gates the *maneuver
decision*, not the safety stop; if a confirmed obstacle is within `safety_distance` the
robot is already commanding zero forward while confirming.)

---

## 7. Recovery strategy (bounded, deterministic, current-data)

Runs **at most once per encounter** (`has_recovered_this_encounter`, reset only after
`clear_drive_duration` of sustained clean DRIVE). Never loops.

1. **Back off** — reverse at `avoid_reverse_speed` until `FRONT ≥ recover_backup_clearance`,
   stopping early if `REAR ≤ rear_clearance_min` (never reverse blindly into something).
2. **Select gap from the current 360° scan** — a *gap* is a contiguous bearing interval
   with min-range ≥ `min_gap_clearance` and angular width ≥ `min_gap_width_deg` (robot
   must fit). If none qualify → **HALT**. Otherwise pick the widest gap; tie-break by
   **smallest `|gap_bearing − goal_heading|`** (prefer the opening closest to the goal
   direction — directly serves the invariant).
3. **Rotate to face** the gap-center bearing (PID; completes on heading tolerance or timeout).
4. **Commit** — drive forward `recover_commit_distance` (bounded).
5. **Resolve** — FRONT clear+margin → **DRIVE** (`target_heading` back to `goal_heading`,
   PID re-acquires the A→B line). Re-block or gap gone → **HALT**.

---

## 8. Anti-oscillation mechanisms (the "commit-lock + hysteresis" core)

- **Commit-lock** — STRAFE/TURN lock their direction on entry and run to completion (§5)
  before ASSESS re-evaluates; a jittering apparent obstacle side cannot flip the command
  mid-maneuver.
- **Hysteresis exit** — returning to clean DRIVE requires
  `FRONT ≥ safety_distance + clear_margin` sustained for `clear_confirm_time`, not a bare
  threshold crossing.
- **Escalation counter** — `consecutive_avoid_count` resets **only** after
  `clear_drive_duration` of sustained clean DRIVE, so a fast HALT↔DRIVE flicker cannot
  reset escalation and mask a genuine trap.
- **Per-encounter one-shots** — `has_recovered_this_encounter` (and the strafe cumulative
  cap) ensure each escape tool fires a bounded number of times before the ladder falls
  through to HALT.

---

## 9. Decision logging (research instrumentation)

Separate from `trial_logger`'s motion CSV, the node emits one **structured decision
record per obstacle encounter** (and per maneuver within it), for research analysis and
debugging. Suggested fields:

`timestamp, encounter_id, state, chosen_maneuver, reason, obstacle_span_deg,
front_distance_m, front_left_m, front_center_m, front_right_m, left_clearance_m,
right_clearance_m, rear_clearance_m, required_clearing_m, cumulative_strafe_m,
consecutive_avoid_count, recovery_triggered, outcome (cleared / escalated / halted),
maneuver_duration_s`.

Emitted at ASSESS (the decision) and at each maneuver's completion (the outcome), to a
dedicated topic and/or CSV. This makes the deterministic decision process fully
auditable — essential for a research write-up and for tuning the §10 parameters against
real runs. `reason` is a short enum/string naming which ladder step fired and why
(e.g. `strafe: narrow+side_clear`, `turn: side_blocked`, `turn: span>max`,
`recover: attempts_exceeded`, `halt: no_gap`).

---

## 10. Parameters

**Tunable (independent):**

| Param | Example | Purpose |
|---|---|---|
| `safety_distance` | 0.30 m | FRONT stop threshold |
| `clear_margin` / `clear_confirm_time` | 0.10 m / 0.5 s | Hysteresis to declare "clear" |
| `obstacle_detect_range` | 0.40 m | Which beams count as "the obstacle" for sizing |
| `obstacle_confirm_scans` | 2 | Consecutive scans to confirm before a maneuver decision |
| `front_arc_deg` / `front_subsector_deg` | 180 / 60 | Forward arc + sub-sector width |
| `side_window_deg` / `rear_window_deg` | 60 / 60 | Lateral + rear window widths |
| `robot_half_width` / `corridor_margin` | 0.11 / 0.05 m | Corridor half-width for clearing math |
| `strafe_speed` / `strafe_timeout` | 0.25 m/s / 1.5 s | Lateral speed + bounded cap |
| `strafe_side_clearance_min` | 0.30 m | Side clearance required to strafe into it |
| `max_obstacle_span_deg` | 50° | Above this → wide → TURN not strafe |
| `max_cumulative_strafe` | 0.60 m | Hard per-encounter lateral cap (long-wall guard) |
| `turn_speed` / `turn_step_deg` / `turn_timeout` | 0.6 rad/s / 30° / 1.5 s | Turn rate, per-turn increment, bounded cap |
| `reverse_trigger_range` / `avoid_reverse_speed` / `rear_clearance_min` | 0.20 / 0.10 / 0.25 m·(m/s)·m | Reverse trigger, speed, rear-safety gate |
| `pass_clearance` / `max_drive_past_distance` | 0.35 / 0.80 m | DRIVE_PAST flank-passed + bounded cap |
| `heading_kp/ki/kd` / `heading_max_correction` / `heading_tol` | 1.0/0.0/0.1 / 0.3 / 5° | Bearing-hold PID + completion tolerance |
| `max_avoid_attempts` / `clear_drive_duration` | 3 / 3.0 s | Escalation trigger + counter reset |
| `recover_backup_clearance` / `recover_commit_distance` | 0.50 / 0.50 m | Recovery back-off + bounded commit |
| `min_gap_clearance` / `min_gap_width_deg` | 0.60 m / 40° | Usable-gap definition (robot must fit) |
| `control_rate_hz` / `scan_timeout` | 10 Hz / 0.5 s | Loop rate + stale-scan watchdog |

**Derived (computed, not configured — single source of truth):**

- `max_strafe_distance = strafe_speed × strafe_timeout`
- `corridor_half = robot_half_width + corridor_margin`
- `clear_threshold = safety_distance + clear_margin`

---

## 11. Edge cases & failure modes

1. **Fixed-height LiDAR blind spot** — thin/overhanging obstacles (pedestal desk) are
   invisible; software cannot fix this, avoidance only reacts to the scan plane.
   Documented limitation; keep obstacles with consistent cross-section at LiDAR height.
2. **Symmetric obstacle dead-center** — LEFT/RIGHT tie → fixed default side; commit-lock
   prevents flip.
3. **Wall / too-wide obstacle** — `max_obstacle_span_deg` + `max_cumulative_strafe` force
   TURN then escalation; never strafes indefinitely.
4. **Moving obstacle / person steps in** — reactive stop still fires; commit-lock may
   briefly commit toward a stale position but bounded completion caps exposure, then
   re-ASSESS. Acceptable at low speed.
5. **Stale / dropped scan** — `scan_timeout` watchdog → HALT.
6. **Open-loop yaw drift** — `goal_heading` held via EKF-fused yaw; drifts over long
   runs → bearing accuracy degrades with distance. Fine for short A→B tracks; noted.
7. **Imperfect strafe (mecanum)** — PID yaw-corrects during STRAFE + bounded distance
   limits residual drift.
8. **Blind reverse** — REAR window mitigates within the scan plane; blind spot remains;
   reverse stays slow/bounded.
9. **Recovery gap away from goal** — tie-break prefers the opening nearest `goal_heading`;
   bounded commit limits added deviation; re-block → HALT.
10. **Threshold chatter** — hysteresis (`clear_margin` + `clear_confirm_time`) eliminates.
11. **Trigger flicker** — `obstacle_confirm_scans` debounce rejects single spurious returns.

---

## 12. Improvements over the current algorithm

1. **Strafe-first preserves the goal bearing** → far less A→B deviation (improves
   trial-data quality) vs current turn-only + single blind drift.
2. **Fixed goal-bearing hold** vs current re-capturing "straight" at each DRIVE restart
   (which lets the robot wander off-line).
3. **360° side + rear awareness** vs current forward-only (which reverses blind and can't
   confirm a strafe side).
4. **Measurable strafe-viability** (span + required lateral clearing + confirmed side
   clearance) vs current's zero lateral reasoning.
5. **Long-wall guard** (`max_cumulative_strafe`) — new.
6. **Precise DRIVE_PAST** flank-passed logic vs current's immediate re-drive.
7. **Deterministic single recovery** using the current 360° scan to pick a fitting gap
   vs current halt-after-N with no gap-seeking.
8. **Formalized commit-lock + hysteresis** → stronger anti-oscillation than the partial
   locking present today.
9. **Heading/clearance-based completion with timeouts** vs current fixed-duration
   maneuvers → closes the loop on the achieved state.
10. **Trigger debounce** (`obstacle_confirm_scans`) → rejects spurious single-scan returns.
11. **Structured decision logging** → the deterministic decision process is fully
   auditable for research and tuning.

---

## 13. Open items for implementation planning

- Confirm sector angle conventions against the LD19's `0..2π` frame end-to-end
  (reuse/extend `scan_utils`).
- Decide decision-log transport: dedicated topic, CSV, or both; and whether it lives in
  `path_tracker` or a small companion node (parallels `trial_logger`).
- Validate `robot_half_width` and the LiDAR mounting offset (measured this session:
  lidar_frame is +7.3 cm forward, +3.9 cm above base_link) against the corridor math.
- Tuning pass on the §10 example values against real granite/wood/concrete/metal runs,
  using the decision log.
