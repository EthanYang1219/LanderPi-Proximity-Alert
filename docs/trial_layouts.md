# Obstacle trial layouts

Reference sheet for `avoidance_trials.csv`'s `layout_id` column. The CSV only
stores the label you type at the prompt (e.g. `layout_A`) — the actual
obstacle geometry for that label lives here. Fill in the "Actual placement"
notes as you tape each layout down; they don't need to be exact to the
millimeter, just close enough to be reproducible on a later date.

**Assumptions carried from [README.md](../README.md):** `target_distance
≈ 2.0 m` (A→B), `safety_distance = 0.20 m` (front stop threshold),
`max_obstacle_width = 0.75 m` (above this, the controller treats the
obstacle as "wide" and turns instead of strafing). Default to a box with a
**consistent cross-section at the LiDAR's mounted height** — the README
documents a real collision caused by a pedestal desk whose thin/gapped
legs were invisible to the LiDAR's fixed scan plane (see the
[LiDAR blind-spot limitation](../README.md#project-overview)). Layout C
below deliberately uses chairs instead and carries a **mandatory static
verification step before any motion trial** for exactly this reason —
do not skip it.

All distances are measured **from point A, along the taped A→B line**,
and lateral offset is measured **from that line**, `+` = right, `-` = left,
matching the sign convention already used for
`ground_truth_lateral_offset_m`.

## Layout A — centered, narrow (baseline)

- **Obstacle:** 1 box, width < 0.75 m (narrow → expect `STRAFE`)
- **Target position:** ~1.0 m from A, centered on the line (0 m lateral offset)
- **Purpose:** baseline replicate of today's first two live trials —
  confirms the strafe-and-clear behavior is repeatable, not a one-off
- **Expected controller behavior:** `ASSESS` → `STRAFE` → `DRIVE` (cleared)
- **Actual placement (fill in when taped):**

## Layout B — offset, narrow

- **Obstacle:** 1 box, width < 0.75 m
- **Target position:** ~1.0 m from A, offset ~0.20 m left of the line
  (`obstacle_lateral_offset ≈ -0.20 m`)
- **Purpose:** tests whether an off-center obstacle still triggers a full
  strafe/turn decision, or whether the robot can clear it with only the
  cross-track correction already pulling it toward center
- **Expected controller behavior:** likely `STRAFE` toward the open (right)
  side, shorter strafe distance than Layout A; possibly no maneuver at all
  if 0.20 m offset is already outside the front arc's danger cone —
  worth noting either way
- **Actual placement (fill in when taped):**

## Layout C — wide, centered (new maneuver type)

- **Obstacle:** 1-2 chairs (4-legged), either a single chair turned to
  present its widest side, or 2 chairs placed side by side / linked to
  jointly exceed 0.75 m of effective width. No wide box was available, so
  this substitutes chairs for the "wall-like, wide" case — see the
  **mandatory verification step** below before this layout is used in any
  motion trial.
- **Target position:** ~1.0 m from A, centered on the line
- **Purpose:** every trial run so far (today's two, plus Layout A/B above)
  is expected to produce `STRAFE`. This layout is the only one in the set
  designed to exercise `TURN` instead — real coverage of a second maneuver
  type is worth more to the paper than a third narrow-obstacle repeat
- **Expected controller behavior:** `ASSESS` → `TURN` → `DRIVE_PAST` →
  `DRIVE` (cleared)

### Required: static LiDAR verification before this layout's first motion trial

Chair legs are exactly the shape that caused the documented pedestal-desk
collision (thin, gapped, possibly outside the scan plane) — do not assume
detection just because the chair "looks wide" to a person. Before running
the robot into this layout for the first time:

1. Place the chair(s) with **no robot motion**.
2. From the robot's static position, check the live LiDAR return against
   the chair at a few distances the robot will actually pass through
   during approach — e.g. `ros2 topic echo /scan_raw` filtered to the
   forward arc, or watch `scan_trace.jsonl`'s `sectors.front` value, at
   roughly 1.0 m, 0.5 m, and 0.3 m standoff.
3. Confirm the front-arc range **drops to a plausible finite value**
   (not `inf` / `range_max`) as the chair enters that standoff — i.e. the
   chair is actually visible to the LiDAR at the height it's mounted at,
   not just to a person looking at it.
4. Only run the live avoidance trial once detection is confirmed at all
   three standoffs. If any standoff shows no detection, reposition the
   chair (turn it, add a second linked chair, lower/raise nothing — the
   LiDAR height is fixed) and re-verify before proceeding; do not run the
   motion trial on an unconfirmed layout.

- **Actual placement (fill in when taped):**
- **Verification result (fill in before first motion trial):**

## Layout D — narrow, late encounter

- **Obstacle:** 1 box, width < 0.75 m
- **Target position:** ~1.6-1.7 m from A (close to B), centered on the line
- **Purpose:** stress-tests detection/decision timing under a short
  reaction window — confirms `obstacle_confirm_scans` debounce and stopping
  distance hold up even with less approach distance to react in
- **Expected controller behavior:** `STRAFE` as in Layout A, but check
  `lidar_stop_range_m` / `front_distance_m` in `decision_log.csv` for a
  tighter margin than Layout A's
- **Actual placement (fill in when taped):**

## Trial-count plan for today

3 trials per layout = 12 baseline. If time allows after all four layouts
are done once, add a 4th pass on whichever layout produced the most
interesting or borderline result (closest call, unexpected maneuver,
etc.) to reach the 15-trial stretch goal — note in that trial's `cause`
field which layout it repeats and why.

## Video

One phone recording per trial, fixed tripod framing the full A→B line and
obstacle zone. Name each file `<layout_id>_trial<N>.mp4` (e.g.
`layout_A_trial1.mp4`) so it's unambiguous which `avoidance_trials.csv`
row it corresponds to. Onboard camera capture is a stretch goal for
today, attempted only if the phone-recording cadence above isn't at risk.
