# Depth-contribution bench test — design

**Date:** 2026-08-11
**Status:** approved, not yet executed
**Relates to:** Task 11 (validation), Task 15 (fusion benefit A/B) of
[2026-08-05-lidar-depth-costmap-fusion.md](../plans/2026-08-05-lidar-depth-costmap-fusion.md)
**Motivated by:** Task 9 Part 2, which established that the POC's only
demonstrated obstacle detection to date is **LiDAR-only**.

---

## 1. Why this test exists

Task 9 Part 2 proved the stop chain end to end on a real obstacle:
`motion → wall → LiDAR → costmap → monitor → gate → /cmd_vel zero → physical halt`.
It also proved, by direct measurement, that **the depth camera contributed
nothing** to that detection: depth's reach in the forward corridor is 0.668 m,
the wall was detected at 0.921 m, and there were zero depth points in the
detection band.

That is not a bug. It is geometry, and it is not fixable by moving the obstacle:

| Quantity | Value |
|---|---|
| Detection window (`window_forward_m`) | 1.000 m |
| Depth reach, forward corridor, 40° pose (measured) | **0.668 m** |
| LiDAR reach | metres |

A floor-standing obstacle enters the LiDAR's window ~0.33 m before depth could
contribute. **No floor-standing obstacle at any distance can demonstrate depth's
contribution**, because the robot always reaches 1.0 m first. Repeating the Part 2
wall test would reproduce the Part 2 result.

So the question this test answers is not "does the robot stop". It is:

> **Does the depth camera add obstacle coverage the LiDAR does not have — and
> does that coverage survive in the costmap?**

## 2. The two hypotheses

**H1 — Coverage.** The LD19 has a fixed scan plane. An object shorter than that
plane is invisible to it (the failure mode behind the 2026-07-22 pedestal-desk
collision). Depth, at the 40° pose, sees the floor from 0.114 m outward and marks
obstacles in the 0.10–0.99 m height band. So a **low object below the scan plane**
should be seen by depth and missed by LiDAR.

**H2 — Clearing erases that coverage.** From `costmap_params.yaml`:

```yaml
scan:        clearing: true    min_obstacle_height: 0.0   max_obstacle_height: 2.0
pointcloud:  clearing: true    min_obstacle_height: 0.10  max_obstacle_height: 0.99
             observation_persistence: 0.3
```

`nav2_costmap_2d::ObstacleLayer` raytraces clearing along every LaserScan beam. A
beam that passes *over* a low object and returns from a wall behind it clears
every cell between the sensor and that return — **including cells depth just
marked**. At `update_frequency: 10.0` against `observation_persistence: 0.3`, the
LiDAR may be continuously erasing depth's marks for precisely the obstacle class
depth exists to catch.

H2 is a hypothesis, not a finding. It is stated so it can be measured rather than
discovered later by collision. **If H2 holds, the fusion costmap does not work for
its intended purpose**, and that is a more important result than H1.

## 3. Scope and safety

**Entirely static. The robot does not move at any point in this test.**

- Launch with `publish_cmd_vel:=false`, so `stop_action_node` creates **no Twist
  publisher at all**. Verified by `ros2 topic info /cmd_vel` → `Publisher count: 0`.
- No motion harness is started. No `/cmd_vel_raw` publisher is created.
- The LiDAR-only control arm — the one arm where the robot would be *expected not
  to stop* — is run **statically**. Proving a sensor cannot see something does not
  require driving at it. This removes the only genuinely dangerous arm of the
  experiment.
- `proximity_alert` and the surface-trial data pipeline are not touched.

## 4. Phases

Each phase gates the next. A failed gate stops the test and is reported as a
finding, not worked around.

Throughout, **"the forward window"** means the monitor's detection region as
reported by `monitor_status`: `window_forward_m 1.000`, `window_half_width_m
0.300`, cells at or above `lethal_threshold 253`. These values are read from the
live diagnostic at test time, not assumed from this document.

### Phase 0 — Geometry

Measure, do not assume:

1. TF `base_footprint` → LiDAR frame. The **z translation is the scan-plane
   height** — a number the repository has never recorded.
2. Physical ruler check of the same height, floor to scan plane.
3. Re-measure depth's forward-corridor reach (the 0.668 m figure assumes the arm
   has not sagged since 2026-08-11).
4. Re-confirm the camera pose per Task 10 Step 1 (±1° tolerance).

Yields the target obstacle height band:

```
lower = 0.10 m                      (pointcloud min_obstacle_height)
upper = LiDAR scan-plane height     (above this, LiDAR sees it; test is void)
```

**Gate:** if `upper − lower < 0.04 m`, the discriminating obstacle class does not
physically exist at this pose. Report that and stop. Do **not** lower
`min_obstacle_height` to widen the band — that parameter exists to reject floor
returns, and changing it to manufacture a testable object would invalidate both
this test and Task 8's floor-filter verification.

### Phase 1 — Empty-floor baseline (negative control)

Nothing in front of the robot. Record over ≥30 s:

- lethal cells in the forward window: expect **0**
- monitor state: expect **CLEAR**
- depth points above the floor filter in the corridor: expect **0**

If the empty floor produces marks, every later measurement is noise. Stop.

### Phase 2 — Object present, fused config as shipped

Object at **0.45 m** — inside depth's reach, inside the 1.0 m window. Four
independent measurements, robot stationary:

| # | Measurement | Source |
|---|---|---|
| 1 | depth points on the object | `/poc_fusion/points` → TF → `base_footprint`, counted in the object footprint and height band |
| 2 | lethal cells in the forward window | `/costmap/costmap_raw` |
| 3 | monitor state → `OBSTACLE` | `/costmap_app/monitor_status` |
| 4 | **persistence of the mark over ≥30 s** | time series of #2, reported as duty cycle |

Measurement 4 tests H2. **A mark that appears and is erased 100 ms later is not
detection**, and reporting only "cells were marked" would hide that. The reported
figure is the fraction of samples in which the object's cells were lethal, plus
the longest continuous marked and unmarked runs.

### Phase 3 — LiDAR-only control (ablation), static

Same object, same position, a control config differing from the fused config in
exactly one line (`observation_sources: "scan"`), per Task 15 Step 1. Diff the two
files and record the diff.

Expect: **0 lethal cells**, monitor `CLEAR`.

**This arm is what makes provenance verified rather than guessed.** Phase 2 marks
combined with Phase 3 no-marks attributes the detection to depth by ablation.
Without it, a mark in Phase 2 proves nothing about which sensor produced it —
the exact error Task 9 Part 2 had to resolve after the fact.

If Phase 3 *does* show marks, the object is not below the scan plane, H1's
premise fails, and the object must be replaced with a shorter one before Phase 2
means anything.

### Phase 4 — The clearing test

Same object, two placements, fused config:

| Placement | Behind the object | Prediction under H2 |
|---|---|---|
| **A** | open space, no return within LiDAR range | marks persist |
| **B** | wall ~1.5 m behind | LiDAR raytraces through the object's cells → **marks erased** |

Report the Phase 2 measurement set for each. A persists / B fails isolates LiDAR
clearing as the mechanism. Both persisting falsifies H2, which is equally a
result worth having.

## 5. What must not happen

- No changes to `min_obstacle_height`, `max_obstacle_height`,
  `observation_persistence`, `clearing`, or any window/threshold parameter to make
  a phase pass. If depth cannot mark the object, or clearing erases it, that is
  the finding.
- No claim that "fusion worked" on the basis of the fused config being loaded.
  `camera_axis: FUSION_ACTIVE` means depth frames are arriving, **not** that depth
  contributed to a detection. Attribution comes from the Phase 3 ablation only.
- No inference of one phase's result from another's.
- If the causal chain cannot be established from the recorded data, the result is
  **INCONCLUSIVE**, not a pass.

## 6. Deliverables

- Per-condition CSV time series of lethal-cell counts, committed under
  `docs/poc_fusion_data/`.
- A new dated entry appended to `docs/poc_fusion_verification.md`. Previous
  evidence is not overwritten.
- The LiDAR-only control config, committed as an additive control file that leaves
  the fused config untouched.
- A determination on H1 and H2 separately, each PASS / FAIL / INCONCLUSIVE.

## 7. Operator setup

Target height is issued after Phase 0. Object requirements:

- **Flat-topped and rigid** — no soft or domed tops; depth needs a surface to
  return from
- **Matte, mid-tone** — not black (absorbs IR), not glossy (scatters it)
- **≥ 15 × 15 cm footprint** so it spans multiple 5 cm costmap cells
- **Height:** within the Phase 0 band, likely 0.10–0.20 m

Placement A needs ~2.5 m of clear floor behind the object. Placement B needs a
flat wall ~1.5 m behind it.

## 8. Out of scope

- Overhanging obstacles. At the 40° pose nothing above 0.246 m is ever in frame,
  so the overhang class is not testable until Task 10a.
- Any motion test. If H1 and H2 both pass, a motion run at the low object becomes
  worth designing — as a separate decision, not an extension of this one.
- Re-running the Task 12 latency collection to resolve its own provenance question.
  Related, but a distinct measurement.
