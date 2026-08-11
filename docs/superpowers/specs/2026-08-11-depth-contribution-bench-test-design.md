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

**H1 — Coverage.** The LD19 scans in a fixed plane. An object whose body **does
not intersect that scan plane** produces no LaserScan return (the failure mode
behind the 2026-07-22 pedestal-desk collision). This is deliberately *not* phrased
as "invisible to LiDAR" — the claim under test is narrow and specific: **does the
LD19's 2D scan plane produce an obstacle return from this object?** Depth, at the
40° pose, marks obstacles in the 0.10–0.99 m height band. So a **low object below
the scan plane** should be observed by depth and not by the scan.

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
3. **Depth reach within the target height band** — see below.
4. Re-confirm the camera pose per Task 10 Step 1 (±1° tolerance).
5. **Scan no-return encoding and effective clearing ranges** — see below.

Items 1–2 yield the target obstacle height band:

```
lower = 0.10 m                      (pointcloud min_obstacle_height)
upper = LiDAR scan-plane height     (above this, the object intersects the
                                     scan plane and the test is void)
```

**Gate:** if `upper − lower < 0.04 m`, the discriminating obstacle class does not
physically exist at this pose. Report that and stop. Do **not** lower
`min_obstacle_height` to widen the band — that parameter exists to reject floor
returns, and changing it to manufacture a testable object would invalidate both
this test and Task 8's floor-filter verification.

#### 0.3 Depth reach must be measured in the target band, not on the floor

The 0.668 m figure from Task 9 Part 2 is the reach of the **forward corridor as a
whole**, dominated by floor returns. It does not establish reach for a raised
object. Because of projection and occlusion geometry, usable reach in the
0.10 m → scan-plane interval may differ substantially from reach at floor level.

Measure instead: **the forward distance range over which `/poc_fusion/points`
yields valid points with `0.10 m ≤ z ≤ scan-plane height` in the forward
corridor.** That interval, not the floor figure, is what constrains object
placement. If it excludes the intended 0.45 m placement, the placement moves to
fit the measurement — not the other way round.

#### 0.5 What does a no-return beam look like, and how far does clearing reach?

This determines whether Phase 4 Placement A is a no-clearing condition at all
(§4 Phase 4). `nav2_costmap_2d::ObstacleLayer` drops `inf` LaserScan returns
unless `inf_is_valid` is set, so those beams clear nothing — but a driver that
reports no-return as `0.0` or as `range_max` produces beams that **do** clear, out
to the raytrace range. Record, from live data and the running node:

- the value `/scan_raw` carries for beams with no return (`inf`, `0.0`,
  `range_max`, or something else), and what fraction of beams that is
- `inf_is_valid`, `raytrace_max_range`, `obstacle_max_range`, `raytrace_min_range`
  as **effectively in force** — `costmap_params.yaml` declares none of them, so
  nav2 defaults apply and must be read off the node rather than assumed

Nothing here is changed. These are read to make Phase 4 interpretable.

### Phase 1 — Empty-floor baseline (negative control)

Nothing in front of the robot. Record over ≥30 s:

- lethal cells in the forward window: expect **0**
- monitor state: expect **CLEAR**
- depth points above the floor filter in the corridor: expect **0**

If the empty floor produces marks, every later measurement is noise. Stop.

### Phase 2 — Object present, fused config as shipped

**Placement at 0.45 m is a designed parameter, not an arbitrary one:** it sits
comfortably inside the depth coverage measured in Phase 0.3 while remaining well
within the monitor's 1.0 m detection window, so neither limit is marginal. If
Phase 0.3 returns a target-band reach below ~0.55 m, the placement moves inward to
preserve that margin, and the new value is recorded with its reason.

**Define the object ROI before recording anything.** Measure the object's footprint
in `base_footprint` — near and far x, left and right y — from its physical
placement. Every count below is reported **inside this ROI**, with the
forward-window total kept as a secondary diagnostic. A bare "37 lethal cells" does
not establish that the cells are the object; cells inside its footprint do.

Five measurements, robot stationary:

| # | Measurement | Source |
|---|---|---|
| 1 | depth points in ROI ∩ target height band; plus their centroid and z range | `/poc_fusion/points` → TF → `base_footprint` |
| 2 | **lethal cells inside the object ROI** | `/costmap/costmap_raw` |
| 3 | lethal cells in the forward window, and the bounding box of all lethal cells (secondary) | `/costmap/costmap_raw` |
| 4 | monitor state | `/costmap_app/monitor_status` |
| 5 | **persistence of #2 over ≥30 s** | time series, reported as duty cycle |

Measurement 5 tests H2. **A mark that appears and is erased 100 ms later is not
detection**, and reporting only "cells were marked" would hide that. Report the
fraction of samples in which ROI cells were lethal, plus the longest continuous
marked and unmarked runs.

**Measurement 4 is integration evidence, not a fusion verdict.** Depth marking the
costmap and the monitor raising `OBSTACLE` are two different claims, and the
second is not a prerequisite for the first. If depth marks the object but the
monitor stays `CLEAR` because the object's cells fall outside its exact window,
that is a monitor/window finding — **not** evidence that fusion failed. The two
are reported separately and never collapsed into one verdict.

### Phase 3 — LiDAR-only control (ablation), static

Same object, same position, a control config differing from the fused config in
exactly one line (`observation_sources: "scan"`), per Task 15 Step 1.

**Diff the two files and require the diff to be exactly that one line** — commit
the diff as evidence. If any other line differs, the ablation is confounded and
the control must be regenerated from the fused config before Phase 3 runs. The
`pointcloud:` block itself stays present but unreferenced, so the only change is
which sources the layer consumes.

Expect: **0 lethal cells**, monitor `CLEAR`.

**This arm is what makes provenance verified rather than guessed.** Phase 2 marks
combined with Phase 3 no-marks attributes the detection to depth by ablation.
Without it, a mark in Phase 2 proves nothing about which sensor produced it —
the exact error Task 9 Part 2 had to resolve after the fact.

If Phase 3 *does* show marks, the object is not below the scan plane, H1's
premise fails, and the object must be replaced with a shorter one before Phase 2
means anything.

### Phase 4 — The clearing test

Same object, two placements, fused config, full Phase 2 measurement set for each:

| Placement | Behind the object |
|---|---|
| **A** | no intentional rear return — ~2.5 m of clear floor |
| **B** | flat wall ~1.5 m behind |

**Placement A is an empirical control, not an assumed no-clearing condition.**
An open-space scan does not guarantee that no raytracing occurs: if the LD19
encodes no-return as `0.0` or `range_max` rather than `inf`, those beams are
valid observations and the `ObstacleLayer` clears along them out to
`raytrace_max_range` — straight through the object's cells, with no wall
involved. Phase 0.5 measures which case holds; A then measures whether clearing
actually occurs under the real scan configuration, rather than presuming it does
not.

Interpretation is therefore a 2×2, and three of the four cells are informative:

| A | B | Interpretation |
|---|---|---|
| persists | erased | Strong evidence the **rear return** causes the additional clearing. H2 supported, mechanism isolated. |
| erased | erased | Clearing occurs **even without a rear wall**. H2 remains plausible, but the wall-specific mechanism is **not** isolated — cross-check against Phase 0.5's no-return encoding. |
| persists | persists | Proposed clearing mechanism **not supported**. H2 fails. |
| unstable | unstable | Investigate before interpreting. Do not report a verdict. |

The "erased / erased" row is the reason A must be measured rather than assumed:
without it, that outcome would be misread as the wall having done the damage.

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

## 6. Verdicts

H1 and H2 are judged **separately**, and neither depends on the monitor firing.

### H1 — depth adds coverage the scan plane does not

| | |
|---|---|
| **PASS** | Depth yields valid points on the object within the target height band and ROI, **and** the Phase 3 LiDAR-only ablation yields no corresponding lethal cells. |
| **FAIL** | Depth yields no valid object observations in the band, **or** the LiDAR-only ablation also marks the object — meaning the object intersects the scan plane and the premise is void. |
| **INCONCLUSIVE** | Depth points exist but cannot be reliably associated with the object, or costmap attribution is ambiguous. |

### H2 — LiDAR clearing suppresses depth-derived obstacle cells under the tested configuration

| | |
|---|---|
| **PASS** | Depth marks appear, but under the clearing condition they are measurably erased or reduced relative to the control condition. |
| **FAIL** | Depth marks persist despite the clearing condition. |
| **INCONCLUSIVE** | Costmap behaviour is unstable, or clearing cannot be distinguished from another cause. |

H2 is only meaningful if H1 passes: there is nothing to erase otherwise.

## 7. Deliverables

- Per-condition CSV time series of ROI and forward-window lethal-cell counts,
  committed under `docs/poc_fusion_data/`.
- **A raw depth-evidence sample per condition** — not the full 30 s cloud, which
  would be unmanageably large, but enough to reconstruct the H1 claim
  independently: transformed XYZ, timestamp, in-ROI / in-band classification, and
  valid-point count, for a few representative frames. Plus a `/scan_raw` sample
  covering the object's bearing, so the "no scan return" claim is checkable rather
  than asserted.
- A new dated entry appended to `docs/poc_fusion_verification.md`. Previous
  evidence is not overwritten.
- The LiDAR-only control config **and its one-line diff**, committed as additive
  control artifacts that leave the fused config untouched.
- Separate determinations for H1 and H2, each PASS / FAIL / INCONCLUSIVE.

## 8. Time-box

This is research validation running against a two-day POC deadline whose priority
is **A → obstacle → avoidance → B → video**. This test must not consume that time.

**If Phase 0 or Phase 1 fails its gate, stop and record the finding.** If the
measurements do not come together within a single controlled block, stop and
record the state reached. Do not debug toward a passing result.

The design is deliberately built so that **"depth does not contribute" is an
acceptable scientific outcome**. That is what keeps this from becoming a rabbit
hole: there is no result here that needs to be forced, and a clean negative is
publishable evidence about the 40° pose rather than a failure to be fixed.

## 9. Operator setup

Target height is issued after Phase 0. Object requirements:

- **Flat-topped and rigid** — no soft or domed tops; depth needs a surface to
  return from
- **Matte, mid-tone** — not black (absorbs IR), not glossy (scatters it)
- **≥ 15 × 15 cm footprint** so it spans multiple 5 cm costmap cells
- **Height:** within the Phase 0 band, likely 0.10–0.20 m

Placement A needs ~2.5 m of clear floor behind the object. Placement B needs a
flat wall ~1.5 m behind it.

## 10. Out of scope

- Overhanging obstacles. At the 40° pose nothing above 0.246 m is ever in frame,
  so the overhang class is not testable until Task 10a.
- Any motion test. If H1 and H2 both pass, a motion run at the low object becomes
  worth designing — as a separate decision, not an extension of this one.
- Re-running the Task 12 latency collection to resolve its own provenance question.
  Related, but a distinct measurement.
