# Documentation

This folder is the project's **evidence and design record**, not its manual.
If you want to understand or run the system, start at the
[top-level README](../README.md); come here when you want to know *why* a
threshold has the value it has, or what happened when something was tested on
the robot.

Everything here is dated and written as a record of work that happened. It is
kept deliberately, including the parts that document approaches that did not
pan out.

## What's in here

| Path | What it is | Read it when |
|---|---|---|
| [`superpowers/specs/`](superpowers/specs/) | Design specs — the reasoning behind each feature before it was built, including rejected alternatives. | You want the argument for a design decision. |
| [`superpowers/plans/`](superpowers/plans/) | Implementation plans — the task breakdowns those specs were built through. | You are tracing where a specific constraint came from. |
| [`poc_fusion_verification.md`](poc_fusion_verification.md) | Dated on-robot verification log for the depth + LiDAR fusion POC, ~6k lines, appended to per task. | You want the raw record of what was measured on the hardware. |
| [`poc_fusion_data/`](poc_fusion_data/) | CSVs captured during that verification (latency, wall events, stop persistence, series-gate runs). | You want the numbers behind a claim in the log. |
| [`poc_trials/`](poc_trials/) | Per-trial evidence folder structure and templates for the physical fusion POC. | You are running or reproducing a fusion trial. |
| [`trial_layouts.md`](trial_layouts.md) | Reference sheet for the obstacle layouts behind `avoidance_trials.csv`'s `layout_id` column. | You are reproducing an avoidance trial. |

The published research dataset itself is **not** here — it lives in
[`../organized_data/`](../organized_data/), indexed by
[`DATA_INDEX.md`](../organized_data/DATA_INDEX.md).

## Task numbering in `poc_fusion`

The fusion POC was built as a numbered task sequence, and those numbers appear
throughout `poc_fusion/`'s comments, parameter descriptions, and the section
headings of `poc_fusion_verification.md`. They are kept because they are the
only way to trace a tuned value back to the measurement that produced it —
several parameters are annotated as placeholders that a later task replaced.

**Task numbers are scoped to a plan, not global.** An unqualified "Task 4"
inside `poc_fusion/` means Task 4 of the fusion plan below. The plans under
`superpowers/plans/` each have their own Task 1, 2, 3 — a reference to "Task 1"
in `proximity_alert/` is a different task entirely.

Tasks of
[`2026-08-05-lidar-depth-costmap-fusion.md`](superpowers/plans/2026-08-05-lidar-depth-costmap-fusion.md):

| Task | Stage | Status |
|---|---|---|
| 0–2 | Environment preflight, TF chain verification, package scaffolding | Done |
| 3 | Pure geometry and detection logic, off-robot | Done |
| 4 | Depth preprocessing (`depth_preprocess_node`) | Done |
| 5 | Point-cloud generation (rectify → project) | Done |
| 6 | Costmap configuration (`nav2_costmap_2d` + lifecycle manager) | Done |
| 7 | Stop monitor (`costmap_stop_monitor_node`) | Done |
| 8 | Launch integration + end-to-end verification | Done — 8/9 goals verified; depth-only overhang detection **pending** |
| 9 | Stop action (`stop_action_node`) | Done |
| 10, 10a | Camera coverage bounds; overhang-test pose | Done |
| 11 | Validation and tuning | **Not started** — several parameters are placeholders awaiting it |
| 12 | Latency budget (`latency_recorder_node`) | Done |
| 13 | Compute budget | **Not collected** |
| 14 | Graceful degradation | Not started |
| 15 | Fusion benefit A/B measurement | Not started |
| 16 | Documentation | Not started |

A separate bench test —
[`2026-08-11-depth-contribution-bench-test-design.md`](superpowers/specs/2026-08-11-depth-contribution-bench-test-design.md)
— was run outside this sequence and **failed its Phase 0 gate**: depth cannot
contribute for floor-standing obstacles at this camera's mounting geometry.
That is the main open question hanging over the whole approach.

## A note on paths

`poc_fusion_verification.md` opens by citing a `.superpowers/sdd/...` path.
That was the authoring location during development; the specs and plans it
refers to are the ones now under [`superpowers/`](superpowers/) here.
