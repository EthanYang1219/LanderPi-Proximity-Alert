# Trial NNN — <one-line description>

**Result: SUCCESS | FAIL | INCONCLUSIVE**
**Date:** YYYY-MM-DD · **Operator:** <name> · **Commit:** `<short sha>`

> Write this so that an undergraduate researcher opening this folder in six
> months immediately understands what was tested, under what conditions, what
> happened, and where the raw evidence is. Assume they remember nothing.

## What the robot was supposed to do

<Plain-language description of the intended behaviour. e.g. "Drive from A to B
in a straight line. Detect the box placed 1.0 m along the path, strafe left to
clear it, return toward the original line, and continue to B.">

## Configuration summary

Full detail is in `trial_metadata.yaml`. The parts that matter for reading
this run:

| | |
|---|---|
| Surface | |
| A→B distance | |
| Forward speed | |
| Avoidance trigger (`safety_distance`) | |
| Fusion gate | actuating / observe-only (`publish_cmd_vel`) |
| Stop window (`window_forward_m`) | |

<Call out anything that differed from the previous trial. If nothing changed,
say so explicitly — "identical configuration to trial_006" is useful.>

## Obstacle setup

<Type, dimensions, exact placement relative to A and to the A→B line. Include
whether it was tall enough for the LiDAR scan plane (> 0.086 m) and whether it
fell inside the depth marking band (0.10–0.99 m). A sketch or photo reference
here is worth a paragraph.>

## What happened

- **Reached B?** yes / no — <if no, where did it stop?>
- **Collision?** yes / no — <what did it hit, at what speed?>
- **Stuck / latched stop?** yes / no — <for how long, and did it recover?>
- **Transit time:** <from `trial_log.csv`> s
- **Avoidance events:** <from `trial_log.csv`>
- **Maneuvers performed:** <from `decision_log.csv` — the sequence of states>

<Then two or three sentences of narrative: what the run actually looked like.>

## Detection provenance

**<LIDAR | DEPTH | BOTH | UNVERIFIED>**

<If UNVERIFIED, say why in one sentence — normally: no ablation was run, and
the fusion stack running is not evidence that depth contributed. Do not
upgrade this on the basis of `camera_axis: FUSION_ACTIVE`, which only means
depth frames were arriving.>

## Evidence in this folder

| File | What it is |
|---|---|
| `trial_metadata.yaml` | Full run configuration |
| `trial_log.csv` | One row: transit time, odom vs. tape distance, slippage, avoidance count, stop range, battery |
| `decision_log.csv` | Per-decision avoidance state machine trace for this trial's time window |
| `scan_trace.jsonl` | Raw LiDAR scans (`ranges` + `sectors`) for this trial's time window |
| `video.mp4` | External camera recording of the run |

<Delete rows for files that are absent, and say why they are absent.>

## Notes and anomalies

<Anything unusual: unexpected stops, sensor dropouts, operator error, low
battery, a scan gap, a run aborted and restarted. Flag anomalies here rather
than editing the data — that is the project convention.>

<If this run contradicts an earlier result, say so plainly and link the
earlier trial.>
