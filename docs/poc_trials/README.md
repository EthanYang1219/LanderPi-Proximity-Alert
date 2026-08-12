# POC trial folders

Self-contained evidence folders for the fusion-costmap physical POC
(A → obstacle → avoidance → B). One folder per trial run.

This is an **additive organization layer**. It does not modify the frozen
`proximity_alert` research-data pipeline in any way — it slices the shared
logs that pipeline already writes, and adds configuration context those logs
never captured.

## Where things live

| What | Where | In git? |
|---|---|---|
| Full trial folder (CSVs, JSONL, video) | `/home/pi/poc_trials/trial_NNN/` | **No** — too large |
| `README.md` + `trial_metadata.yaml` copies | `docs/poc_trials/trial_NNN/` | **Yes** |
| Templates | `docs/poc_trials/templates/` | Yes |

Trial numbering is **global** — `trial_001`, `trial_002`, … across every POC
run regardless of surface, obstacle, or configuration. The number is never
reused and never renumbered.

## Folder contents

```
trial_007/
├── README.md              <- written by the operator after the run
├── trial_metadata.yaml    <- configuration, filled in BEFORE the run
├── trial_log.csv          <- sliced from the shared per-trial CSV
├── decision_log.csv       <- sliced from the shared decision log
├── scan_trace.jsonl       <- sliced from the shared scan trace
└── video.mp4              <- external camera recording of the run
```

## What is captured automatically vs. manually

The frozen pipeline already records a great deal. The point of
`trial_metadata.yaml` is to record only what it *doesn't*.

**Automatic** (do not re-enter these by hand):

- `trial_log.csv` — one row, written at trial END by `trial_logger`.
  Transit time, odom vs. ground-truth distance, slippage, avoidance event
  count, LiDAR stop range, battery bucket. Columns documented in
  [organized_data/raw/trial_logs/README.md](../../organized_data/raw/trial_logs/README.md).
- `decision_log.csv` — one row per avoidance state-machine decision, written
  by `decision_logger`. State, chosen maneuver, reason, per-sector clearances,
  cumulative strafe, outcome, maneuver duration.
- `scan_trace.jsonl` — one line per LiDAR scan tick: full `ranges` array plus
  the derived `sectors` digest.

**Manual** — the configuration under which the run happened. None of it
appears in any log:

- Which obstacle, what size, where it was placed
- Robot speed and avoidance parameters actually in force
- Camera mounting angle and RGB-D configuration
- Which git commit was deployed
- Whether the fusion gate was actuating or observing only

## How slicing works

None of the three shared logs carry a trial ID. `decision_log.csv` and
`scan_trace.jsonl` are **shared, append-only, and global** — every trial ever
run writes into the same file.

The only available key is time. `trial_log.csv` gives both ends of the window:

```
trial_end   = row["timestamp"]
trial_start = trial_end - row["transit_time_s"]
```

`transit_time_s` is measured start-of-motion → confirmed stop, so this window
covers the motion phase only. Slicing pads the window at both ends
(`slice_pad_s` in the metadata) to catch the approach and the settle.

**Caveats that must be respected when reading a sliced folder:**

- The host is HKT (UTC+8); ROS/container timestamps are UTC. Slicing must
  normalize before comparing, or it will silently return an empty window.
- The window is *derived*, not recorded. If two trials run back-to-back
  within the pad, their decision rows can overlap. Keep ≥ 10 s between runs.
- `scan_trace.jsonl` uses `stamp_sec`/`stamp_nanosec` (ROS time), not the
  wall-clock strings the CSVs use. These are different clocks — do not assume
  they agree without checking.

## Procedure for one trial

1. Copy `templates/trial_metadata.template.yaml` into the new folder as
   `trial_metadata.yaml`. Defaults are pre-populated from the config files
   in the repo — **verify them against what is actually deployed**, and fill
   in every `TODO` field before the robot moves.
2. Set up and measure the obstacle. Record dimensions and placement.
3. Start the external video recording.
4. Run the trial.
5. Slice the logs into the folder.
6. Write `README.md` from `templates/trial_README.template.md`.
7. Copy `README.md` and `trial_metadata.yaml` into `docs/poc_trials/trial_NNN/`
   and commit those two files only.

## Honesty rules for these folders

These carry the same standards as the rest of the project:

- A trial that fails is written up as a failure, with its evidence intact.
  Do not delete a folder because the run went badly.
- **Detection provenance is not inferable from these logs.** The fusion stack
  running is not evidence the depth camera contributed. Unless an ablation
  was run, `detection_provenance` is `UNVERIFIED`. See
  [poc_fusion_verification.md](../poc_fusion_verification.md).
- Never edit a sliced CSV to "clean up" a run. Flag anomalies in the README,
  matching the convention used in `organized_data/`.
