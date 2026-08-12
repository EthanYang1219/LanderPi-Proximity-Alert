# Data index

Quick reference for every data file in this project. For full column
definitions and context, see the `README.md` in each subfolder.

| File | Location | What it is | Source |
|---|---|---|---|
| `carpet.csv` | `raw/trial_logs/` | Per-trial motion log (transit time, odom vs. ground-truth distance, slippage, avoidance events, battery) — carpet surface | `trial_logger` node (auto) |
| `granite.csv` | `raw/trial_logs/` | Same as above — granite surface. Has flagged data-integrity issues, see [raw/trial_logs/README.md](raw/trial_logs/README.md#known-data-integrity-flags-not-corrected-here--see-callers-guidance) | `trial_logger` node (auto) |
| `wood.csv` | `raw/trial_logs/` | Same as above — wood surface, 15 trials. Has a flagged outlier row, see [raw/trial_logs/README.md](raw/trial_logs/README.md#known-data-integrity-flags-not-corrected-here--see-callers-guidance) | `trial_logger` node (auto) |
| `decision_log.csv` | `raw/decision_logs/` | Per-decision avoidance log (state machine transitions, clearances, maneuver outcomes), all surfaces | `decision_logger` node (auto) |
| `scan_trace.jsonl` | not copied — see [raw/scan_traces/README.md](raw/scan_traces/README.md) | Per-scan-tick raw LiDAR trace (~1 GB, lives at `trials/scan_trace.jsonl` / `/home/pi/docker/tmp/trials/`) | `scan_trace_logger` node (auto) |
| `floor_test_log.csv` | `processed/` | Original hand-maintained PID/safety-distance tuning log (legacy schema) | manual |
| `wood_floor_test_log.csv` | `processed/` | Separate per-surface (wood) hand-maintained tuning log, expanded schema | manual |

## Fusion-costmap POC trials

Trials for the fusion-costmap POC are organized differently — one
self-contained folder per run, slicing the shared logs above by time window
and adding the run configuration those logs never captured. See
[docs/poc_trials/README.md](../docs/poc_trials/README.md). The full folders
live outside git at `/home/pi/poc_trials/`; only each trial's `README.md` and
`trial_metadata.yaml` are committed.

## Where the live data actually is

`raw/` and `processed/` here are **copies**, taken 2026-07-29, for a clean
browsable snapshot. The live files this repo actually reads/writes are:

- `trials/carpet.csv`, `trials/granite.csv`, `trials/wood.csv`,
  `trials/decision_log.csv`, `trials/scan_trace.jsonl` — symlinks to
  `/home/pi/docker/tmp/trials/`
  (the container's bind-mounted logging destination). These update every
  time a trial/decision/scan is logged.
- `trials/floor_test_log.csv`, `trials/wood_floor_test_log.csv` — real files,
  hand-edited or updated by `floor_test_reconcile.py`.

If you need current data, re-copy from `trials/` rather than trusting the
snapshot in `organized_data/` to be up to date.

## Folder structure

```
organized_data/
├── DATA_INDEX.md          <- this file
├── raw/                   <- machine-written logs, never hand-edited
│   ├── README.md
│   ├── trial_logs/
│   │   ├── README.md
│   │   ├── carpet.csv
│   │   ├── granite.csv
│   │   └── wood.csv
│   ├── decision_logs/
│   │   ├── README.md
│   │   └── decision_log.csv
│   └── scan_traces/
│       └── README.md      <- data not copied (see file for why)
└── processed/              <- hand-maintained session summaries
    ├── README.md
    ├── floor_test_log.csv
    └── wood_floor_test_log.csv
```
