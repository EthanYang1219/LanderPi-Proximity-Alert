# Scan trace logger — design spec

**Date:** 2026-07-24
**Status:** Approved for implementation planning

## 1. Problem

The current logging setup (`trial_logger` → per-trial motion/slippage CSV, `decision_logger` → per-*decision* CSV) only produces a record when `path_tracker`'s `AvoidanceController` actually notices an obstacle and reacts. That covers cases like the strafe-past-a-box scenario well, but it structurally cannot capture a **total miss**: on 2026-07-23/24 hardware testing, the robot drove past a box successfully but then failed to detect a chair leg. A thin, low-cross-section obstacle like a chair leg may fall entirely outside the LD19's fixed 2D scan plane (the same class of blind spot already documented for the pedestal-desk incident) — in which case `front` reads clear the entire time, no encounter is ever entered, and neither existing CSV has any row to show for it. The failure is invisible to the data even though it happened.

There are two different failure mechanisms that look identical from the outside (robot didn't react) but require different fixes:

1. **Sensor-geometry blind spot** — the obstacle's cross-section never intersected the LiDAR's scan plane. Not fixable by tuning `safety_distance` or any other `AvoidanceConfig` parameter; only fixable by changing sensor placement/count.
2. **Threshold miss** — the obstacle *was* in the scan, but its range/width fell outside `obstacle_detect_range` or a similar gate. Fixable by tuning parameters.

Distinguishing these two requires access to the raw beam data around the moment of the miss, not just the reduced sector summary `path_tracker` currently computes and discards each tick.

## 2. Goal

Add a new, independent logging node that continuously records the **raw LiDAR scan** (every beam, not just reduced sectors) for the full duration of a trial, so a miss discovered after the fact can be traced back to raw sensor data and classified as (1) or (2) above.

## 3. Non-goals

- Not attempting to fix the chair-leg detection failure itself (no new avoidance logic, no new sensor).
- Not replacing `decision_log.csv` or the trial CSV — this is a third, independent log, same relationship the other two already have to each other.
- Not doing real-time miss detection/alerting. This is an offline-analysis aid: capture now, diagnose later.
- Not downsampling or binning the raw ranges — any fixed-width binning risks losing exactly the thin-obstacle signal this log exists to catch.

## 4. Architecture

A fourth node, `scan_trace_logger.py`, following the same independence principle already established by `trial_logger` (watches `/odom` on its own) and `decision_logger` (watches `/avoidance_decision` on its own): it subscribes **directly to `/scan_raw`**, with no dependency on `path_tracker`'s internals or process lifetime. Any node can be developed, run, or omitted without affecting the others.

Per scan message received, it:

1. Extracts the raw scan fields needed to fully reconstruct beam geometry later (see §5).
2. Calls the existing pure `reduce_to_sectors(...)` from `scan_utils.py` — the same function `path_tracker` already uses — to attach the reduced sector summary alongside the raw data. This means a coarse pass ("was front ever below X") doesn't require re-deriving sector logic from raw ranges by hand; the raw ranges remain available for the deep-dive blind-spot check.
3. Appends one JSON Lines (`.jsonl`) record to the configured output file.

No new topics are published; this node only consumes and persists.

## 5. Record schema

One JSON object per line (JSON Lines, not a single JSON array — so the file is streamable/appendable and a partially-written file from an interrupted run is still readable up to the last complete line):

```json
{
  "timestamp": "2026-07-24 14:32:07.183241",
  "angle_min": -3.14159,
  "angle_increment": 0.01745,
  "range_min": 0.12,
  "range_max": 25.0,
  "ranges": [0.45, 0.46, "inf", ...],
  "sectors": {
    "front": 0.45, "front_left": 1.2, "front_center": 0.45, "front_right": 0.9,
    "left": 1.4, "right": 0.9, "rear": 2.1
  }
}
```

Field notes:

- `timestamp` — wall-clock, **microsecond precision** (`%Y-%m-%d %H:%M:%S.%f`). This is a deliberate difference from `trial_logger`'s/`decision_logger`'s second-resolution timestamps: correlating "which tick was the robot passing the chair leg" against the trial CSV's start/end times needs sub-second resolution, since a scan tick is ~100ms apart at the LD19's observed ~10Hz rate.
- `angle_min`, `angle_increment`, `range_min`, `range_max` — copied verbatim from the incoming `LaserScan` message. Together with `ranges`, this fully reconstructs each beam's angle via `angle = angle_min + i * angle_increment`, matching the existing convention in `scan_utils.py`.
- `ranges` — the full, unmodified ranges array. Non-finite values (`inf`, `nan`) are serialized as the string `"inf"` / `"nan"` (Python's `json` module cannot round-trip float `inf`/`nan` through standard JSON; using strings for these two sentinel cases keeps the file valid JSON per line while staying unambiguous — a finite range is a JSON number, a non-finite one is one of exactly two strings).
- `sectors` — the dict returned by `reduce_to_sectors`, reusing `path_tracker`'s existing default parameters (`front_arc_deg`, `front_subsector_deg`, `side_window_deg`, `rear_window_deg`), independently declared as ROS params on this node (matching how `trial_logger`/`decision_logger` each independently declare their own `csv_path` rather than sharing config with `path_tracker`).

## 6. File location and naming

Follows the existing convention exactly:

- ROS param `csv_path` (kept as the param name for consistency with the other two loggers, even though the file is `.jsonl` not `.csv`), default `/home/ubuntu/shared/trials/scan_trace.jsonl`.
- Bind-mounted to `/home/pi/docker/tmp/trials/scan_trace.jsonl` on the Pi, same as `decision_log.csv` and the per-surface trial CSVs.
- Symlinked into the repo's `trials/` folder (`trials/scan_trace.jsonl → /home/pi/docker/tmp/trials/scan_trace.jsonl`) for VS Code visibility, same as the other two.

One continuous file per logging session (per the approved design decision): no per-trial file splitting, no trial-boundary detection logic in this node. A trial's window is sliced out afterward by filtering on timestamp overlap against `trial_logger`'s/`decision_logger`'s rows for that trial.

## 7. Post-hoc miss-diagnosis workflow

This is analysis guidance, not code, but worth stating since it's the entire reason this log exists:

1. From the trial CSV, get the trial's approximate start/end wall-clock window (from `timestamp` and `transit_time_s`).
2. Filter `scan_trace.jsonl` to that window.
3. For a suspected miss at a known approximate bearing (e.g. "the chair leg was roughly ahead-left"), inspect the raw `ranges` at that bearing across the window:
   - If every beam near that bearing reads `range_max` or `"inf"` the entire time → the obstacle was never in the scan plane at all → sensor-geometry blind spot (§1, case 1).
   - If some beams show a finite reading close to the obstacle's actual distance, but `sectors.front` (or the relevant sub-sector) never crossed `safety_distance`/`obstacle_detect_range` → thresholding miss → tunable (§1, case 2).

## 8. Testing plan

Same split already used by every other module in this package:

- A pure, ROS-free piece: a `ScanTraceRecord` dataclass (mirroring `DecisionRecord`'s shape) with `to_json`/`from_json`, handling the `inf`/`nan` string-sentinel encoding/decoding — unit-tested on the host, no rclpy required.
- A thin ROS wrapper (`scan_trace_logger.py`) that subscribes to `/scan_raw`, builds a record each callback, and appends it — tested in-container (needs rclpy), following the existing pattern in `test_decision_logger_csv.py`.

## 9. Open questions / edge cases considered

- **File growth.** Trials are short (~10-35s observed) and this is opt-in per logging session (you start the node alongside the other two, same as today), so unbounded growth isn't a concern for the current single-session-at-a-time workflow. Not adding rotation/truncation now (YAGNI) — revisit if a session runs unattended for a long time.
- **`ranges` array length varies slightly between scans.** Not assumed fixed-width; stored as whatever length arrives, exactly as published. Reconstruction always uses `angle_min + i * angle_increment` per stored `ranges` length, matching `scan_utils.py`'s existing indexing convention.
- **Why JSON Lines over CSV.** A LaserScan's `ranges` is a ~450-element variable-length float array; CSV has no clean way to represent that as a single field, and a fixed-column-per-beam CSV would be brittle against beam-count drift. JSON Lines keeps the array structured and each line independently parseable.
