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

**Scan topic confirmed.** `/scan_raw` is the actual, currently-published topic on hardware (`ros2 topic list` on the running container shows `/scan_raw`; no `/scan` topic exists). No ambiguity — this node subscribes to `/scan_raw` directly, same topic `path_tracker` uses.

Per scan message received, it:

1. Extracts the raw scan fields needed to fully reconstruct beam geometry later (see §5), keyed off the message's own `header.stamp` rather than wall-clock time (see §5's `stamp_sec`/`stamp_nanosec` note).
2. Assigns the next `scan_number` (see §5).
3. Calls the existing pure `reduce_to_sectors(...)` from `scan_utils.py` — the same function `path_tracker` already uses — to attach the reduced sector summary alongside the raw data. This means a coarse pass ("was front ever below X") doesn't require re-deriving sector logic from raw ranges by hand; the raw ranges remain available for the deep-dive blind-spot check.
4. Appends one JSON Lines (`.jsonl`) record to the configured output file.

No new topics are published; this node only consumes and persists.

## 5. Record schema

One JSON object per line (JSON Lines, not a single JSON array — so the file is streamable/appendable and a partially-written file from an interrupted run is still readable up to the last complete line):

```json
{
  "scan_number": 4821,
  "stamp_sec": 1784824869,
  "stamp_nanosec": 454205692,
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

- `scan_number` — a monotonically increasing integer this node assigns itself, starting at 0 and incrementing once per received `/scan_raw` message, in receipt order. ROS 2's `std_msgs/Header` no longer carries a `seq` field (dropped from ROS 1), so nothing upstream provides a sequence number — this node is the source of it. Storing it makes referencing/correlating a specific scan ("scan #4821 shows...") far more convenient than always having to quote a full timestamp pair, and gives an unambiguous, gap-detectable ordering even if two scans somehow shared a timestamp.
- `stamp_sec` / `stamp_nanosec` — copied directly from the incoming `LaserScan` message's `header.stamp` (ROS `Time`: int32 seconds + uint32 nanoseconds), **not** wall-clock time (`datetime.now()`). Confirmed on hardware that the driver populates this with real, monotonically-increasing values (~100ms apart, matching the LD19's ~10Hz rate) rather than leaving it zeroed. Using the message's own stamp — rather than whatever wall-clock time this node happens to observe it at — keeps this log on the same clock as every other ROS message (`/odom`, `/cmd_vel`, etc.), which is what makes cross-message correlation for offline analysis valid; wall-clock timestamps would carry extra DDS transport/scheduling jitter that has nothing to do with when the LiDAR actually took the reading. Stored as the two raw ROS `Time` integer fields (not combined into a single float) to avoid float64 precision loss at 10-digit second values.
- `angle_min`, `angle_increment`, `range_min`, `range_max` — copied verbatim from the incoming `LaserScan` message. Together with `ranges`, this fully reconstructs each beam's angle via `angle = angle_min + i * angle_increment`, matching the existing convention in `scan_utils.py`.
- `ranges` — the full, unmodified ranges array. Non-finite values (`inf`, `nan`) are serialized as the string `"inf"` / `"nan"` (Python's `json` module cannot round-trip float `inf`/`nan` through standard JSON; using strings for these two sentinel cases keeps the file valid JSON per line while staying unambiguous — a finite range is a JSON number, a non-finite one is one of exactly two strings).
- `sectors` — the dict returned by `reduce_to_sectors`, reusing `path_tracker`'s existing default parameters (`front_arc_deg`, `front_subsector_deg`, `side_window_deg`, `rear_window_deg`), independently declared as ROS params on this node (matching how `trial_logger`/`decision_logger` each independently declare their own `csv_path` rather than sharing config with `path_tracker`). **This is intentionally duplicated data** — every value in `sectors` is always fully recomputable from `ranges` plus the four sector-window parameters, so it adds no new information. It's stored anyway purely to speed up coarse offline analysis (e.g. "was `front` ever below X across this whole trace") without needing to re-run `reduce_to_sectors` over the raw arrays first. The trade-off is a larger file for a faster common-case query; `ranges` remains the ground truth if `sectors` and a from-scratch recomputation ever need to be cross-checked.

## 6. File location and naming

Follows the existing convention exactly:

- ROS param `csv_path` (kept as the param name for consistency with the other two loggers, even though the file is `.jsonl` not `.csv`), default `/home/ubuntu/shared/trials/scan_trace.jsonl`.
- Bind-mounted to `/home/pi/docker/tmp/trials/scan_trace.jsonl` on the Pi, same as `decision_log.csv` and the per-surface trial CSVs.
- Symlinked into the repo's `trials/` folder (`trials/scan_trace.jsonl → /home/pi/docker/tmp/trials/scan_trace.jsonl`) for VS Code visibility, same as the other two.

One continuous file per logging session (per the approved design decision): no per-trial file splitting, no trial-boundary detection logic in this node. A trial's window is sliced out afterward by filtering on timestamp overlap against `trial_logger`'s/`decision_logger`'s rows for that trial.

## 7. Post-hoc miss-diagnosis workflow

This is analysis guidance, not code, but worth stating since it's the entire reason this log exists:

1. From the trial CSV, get the trial's approximate start/end wall-clock window (from `timestamp` and `transit_time_s`). `trial_logger`/`decision_logger` still record wall-clock time; `scan_trace.jsonl` records ROS `header.stamp` (epoch seconds + nanoseconds). Since this project runs on the system clock (no `use_sim_time`, no simulation), these are the same underlying clock — convert `stamp_sec` to local wall-clock (e.g. `datetime.fromtimestamp(stamp_sec)`) to line the window up against the trial CSV's timestamps.
2. Filter `scan_trace.jsonl` to that window. Once the rough window is found, `scan_number` gives a convenient, unambiguous way to reference or re-locate specific scans within it (e.g. "the miss is visible starting around scan #4821") without repeating a full timestamp pair.
3. For a suspected miss at a known approximate bearing (e.g. "the chair leg was roughly ahead-left"), inspect the raw `ranges` at that bearing across the window:
   - If every beam near that bearing reads `range_max` or `"inf"` the entire time → the obstacle was never in the scan plane at all → sensor-geometry blind spot (§1, case 1).
   - If some beams show a finite reading close to the obstacle's actual distance, but `sectors.front` (or the relevant sub-sector) never crossed `safety_distance`/`obstacle_detect_range` → thresholding miss → tunable (§1, case 2).

## 8. Testing plan

Same split already used by every other module in this package:

- A pure, ROS-free piece: a `ScanTraceRecord` dataclass (mirroring `DecisionRecord`'s shape) with `to_json`/`from_json`, handling the `inf`/`nan` string-sentinel encoding/decoding — unit-tested on the host, no rclpy required.
- A thin ROS wrapper (`scan_trace_logger.py`) that subscribes to `/scan_raw`, builds a record each callback, and appends it — tested in-container (needs rclpy), following the existing pattern in `test_decision_logger_csv.py`.

## 9. Open questions / edge cases considered

- **Intended logging duration.** This logger is designed and scoped for **bounded trial sessions** — you start it alongside the other two nodes, run one or a handful of short trials (~10-35s each, observed), then stop it, the same opt-in workflow as today. Long-duration or unattended logging, log rotation, and compression are **intentionally out of scope** for this design; the node has no size cap, rotation, or truncation logic. If a future use case needs multi-hour or continuous unattended capture, that's a separate design problem (rotation policy, compression, retention) and should not be retrofitted onto this node without revisiting this spec.
- **`ranges` array length varies slightly between scans.** Not assumed fixed-width; stored as whatever length arrives, exactly as published. Reconstruction always uses `angle_min + i * angle_increment` per stored `ranges` length, matching `scan_utils.py`'s existing indexing convention.
- **Why JSON Lines over CSV.** A LaserScan's `ranges` is a ~450-element variable-length float array; CSV has no clean way to represent that as a single field, and a fixed-column-per-beam CSV would be brittle against beam-count drift. JSON Lines keeps the array structured and each line independently parseable.
