# Scan traces (raw) — not duplicated here

`scan_trace_logger` writes one JSON object per raw LiDAR scan tick to
`scan_trace.jsonl`, for diagnosing detection misses that never trigger an
avoidance encounter (e.g. a thin chair leg outside the LiDAR's scan plane).

**This file is intentionally not copied into `organized_data/`** — as of
2026-07-29 it is **~1.04 GB** (`/home/pi/docker/tmp/trials/scan_trace.jsonl`,
symlinked at `trials/scan_trace.jsonl` in the repo root). Copying it would
bloat this folder for no benefit; work with it in place instead.

## Columns (per JSON line)

| Field | Meaning |
|---|---|
| `scan_number` | Auto-incrementing scan counter |
| `stamp_sec` / `stamp_nanosec` | Timestamp from the LaserScan message's own `header.stamp` (not wall-clock) — stays on the same clock as `/odom` and every other ROS message, for cross-message correlation |
| `angle_min` | Start angle of the scan, radians |
| `angle_increment` | Angular step between range readings, radians |
| `range_min` / `range_max` | Sensor's valid range bounds, meters |
| `ranges` | Full array of raw per-beam distances, meters (`"nan"` where no valid return) |
| `sectors` | Same scan reduced into `front`, `front_left`, `front_center`, `front_right`, `left`, `right`, `rear` clearances — the same reduction `path_tracker` uses for avoidance decisions |

A process killed mid-write can only ever corrupt the last line of the file —
skip a line that fails to parse rather than treating it as file corruption.

See the main [README.md](../../../README.md#three-logs-all-on-your-computer)
for the full design notes and the post-hoc miss-diagnosis workflow.
