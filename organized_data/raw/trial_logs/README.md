# Trial logs (raw)

One row per A→B motion trial, written automatically by the `trial_logger` ROS 2
node (`proximity_alert/proximity_alert/trial_logger.py`) — never hand-edited.

**These are copies.** The live files are symlinks in the repo's `trials/`
folder (`trials/carpet.csv`, `trials/granite.csv`), pointing at
`/home/pi/docker/tmp/trials/` — the bind mount `trial_logger` actually writes
to. Do not treat these copies as the source of truth; re-copy from `trials/`
after new sessions if you need fresh data here.

## Files
- `carpet.csv` — trials run on carpet.
- `granite.csv` — trials run on granite.
- `wood.csv` — trials run on wood (15 trials). Was sitting in the container's
  bind mount unlinked/uncopied until 2026-07-29 — see caveat below.

## Columns

| Column | Meaning | Units |
|---|---|---|
| `timestamp` | Wall-clock time the trial was logged | `YYYY-MM-DD HH:MM:SS` |
| `surface` | Surface material entered at the prompt | text |
| `trial_num` | Auto-incrementing trial counter for that CSV file | integer |
| `transit_time_s` | Seconds from start-of-motion to confirmed stop | seconds |
| `odom_distance_m` | Straight-line distance from start to stop per the robot's dead-reckoned `/odom` estimate. Not real wheel-encoder feedback — only trust as a slippage signal when `avoidance_events == 0` | meters |
| `ground_truth_distance_m` | Straight-line distance from start to stop, per tape measure | meters |
| `slippage_error_m` | `odom_distance_m - ground_truth_distance_m` | meters |
| `slippage_pct` | Slippage error as a percentage of ground-truth distance | percent |
| `avoidance_events` | Count of obstacle-avoidance maneuvers triggered during the trial. Non-zero means this was not a clean A→B run — filter out or analyze separately from clean-run slippage stats | integer |
| `lidar_stop_range_m` | Actual LiDAR range to the closest forward obstacle at the moment the trial finalized (the real stop clearance). Blank if nothing valid was in the arc at stop | meters |
| `notes` | Freeform text entered at logging time for anything unusual observed | text |
| `battery_level` | Rough `High`/`Medium`/`Low` estimate from `/ros_robot_controller/battery` at the moment the trial finalized (2S Li-ion, ~6.0V-8.4V range). Blank if no reading had arrived yet, or if the row predates this column | text |

## Known data-integrity flags (not corrected here — see caller's guidance)
- `granite.csv` trial 8 has `surface=wood` and trial 9 has `surface=other`
  (note says "good run on carpet") inside a file that's otherwise all
  `granite` rows — likely mislabeled at entry time, not re-run on the wrong
  surface.
- `granite.csv` trials 4-5 have outlier `ground_truth_distance_m` values
  (23.2m, 22.5m) that the trial's own notes already flag as bad ("need to
  adjust starting position," "forgot to renew starting position").
- `wood.csv` trial 1 has `ground_truth_distance_m = 13.10m` versus ~0.12-0.15m
  for every other wood trial, driving `slippage_pct` to -96.22% — its own
  note ("low battery run") doesn't explain a 10x-off tape measurement, so
  this looks like a data-entry error (e.g. wrong units or wrong tape
  reading) rather than a real result. Exclude from slippage stats.

These were flagged, not altered, per project convention of surfacing
anomalies rather than silently fixing them.
