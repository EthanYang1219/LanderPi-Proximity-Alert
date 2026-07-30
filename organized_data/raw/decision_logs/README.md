# Decision logs (raw)

One row per obstacle-avoidance *decision*, written automatically by the
`decision_logger` ROS 2 node from JSON records published on
`/avoidance_decision` by `path_tracker`'s `AvoidanceController` state machine.
Never hand-edited.

**This is a copy.** The live file is a symlink in the repo's `trials/` folder
(`trials/decision_log.csv`), pointing at `/home/pi/docker/tmp/trials/` — the
bind mount `decision_logger` actually writes to.

## Files
- `decision_log.csv` — every avoidance decision across all sessions/surfaces
  (not split per-surface).

## Columns

| Column | Meaning |
|---|---|
| `timestamp` | Wall-clock time the decision was logged |
| `encounter_id` | Groups every decision belonging to the same obstacle encounter (from first detection through resolution) |
| `state` | Current state in the 7-state `AvoidanceController` machine: `DRIVE`, `ASSESS`, `STRAFE`, `TURN`, `DRIVE_PAST`, `RECOVER`, `HALT` |
| `chosen_maneuver` | The maneuver selected this tick |
| `reason` | Short text explanation for why that maneuver was chosen (e.g. "turn: span>max") |
| `obstacle_span_deg` | Angular width of the detected obstacle, degrees |
| `front_distance_m` | Reduced LiDAR distance in the FRONT sector | meters |
| `front_left_m` / `front_center_m` / `front_right_m` | Reduced LiDAR distance in each FRONT sub-sector | meters |
| `left_clearance_m` / `right_clearance_m` / `rear_clearance_m` | Reduced LiDAR distance in the LEFT/RIGHT/REAR sectors | meters |
| `required_clearing_m` | Clearance needed for the maneuver under consideration to be judged safe | meters |
| `cumulative_strafe_m` | Total lateral distance strafed so far this encounter (capped by `max_cumulative_strafe`) | meters |
| `consecutive_avoid_count` | Consecutive failed avoidance cycles this encounter (escalates toward `RECOVER` at `max_avoid_attempts`) | integer |
| `recovery_triggered` | Whether this row's decision entered the `RECOVER` state | boolean |
| `outcome` | Result of the maneuver (e.g. "committed") | text |
| `maneuver_duration_s` | How long the maneuver took/ran | seconds |

See the main [README.md](../../../README.md#refined-obstacle-avoidance) for the
full state-machine description and parameter definitions.
