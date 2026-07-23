# LanderPi Proximity Alert

A LiDAR-based Advanced Driver Assistance System (ADAS) proximity alert for a mobile robot, developed as part of an undergraduate research project at the University of Prince Edward Island (UPEI) under the supervision of Dr. Bingxian Mu.

The system drives a mobile robot from a fixed point A to point B, reactively stopping and turning away from obstacles detected by an onboard LiDAR, while logging transit time and odometry drift across four surface materials: granite, concrete, wood, and metal.

## Table of contents

- [Project overview](#project-overview)
- [Hardware and software stack](#hardware-and-software-stack)
- [Repository structure](#repository-structure)
- [Running it in VS Code (quick start)](#running-it-in-vs-code-quick-start)
- [Setup](#setup)
- [Usage](#usage)
- [Data collected](#data-collected)
- [Parameters](#parameters)
- [Roadmap](#roadmap)
- [Troubleshooting](#troubleshooting)
- [Authors](#authors)

## Project overview

This project measures how surface material affects a mobile robot's real-world driving performance versus its own internal estimate of that performance. Concretely, for each surface the robot is driven along a fixed marked track from point A to point B, and the following are compared:

- **Transit time** — how long the run took
- **Odometry-estimated distance** — the robot's own dead-reckoned estimate of how far it traveled (see the important caveat on what this actually measures, below)
- **Ground-truth distance** — how far it actually traveled, read by hand off a marked/taped track

The gap between odometry and ground truth quantifies wheel slippage, which is expected to vary by surface (granite and metal are expected to slip more than wood or concrete, for example). A LiDAR-based safety stop is layered on top so the robot avoids collisions with obstacles placed along or at the end of the track, independent of the distance measurement itself.

**Important limitation: this robot has no wheel-encoder feedback.** `/odom` (and the `odom_distance_m` column below) comes from `odom_publisher_node.py` in the vendor driver stack, which dead-reckons position by integrating the *commanded* velocity (`/cmd_vel`) over time — there is no wheel-encoder tick reading anywhere in the drive stack (confirmed by searching the full `driver/` tree). This means `odom_distance_m` measures "how far the commanded motion, integrated open-loop, thinks it went," not "how far the wheels actually turned." Two consequences for interpreting trial data:

1. On a clean, single-pass, straight-line run (`disable_avoidance:=true`, no obstacle-avoidance turns), `odom_distance_m` should closely track `forward_speed × transit_time_s` regardless of surface — so a gap against `ground_truth_distance_m` is a reasonable (if indirect) proxy for real slippage, since the *commanded* motion is known and constant.
2. On any run where obstacle-avoidance actually triggered (reverse/turn maneuvers), `odom_distance_m` is unreliable as a slippage signal: heading (`pose_yaw`) is *also* integrated open-loop from commanded `angular_z`, so it inherits error from the known mecanum-rotation quirk (see `path_tracker.py`'s `AVOIDING` state comments) on every turn, compounding across attempts. A large mismatch between `odom_distance_m` and `ground_truth_distance_m` on such a run reflects accumulated dead-reckoning drift through multiple maneuvers, not real-world wheel slip — see `avoidance_events` in [Data collected](#data-collected) below, and exclude any trial where it's non-zero from slippage analysis.

**Important limitation: the LiDAR has a fixed-height blind spot.** The LD19 scans a single 2D plane at whatever height it's mounted — it cannot see anything above or below that plane. Confirmed on real hardware: a round pedestal desk (narrow base, wide overhanging top) was not detected in time and the robot made low-speed contact with it before the safety stop or avoidance maneuver engaged, because the desk's base/legs were too thin or outside the scan plane at the point of approach, even though the tabletop clearly overhung the robot's path above the LiDAR's height. This is a physical sensor-geometry limitation, not a `path_tracker` bug — **lowering `safety_distance` would not have helped**, since the obstacle was effectively invisible to the sensor until contact, not merely detected late. Avoid pedestal-style, overhanging, or otherwise thin-profile-at-LiDAR-height obstacles when placing point B for a trial; use obstacles with a consistent cross-section at the LiDAR's mounted height (a box, a flat wall) instead.

Raw per-trial data is written to CSV so it can be handed off directly for statistical analysis.

## Hardware and software stack

| Component | Detail |
|---|---|
| Chassis | Hiwonder LanderPi, Mecanum wheels |
| Main compute | Raspberry Pi 5 (ROS 2 controller) |
| Low-level control | STM32 microcontroller (motor commands) |
| LiDAR | LD19 (the running driver node identifies as `LD19`/`LDLiDAR_LD19`; worth double-checking against this unit's actual spec sheet if "MS200" is documented elsewhere) |
| Odometry | Wheel encoders |
| Alert (planned) | Onboard buzzer, I2C |
| OS | Ubuntu 22.04 LTS |
| Middleware | ROS 2 Humble, running inside Docker |
| Language | Python 3 (`rclpy`) |

The buzzer is intentionally not wired into the current nodes — it is a separate, later integration step (see [Roadmap](#roadmap)) so it doesn't add risk while the motion-tracking behavior is still being validated.

## Repository structure

```
.
├── proximity_alert/            # The ROS 2 package (ament_python)
│   ├── package.xml
│   ├── setup.py / setup.cfg
│   └── proximity_alert/
│       ├── path_tracker.py           # Drives A -> B, reactive LiDAR obstacle avoidance
│       ├── avoidance.py              # Pure, ROS-free obstacle-avoidance state machine
│       ├── trial_logger.py           # Logs transit time, odometry distance, ground truth per trial
│       ├── decision_logger.py        # Logs each avoidance decision to its own CSV
│       ├── floor_test_reconcile.py   # Fills floor_test_log.csv's Time/Stop-clearance from real trial data
│       ├── scan_trace_record.py      # Pure JSON-Lines record for one raw scan tick
│       └── scan_trace_logger.py      # Logs every raw LiDAR scan continuously, for post-hoc miss diagnosis
├── trials/                     # CSVs, gitignored (not committed) -- see below
│   ├── floor_test_log.csv      # Hand-maintained PID/safety-distance tuning session report
│   ├── granite.csv             # -> symlink to /home/pi/docker/tmp/trials/granite.csv
│   ├── decision_log.csv        # -> symlink to /home/pi/docker/tmp/trials/decision_log.csv
│   └── scan_trace.jsonl        # -> symlink to /home/pi/docker/tmp/trials/scan_trace.jsonl
└── README.md
```

`trials/granite.csv` and `trials/decision_log.csv` are symlinks into the container's bind-mounted shared folder (see [Three logs](#three-logs-all-on-your-computer) below) — they exist purely so both logs show up directly in this repo's VS Code Explorer/file tree instead of requiring you to browse to `/home/pi/docker/tmp/trials/` separately. They live-update as the nodes write to them. If you log a new surface (e.g. `concrete.csv`), symlink it the same way:

```bash
ln -sf /home/pi/docker/tmp/trials/concrete.csv trials/concrete.csv
```

`path_tracker.py` and `trial_logger.py` are independent ROS 2 nodes. `trial_logger.py` does not modify or depend on the internals of `path_tracker.py` — it only observes `/odom`, so either node can be developed, tested, or replaced without breaking the other.

## Running it in VS Code (quick start)

The robot's ROS 2 stack already runs in a Docker container named `MentorPi` on the Pi — you don't need to install ROS yourself. Do this from a VS Code integrated terminal (open the repo folder via the Remote-SSH extension if you're connecting from another machine, or directly if VS Code is running on the Pi itself):

1. **Copy the package into the container.** `docker cp` nests the source inside the destination if the destination already exists, silently leaving a stale duplicate that colcon keeps building instead of your edits — this bit us mid-session (see [Troubleshooting](#troubleshooting)). Always clear the destination first:

   ```bash
   docker exec -u ubuntu MentorPi rm -rf /home/ubuntu/ros2_ws/src/proximity_alert/proximity_alert
   docker cp proximity_alert/proximity_alert MentorPi:/home/ubuntu/ros2_ws/src/proximity_alert/proximity_alert
   ```

2. **Build it** as the `ubuntu` user via `zsh` (not `bash` + `/opt/ros/humble/setup.bash` — that fails in this container; `~/.zshrc` is what actually sets up the workspace environment correctly):

   ```bash
   docker exec -u ubuntu MentorPi zsh -lc "source ~/.zshrc && cd ~/ros2_ws && colcon build --packages-select proximity_alert"
   ```

   If a previous build was ever run as `root` (e.g. via plain `docker exec` without `-u ubuntu`), leftover root-owned files under `build/proximity_alert` or `install/proximity_alert` will make this fail with `Permission denied`. Fix with:

   ```bash
   docker exec -u root MentorPi bash -c "chown -R ubuntu:ubuntu /home/ubuntu/ros2_ws/build/proximity_alert /home/ubuntu/ros2_ws/install/proximity_alert"
   ```

3. **Run each node in its own terminal**, as the `ubuntu` user via `zsh` (this loads `need_compile` and other env vars some of the robot's own launch files expect):

   ```bash
   # Terminal A — drive the robot
   docker exec -it -u ubuntu MentorPi zsh -lc "source ~/.zshrc && source ~/ros2_ws/install/setup.bash && ros2 run proximity_alert path_tracker --ros-args -p safety_distance:=0.30 -r scan:=/scan_raw"
   ```

   ```bash
   # Terminal B — log the trial
   docker exec -it -u ubuntu MentorPi zsh -lc "source ~/.zshrc && source ~/ros2_ws/install/setup.bash && ros2 run proximity_alert trial_logger --ros-args -p csv_path:=/home/ubuntu/shared/trials/granite.csv"
   ```

   ```bash
   # Terminal C — log the avoidance decisions (separate CSV)
   docker exec -it -u ubuntu MentorPi zsh -lc "source ~/.zshrc && source ~/ros2_ws/install/setup.bash && ros2 run proximity_alert decision_logger --ros-args -p csv_path:=/home/ubuntu/shared/trials/decision_log.csv"
   ```

   ```bash
   # Terminal D — log every raw LiDAR scan (for diagnosing total detection misses)
   docker exec -it -u ubuntu MentorPi zsh -lc "source ~/.zshrc && source ~/ros2_ws/install/setup.bash && ros2 run proximity_alert scan_trace_logger --ros-args -p csv_path:=/home/ubuntu/shared/trials/scan_trace.jsonl"
   ```

   Pointing `csv_path` at `/home/ubuntu/shared/...` writes the CSV into the container's shared folder, which is bind-mounted to `~/docker/tmp` (i.e. `/home/pi/docker/tmp/trials/`) on the Pi — so both the trial log and the decision log appear in your local file manager and survive container restarts.

After step 2, repeat only step 3 for future runs — you only need to rebuild when you change `path_tracker.py`/`trial_logger.py` (repeat steps 1–2 each time).

## Setup

1. **Docker + ROS 2 Humble.** The project's Docker container (`MentorPi`, image `ros:humble`) is already running on the robot's Pi — confirm it can see the LanderPi's ROS 2 stack with `docker exec -u ubuntu MentorPi zsh -lc "source ~/.zshrc && ros2 topic list"` (`/scan_raw`, `/odom`, `/cmd_vel` should be visible). Always include `-u ubuntu` — running as `root` silently breaks FastRTPS's shared-memory transport between nodes (discovery still matches over UDP, but zero data ever delivers), which cost an entire debugging session before it was traced to this.
2. **Copy the package in** and **build** it — see [Running it in VS Code](#running-it-in-vs-code-quick-start) above for the exact commands.

## Usage

Run each node in its own terminal (or SSH session) — see [Running it in VS Code](#running-it-in-vs-code-quick-start) for the exact commands.

Then, for each trial:

1. Place the robot at point A on the marked track for the current surface.
2. Place an obstacle (or rely on the track's natural endpoint) at point B.
3. Let the robot drive. `trial_logger` detects the start automatically when the robot begins moving, and the end automatically once `path_tracker`'s obstacle stop has held for one second.
4. When prompted in Terminal 2, read the robot's actual stop position off your tape-measure marks and enter the surface name and ground-truth distance.
5. Repeat for each trial and surface, pointing `csv_path` at a different file (or the same file — `trial_num` increments automatically) per surface.

## Data collected

Each row appended to the CSV represents one trial:

| Column | Meaning |
|---|---|
| `timestamp` | Wall-clock time the trial was logged |
| `surface` | Surface material entered at the prompt |
| `trial_num` | Auto-incrementing trial counter for that CSV file |
| `transit_time_s` | Seconds from start-of-motion to confirmed stop |
| `odom_distance_m` | Straight-line distance from start to stop, per the robot's dead-reckoned `/odom` estimate. **Not real wheel-encoder feedback** — see the caveat in [Project overview](#project-overview); only trust this as a slippage signal on `avoidance_events == 0` trials |
| `ground_truth_distance_m` | Straight-line distance from start to stop, per tape measure |
| `slippage_error_m` | `odom_distance_m - ground_truth_distance_m` |
| `slippage_pct` | Slippage error as a percentage of ground-truth distance |
| `avoidance_events` | Count of obstacle-avoidance maneuvers `path_tracker` triggered during the trial (detected via reverse `/cmd_vel` commands, which only occur in its `AVOIDING` state). **Non-zero means this was not a clean A→B run** and should be filtered out or analyzed separately from clean-run slippage stats. |
| `lidar_stop_range_m` | The actual LiDAR range to the closest obstacle in the forward arc at the moment the trial finalized (i.e. the real stop clearance), captured from `path_tracker`'s `/forward_min_range` topic. Useful for checking proximity-trigger accuracy against the `safety_distance` parameter and whether it varies by surface. Blank if nothing valid was in the arc at stop. |
| `notes` | Freeform text entered at logging time for anything unusual observed (e.g. "motors fought each other on the turn", "oscillated near desk", "false stop") |

## Refined obstacle avoidance

`path_tracker` runs a deterministic 7-state machine (`AvoidanceController`, a pure module unit-tested off-hardware) governed by one invariant: **always take the maneuver that minimizes deviation from the goal heading while maintaining safety.** That ordering falls out of it — strafe (holds the bearing) → turn-and-drive (bounded deviation) → one bounded recovery → halt:

- **DRIVE** — forward at `forward_speed`, PID holds the goal heading captured at A.
- **ASSESS** — on a confirmed obstacle, a strafe-first ladder: sidestep if a bounded strafe toward a *confirmed-clear* side clears the (narrow) obstacle; else turn toward the more-open side and drive past; else escalate.
- **STRAFE / TURN / DRIVE_PAST** — the maneuvers, each committed (run to completion) so an obstacle's jittering apparent side can't cause oscillation.
- **RECOVER** — after repeated failures, one bounded routine: back off, pick the widest fitting gap from the current 360° scan, rotate to face it, commit once.
- **HALT** — stop and flag; re-check each tick.

The LiDAR is reduced each scan into FRONT (+ front sub-sectors), LEFT, RIGHT, and REAR clearances. Every decision is published as a JSON record on `/avoidance_decision` and logged by `decision_logger` (see [Data collected](#data-collected)). Set `disable_avoidance:=true` to halt on any obstacle with no turn/strafe (used for the clean go-and-stop distance runs).

## Parameters

**`path_tracker`** (every field is a ROS parameter; only the commonly-tuned ones are shown — see `AvoidanceConfig` in `proximity_alert/avoidance.py` for the full list)

| Parameter | Default | Description |
|---|---|---|
| `safety_distance` | `0.30` | FRONT stop threshold, meters |
| `forward_speed` | `0.50` | Constant forward speed, m/s |
| `obstacle_confirm_scans` | `2` | Consecutive close scans required before a maneuver decision (debounce) |
| `strafe_speed` / `strafe_timeout` | `0.25` / `1.5` | Lateral speed and per-strafe time cap (m/s, s) |
| `strafe_side_clearance_min` | `0.30` | Side clearance required to strafe into it, meters |
| `max_obstacle_width` | `0.50` | Above this *physical* lateral width (meters), the obstacle is "wide" (a wall) → turn, not strafe. Keyed on physical width, not angular span: at trigger range any real object subtends a large angle, so an angular-span gate would block strafing entirely. |
| `max_cumulative_strafe` | `0.60` | Hard per-encounter lateral cap (long-wall guard), meters |
| `turn_speed` / `turn_step_deg` / `turn_timeout` | `0.6` / `30.0` / `1.5` | Turn rate, per-turn increment, time cap (rad/s, deg, s) |
| `max_avoid_attempts` | `3` | Failed cycles before escalating to recovery |
| `clear_drive_duration` | `3.0` | Sustained clean-drive time that closes an encounter and resets counters, seconds |
| `min_gap_clearance` / `min_gap_width_deg` | `0.60` / `40.0` | What counts as a usable recovery gap (robot must fit) |
| `disable_avoidance` | `false` | Halt on any obstacle, no turn/strafe (clean go-and-stop runs) |

Derived (computed, not configured): `max_strafe_distance = strafe_speed × strafe_timeout`; `corridor_half = robot_half_width + corridor_margin`; `clear_threshold = safety_distance + clear_margin`.

**`trial_logger`**

| Parameter | Default | Description |
|---|---|---|
| `csv_path` | `trial_log.csv` | Output CSV file path (point at `/home/ubuntu/shared/trials/<surface>.csv` to land on the host — see below) |
| `move_velocity_threshold` | `0.03` | Speed above which the robot is considered moving, m/s |
| `stop_velocity_threshold` | `0.02` | Speed below which the robot is considered stopped, m/s |
| `stop_confirm_duration` | `1.0` | Seconds of continuous stopped-ness before a trial is finalized |

**`decision_logger`**

| Parameter | Default | Description |
|---|---|---|
| `csv_path` | `/home/ubuntu/shared/trials/decision_log.csv` | Output CSV for the avoidance decision log (host path — see below) |

### Three logs, all on your computer

The logs are deliberately kept in **separate files** so the motion/slippage data, the avoidance-decision data, and the raw scan data stay clean and independently analyzable:

- **Trial motion log** (`trial_logger`) — one row per A→B trial: transit time, odometry vs. ground-truth distance, slippage, `avoidance_events`, `lidar_stop_range_m`.
- **Decision log** (`decision_logger`) — one row per avoidance *decision*, for research/debugging: `timestamp, encounter_id, state, chosen_maneuver, reason, obstacle_span_deg, front_distance_m, front_left_m, front_center_m, front_right_m, left_clearance_m, right_clearance_m, rear_clearance_m, required_clearing_m, cumulative_strafe_m, consecutive_avoid_count, recovery_triggered, outcome, maneuver_duration_s`.
- **Scan trace log** (`scan_trace_logger`) — one row per raw LiDAR scan tick, for diagnosing detection misses that never trigger an avoidance encounter at all (e.g. a thin chair leg outside the LiDAR's scan plane): `scan_number, stamp_sec, stamp_nanosec, angle_min, angle_increment, range_min, range_max, ranges, sectors`. JSON Lines (`.jsonl`), not CSV — `ranges` is a variable-length array that doesn't fit CSV's fixed-column shape. `stamp_sec`/`stamp_nanosec` come from the LaserScan message's own `header.stamp`, not wall-clock time, so this log stays on the same clock as `/odom` and every other ROS message for valid cross-message correlation. A process killed mid-write can only ever corrupt the last line of the file — skip a line that fails to parse rather than treating it as corruption. The node's `front_arc_deg`/`front_subsector_deg`/`side_window_deg`/`rear_window_deg` params default to match `path_tracker`'s current `AvoidanceConfig` values; if those get tuned in `avoidance.py`, update this node's defaults too or the logged `sectors` will stop reflecting what the controller actually saw. See `docs/superpowers/specs/2026-07-24-scan-trace-logger-design.md` for the full design and the post-hoc miss-diagnosis workflow.

All three default to (or should be pointed at) `/home/ubuntu/shared/trials/` inside the container, which is bind-mounted to **`/home/pi/docker/tmp/trials/`** on the Pi — so all three logs appear directly in your local file manager (and survive container restarts) with no `docker` digging. This repo's `trials/` folder also symlinks straight to them (see [Repository structure](#repository-structure)) so they show up in VS Code too.

### Reconciling floor_test_log.csv

`floor_test_log.csv` (the hand-maintained Google-Sheet-schema report of PID/safety-distance tuning sessions) is never written by any ROS node — it's a manual transcription of Time and Stop clearance from the real trial CSV, plus the `safety_distance`/`Kp`/`Ki`/`Kd` you ran with (which aren't persisted anywhere else). That transcription step is easy to forget. Run this after a session to auto-fill whatever's derivable from the trial data, on the host (no ROS needed):

```bash
python3 -m proximity_alert.floor_test_reconcile --floor-log trials/floor_test_log.csv --trial-csv trials/granite.csv
```

It only fills Time/Stop-clearance, never fabricates Safety Distance/Speed/Kp/Ki/Kd, and refuses to guess when two session rows share the same date (nothing to disambiguate which trials belong to which row) — it reports both cases so you can fill them by hand instead of silently leaving (or corrupting) a blank.

## Roadmap

- [x] Reactive obstacle-avoidance driver (`path_tracker.py`)
- [x] Per-trial transit time / odometry / ground-truth logging (`trial_logger.py`)
- [ ] Re-integrate the I2C buzzer so it fires simultaneously with the obstacle-avoidance stop
- [ ] Collect full trial sets across all four surfaces
- [ ] Statistical analysis of slippage by surface (Haotian)
- [ ] Methodology and Results sections, IEEE conference format

## Troubleshooting

- **`trial_logger` never detects a trial end.** Check that `path_tracker`'s obstacle stop is actually driving `linear.x` to zero (watch `ros2 topic echo /cmd_vel`) and that `/odom` twist values are reasonably close to zero when stationary — noisy odometry may need a higher `stop_velocity_threshold`.
- **Robot doesn't stop in time / stops too early.** Adjust `safety_distance` on `path_tracker`; the LiDAR's `range_min`/`range_max` limits also bound how close/far it can reliably see.
- **Robot makes contact with an obstacle that has a thin or overhanging profile (e.g. a pedestal desk, chair legs).** This is very likely the LiDAR's fixed-height blind spot, not a `safety_distance` or code issue — see the limitation note in [Project overview](#project-overview). Reposition the obstacle so it has a consistent cross-section at the LiDAR's mounted height, don't just lower `safety_distance`.
- **No `/scan_raw` or `/odom` data.** Confirm the LanderPi's sensor drivers are running inside the `MentorPi` container (`docker exec MentorPi bash -lc "source /opt/ros/humble/setup.bash && ros2 node list"` should show `LD19`, `ekf_filter_node`, etc.) before starting either node. If the list comes back empty, the driver stack itself has died and needs restarting — see `~/robot_pi/tool/bringup.sh` on the Pi.
- **`path_tracker` holds still and logs "No fresh scan within scan_timeout."** This is the scan-freshness watchdog working as intended — the LiDAR isn't currently publishing. Check `ros2 topic hz /scan_raw`; this LD19 has been observed to intermittently stop publishing mid-session.
- **Edits to a `.py` file don't seem to take effect after rebuilding.** `docker cp` nests the source inside the destination directory if the destination already exists, rather than overwriting it — running the copy step twice without clearing the destination first silently produces a stale duplicate package tree that colcon keeps building from instead of your latest edit. This happened mid-session and cost real time to trace. Always `rm -rf` the destination package dir before `docker cp` (see [Running it in VS Code](#running-it-in-vs-code-quick-start)), and if in doubt, `find ~/ros2_ws/src/proximity_alert -name '<file>.py'` inside the container to check for more than one copy.
- **Rebuild fails with `Permission denied` on files under `build/` or `install/`.** A previous build ran as `root` (e.g. a bare `docker exec` without `-u ubuntu`) and left root-owned artifacts that the `ubuntu` user can't overwrite. `chown -R ubuntu:ubuntu` those two directories (command in [Running it in VS Code](#running-it-in-vs-code-quick-start)) and rebuild.

## Authors

- Ethan — programming, physical data collection
- Haotian — statistical analysis, lead author
- Supervised by Dr. Bingxian Mu, University of Prince Edward Island
