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
│       ├── path_tracker.py     # Drives A -> B, reactive LiDAR obstacle avoidance
│       └── trial_logger.py     # Logs transit time, odometry distance, ground truth per trial
├── trials/                     # Suggested output location for per-surface CSV logs (not committed)
└── README.md
```

`path_tracker.py` and `trial_logger.py` are independent ROS 2 nodes. `trial_logger.py` does not modify or depend on the internals of `path_tracker.py` — it only observes `/odom`, so either node can be developed, tested, or replaced without breaking the other.

## Running it in VS Code (quick start)

The robot's ROS 2 stack already runs in a Docker container named `MentorPi` on the Pi — you don't need to install ROS yourself. Do this from a VS Code integrated terminal (open the repo folder via the Remote-SSH extension if you're connecting from another machine, or directly if VS Code is running on the Pi itself):

1. **Copy the package into the container:**

   ```bash
   docker cp proximity_alert MentorPi:/home/ubuntu/ros2_ws/src/proximity_alert
   ```

2. **Build it** (must run under `bash`, not `zsh` — colcon's environment hooks need it):

   ```bash
   docker exec MentorPi bash -lc "source /opt/ros/humble/setup.bash && cd /home/ubuntu/ros2_ws && colcon build --packages-select proximity_alert"
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

   Pointing `csv_path` at `/home/ubuntu/shared/...` writes the CSV into the container's shared folder, which is bind-mounted to `~/docker/tmp` on the Pi — so your trial data survives even if the container restarts.

After step 2, repeat only step 3 for future runs — you only need to rebuild when you change `path_tracker.py`/`trial_logger.py` (repeat steps 1–2 each time).

## Setup

1. **Docker + ROS 2 Humble.** The project's Docker container (`MentorPi`, image `ros:humble`) is already running on the robot's Pi — confirm it can see the LanderPi's ROS 2 stack with `docker exec MentorPi bash -lc "source /opt/ros/humble/setup.bash && ros2 topic list"` (`/scan_raw`, `/odom`, `/cmd_vel` should be visible).
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

## Parameters

**`path_tracker`**

| Parameter | Default | Description |
|---|---|---|
| `safety_distance` | `0.30` | Stop threshold, meters |
| `forward_speed` | `0.15` | Constant forward speed, m/s |
| `turn_speed` | `0.6` | Turn-away angular speed, rad/s |
| `scan_arc_deg` | `180.0` | Forward arc monitored for obstacles, degrees |
| `drift_speed` | `0.15` | Sideways strafe speed for the last-resort drift-around maneuver, m/s |
| `drift_duration` | `1.0` | How long the robot strafes sideways before re-checking for a clear path, seconds |

**`trial_logger`**

| Parameter | Default | Description |
|---|---|---|
| `csv_path` | `trial_log.csv` | Output CSV file path |
| `move_velocity_threshold` | `0.03` | Speed above which the robot is considered moving, m/s |
| `stop_velocity_threshold` | `0.02` | Speed below which the robot is considered stopped, m/s |
| `stop_confirm_duration` | `1.0` | Seconds of continuous stopped-ness before a trial is finalized |

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

## Authors

- Ethan — programming, physical data collection
- Haotian — statistical analysis, lead author
- Supervised by Dr. Bingxian Mu, University of Prince Edward Island
