# LanderPi Proximity Alert

A LiDAR-based Advanced Driver Assistance System (ADAS) proximity alert for a mobile robot, developed as part of an undergraduate research project at the University of Prince Edward Island (UPEI) under the supervision of Dr. Bingxian Mu.

The system drives a mobile robot from a fixed point A to point B, reactively stopping and turning away from obstacles detected by an onboard LiDAR, while logging transit time and odometry drift across four surface materials: granite, concrete, wood, and metal.

## Table of contents

- [Project overview](#project-overview)
- [Hardware and software stack](#hardware-and-software-stack)
- [Repository structure](#repository-structure)
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
- **Odometry-estimated distance** — how far the robot's wheel encoders think it traveled
- **Ground-truth distance** — how far it actually traveled, read by hand off a marked/taped track

The gap between odometry and ground truth quantifies wheel slippage, which is expected to vary by surface (granite and metal are expected to slip more than wood or concrete, for example). A LiDAR-based safety stop is layered on top so the robot avoids collisions with obstacles placed along or at the end of the track, independent of the distance measurement itself.

Raw per-trial data is written to CSV so it can be handed off directly for statistical analysis.

## Hardware and software stack

| Component | Detail |
|---|---|
| Chassis | Hiwonder LanderPi, Mecanum wheels |
| Main compute | Raspberry Pi 5 (ROS 2 controller) |
| Low-level control | STM32 microcontroller (motor commands) |
| LiDAR | MS200 TOF LiDAR |
| Odometry | Wheel encoders |
| Alert (planned) | Onboard buzzer, I2C |
| OS | Ubuntu 22.04 LTS |
| Middleware | ROS 2 Humble, running inside Docker |
| Language | Python 3 (`rclpy`) |

The buzzer is intentionally not wired into the current nodes — it is a separate, later integration step (see [Roadmap](#roadmap)) so it doesn't add risk while the motion-tracking behavior is still being validated.

## Repository structure

```
.
├── path_tracker.py     # Drives A -> B, reactive LiDAR obstacle avoidance
├── trial_logger.py      # Logs transit time, odometry distance, ground truth per trial
├── trials/               # Suggested output location for per-surface CSV logs (not committed)
└── README.md
```

`path_tracker.py` and `trial_logger.py` are independent ROS 2 nodes. `trial_logger.py` does not modify or depend on the internals of `path_tracker.py` — it only observes `/odom`, so either node can be developed, tested, or replaced without breaking the other.

## Setup

1. **Docker + ROS 2 Humble.** Build or pull the project's Docker image with ROS 2 Humble installed, and confirm it can reach the LanderPi's ROS 2 stack (`/scan`, `/odom`, `/cmd_vel` topics should be visible via `ros2 topic list`).
2. **Copy the nodes into your package.** Place `path_tracker.py` and `trial_logger.py` into your ROS 2 package's Python module directory (wherever your other nodes already live), and register them as executables in `setup.py`:

   ```python
   entry_points={
       "console_scripts": [
           "path_tracker = <your_package>.path_tracker:main",
           "trial_logger = <your_package>.trial_logger:main",
       ],
   },
   ```

3. **Build.**

   ```bash
   cd ~/ros2_ws
   colcon build --packages-select <your_package>
   source install/setup.bash
   ```

## Usage

Run each node in its own terminal (or SSH session).

**Terminal 1 — drive the robot:**

```bash
ros2 run <your_package> path_tracker --ros-args -p safety_distance:=0.30
```

**Terminal 2 — log the trial:**

```bash
ros2 run <your_package> trial_logger --ros-args -p csv_path:=trials/granite.csv
```

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
| `odom_distance_m` | Straight-line distance from start to stop, per wheel odometry |
| `ground_truth_distance_m` | Straight-line distance from start to stop, per tape measure |
| `slippage_error_m` | `odom_distance_m - ground_truth_distance_m` |
| `slippage_pct` | Slippage error as a percentage of ground-truth distance |

## Parameters

**`path_tracker`**

| Parameter | Default | Description |
|---|---|---|
| `safety_distance` | `0.30` | Stop threshold, meters |
| `forward_speed` | `0.15` | Constant forward speed, m/s |
| `turn_speed` | `0.6` | Turn-away angular speed, rad/s |
| `scan_arc_deg` | `180.0` | Forward arc monitored for obstacles, degrees |

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
- **Robot doesn't stop in time / stops too early.** Adjust `safety_distance` on `path_tracker`; the LiDAR's `range_min`/`range_max` limits from the MS200 driver also bound how close/far it can reliably see.
- **No `/scan` or `/odom` data.** Confirm the LanderPi's sensor drivers are running inside the Docker container and `ros2 topic list` shows both topics before starting either node.

## Authors

- Ethan — programming, physical data collection
- Haotian — statistical analysis
- Supervised by Dr. Bingxian Mu, University of Prince Edward Island
