# LanderPi Proximity Alert

A LiDAR-based Advanced Driver Assistance System (ADAS) proximity alert for a mobile robot, developed as part of an undergraduate research project at the University of Prince Edward Island (UPEI) under the supervision of Dr. Bingxian Mu.

The system drives a mobile robot from a fixed point A to point B, reactively stopping and turning away from obstacles detected by an onboard LiDAR, while logging transit time and odometry drift across three surface materials: granite, concrete, and wood. 

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

This project measures how surface material affects a mobile robot's real-world driving performance versus its own internal estimate of that performance. Concretely, for each surface, the robot is driven along a fixed marked track from point A to point B, and the following are compared:

- **Transit time** — how long the run took
- **Odometry-estimated distance** — the robot's own dead-reckoned estimate of how far it travelled (see the important caveat on what this actually measures, below)
- **Ground-truth distance** — how far it actually travelled, read by hand off a marked/taped track

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
| Odometry | Integrated from /cmd_vel |
| Alert (planned) | Onboard buzzer, I2C |
| OS | Ubuntu 22.04 LTS |
| Middleware | ROS 2 Humble, running inside Docker |
| Language | Python 3 (`rclpy`) |

The buzzer is intentionally not wired into the current nodes — it is a separate, later integration step (see [Roadmap](#roadmap)) so it doesn't add risk while the motion-tracking behaviour is still being validated.

## Repository structure

```
.
├── proximity_alert/            # The ROS 2 package (ament_python)
│   ├── package.xml
│   ├── setup.py / setup.cfg
│   └── proximity_alert/
│       ├── path_tracker.py           # Drives A -> B, reactive LiDAR obstacle avoidance, and plays the obstacle audio alert (on by default)
│       ├── avoidance.py              # Pure, ROS-free obstacle-avoidance state machine
│       ├── trial_logger.py           # Logs transit time, odometry distance, ground truth per trial
│       ├── decision_logger.py        # Logs each avoidance decision to its own CSV
│       ├── floor_test_reconcile.py   # Fills floor_test_log.csv's Time/Stop-clearance from real trial data
│       ├── scan_trace_record.py      # Pure JSON-Lines record for one raw scan tick
│       ├── scan_trace_logger.py      # Logs every raw LiDAR scan continuously, for post-hoc miss diagnosis
│       └── audio_trigger.py          # Pure once-per-encounter trigger logic for the obstacle audio alert (used by path_tracker.py)
├── trials/                     # CSVs, gitignored (not committed) -- see below
│   ├── floor_test_log.csv      # Hand-maintained PID/safety-distance tuning session report
│   ├── lateral_offset_trials.csv  # Hand-maintained return-to-line (cross-track) trial report
│   ├── granite.csv             # -> symlink to /home/pi/docker/tmp/trials/granite.csv
│   ├── decision_log.csv        # -> symlink to /home/pi/docker/tmp/trials/decision_log.csv
│   └── scan_trace.jsonl        # -> symlink to /home/pi/docker/tmp/trials/scan_trace.jsonl
└── README.md
```

`trials/granite.csv`, `trials/decision_log.csv`, and `trials/scan_trace.jsonl` are symlinks into the container's bind-mounted shared folder (see [Three logs](#three-logs-all-on-your-computer) below) — they exist purely so all three logs show up directly in this repo's VS Code Explorer/file tree instead of requiring you to browse to `/home/pi/docker/tmp/trials/` separately. They live-update as the nodes write to them. If you log a new surface (e.g. `hpl.csv` for plastic laminate), symlink it the same way:

```bash
ln -sf /home/pi/docker/tmp/trials/hpl.csv trials/hpl.csv
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
   docker exec -it -u ubuntu MentorPi zsh -lc "source ~/.zshrc && source ~/ros2_ws/install/setup.bash && ros2 run proximity_alert path_tracker --ros-args -p safety_distance:=0.20 -p target_distance:=2.0 -r scan:=/scan_raw"
   ```

   `target_distance` is declared as a double parameter -- pass the decimal
   form (`2.0`, not `2`), or `--ros-args` raises
   `InvalidParameterTypeException` and the node never starts.

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

   Pointing `csv_path` at `/home/ubuntu/shared/...` writes the file into the container's shared folder, which is bind-mounted to `~/docker/tmp` (i.e. `/home/pi/docker/tmp/trials/`) on the Pi — so all three logs appear in your local file manager and survive container restarts.

   Terminal A also plays the obstacle audio alert by default (no separate terminal needed — see [Obstacle audio alert](#obstacle-audio-alert)); pass `-p audio_alert_enabled:=false` there to disable it.

   **Running with `target_distance` set to test the return-to-line correction?** No node logs the lateral (cross-track) offset automatically — measure it against your taped A→B line once the robot stops, and record it by hand in [`trials/lateral_offset_trials.csv`](trials/lateral_offset_trials.csv) (see [Lateral offset trials](#lateral-offset-trials-lateral_offset_trialscsv) below).

After step 2, repeat only step 3 for future runs — you only need to rebuild when you change `path_tracker.py`/`trial_logger.py`/`decision_logger.py`/`scan_trace_logger.py`/`scan_trace_record.py`/`audio_trigger.py` (repeat steps 1–2 each time).

## Setup

1. **Docker + ROS 2 Humble.** The project's Docker container (`MentorPi`, image `ros:humble`) is already running on the robot's Pi — confirm it can see the LanderPi's ROS 2 stack with `docker exec -u ubuntu MentorPi zsh -lc "source ~/.zshrc && ros2 topic list"` (`/scan_raw`, `/odom`, `/cmd_vel` should be visible). Always include `-u ubuntu` — running as `root` silently breaks FastRTPS's shared-memory transport between nodes (discovery still matches over UDP, but zero data ever delivers), which cost an entire debugging session before it was traced to this.
2. **Copy the package in** and **build** it — see [Running it in VS Code](#running-it-in-vs-code-quick-start) above for the exact commands.

## Usage

Run each node in its own terminal (or SSH session) — see [Running it in VS Code](#running-it-in-vs-code-quick-start) for the exact commands.

Then, for each trial:

1. Place the robot at point A on the marked track for the current surface.
2. Place an obstacle (or rely on the track's natural endpoint) at point B.
3. Let the robot drive. `trial_logger` detects the start automatically when the robot begins moving, and the end automatically once `path_tracker`'s obstacle stop has held for one second.
4. When prompted in Terminal 2, read the robot's actual stop position off your tape-measure marks and enter the surface name (granite/concrete/wood/metal/hpl) and ground-truth distance.
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
| `lidar_stop_range_m` | The actual LiDAR range to the closest obstacle in the forward arc at the moment the trial finalized (i.e. the real stop clearance), captured from `path_tracker`'s `/forward_min_range` topic. Useful for checking proximity-trigger accuracy against the `safety_distance` parameter and whether it varies by surface. Blank if nothing valid was in the arc at stop. **Measure `ground_truth_distance_m`/stop clearance from the LiDAR unit itself** (the rotating sensor housing), not the chassis front edge — `safety_distance` and this column are both computed against the LiDAR's own raw range, so that's the only measurement point directly comparable to them. Expect the real stop clearance to consistently undershoot the configured `safety_distance` by a few centimeters (at the defaults, `obstacle_confirm_scans=2` debounce scans at the LD19's ~10Hz rate, times `forward_speed`, accounts for ~5cm of it) — that's expected stop latency, not a measurement or code error. If your tape measurement doesn't match this logged value, that's the discrepancy worth investigating; a gap against `safety_distance` alone is not. |
| `notes` | Freeform text entered at logging time for anything unusual observed (e.g. "motors fought each other on the turn", "oscillated near desk", "false stop") |
| `battery_level` | Rough `High`/`Medium`/`Low` estimate captured from `/ros_robot_controller/battery` (raw millivolts) at the moment the trial finalized, assuming a 2S Li-ion pack (6.0V empty - 8.4V full). Not a precise state-of-charge reading -- just enough to flag "was the pack getting low during this session." Blank if no reading had arrived yet. |

## Refined obstacle avoidance

`path_tracker` runs a deterministic 7-state machine (`AvoidanceController`, a pure module unit-tested off-hardware) governed by one invariant: **always take the maneuver that minimizes deviation from the goal heading while maintaining safety.** That ordering falls out of it — strafe (holds the bearing) → turn-and-drive (bounded deviation) → one bounded recovery → halt:

- **DRIVE** — forward at `forward_speed`, PID holds the goal heading captured at A.
- **ASSESS** — on a confirmed obstacle, a strafe-first ladder: sidestep if a bounded strafe toward a *confirmed-clear* side clears the (narrow) obstacle; else turn toward the more-open side and drive past; else escalate.
- **STRAFE / TURN / DRIVE_PAST** — the maneuvers, each committed (run to completion) so an obstacle's jittering apparent side can't cause oscillation.
- **RECOVER** — after repeated failures, one bounded routine: back off, pick the widest fitting gap from the current 360° scan, rotate to face it, commit once.
- **HALT** — stop and flag; re-check each tick.

The LiDAR is reduced each scan into FRONT (+ front sub-sectors), LEFT, RIGHT, and REAR clearances. Every decision is published as a JSON record on `/avoidance_decision` and logged by `decision_logger` (see [Data collected](#data-collected)). Set `disable_avoidance:=true` to halt on any obstacle with no turn/strafe (used for the clean go-and-stop distance runs).

## Parameters

**`path_tracker`** (every field is a ROS parameter; only the commonly-tuned ones are shown — see `AvoidanceConfig` in `proximity_alert/avoidance.py` for the full list). This includes `heading_kp`/`heading_ki`/`heading_kd` (the heading-hold PID, [Data collected](#data-collected)'s `Kp`/`Ki`/`Kd` columns) — override per run/per surface with e.g. `--ros-args -p heading_kd:=0.2` without touching the shared defaults.

| Parameter | Default | Description |
|---|---|---|
| `safety_distance` | `0.20` | FRONT stop threshold, meters |
| `forward_speed` | `0.20` | Constant forward speed, m/s. Set to match what `/cmd_vel` actually delivers (see "Speed clamp" below), not an aspirational value — every distance-based timeout in this table (`max_drive_past_distance`, `recover_commit_distance`, `max_cumulative_strafe`) is `speed × elapsed_time`, so a mismatch here makes those give up before covering their configured real distance |
| `obstacle_confirm_scans` | `2` | Consecutive close scans required before a maneuver decision (debounce) |
| `strafe_speed` / `strafe_timeout` | `0.20` / `2` | Lateral speed and per-strafe time cap (m/s, s). `strafe_speed` is subject to the same `/cmd_vel` clamp as `forward_speed` |
| `strafe_side_clearance_min` | `0.20` | Side clearance required to strafe into it, meters |
| `max_obstacle_width` | `0.75` | Above this *physical* lateral width (meters), the obstacle is "wide" (a wall) → turn, not strafe. Keyed on physical width, not angular span: at trigger range any real object subtends a large angle, so an angular-span gate would block strafing entirely. |
| `max_cumulative_strafe` | `1.25` | Hard per-encounter lateral cap (long-wall guard), meters |
| `turn_speed` / `turn_step_deg` / `turn_timeout` | `0.75` / `30.0` / `1.5` | Turn rate, per-turn increment, time cap (rad/s, deg, s) |
| `turn_radius` | `0.0` | Reverse-arc radius during TURN/RECOVER, meters. Those states already command reverse + rotation together, so they trace an arc of radius `reverse_speed / turn_speed` — at legacy defaults a tight ~0.13m. Set > 0 to control that geometry directly (reverse speed becomes `turn_radius × turn_speed`): bigger = a wider, longer sweep instead of an almost-in-place pivot. `0` keeps the legacy fixed `avoid_reverse_speed` |
| `rear_taper_zone` | `0.0` | Distance above `rear_clearance_min` over which the reverse component fades out linearly rather than snapping to zero, meters. The hard cutoff makes a turn lurch from arc to pure pivot the moment clearance runs low; a taper degrades smoothly. `0` keeps the legacy hard cutoff |
| `max_avoid_attempts` | `3` | Failed cycles before escalating to recovery |
| `clear_drive_distance` | `0.3` | Confirmed-clear straight-line displacement that closes an encounter and resets its counters (`cumulative_strafe`, attempt count), meters. Measured from odometry — see [Distance-based encounter close](#distance-based-encounter-close) |
| `encounter_close_confirm_scans` | `3` | Consecutive clear scans required before `clear_drive_distance` even starts accumulating. Mirrors `obstacle_confirm_scans` on the exit side, so one noisy clear reading can't start (or falsely advance) the measurement |
| `odom_jump_threshold` | `0.15` | Per-tick position delta above which motion is treated as a discontinuous odometry jump (e.g. a localization reset) rather than real travel, meters. The in-progress measurement is abandoned and restarts after the next confirmed-clear streak |
| `min_gap_clearance` / `min_gap_width_deg` | `0.50` / `40.0` | What counts as a usable recovery gap (robot must fit) |
| `disable_avoidance` | `false` | Halt on any obstacle, no turn/strafe (clean go-and-stop runs) |
| `target_distance` | `0.0` | Distance **along the A→B line** (the projection, not straight-line displacement) after which the robot stops and reports `arrived_target_distance`. `0.0` disables — no stop condition and no odom watchdog, exactly as before this parameter existed (distance tracking itself still runs internally either way; nothing consumes it when disabled). Arrival is deferred until the avoidance state machine is back in `DRIVE`, so a strafe or turn finishes before the stop — this can cost real overshoot, up to roughly a metre if a full maneuver ladder (strafe, then turn, then drive-past) runs before DRIVE is reached again |
| `cross_track_kp` | `0.6` | Gain from cross-track error (m) to crab velocity (m/s) for the return-to-line correction. Derived from the *distance* you want re-centering to take, not a time: `ė = -kp·e` settles 95% in 3 time constants, so **`kp = 3v/D`** — at the delivered `v = 0.20` m/s and `D = 1.0` m, `kp = 0.6`. **Rescale if `forward_speed` changes**, or re-centering stretches over a proportionally longer distance. `0.0` disables the correction entirely |
| `cross_track_max_speed` | `0.10` | Crab velocity clamp, m/s. Derived from the largest acceptable crab angle: `v_lat = tan(angle) × forward_speed`; past ~27° the LiDAR's forward arc stops covering the direction the robot is actually travelling. Higher = re-centers sooner but drives increasingly sideways-on |
| `cross_track_deadband` | `0.03` | Offset below which the correction is exactly zero, meters. Set from the smallest offset you can actually measure on the floor (~1 cm ruler-tip) times a small factor — correcting below your own measurement resolution just chatters |
| `cross_track_ramp_time` | `0.3` | Seconds to fade the crab in after a maneuver hands back to `DRIVE`. Derived from the chassis acceleration limit in the platform's own `ekf.yaml` (1.3 m/s²): reaching 0.10 m/s needs ≥ 0.077 s, rounded up for margin. `0` = full strength immediately |
| `cross_track_tolerance` | `0.05` | How close to the line counts as "on it" for the arrival centering phase, meters. After covering `target_distance` the robot stops driving forward and keeps crabbing until within this band, then reports `arrived_target_distance`. `0.0` disables centering, so arrival latches the instant the distance is covered |
| `centering_timeout` | `5.0` | Max seconds spent centering before declaring arrival regardless of remaining offset. Guarantees termination when the correction is gated off or a flank is blocked; the leftover offset is then measured on the floor rather than asserted by the robot |
| `odom_timeout_sec` | `1.0` | Odometry watchdog, seconds. If `target_distance` is set and no `/odom` message arrives within this window, the robot stops and reports `stopped_odom_fault` rather than driving on a stale distance estimate. Ignored entirely when `target_distance` is `0.0` |
| `audio_alert_enabled` | `true` | Plays `wav_path` through the USB speaker once per obstacle encounter — see [Obstacle audio alert](#obstacle-audio-alert). Set `false` to disable |
| `wav_path` | `/home/ubuntu/shared/audio/obstacle_alert.wav` | WAV file to play (host path — see below) |
| `alsa_device` | `plughw:2,0` | ALSA device string for the robot's USB speaker (confirmed via `aplay -l` inside the container) |

Derived (computed, not configured): `max_strafe_distance = strafe_speed × strafe_timeout`; `corridor_half = robot_half_width + corridor_margin`; `clear_threshold = safety_distance + clear_margin`.

### Distance-based encounter close

An *encounter* is one obstacle, from the first confirmed detection until the
robot has clearly driven past it. While an encounter is open, the attempt count
and `cumulative_strafe` budget keep accruing; when it closes, they reset.

That close used to be a *time* value (`clear_drive_duration`, 3.0 s). The
problem: three seconds is not a place. Whether it corresponds to 10 cm or a
full metre depends entirely on `forward_speed`, so tuning it against real
obstacle spacing meant doing the arithmetic in your head every time — and two
obstacles spaced closer together than that interval were merged into a single
encounter, the second one inheriting the first's leftover attempt count and
strafe budget and escalating to RECOVER or HALT far sooner than it should have.

It is now a distance: `clear_drive_distance` (0.3 m) of confirmed-clear
straight-line displacement, taken from odometry. You set it directly in the
same units you measure the floor in.

Three details worth knowing:

- **`encounter_close_confirm_scans` (3) gates the start.** The measurement only
  begins after that many consecutive clear scans, mirroring what
  `obstacle_confirm_scans` does on the entry side, so a single noisy clear
  reading can't start or falsely advance it. A re-block before the threshold is
  reached discards the partial measurement outright — interrupted clear
  stretches never sum.
- **It is straight-line displacement, not integrated path length.** A robot
  that strafes and curves around an obstacle covers more ground than its net
  displacement, so this under-reports. That is deliberate: it errs toward
  keeping an encounter open, never toward closing one early.
- **`odom_jump_threshold` (0.15 m) rejects discontinuities.** A per-tick jump
  larger than this is a localization reset, not travel, so the in-progress
  measurement is abandoned rather than credited with a bogus few metres. Note
  it is *not* scaled by elapsed `dt`: at the confirmed 0.20 m/s clamp even a
  sluggish 5 Hz loop only covers ~0.04 m per tick, so 0.15 m is roughly 4×
  margin. A stalled control loop that legitimately covers more ground in one
  delayed tick could trip it — a known, accepted simplification, not a hidden
  gap. If long scheduling stalls ever become real, the fix is a Δt-aware
  version (`expected = forward_speed × dt`, jump if `actual > expected + margin`).

**These three defaults are starting points, not measured constants**, and need
on-hardware validation against real obstacle spacing and real odometry noise.

**Known limitation:** obstacles spaced closer together than roughly
`clear_drive_distance` plus the confirm-scan debounce distance are still
treated as one encounter. Distance alone cannot resolve that; telling "same
obstacle" from "new obstacle" would need a different signal entirely, which is
out of scope here.

### Speed clamp — why `forward_speed`/`strafe_speed` default to 0.20, not 0.50

`path_tracker` publishes to `/cmd_vel`. On this platform that topic is read by
the vendor's phone-app control node (`odom_publisher_node.py`), which clamps
`linear.x`/`linear.y` to **±0.20 m/s** before the command reaches either the
motors or `/odom` — a sensible limit for manual driving, but it applies to
every publisher on that topic, `path_tracker` included. Every other
autonomous node on this robot (`lidar_app`, `line_following`,
`object_tracking`, etc.) avoids it by publishing to `/controller/cmd_vel`
instead.

Confirmed on hardware, not just read from the driver source: a `target_distance:=1.0`
run's own log —

```
[INFO] path_tracker started: forward_speed=0.5m/s ... target_distance=1.0
[WARN] No fresh scan within scan_timeout; holding.   (x3, ~1s apart, ~2.16s of startup)
[INFO] Target distance 1.00 m reached (along-track 1.01 m, final offset +0.011 m) -- stopping.
```

— took 7.57 s end to end. Subtracting the ~2.16 s scan warm-up leaves ≤5.41 s
to cover 1.01 m: **≥0.187 m/s actual**, against an expected 2.00 s if the
configured 0.50 m/s were really reaching the motors — a 3.4 second gap,
independently corroborated by a tape-and-stopwatch trial (~100 cm in 5.7 s,
0.175 m/s).

**Fixed by aligning the config to reality (`forward_speed`/`strafe_speed` set to
`0.20`), not by switching topics.** Every distance-based counter in the
controller — `_drive_past_dist` vs. `max_drive_past_distance`, `_commit_dist`
vs. `recover_commit_distance`, `cumulative_strafe` vs. `max_cumulative_strafe`
— is `configured_speed × elapsed_time`. With the old `0.50` default those all
overestimated real distance travelled by ~2.5×, so e.g. `DRIVE_PAST` was
giving up after ~32 cm of *real* travel while believing it had driven 80 cm —
often not enough to actually clear an obstacle. Switching to
`/controller/cmd_vel` instead would fix the clamp but make the robot 2.5×
faster with no watchdog on the platform, invalidate every trial logged so
far, and need a fresh re-tune; that's left as a deliberate, separate future
change if higher speed is ever wanted.

### Arrival status (`/path_tracker/status`)

`path_tracker` publishes a `std_msgs/String` on `/path_tracker/status` every
control tick, reporting what is currently governing the robot:

| Value | Meaning |
|---|---|
| `driving` | Driving forward, or mid-avoidance-maneuver |
| `centering` | Along-track distance is covered, but the robot is still further than `cross_track_tolerance` off the A→B line. Forward motion has stopped; it is crabbing sideways onto the line. Ends in `arrived_target_distance` either on reaching the line or at `centering_timeout` |
| `arrived_target_distance` | `target_distance` reached while in `DRIVE` (and centering finished); stopped and latched |
| `arrived_obstacle` | The avoidance state machine reached `HALT` — it ran out of options. Note this is recoverable: if the obstacle is removed and the path stays clear, it returns to `driving` |
| `stopped_odom_fault` | `/odom` went stale while `target_distance` was set; stopped as a precaution |
| `stopped_scan_fault` | The scan-side counterpart to `stopped_odom_fault`: no fresh LiDAR scan within `scan_timeout`, so the node holds rather than driving blind. Never reported while an arrival is already latched — `arrived_target_distance` takes priority, so a LiDAR dropout after arrival still reports arrived |

### Return-to-line correction — and what it cannot do

Holding the goal *heading* was never enough to stay on the A→B line. A strafe
leaves the robot pointing the right way but bodily offset, so before this
correction existed it would clear an obstacle and then drive on **parallel to
the original line, permanently offset**. The fix crabs it back: a `linear.y`
command proportional to cross-track error, applied only in `DRIVE`.

Crab, not steer, because the chassis is mecanum. Stanley, Pure Pursuit and
line-of-sight guidance all exist to solve this on car-like bases that *cannot*
move sideways; their machinery (lookahead, `atan(k·e/v)`, curvature limits) is
there to turn lateral error into a steering angle without oscillating. This
robot can move sideways directly — and lateral strafe is the one motion the
platform does cleanly, since pure rotation hits the documented vendor
kinematics quirk. Crabbing also keeps the LiDAR pointed down the path, so the
forward-arc sectors keep meaning what the avoidance machine was tuned for.
The closed loop is a plain first-order lag (`ė = -kp·e`): exponential decay,
no overshoot for any `kp > 0`, and saturation only slows it.

**The correction never closes on the obstacle it just avoided.** A strafe
commands `linear_x = 0`, so it makes *no* forward progress — it ends with the
robot level with the obstacle, offset by the bare minimum that uncovered its
front arc. Crabbing straight back would drive into its flank, and worse, form
a stable limit cycle (strafe out → front clears → crab back → front blocks →
strafe out) that hangs the run. So the gate is asymmetric: moving *away* from
the avoided side is never blocked, while moving *toward* it requires that
side's live LiDAR clearance to exceed `pass_clearance`. Gating on measured
clearance rather than "drive forward N metres first" makes it self-timing —
it waits as long as the obstacle actually needs, which differs for a wall
versus a cone.

> **Scope limit — this corrects commanded displacement, not slip.**
> `/odom`'s x/y on this platform is an **open-loop integral of commanded
> velocity**: there are no wheel encoders, and the EKF's only configured
> exteroceptive x/y source (`odom1: odom_rf2o`, laser odometry) is **not
> running** — 0 publishers, so `odom0` contributes velocities only. The
> correction therefore undoes lateral displacement the robot *commanded* —
> which is exactly what a strafe-based avoidance maneuver produces, and the
> bug this was written for — but it **cannot see wheel slip or dead-reckoning
> drift**. If a strafe slips, odom still reports a perfect strafe and the
> robot will happily "re-center" onto a line that has itself drifted. Yaw is
> genuinely EKF-fused with the IMU and is trustworthy; position is not.
> Accuracy degrades with distance and with the number of maneuvers.
> **True path following requires an external position source** — starting the
> rf2o laser-odometry node, or equivalent. Measure the final offset on the
> floor (`centering` reports what the robot *believes*); don't take the
> robot's own number as ground truth.

**One `path_tracker` process per trial.** Arrival latches permanently — once
`arrived_target_distance` is reached the node stays stopped and will not
drive again. Ctrl-C and relaunch between runs. This is deliberate: an
auto-reset could be tripped by nudging the robot between trials.

**Launch order matters.** `path_tracker` latches its start position (`start_pos`)
on the first `/odom` message it receives after the node starts, not at any
later "trial start" moment. Place the robot at point A **first**, then launch
`path_tracker` — launching first and moving the robot to A afterward measures
`target_distance` from the wrong origin.

**`trial_logger`**

| Parameter | Default | Description |
|---|---|---|
| `csv_path` | `trial_log.csv` | Output CSV file path (point at `/home/ubuntu/shared/trials/<surface>.csv` to land on the host — see below) |
| `surface` | `""` | Surface material logged per trial. Empty (default) infers it from `csv_path`'s filename — e.g. `.../granite.csv` → `granite` — no terminal prompt needed; set explicitly to override (e.g. a shared/misc log whose filename doesn't match the surface) |
| `move_velocity_threshold` | `0.03` | Speed above which the robot is considered moving, m/s |
| `stop_velocity_threshold` | `0.02` | Speed below which the robot is considered stopped, m/s |
| `stop_confirm_duration` | `1.0` | Seconds of continuous stopped-ness before a trial is finalized |

**`decision_logger`**

| Parameter | Default | Description |
|---|---|---|
| `csv_path` | `/home/ubuntu/shared/trials/decision_log.csv` | Output CSV for the avoidance decision log (host path — see below) |

### Lateral offset trials (`lateral_offset_trials.csv`)

No node measures ground-truth cross-track offset — it can only ever be read off a tape measure on the floor, so [`trials/lateral_offset_trials.csv`](trials/lateral_offset_trials.csv) is a hand-maintained sheet, same pattern as `floor_test_log.csv` (header row, one row per session, blank cells you fill in after each run — not written by any ROS node).

| Column | Fill in with |
|---|---|
| `Date and Session #` | Same convention as the other sheets |
| `Target Distance (m)` / `± Target Distance Uncertainity (m)` | The `target_distance` you ran with, and your tape/ruler uncertainty |
| `Forward Speed (m/s)` / `± Speed Uncertainity (m/s)` | Pre-filled `0.20` — the real delivered speed, see [Speed clamp](#speed-clamp--why-forward_speedstrafe_speed-default-to-020-not-050) |
| `Cross Track Kp` / `Max Speed (m/s)` / `Tolerance (m)` | The `cross_track_*` values that run used — pre-filled with the shipped defaults so each row records exactly which tuning produced its result, useful once you start varying them |
| `Along-Track Stop Distance (m)` | Where it actually stopped along the tape, checked against `Target Distance` |
| `Lateral Offset at Stop (m)` / `± Lateral Offset Uncertainity (m)` | **The measurement this sheet exists for** — sideways offset from the taped A→B line at the moment it stopped. Pick a left/right sign convention and note it consistently |
| `Obstacle Side (Left/Right)` | Which side you placed the obstacle / which way it dodged — lets you check later whether the correction converges the same from both directions |
| `Measurement Method`, `Surface Type`, `Obstacle Type` | Same as the other sheets |
| `Centering Outcome` | `Completed`, `Timed Out`, or `Not Triggered` — reflects the `driving` → `centering` → `arrived_target_distance` sequence on `/path_tracker/status` (see [Arrival status](#arrival-status-path_trackerstatus)) |
| `Notes`, `Bugs/Issues`, `Battery level` | Same as the other sheets |

### Obstacle audio alert

`path_tracker` plays `wav_path` through the USB speaker (`aplay`, non-blocking) the first time an obstacle encounter enters a non-`DRIVE` state — once per `encounter_id`, not on every escalation step (strafe → turn → recover → halt) within it, and never during ordinary clear-path driving. It's on by default (`audio_alert_enabled:=true`); set `-p audio_alert_enabled:=false` to turn it off. This uses the same trigger logic (`audio_trigger.py`) as the original standalone `obstacle_audio` prototype node, now folded directly into `path_tracker` so it runs with no extra terminal — a non-blocking `aplay` call can never stall the control loop.

**A WAV file must exist or nothing plays.** `aplay` is spawned non-blocking with its stderr discarded, so a missing file used to produce no visible error — the robot just silently never beeped. `path_tracker` now logs an `ERROR` at startup if `wav_path` doesn't exist. To generate a quick test beep (no dependencies, run on the Pi):

```bash
mkdir -p /home/pi/docker/tmp/audio
python3 -c "
import math, struct, wave
r, d, f = 44100, 0.35, 880.0
n = int(r*d)
fr = b''.join(struct.pack('<h', int(32767*0.5*min(1,min(i,n-i)/(0.02*r))*math.sin(2*math.pi*f*i/r))) for i in range(n))
w = wave.open('/home/pi/docker/tmp/audio/obstacle_alert.wav','w'); w.setnchannels(1); w.setsampwidth(2); w.setframerate(r); w.writeframes(fr); w.close()"
```

Verify it plays with the exact command the node uses:

```bash
docker exec -u ubuntu MentorPi aplay -D plughw:2,0 /home/ubuntu/shared/audio/obstacle_alert.wav
```

**To use your own dialogue clip:** drop a WAV file at `/home/pi/docker/tmp/audio/obstacle_alert.wav` on the Pi (create the `audio/` folder if it doesn't exist yet) — that's the host side of the same bind mount the CSV/JSONL logs already use, so it lands at `/home/ubuntu/shared/audio/obstacle_alert.wav` inside the container automatically, with no container restart needed. See `docs/superpowers/specs/2026-07-24-obstacle-audio-alert-design.md` for the original design (the trigger logic it describes is unchanged; only which node calls it moved).

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
- [ ] Collect full trial sets across all five surfaces
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
