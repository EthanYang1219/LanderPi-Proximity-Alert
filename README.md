# LanderPi Proximity Alert

LiDAR-based proximity detection and reactive obstacle avoidance on a mecanum-wheeled
mobile robot, built to measure **how surface material affects the gap between a robot's
real-world travel and its own estimate of that travel.**

Undergraduate robotics research at the University of Prince Edward Island.

---

## Overview

A robot with no wheel encoders only knows how far it has gone by integrating the speed
it *asked* for. On a surface where the wheels slip, that estimate drifts — and the amount
it drifts is a property of the surface.

This project measures that drift directly. For each surface material, the robot drives a
fixed, marked track from point A to point B while three quantities are recorded:

- **Transit time** — how long the run took
- **Odometry distance** — the robot's own dead-reckoned estimate
- **Ground-truth distance** — measured by hand off a taped track

The difference between the last two quantifies wheel slippage. A LiDAR safety layer runs
on top so the robot can share the track with obstacles without colliding — and that safety
layer grew into the larger half of the codebase.

The repository contains two independent systems:

| Package | What it is | Status |
|---|---|---|
| **`proximity_alert`** | LiDAR-only A→B driving with a 7-state reactive avoidance machine, plus the logging pipeline that produced the research dataset. | Field-validated; used to collect the published data |
| **`poc_fusion`** | A proof of concept fusing an Aurora depth camera with the LiDAR into a single Nav2 costmap, responding with a stop only. | **In progress** — see [status](#poc_fusion-status) |

They share the robot and the `/scan_raw` topic, and nothing else. `poc_fusion` deliberately
does not touch `avoidance.py`, so the surface-trial dataset stays comparable across the
whole collection period.

---

## Hardware and software

| Component | Detail |
|---|---|
| Chassis | Hiwonder LanderPi, mecanum wheels (wheelbase 0.216 m, track 0.195 m) |
| Main compute | Raspberry Pi 5 |
| Low-level control | STM32 (motor commands; holds the last commanded velocity indefinitely) |
| LiDAR | LD19, single fixed scan plane, ~10 Hz |
| Depth camera | Aurora, arm-mounted, 640×400 mono16 @ 14.7 Hz (`poc_fusion` only) |
| Odometry | Integrated open-loop from `/cmd_vel` — **no wheel encoders** |
| Audible alert | USB speaker via ALSA `aplay`; `poc_fusion` additionally drives an I2C buzzer |
| OS / middleware | Ubuntu 22.04, ROS 2 Humble in Docker (`MentorPi` container) |
| Language | Python 3 (`rclpy`) |

---

## How it works

```
LD19 ──▶ scan_utils ──▶ AvoidanceController ──▶ path_tracker ──▶ motion_watchdog ──▶ motors
         (sectors,       (7-state machine,       (ROS shell)      (stale ⇒ stop)
          sizing,         pure & ROS-free)
          gap search)
/odom ─▶ nav_utils ──────────┘
         (along-track, cross-track)
```

**Sensing.** Each `LaserScan` is reduced once into validated `(bearing, range)` pairs, then
into seven named sector clearances: front, front-left/center/right, left, right, rear. The
obstacle ahead is measured in **metres of lateral width**, not angular span — angular span
grows as an obstacle approaches, so it cannot answer "can I fit past this?". A best escape
gap is selected from the full 360° sweep.

**Deciding.** Those products feed a pure state machine at 20 Hz, governed by one invariant:
*take the manoeuvre that minimises deviation from the goal heading while maintaining safety.*
The escalation order falls out of that:

| State | Behaviour |
|---|---|
| `DRIVE` | Forward at `forward_speed`, PID holding the heading captured at A |
| `ASSESS` | Obstacle confirmed — choose a manoeuvre, trying both sides before giving up on strafing |
| `STRAFE` | Mecanum sidestep. Preferred: it holds the bearing exactly |
| `TURN` → `DRIVE_PAST` | Fallback for obstacles too wide to sidestep |
| `RECOVER` | Once per encounter — back off, rotate to the widest fitting gap, commit |
| `HALT` | Stop and flag. Recoverable: re-checked every tick |

Manoeuvres run to completion so an obstacle's jittering apparent side cannot cause
oscillation. Arrival is decided *after* the controller runs and only gates whether its
output is forwarded, so adding the fixed-distance A→B stop did not alter avoidance behaviour.

**Acting.** `path_tracker` publishes `Twist` on `/cmd_vel`, plays a once-per-encounter
audio alert, and emits a structured JSON record per decision. `motion_watchdog` sits between
the command source and the motors and forces a stop the moment its input goes stale — the
STM32 has no watchdog of its own and will otherwise hold the last command forever.

### Topics

| Topic | Type | Direction |
|---|---|---|
| `/scan_raw` | `sensor_msgs/LaserScan` | in |
| `/odom` | `nav_msgs/Odometry` | in |
| `/cmd_vel` | `geometry_msgs/Twist` | out |
| `/forward_min_range` | `std_msgs/Float32` | out — live forward-arc range |
| `/avoidance_decision` | `std_msgs/String` (JSON) | out — one record per decision |
| `/path_tracker/status` | `std_msgs/String` | out — driving / arrived / fault |

---

## Repository structure

```
proximity_alert/            ROS 2 package — A→B driving, avoidance, research logging
  proximity_alert/
    path_tracker.py         Main node: ROS shell around the controller
    avoidance.py            Pure 7-state machine + AvoidanceConfig (all tunables)
    scan_utils.py           Pure LiDAR reduction: beams → sectors, sizing, gap search
    nav_utils.py            Pure along-track / cross-track geometry
    motion_watchdog.py      Fail-safe cmd_vel forwarder  (+ _logic.py, pure)
    trial_logger.py         Per-trial transit time, odometry, ground truth
    decision_logger.py      Per-decision CSV        (+ decision_record.py, pure)
    scan_trace_logger.py    Raw per-scan JSONL trace (+ scan_trace_record.py, pure)
    floor_test_reconcile.py Fills the hand-kept tuning log from real trial data
    audio_trigger.py        Pure once-per-encounter alert logic
  test/                     pytest suite

poc_fusion/                 ROS 2 package — depth + LiDAR costmap fusion POC
  poc_fusion/               Four nodes; lib/ holds their pure, ROS-free logic
  config/                   YAML parameters per node
  launch/                   Full pipeline bring-up
  tools/                    Stopping-distance measurement
  test/                     pytest suite

organized_data/             The published research dataset (CSVs + column docs)
docs/                       Design specs, verification log, trial layouts, operations
scripts/                    Host → container deploy helper
```

The clearest reading order is `avoidance.py` (the decisions), then `scan_utils.py` (what it
decides on), then `path_tracker.py` (how it reaches the robot).

---

## Setup and running

The ROS 2 stack runs inside the `MentorPi` Docker container on the robot's Pi. Bring-up
commands, the deploy step, and the shutdown procedure are in
**[docs/OPERATIONS.md](docs/OPERATIONS.md)** — read the shutdown section before running
anything that moves.

A minimal run, each node in its own terminal:

```bash
ros2 run proximity_alert path_tracker --ros-args -p target_distance:=2.0
ros2 run proximity_alert motion_watchdog
ros2 run proximity_alert trial_logger
```

`trial_logger` detects the start of a trial from motion and the end from a held stop, then
prompts for the surface name and the tape-measured distance.

---

## Testing

Both packages carry a pytest suite, and most of it runs off-robot. **There is no working
top-level test command** — each package's imports resolve only from inside its own directory.

```bash
cd poc_fusion     && python3 -m pytest test/ -q
cd proximity_alert && python3 -m pytest test/ -q --continue-on-collection-errors
```

| Suite | Requires | Expected |
|---|---|---|
| `poc_fusion` | numpy, scipy, pyyaml | 162 passed, 2 skipped |
| `proximity_alert`, pure modules | stdlib only | 139 passed |
| `proximity_alert`, 6 ROS modules | `rclpy` | 6 collection errors off-robot — run inside the container |

`--continue-on-collection-errors` is required off-robot: without it the `rclpy` import
failures abort the whole run and **zero** tests execute. The six errors are the host
environment, not a regression.

Everything above is software validation. Behaviour on the physical robot is verified
separately and recorded in [docs/poc_fusion_verification.md](docs/poc_fusion_verification.md).

---

## Configuration

Every tunable for the avoidance system lives in one place: the `AvoidanceConfig` dataclass
in [`avoidance.py`](proximity_alert/proximity_alert/avoidance.py). Each field carries an
inline comment explaining what raising or lowering it actually does. Every field is
automatically declared as a ROS parameter, so anything can be overridden per run without
editing shared defaults:

```bash
ros2 run proximity_alert path_tracker --ros-args -p safety_distance:=0.25 -p heading_kd:=0.2
```

The ones that matter most:

| Parameter | Default | Effect |
|---|---|---|
| `safety_distance` | 0.20 m | How close something ahead gets before the robot reacts |
| `forward_speed` | 0.20 m/s | Drive speed. Must match what the vendor controller actually delivers, or every distance and timeout below it starts lying |
| `max_obstacle_width` | 0.75 m | Above this, turn around it instead of sidestepping |
| `target_distance` | 0.0 | A→B run length; `0.0` disables the fixed-distance stop entirely |
| `disable_avoidance` | `false` | Halt on any obstacle, no manoeuvre — used for clean straight-line distance runs |
| `heading_kp/ki/kd` | 5.0 / 0.05 / 0.2 | Heading-hold PID |

`poc_fusion` is configured through YAML in [`poc_fusion/config/`](poc_fusion/config/) instead.

---

## Data

The published dataset is in [`organized_data/`](organized_data/), indexed by
[`DATA_INDEX.md`](organized_data/DATA_INDEX.md): per-surface trial logs (carpet, granite,
wood), the decision log, and the hand-maintained tuning logs. Every column is documented in
the README beside the data, including known integrity flags on specific rows.

Raw scan traces are **not** committed — `scan_trace.jsonl` runs to about 1 GB per session.

---

## Limitations

These are real, confirmed on hardware, and they bound what the data can support.

**No wheel encoders.** `/odom` integrates *commanded* velocity, so `odom_distance_m` measures
"how far the commanded motion thinks it went", not how far the wheels turned. Two
consequences:

1. On a clean straight-line run (`disable_avoidance:=true`), the gap against ground truth is
   a reasonable indirect proxy for slippage, because the commanded motion is known and constant.
2. On any run where avoidance triggered, it is **not**. Heading is also integrated open-loop,
   so it accumulates error through every turn. Filter on `avoidance_events == 0` before using
   a trial for slippage analysis.

**The LiDAR has a fixed-height blind spot.** The LD19 scans one plane and cannot see above or
below it. A round pedestal desk — narrow base, wide overhanging top — was not detected, and the
robot made low-speed contact with it. Lowering `safety_distance` would not have helped: the
obstacle was effectively invisible until contact, not merely detected late. Use obstacles with
a consistent cross-section at the LiDAR's mounted height.

**Cross-track correction only undoes commanded displacement.** The return-to-line crab closes
offset measured in the same open-loop odometry described above, so it cannot see wheel slip or
dead-reckoning drift. True path following needs an external position source.

**Debouncing counts control ticks, not scans.** The control loop runs at 20 Hz and the LD19
publishes at ~10 Hz, so two consecutive ticks may be the same scan counted twice. This affects
every debounce in the controller.

### `poc_fusion` status

An in-progress proof of concept, not a finished subsystem. End-to-end integration of the
depth + LiDAR chain **was** verified on the robot (2026-08-10: sensor ingress, costmap
representation, clear→CLEAR, obstacle→stop, and recovery, all with recorded sample counts).
Still open:

- Depth-only detection of an overhang was never live-verified.
- A bench test on 2026-08-11 **failed its gate**: at this camera's mounting geometry, depth
  cannot contribute for floor-standing obstacles. This is the main open question over the
  whole approach.
- Validation/tuning, compute budget, graceful degradation and the fusion-benefit A/B
  measurement were not carried out — several parameters remain deliberate placeholders.

`stop_action_node` is off by default in the launch file. It is a **series gate** on `cmd_vel`;
read its `REQUIRED TOPOLOGY` section before enabling it.

---

## Documentation

| Where | What |
|---|---|
| [docs/OPERATIONS.md](docs/OPERATIONS.md) | Running the robot: bring-up, shutdown, recovery from known failures |
| [docs/README.md](docs/README.md) | Index of the design and evidence record, and the `poc_fusion` task-number key |
| [docs/poc_fusion_verification.md](docs/poc_fusion_verification.md) | Dated on-robot verification log |
| [organized_data/DATA_INDEX.md](organized_data/DATA_INDEX.md) | Every data file and what produced it |

---

## Authors

- **Ethan Yang** — programming, physical data collection
- **Haotian** — statistical analysis, lead author
- Supervised by **Dr. Bingxian Mu**, University of Prince Edward Island

## License

Not yet declared. Both `package.xml` files and both `setup.py` files currently carry a
placeholder pending confirmation of the appropriate licence for this work.
