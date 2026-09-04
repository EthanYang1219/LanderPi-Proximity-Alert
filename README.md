# LanderPi Proximity Alert

LiDAR-based proximity detection and reactive obstacle avoidance on a mecanum-wheeled
mobile robot, built to measure **how surface material affects the gap between a robot's
real-world travel and its own estimate of that travel.**

Undergraduate robotics research, University of Prince Edward Island.

## Overview

A robot with no wheel encoders only knows how far it has gone by integrating the speed it
*asked* for. Where the wheels slip, that estimate drifts — and how much it drifts is a
property of the surface.

For each surface material, the robot drives a fixed taped track from A to B while transit
time, odometry-estimated distance, and hand-measured ground-truth distance are recorded.
The gap between the last two quantifies wheel slippage. A LiDAR safety layer runs on top so
the robot can share the track with obstacles — and that layer grew into the larger half of
the codebase.

Two independent packages, sharing the robot and the `/scan_raw` topic and nothing else:

| Package | What it is | Status |
|---|---|---|
| **`proximity_alert`** | LiDAR-only A→B driving, a 7-state reactive avoidance machine, and the logging that produced the dataset | Field-validated |
| **`poc_fusion`** | Depth camera + LiDAR fused into one Nav2 costmap, responding with a stop only | [In progress](#poc_fusion-status) |

`poc_fusion` deliberately never touches `avoidance.py`, so the surface trials stay
comparable across the whole collection period.

## Hardware and software

| | |
|---|---|
| Chassis | Hiwonder LanderPi, mecanum (wheelbase 0.216 m, track 0.195 m) |
| Compute | Raspberry Pi 5 + STM32 (holds the last commanded velocity indefinitely) |
| LiDAR | LD19, single fixed scan plane, ~10 Hz |
| Depth camera | Aurora, arm-mounted, 640×400 mono16 @ 14.7 Hz — `poc_fusion` only |
| Odometry | Integrated open-loop from `/cmd_vel` — **no wheel encoders** |
| Stack | Ubuntu 22.04, ROS 2 Humble in Docker (`MentorPi`), Python 3 / `rclpy` |

## How it works

```
LD19 ──▶ scan_utils ──▶ AvoidanceController ──▶ path_tracker ──▶ motion_watchdog ──▶ motors
         (sectors,       (7-state machine,       (ROS shell)      (stale ⇒ stop)
          sizing, gap)     pure & ROS-free)
/odom ─▶ nav_utils ──────────┘
```

**Sensing.** Each scan is reduced once into validated `(bearing, range)` pairs, then into
seven sector clearances: front, front-left/center/right, left, right, rear. The obstacle
ahead is measured in **metres of lateral width**, not angular span — span grows as an
obstacle nears, so it cannot answer "can I fit past this?".

**Deciding.** Those feed a pure state machine at 20 Hz under one invariant: *take the
manoeuvre that minimises deviation from the goal heading while maintaining safety.* The
escalation order falls out of it.

| State | Behaviour |
|---|---|
| `DRIVE` | Forward, PID holding the heading captured at A |
| `ASSESS` | Obstacle confirmed — choose a manoeuvre, trying both sides before abandoning strafe |
| `STRAFE` | Mecanum sidestep. Preferred: it holds the bearing exactly |
| `TURN` → `DRIVE_PAST` | Fallback for obstacles too wide to sidestep |
| `RECOVER` | Once per encounter — back off, rotate to the widest fitting gap, commit |
| `HALT` | Stop and flag. Recoverable: re-checked every tick |

Manoeuvres run to completion, so an obstacle's jittering apparent side cannot cause
oscillation. Arrival is decided *after* the controller runs and only gates whether its
output is forwarded — which is why the fixed-distance A→B stop could be added without
altering avoidance behaviour.

**Acting.** `path_tracker` publishes `Twist` on `/cmd_vel`, plays a once-per-encounter audio
alert, and emits a JSON record per decision on `/avoidance_decision`. `motion_watchdog` sits
between the command source and the motors and forces a stop the moment its input goes
stale — the STM32 has no watchdog and will otherwise hold the last command forever.

Other topics: `/scan_raw` and `/odom` in; `/forward_min_range` and `/path_tracker/status` out.

## Repository structure

```
proximity_alert/    A→B driving, avoidance, research logging   (6 nodes)
poc_fusion/         depth + LiDAR costmap fusion POC           (4 nodes, config/, launch/)
organized_data/     the published dataset + column docs
docs/               design specs, verification log, operations
scripts/            host → container deploy helper
```

Within each package, `lib/` (or the bare modules in `proximity_alert/`) hold pure, ROS-free
logic that is unit-tested off-robot; the `*_node.py` files are thin ROS shells around them.
Every module carries a docstring explaining what it owns and why.

Clearest reading order: [`avoidance.py`](proximity_alert/proximity_alert/avoidance.py) (the
decisions) → [`scan_utils.py`](proximity_alert/proximity_alert/scan_utils.py) (what it
decides on) → [`path_tracker.py`](proximity_alert/proximity_alert/path_tracker.py) (how it
reaches the robot).

## Running it

The stack runs inside the `MentorPi` container on the robot's Pi. Bring-up, deploy and
shutdown are in **[docs/OPERATIONS.md](docs/OPERATIONS.md)** — read the shutdown section
before running anything that moves.

```bash
ros2 run proximity_alert path_tracker --ros-args -p target_distance:=2.0
ros2 run proximity_alert motion_watchdog     # not optional — see How it works
ros2 run proximity_alert trial_logger
```

`trial_logger` detects a trial's start from motion and its end from a held stop, then
prompts for the surface name and the tape-measured distance.

## Testing

Most of the suite runs off-robot. **There is no working top-level test command** — each
package's imports resolve only from inside its own directory.

```bash
cd poc_fusion      && python3 -m pytest test/ -q                                # 162 passed, 2 skipped
cd proximity_alert && python3 -m pytest test/ -q --continue-on-collection-errors # 139 passed, 6 errors
```

`--continue-on-collection-errors` is required off-robot: six modules import `rclpy`, and
without the flag those collection failures abort the run so **zero** tests execute. Those
six errors are the host environment, not a regression — run the same command inside the
container to exercise them.

That is software validation only. On-robot behaviour is recorded in
[docs/poc_fusion_verification.md](docs/poc_fusion_verification.md).

## Configuration

Every avoidance tunable lives in the `AvoidanceConfig` dataclass in
[`avoidance.py`](proximity_alert/proximity_alert/avoidance.py), each field carrying an inline
comment on what raising or lowering it actually does. All of them are auto-declared as ROS
parameters, so anything can be overridden per run:

```bash
ros2 run proximity_alert path_tracker --ros-args -p safety_distance:=0.25
```

The ones most often touched: `safety_distance` (0.20 m, reaction threshold), `forward_speed`
(0.20 m/s — must match what the vendor controller actually delivers, or every distance and
timeout derived from it starts lying), `max_obstacle_width` (0.75 m, above which it turns
rather than sidesteps), `target_distance` (`0.0` disables the fixed-distance stop), and
`disable_avoidance` (halt on any obstacle, used for clean straight-line runs).

`poc_fusion` is configured through YAML in [`poc_fusion/config/`](poc_fusion/config/).

## Data

The dataset is in [`organized_data/`](organized_data/) — per-surface trial logs, the decision
log, and the hand-kept tuning logs — indexed with full column definitions and known
integrity flags in [`DATA_INDEX.md`](organized_data/DATA_INDEX.md). Raw scan traces are not
committed; `scan_trace.jsonl` runs to roughly 1 GB per session.

## Limitations

Confirmed on hardware, and they bound what the data can support.

**No wheel encoders.** `/odom` integrates *commanded* velocity, so `odom_distance_m` measures
how far the commanded motion thinks it went, not how far the wheels turned. On a clean
straight-line run (`disable_avoidance:=true`) the gap against ground truth is a reasonable
indirect proxy for slippage, because the commanded motion is known and constant. On any run
where avoidance triggered it is **not** — heading is also integrated open-loop and
accumulates error through every turn. Filter on `avoidance_events == 0` before using a trial
for slippage analysis.

**The LiDAR has a fixed-height blind spot.** The LD19 scans one plane. A round pedestal desk
— narrow base, wide overhanging top — was never detected, and the robot made low-speed
contact with it. Lowering `safety_distance` would not have helped: the obstacle was
invisible until contact, not merely detected late. Use obstacles with a consistent
cross-section at the LiDAR's mounted height.

**Cross-track correction only undoes commanded displacement.** The return-to-line crab closes
offset measured in the same open-loop odometry above, so it cannot see slip or drift. True
path following needs an external position source.

**Debouncing counts control ticks, not scans.** The loop runs at 20 Hz and the LD19 publishes
at ~10 Hz, so two consecutive ticks may be the same scan counted twice.

### `poc_fusion` status

A proof of concept, not a finished subsystem. End-to-end integration of the depth + LiDAR
chain **was** verified on the robot (2026-08-10, with recorded sample counts). Still open:
depth-only overhang detection was never live-verified; a bench test on 2026-08-11 **failed
its gate** — at this camera's mounting geometry depth cannot contribute for floor-standing
obstacles, which is the main open question over the approach; and validation/tuning, compute
budget, graceful degradation and the fusion-benefit A/B were not carried out, so several
parameters remain deliberate placeholders. Per-task status is in
[docs/README.md](docs/README.md).

`stop_action_node` is off by default in the launch file. It is a **series gate** on
`cmd_vel` — read its `REQUIRED TOPOLOGY` section before enabling it.

## Documentation

[docs/OPERATIONS.md](docs/OPERATIONS.md) — running the robot, shutdown, recovery from known
failures · [docs/README.md](docs/README.md) — index of the design and evidence record ·
[docs/poc_fusion_verification.md](docs/poc_fusion_verification.md) — dated on-robot log ·
[organized_data/DATA_INDEX.md](organized_data/DATA_INDEX.md) — every data file and its source.

## Authors

**Ethan Yang** — programming, physical data collection ·
**Haotian** — statistical analysis, lead author ·
supervised by **Dr. Bingxian Mu**, University of Prince Edward Island.

**License:** not yet declared. Both `package.xml` and both `setup.py` files carry a
placeholder pending confirmation of the appropriate licence for this work.
