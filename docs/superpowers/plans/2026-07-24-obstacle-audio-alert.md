# Obstacle Audio Alert Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a fifth, independent ROS 2 node (`obstacle_audio`) that plays a WAV dialogue clip through the robot's USB speaker once per obstacle encounter, as a simple observational prototype with zero changes to `path_tracker`'s control loop.

**Architecture:** A pure, ROS-free `should_play(decision, last_played_encounter_id)` function (unit-tested on the host) plus a thin `obstacle_audio.py` ROS node that subscribes to the existing `/avoidance_decision` topic, calls `should_play`, and spawns `aplay` non-blocking via `subprocess.Popen` when it returns `True`. No new topics published; no dependency on `path_tracker`/`trial_logger`/`decision_logger`/`scan_trace_logger` — same independence principle already used by every other logger/companion node in this package.

**Tech Stack:** Python 3, rclpy (ROS 2 Humble), `std_msgs/String` (the existing `/avoidance_decision` transport), stdlib `subprocess`, ALSA `aplay` (already confirmed installed and working against the robot's USB speaker, card 2, from inside the `MentorPi` container — no new dependency), pytest.

## Global Constraints

- Full design spec (read before starting): `docs/superpowers/specs/2026-07-24-obstacle-audio-alert-design.md`.
- Trigger rule is exactly: play **once per obstacle encounter**, not on every state-transition escalation within it. Concretely: `should_play(decision, last_played_encounter_id)` returns `True` only when `decision.state != "DRIVE"` **and** `decision.encounter_id != last_played_encounter_id`. It returns `False` for any `DRIVE`-state decision (including the `cleared` record `_complete_to_drive` emits) regardless of `encounter_id`.
- No new topics are published by this node — it only subscribes to `/avoidance_decision` (existing `std_msgs/String`, JSON-encoded `DecisionRecord`, already published by `path_tracker`) and shells out to `aplay`.
- Playback is **non-blocking**: `subprocess.Popen(...)`, never `subprocess.run`/`.wait()` — this node must never block waiting for playback to finish, and must never touch `path_tracker`'s control loop or process.
- No new Python audio dependency. Use `aplay` via `subprocess.Popen(["aplay", "-D", alsa_device, wav_path])` exactly — confirmed working against the container's passed-through USB speaker with zero extra container configuration.
- ROS params on the new node: `wav_path` (default `/home/ubuntu/shared/audio/obstacle_alert.wav`), `alsa_device` (default `plughw:2,0`, the confirmed USB speaker device string from `aplay -l` inside the container), and `decision_topic` (default `/avoidance_decision`, parameterized so the node survives a topic rename).
- The node must never crash from an external dependency. Exactly two failure points get graceful handling (log a warning, keep the node alive), nothing more (no retry, no re-queue, no alerting — this is an observational prototype): (1) malformed `/avoidance_decision` JSON → `try/except (ValueError, TypeError)` around `DecisionRecord.from_json`, log and return early; (2) `aplay` not launchable → `try/except OSError` around the `subprocess.Popen` call only. A *missing WAV file* is deliberately NOT guarded — `Popen` doesn't raise for it (`aplay` spawns fine and errors to its own stderr), so the node stays alive naturally; the `OSError` guard is specifically for `aplay`-not-installed.
- `_last_played_encounter_id` is set when the node decides to play (before the `Popen` attempt), so a failed launch never retries within the same encounter.
- Overlapping `aplay` processes (two encounters in rapid succession) are left unguarded intentionally — cosmetic doubled sound only; mark this with a code comment so it reads as deliberate, not an oversight.
- Container package layout landmine (confirmed in earlier work on this branch): the colcon package root inside the container is `~/ros2_ws/src/proximity_alert/`, whose Python module directory is `~/ros2_ws/src/proximity_alert/proximity_alert/` (exactly two "proximity_alert" segments after `src/`). A stray, dead, third-nested `proximity_alert/proximity_alert/proximity_alert/` directory has existed before from a `docker cp` mistake — never write there. `docker cp` silently nests the source inside an existing destination rather than overwriting it, so always clear the destination first.
- Every `docker exec`/`docker cp` interaction with the container **must** use `-u ubuntu` (or immediately `chown ubuntu:ubuntu` afterward) — running as `root` leaves root-owned build artifacts that break subsequent `ubuntu`-user builds with `Permission denied`, and separately breaks FastRTPS shared-memory transport between ROS nodes.

---

### Task 1: `should_play` — pure trigger logic, host-tested

**Files:**
- Create: `proximity_alert/proximity_alert/audio_trigger.py`
- Test: `proximity_alert/test/test_audio_trigger.py`

**Interfaces:**
- Consumes: `DecisionRecord` from `proximity_alert.decision_record` (existing, unchanged) — specifically its `.state: str` and `.encounter_id: int` fields.
- Produces: `should_play(decision: DecisionRecord, last_played_encounter_id: int | None) -> bool`. Consumed by Task 2's `obstacle_audio.py`.

- [ ] **Step 1: Write the failing tests**

Create `proximity_alert/test/test_audio_trigger.py`:

```python
from proximity_alert.audio_trigger import should_play
from proximity_alert.decision_record import DecisionRecord


def _decision(state="STRAFE", encounter_id=1):
    return DecisionRecord(
        timestamp="2026-07-24 10:00:00", encounter_id=encounter_id, state=state,
        chosen_maneuver=state if state != "DRIVE" else "NONE", reason="test",
        obstacle_span_deg=10.0, front_distance_m=0.29, front_left_m=0.5,
        front_center_m=0.29, front_right_m=0.6, left_clearance_m=1.2,
        right_clearance_m=0.4, rear_clearance_m=2.0, required_clearing_m=0.22,
        cumulative_strafe_m=0.0, consecutive_avoid_count=1,
        recovery_triggered=False, outcome="committed", maneuver_duration_s=0.0,
    )


def test_plays_on_first_detection_in_new_encounter():
    decision = _decision(state="STRAFE", encounter_id=1)
    assert should_play(decision, last_played_encounter_id=None) is True


def test_does_not_replay_within_same_encounter():
    # Same encounter_id escalating STRAFE -> TURN -- already played once, stay silent.
    decision = _decision(state="TURN", encounter_id=1)
    assert should_play(decision, last_played_encounter_id=1) is False


def test_plays_again_on_new_encounter():
    decision = _decision(state="STRAFE", encounter_id=2)
    assert should_play(decision, last_played_encounter_id=1) is True


def test_never_plays_on_drive_state():
    # Includes the "cleared" record _complete_to_drive emits when an encounter ends.
    decision = _decision(state="DRIVE", encounter_id=1)
    assert should_play(decision, last_played_encounter_id=None) is False
    assert should_play(decision, last_played_encounter_id=5) is False
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd proximity_alert && PYTHONPATH=. python3 -m pytest test/test_audio_trigger.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'proximity_alert.audio_trigger'`

- [ ] **Step 3: Write the minimal implementation**

Create `proximity_alert/proximity_alert/audio_trigger.py`:

```python
"""Pure, ROS-free trigger logic for the obstacle_audio node.

Decides whether a newly-received /avoidance_decision record should play the
alert WAV: once per obstacle encounter (a new encounter_id in a non-DRIVE
state), never on every escalation step within the same encounter, and never
on DRIVE (including the "cleared" record emitted when an encounter ends).
See docs/superpowers/specs/2026-07-24-obstacle-audio-alert-design.md.
"""
from proximity_alert.decision_record import DecisionRecord


def should_play(decision: DecisionRecord, last_played_encounter_id):
    if decision.state == "DRIVE":
        return False
    return decision.encounter_id != last_played_encounter_id
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd proximity_alert && PYTHONPATH=. python3 -m pytest test/test_audio_trigger.py -v`
Expected: 4 passed

- [ ] **Step 5: Commit**

```bash
git add proximity_alert/proximity_alert/audio_trigger.py proximity_alert/test/test_audio_trigger.py
git commit -m "feat: add should_play, pure once-per-encounter trigger logic for obstacle audio alert"
```

---

### Task 2: `obstacle_audio` ROS node

**Files:**
- Create: `proximity_alert/proximity_alert/obstacle_audio.py`
- Test: `proximity_alert/test/test_obstacle_audio.py`
- Modify: `proximity_alert/setup.py`

**Interfaces:**
- Consumes: `should_play(decision, last_played_encounter_id) -> bool` from Task 1 (`proximity_alert.audio_trigger`); `DecisionRecord.from_json(text) -> DecisionRecord` from `proximity_alert.decision_record` (existing, unchanged).
- Produces: `ObstacleAudio` class (importable for tests) and a `main()` entry point registered as the `obstacle_audio` console script.

This task's test needs `rclpy`, which is not installed on the host — it must be copied into the container and run there (see `test_decision_logger_csv.py` for the existing precedent of an in-container-only test). The test also needs to verify a subprocess was spawned with the right arguments **without actually spawning `aplay`** during the test run — do this by patching `subprocess.Popen` at the module level (`proximity_alert.obstacle_audio.subprocess.Popen`), matching the spec's stated approach.

- [ ] **Step 1: Write the failing test**

Create `proximity_alert/test/test_obstacle_audio.py`:

```python
from unittest.mock import patch

import rclpy
from std_msgs.msg import String

from proximity_alert.decision_record import DecisionRecord
from proximity_alert.obstacle_audio import ObstacleAudio


def _decision_json(state="STRAFE", encounter_id=1):
    rec = DecisionRecord(
        timestamp="2026-07-24 10:00:00", encounter_id=encounter_id, state=state,
        chosen_maneuver=state if state != "DRIVE" else "NONE", reason="test",
        obstacle_span_deg=10.0, front_distance_m=0.29, front_left_m=0.5,
        front_center_m=0.29, front_right_m=0.6, left_clearance_m=1.2,
        right_clearance_m=0.4, rear_clearance_m=2.0, required_clearing_m=0.22,
        cumulative_strafe_m=0.0, consecutive_avoid_count=1,
        recovery_triggered=False, outcome="committed", maneuver_duration_s=0.0,
    )
    return rec.to_json()


def test_spawns_aplay_on_new_encounter():
    rclpy.init()
    node = ObstacleAudio()
    with patch("proximity_alert.obstacle_audio.subprocess.Popen") as mock_popen:
        node.on_decision(String(data=_decision_json(state="STRAFE", encounter_id=1)))
    node.destroy_node()
    rclpy.shutdown()
    mock_popen.assert_called_once_with(
        ["aplay", "-D", "plughw:2,0", "/home/ubuntu/shared/audio/obstacle_alert.wav"]
    )


def test_does_not_spawn_aplay_again_within_same_encounter():
    rclpy.init()
    node = ObstacleAudio()
    with patch("proximity_alert.obstacle_audio.subprocess.Popen") as mock_popen:
        node.on_decision(String(data=_decision_json(state="STRAFE", encounter_id=1)))
        node.on_decision(String(data=_decision_json(state="TURN", encounter_id=1)))
    node.destroy_node()
    rclpy.shutdown()
    assert mock_popen.call_count == 1


def test_does_not_spawn_aplay_on_drive_state():
    rclpy.init()
    node = ObstacleAudio()
    with patch("proximity_alert.obstacle_audio.subprocess.Popen") as mock_popen:
        node.on_decision(String(data=_decision_json(state="DRIVE", encounter_id=1)))
    node.destroy_node()
    rclpy.shutdown()
    mock_popen.assert_not_called()


def test_survives_malformed_decision_json():
    # A bad message on the topic must not kill the subscription or crash.
    rclpy.init()
    node = ObstacleAudio()
    with patch("proximity_alert.obstacle_audio.subprocess.Popen") as mock_popen:
        node.on_decision(String(data="{not valid json"))   # must not raise
    node.destroy_node()
    rclpy.shutdown()
    mock_popen.assert_not_called()


def test_survives_aplay_not_installed():
    # Popen raising OSError (e.g. aplay binary missing) must not crash the node.
    rclpy.init()
    node = ObstacleAudio()
    with patch("proximity_alert.obstacle_audio.subprocess.Popen",
               side_effect=FileNotFoundError("aplay")):
        node.on_decision(String(data=_decision_json(state="STRAFE", encounter_id=1)))  # must not raise
    node.destroy_node()
    rclpy.shutdown()


def test_construct_and_destroy_cleanly():
    # Lifecycle: constructing and destroying the node tears down its
    # subscription/resources without error.
    rclpy.init()
    node = ObstacleAudio()
    node.destroy_node()
    rclpy.shutdown()
```

- [ ] **Step 2: Run test to verify it fails**

This requires rclpy — copy the new test and (not-yet-existing) module into the container and run there:

```bash
docker cp proximity_alert/test/test_obstacle_audio.py MentorPi:/home/ubuntu/ros2_ws/src/proximity_alert/test/test_obstacle_audio.py
docker exec -u root MentorPi bash -lc "chown ubuntu:ubuntu /home/ubuntu/ros2_ws/src/proximity_alert/test/test_obstacle_audio.py"
docker exec -u ubuntu MentorPi zsh -lc "source ~/.zshrc >/dev/null 2>&1 && source ~/ros2_ws/install/setup.bash >/dev/null 2>&1 && cd ~/ros2_ws/src/proximity_alert && python3 -m pytest test/test_obstacle_audio.py -v"
```

Expected: FAIL with `ModuleNotFoundError: No module named 'proximity_alert.obstacle_audio'`

- [ ] **Step 3: Write the minimal implementation**

Create `proximity_alert/proximity_alert/obstacle_audio.py`:

```python
#!/usr/bin/env python3
"""Companion node: plays a WAV dialogue clip through the robot's USB speaker
once per obstacle encounter, independent of path_tracker/trial_logger/
decision_logger/scan_trace_logger.

Prototype only -- observational, not a safety feature. Subscribes to the
existing /avoidance_decision topic and shells out to `aplay` (ALSA)
non-blocking, so a slow or failed playback can never stall this node, let
alone path_tracker's control loop, which this node never touches.
See docs/superpowers/specs/2026-07-24-obstacle-audio-alert-design.md.
"""
import subprocess

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from proximity_alert.audio_trigger import should_play
from proximity_alert.decision_record import DecisionRecord


class ObstacleAudio(Node):
    def __init__(self):
        super().__init__("obstacle_audio")
        self.declare_parameter("wav_path", "/home/ubuntu/shared/audio/obstacle_alert.wav")
        self.declare_parameter("alsa_device", "plughw:2,0")
        self.declare_parameter("decision_topic", "/avoidance_decision")
        self.wav_path = self.get_parameter("wav_path").value
        self.alsa_device = self.get_parameter("alsa_device").value
        decision_topic = self.get_parameter("decision_topic").value
        self._last_played_encounter_id = None

        self.create_subscription(String, decision_topic, self.on_decision, 50)
        self.get_logger().info(
            f"obstacle_audio up. Will play '{self.wav_path}' via '{self.alsa_device}' "
            f"on obstacle detection from '{decision_topic}'."
        )

    def on_decision(self, msg: String):
        # A companion node must never die from bad input it doesn't control:
        # a malformed message on the topic is logged and skipped, not fatal.
        try:
            decision = DecisionRecord.from_json(msg.data)
        except (ValueError, TypeError) as e:
            self.get_logger().warn(f"Ignoring unparseable /avoidance_decision message: {e}")
            return
        if should_play(decision, self._last_played_encounter_id):
            # Mark as played BEFORE launching so a failed launch doesn't retry
            # within this encounter (retry within one encounter is suppressed
            # by the once-per-encounter gate anyway).
            self._last_played_encounter_id = decision.encounter_id
            # aplay is spawned non-blocking. Overlapping playback (two
            # encounters in quick succession) is left unguarded intentionally:
            # cosmetic doubled sound only, not worth added state for a prototype.
            # The try/except catches only aplay-not-installed (OSError); a
            # missing WAV doesn't raise here -- aplay handles that itself.
            try:
                subprocess.Popen(["aplay", "-D", self.alsa_device, self.wav_path])
            except OSError as e:
                self.get_logger().warn(
                    f"Could not launch aplay ({e}); is ALSA's aplay installed in this container?"
                )


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAudio()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
```

Add the console-script entry point. In `proximity_alert/setup.py`, change:

```python
        'console_scripts': [
            'path_tracker = proximity_alert.path_tracker:main',
            'trial_logger = proximity_alert.trial_logger:main',
            'decision_logger = proximity_alert.decision_logger:main',
            'floor_test_reconcile = proximity_alert.floor_test_reconcile:main',
            'scan_trace_logger = proximity_alert.scan_trace_logger:main',
        ],
```

to:

```python
        'console_scripts': [
            'path_tracker = proximity_alert.path_tracker:main',
            'trial_logger = proximity_alert.trial_logger:main',
            'decision_logger = proximity_alert.decision_logger:main',
            'floor_test_reconcile = proximity_alert.floor_test_reconcile:main',
            'scan_trace_logger = proximity_alert.scan_trace_logger:main',
            'obstacle_audio = proximity_alert.obstacle_audio:main',
        ],
```

Copy the new/changed files into the container at the **correct, verified** module path (`~/ros2_ws/src/proximity_alert/proximity_alert/`, not the triple-nested one — see Global Constraints) and rebuild. Also copy Task 1's files in now — they were only run on the host in Task 1 (pure Python, no rclpy needed there), but the full in-container regression check below needs them present too, exactly like every other pure-module test in this package:

```bash
docker cp proximity_alert/proximity_alert/audio_trigger.py MentorPi:/home/ubuntu/ros2_ws/src/proximity_alert/proximity_alert/audio_trigger.py
docker cp proximity_alert/proximity_alert/obstacle_audio.py MentorPi:/home/ubuntu/ros2_ws/src/proximity_alert/proximity_alert/obstacle_audio.py
docker cp proximity_alert/test/test_audio_trigger.py MentorPi:/home/ubuntu/ros2_ws/src/proximity_alert/test/test_audio_trigger.py
docker cp proximity_alert/setup.py MentorPi:/home/ubuntu/ros2_ws/src/proximity_alert/setup.py
docker exec -u root MentorPi bash -lc "chown -R ubuntu:ubuntu /home/ubuntu/ros2_ws/src/proximity_alert/proximity_alert/audio_trigger.py /home/ubuntu/ros2_ws/src/proximity_alert/proximity_alert/obstacle_audio.py /home/ubuntu/ros2_ws/src/proximity_alert/test/test_audio_trigger.py /home/ubuntu/ros2_ws/src/proximity_alert/setup.py /home/ubuntu/ros2_ws/build/proximity_alert /home/ubuntu/ros2_ws/install/proximity_alert 2>/dev/null; echo chowned"
docker exec -u ubuntu MentorPi zsh -lc "source ~/.zshrc >/dev/null 2>&1 && cd ~/ros2_ws && rm -rf build/proximity_alert install/proximity_alert && colcon build --packages-select proximity_alert 2>&1 | tail -5"
```

- [ ] **Step 4: Run test to verify it passes**

```bash
docker exec -u ubuntu MentorPi zsh -lc "source ~/.zshrc >/dev/null 2>&1 && source ~/ros2_ws/install/setup.bash >/dev/null 2>&1 && cd ~/ros2_ws/src/proximity_alert && python3 -m pytest test/test_obstacle_audio.py -v"
```

Expected: 6 passed

Then run the **full** in-container suite to confirm nothing else regressed:

```bash
docker exec -u ubuntu MentorPi zsh -lc "source ~/.zshrc >/dev/null 2>&1 && source ~/ros2_ws/install/setup.bash >/dev/null 2>&1 && cd ~/ros2_ws/src/proximity_alert && python3 -m pytest test/ -q"
```

Expected: all tests pass (55 existing + 4 Task-1 tests now copied in + 6 Task-2 tests = 65; exact prior count may differ slightly by the time this runs — the key check is 0 failed).

- [ ] **Step 5: Commit**

```bash
git add proximity_alert/proximity_alert/obstacle_audio.py proximity_alert/test/test_obstacle_audio.py proximity_alert/setup.py
git commit -m "feat: add obstacle_audio node, plays WAV once per obstacle encounter"
```

---

### Task 3: Documentation

**Files:**
- Modify: `README.md`

**Interfaces:**
- None (documentation only).

- [ ] **Step 1: Add the new files to the Repository structure diagram**

In `README.md`, find this block:

```
│       ├── decision_logger.py        # Logs each avoidance decision to its own CSV
│       ├── floor_test_reconcile.py   # Fills floor_test_log.csv's Time/Stop-clearance from real trial data
│       ├── scan_trace_record.py      # Pure JSON-Lines record for one raw scan tick
│       └── scan_trace_logger.py      # Logs every raw LiDAR scan continuously, for post-hoc miss diagnosis
├── trials/                     # CSVs, gitignored (not committed) -- see below
```

Replace it with:

```
│       ├── decision_logger.py        # Logs each avoidance decision to its own CSV
│       ├── floor_test_reconcile.py   # Fills floor_test_log.csv's Time/Stop-clearance from real trial data
│       ├── scan_trace_record.py      # Pure JSON-Lines record for one raw scan tick
│       ├── scan_trace_logger.py      # Logs every raw LiDAR scan continuously, for post-hoc miss diagnosis
│       ├── audio_trigger.py          # Pure once-per-encounter trigger logic for the obstacle audio alert
│       └── obstacle_audio.py         # Plays a WAV clip through the USB speaker once per obstacle encounter (prototype)
├── trials/                     # CSVs, gitignored (not committed) -- see below
```

- [ ] **Step 2: Add a Terminal E run command to Quick Start**

In `README.md`, find:

```
   ```bash
   # Terminal D — log every raw LiDAR scan (for diagnosing total detection misses)
   docker exec -it -u ubuntu MentorPi zsh -lc "source ~/.zshrc && source ~/ros2_ws/install/setup.bash && ros2 run proximity_alert scan_trace_logger --ros-args -p csv_path:=/home/ubuntu/shared/trials/scan_trace.jsonl"
   ```
```

Add immediately after it:

```

   ```bash
   # Terminal E — play a dialogue clip when an obstacle is detected (prototype)
   docker exec -it -u ubuntu MentorPi zsh -lc "source ~/.zshrc && source ~/ros2_ws/install/setup.bash && ros2 run proximity_alert obstacle_audio --ros-args -p wav_path:=/home/ubuntu/shared/audio/obstacle_alert.wav -p alsa_device:=plughw:2,0"
   ```
```

- [ ] **Step 3: Update the rebuild-trigger file list**

In `README.md`, find:

```
After step 2, repeat only step 3 for future runs — you only need to rebuild when you change `path_tracker.py`/`trial_logger.py`/`decision_logger.py`/`scan_trace_logger.py`/`scan_trace_record.py` (repeat steps 1–2 each time).
```

Replace it with:

```
After step 2, repeat only step 3 for future runs — you only need to rebuild when you change `path_tracker.py`/`trial_logger.py`/`decision_logger.py`/`scan_trace_logger.py`/`scan_trace_record.py`/`obstacle_audio.py`/`audio_trigger.py` (repeat steps 1–2 each time).
```

- [ ] **Step 4: Add an `obstacle_audio` parameter table**

In `README.md`, find:

```
**`decision_logger`**

| Parameter | Default | Description |
|---|---|---|
| `csv_path` | `/home/ubuntu/shared/trials/decision_log.csv` | Output CSV for the avoidance decision log (host path — see below) |

### Three logs, all on your computer
```

Replace it with:

```
**`decision_logger`**

| Parameter | Default | Description |
|---|---|---|
| `csv_path` | `/home/ubuntu/shared/trials/decision_log.csv` | Output CSV for the avoidance decision log (host path — see below) |

**`obstacle_audio`** (prototype — plays a WAV dialogue clip through the robot's USB speaker once per obstacle encounter; see [Obstacle audio alert](#obstacle-audio-alert-prototype) below)

| Parameter | Default | Description |
|---|---|---|
| `wav_path` | `/home/ubuntu/shared/audio/obstacle_alert.wav` | WAV file to play (host path — see below) |
| `alsa_device` | `plughw:2,0` | ALSA device string for the robot's USB speaker (confirmed via `aplay -l` inside the container) |
| `decision_topic` | `/avoidance_decision` | Topic to listen on for obstacle-detection events (parameterized so a topic rename doesn't require editing the node) |

### Obstacle audio alert (prototype)

`obstacle_audio` subscribes to the existing `/avoidance_decision` topic and plays `wav_path` through the USB speaker (`aplay`, non-blocking) the first time an encounter enters a non-`DRIVE` state — once per `encounter_id`, not on every escalation step (strafe → turn → recover → halt) within it, and never during ordinary clear-path driving. It is fully independent: no changes to `path_tracker`'s control loop, no new topics.

**To use your own dialogue clip:** drop a WAV file at `/home/pi/docker/tmp/audio/obstacle_alert.wav` on the Pi (create the `audio/` folder if it doesn't exist yet) — that's the host side of the same bind mount the CSV/JSONL logs already use, so it lands at `/home/ubuntu/shared/audio/obstacle_alert.wav` inside the container automatically, with no container restart needed. See `docs/superpowers/specs/2026-07-24-obstacle-audio-alert-design.md` for the full design.

### Three logs, all on your computer
```

- [ ] **Step 5: Commit**

```bash
git add README.md
git commit -m "docs: document obstacle_audio node, Terminal E run command, WAV placement"
```

---

## Plan self-review notes

- **Spec coverage:** "Architecture" section → Task 2. "Trigger logic" section → Task 1 (`should_play`) + Task 2 (calling it from `on_decision`). "Configuration" section (`wav_path`/`alsa_device`/`decision_topic` params) → Task 2. "Error handling" (graceful on the two external failure points, nothing more) → Task 2's `try/except (ValueError, TypeError)` around `from_json` and `try/except OSError` around `Popen`, each with a covering test (`test_survives_malformed_decision_json`, `test_survives_aplay_not_installed`). "Testing" section → Tasks 1 and 2, including mocking `Popen` rather than exercising real playback, plus the lifecycle test (`test_construct_and_destroy_cleanly`). "Open questions" (overlapping `aplay` processes, missing WAV file) → both explicitly left unguarded per the spec's own YAGNI call, the overlapping case now marked intentional in a code comment.
- **Placeholder scan:** none found — every step has complete, runnable code or exact commands with expected output.
- **Type consistency:** `should_play(decision, last_played_encounter_id)` signature and `DecisionRecord.state`/`.encounter_id` field names are identical across Task 1's definition, Task 1's tests, Task 2's `on_decision` call site, and Task 2's tests. `ObstacleAudio`'s `wav_path`/`alsa_device`/`decision_topic` param names match between the class definition and every reference. The five wrapper tests observe behavior only through the mocked `Popen` call and node lifecycle (not direct attribute access), so no attribute name can drift silently past a green test.
- **Review-feedback incorporation (this pass):** folded in the five review points — (1) graceful `Popen` failure via `try/except OSError` + test; (2) `test_construct_and_destroy_cleanly` lifecycle test; (3) overlapping-playback left intentional, now stated in a code comment; (4) `decision_topic` made a ROS param; (5) graceful malformed-JSON handling via `try/except (ValueError, TypeError)` + test. The spec was updated in lockstep so it remains the accurate design-of-record.
