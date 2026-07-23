# Obstacle audio alert — design spec

**Date:** 2026-07-24
**Status:** Approved for implementation planning

## Problem

The robot has no audible signal when it detects an obstacle — only the CSV/JSONL logs record what happened, after the fact. A simple audio cue (a WAV dialogue clip) played through the robot's onboard USB speaker when an obstacle is detected would make the behavior observable in real time. This is an early prototype: functionality over polish.

## Goal

Play a WAV file through the robot's USB speaker once per obstacle encounter, with zero risk to the existing, already-validated `path_tracker` control loop.

## Non-goals

- Not a safety feature — purely observational/demonstrative for this prototype.
- Not handling multiple simultaneous/overlapping sounds, volume control, or a sound library — one fixed WAV file, one trigger condition.
- Not touching `path_tracker`'s control loop or the `AvoidanceController` — this only *listens* to what it already publishes.
- Not adding a new Python audio dependency — `aplay` (ALSA) is already confirmed working against the robot's USB speaker from inside the container, with zero extra setup.

## Architecture

A fourth independent node, `obstacle_audio.py`, following the same decoupling principle as `decision_logger`/`scan_trace_logger`: it subscribes to the existing `/avoidance_decision` topic (JSON `DecisionRecord`, published once per state *transition*, not every control tick) and has no dependency on `path_tracker`'s internals or process lifetime.

**Confirmed on hardware:** the `MentorPi` container has `/dev/snd` passed through and sees the same USB speaker (`card 2, USB PnP Audio Device`) as the host; `aplay -D plughw:2,0 <file>.wav` needs no extra container configuration.

**Rejected alternatives:**
- Triggering playback from inside `path_tracker` — couples audio I/O into the safety-critical control loop; the README already documents deliberately keeping the (still-unwired) buzzer separate from the motion node for this exact reason. A blocking or slow subprocess spawn inside the 10Hz callback risks control-loop jitter.
- A Python audio library (`playsound`/`simpleaudio`) instead of shelling out to `aplay` — adds a dependency for no benefit; `aplay` already works against this hardware.

## Trigger logic

On each `/avoidance_decision` message:
1. Parse the JSON `DecisionRecord` (reusing the existing `DecisionRecord.from_json`).
2. If `state != "DRIVE"` **and** `encounter_id` is different from the last `encounter_id` this node played audio for, spawn `aplay` non-blocking and record this `encounter_id` as played.
3. Otherwise, do nothing.

This fires exactly once per obstacle encounter, not on every escalation step (strafe→turn→recover→halt) within the same encounter, and never during ordinary clear-path driving (`DRIVE` decisions, e.g. the `cleared` record, are ignored). No explicit reset logic is needed: the controller only ever increments `encounter_id`, so "different from last played" is equivalent to "new encounter" without needing to track more state.

## Configuration

ROS params on the new node:
- `wav_path` (default `/home/ubuntu/shared/audio/obstacle_alert.wav`) — same bind-mount convention as the existing CSV/JSONL logs (`/home/pi/docker/tmp/audio/` on the host), so a new WAV file can be dropped in without a container rebuild.
- `alsa_device` (default `plughw:2,0`) — the confirmed USB speaker device string.
- `decision_topic` (default `/avoidance_decision`) — the topic to subscribe to. Parameterized rather than hardcoded so the node keeps working if the decision topic is ever renamed, at near-zero cost.

## Error handling

The node must **never crash from an external dependency it doesn't control** — a companion node that dies silently is worse than useless. Two failure points get graceful handling (log a warning, keep the node alive and listening); nothing else does, since this is an observational prototype, not a safety system (no retry, no re-queue, no alerting):

1. **Malformed `/avoidance_decision` JSON.** If another publisher ever puts non-JSON or structurally-wrong data on the topic, `DecisionRecord.from_json` raises `ValueError`/`TypeError`. Wrap the parse in a `try/except` that logs a warning and returns early, so one bad message doesn't kill the subscription.
2. **`aplay` not launchable.** `subprocess.Popen(["aplay", ...])` raises `OSError` (e.g. `FileNotFoundError`) only if the `aplay` binary itself can't be executed. Wrap just the `Popen` call in a `try/except OSError` that logs a warning. Note: a *missing WAV file* does **not** raise here — `Popen` spawns `aplay` fine and `aplay` handles the missing file by erroring to its own stderr, so the node stays alive naturally in that case; the `try/except` is specifically for `aplay`-not-installed / unexecutable.

`_last_played_encounter_id` is updated when the node *decides* to play (before the `Popen` attempt), so a failed launch does not cause a retry storm within the same encounter — retrying within one encounter is already suppressed by the once-per-encounter gate regardless.

## Testing

Split following the project's established pure/wrapper pattern:
- A pure, ROS-free function `should_play(decision: DecisionRecord, last_played_encounter_id: int | None) -> bool`, unit-tested on the host with no rclpy.
- A thin ROS wrapper (`obstacle_audio.py`) that subscribes to `decision_topic`, calls `should_play`, and spawns `aplay` — tested in-container (needs rclpy), following the existing pattern in `test_decision_logger_csv.py`. The subprocess call itself is not exercised against real audio hardware; the test patches `subprocess.Popen` and verifies it is invoked with the right argument list when `should_play` returns `True`, and not invoked when it returns `False`. Additional wrapper tests cover the two graceful-failure paths (malformed JSON → no crash, no playback; `Popen` raising `FileNotFoundError` → no crash) and a construct-then-destroy lifecycle test to confirm the node's subscription/resources tear down cleanly.

## Open questions / edge cases considered

- **Overlapping `aplay` processes.** If `should_play` somehow returned `True` twice in rapid succession (shouldn't happen given the `encounter_id` gate, but not architecturally prevented), two `aplay` processes could run concurrently. Left unguarded **intentionally** in this prototype — the trigger logic already makes it very unlikely, and the consequence (a doubled-up sound) is cosmetic, not harmful. Documented here and in a code comment so it's a known, deliberate choice rather than an oversight.
- **WAV file missing at runtime.** `aplay` will simply error to its own stderr; the node keeps running and keeps listening for the next encounter. No node-level crash (this is why the `try/except` around `Popen` targets `OSError` for the not-installed case, not the missing-file case, which never reaches `Popen`).
