# Scan Trace Logger Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a fourth, independent ROS 2 node (`scan_trace_logger`) that continuously records the full raw LiDAR scan (every beam, not just reduced sectors) plus a reduced-sector summary to a JSON-Lines file, so a total detection miss (e.g. a thin chair leg the LiDAR never saw) can be diagnosed after the fact from raw sensor data, even though it never triggers an avoidance encounter and so has no row in either existing log.

**Architecture:** A pure, ROS-free `ScanTraceRecord` dataclass (JSON serialization only, unit-tested on the host) plus a thin `scan_trace_logger.py` ROS node that subscribes directly to `/scan_raw`, builds one `ScanTraceRecord` per incoming `LaserScan`, and appends it as one line to a `.jsonl` file. No new topics published; no dependency on `path_tracker`, `trial_logger`, or `decision_logger` — same independence principle already used by the other two logger nodes.

**Tech Stack:** Python 3, rclpy (ROS 2 Humble), `sensor_msgs/LaserScan`, stdlib `json`/`math`/`os`, pytest.

## Global Constraints

- Full design spec (read before starting): `docs/superpowers/specs/2026-07-24-scan-trace-logger-design.md`.
- Record schema is exactly: `scan_number` (int), `stamp_sec` (int), `stamp_nanosec` (int), `angle_min`/`angle_increment`/`range_min`/`range_max` (float), `ranges` (list of float, non-finite values as sentinel strings), `sectors` (dict of 7 named floats from `reduce_to_sectors`, same sentinel handling).
- `stamp_sec`/`stamp_nanosec` come from `LaserScan.header.stamp`, **never** wall-clock time (`datetime.now()`/`time.strftime`) — this is a deliberate, spec-mandated choice so the trace stays on the same clock as every other ROS message.
- Non-finite floats (`inf`, `-inf`, `nan`) anywhere in `ranges` or `sectors` are serialized as the literal strings `"inf"`, `"-inf"`, `"nan"` — **not** Python's non-standard `Infinity`/`-Infinity`/`NaN` JSON extension tokens — so every line stays valid standard JSON, parseable by strict JSON readers (`pandas.read_json`, `jq`, non-Python tooling).
- Output defaults to `/home/ubuntu/shared/trials/scan_trace.jsonl` (ROS param `csv_path`, kept as the param name for consistency with `trial_logger`/`decision_logger` even though the file is `.jsonl`).
- Sector-window parameters (`front_arc_deg`, `front_subsector_deg`, `side_window_deg`, `rear_window_deg`) are declared as this node's own independent ROS params (matching how `trial_logger`/`decision_logger` each independently declare `csv_path` rather than sharing `path_tracker`'s config) — defaults `180.0`/`60.0`/`60.0`/`60.0` to match `path_tracker`'s current defaults.
- **These four defaults must be kept manually in sync with `path_tracker`'s `AvoidanceConfig` fields of the same name.** There is no shared config source between the two nodes (by design — the independence principle above), so if `path_tracker`'s `front_arc_deg`/`front_subsector_deg`/`side_window_deg`/`rear_window_deg` defaults are ever tuned, this node's matching defaults must be updated too, or the logged `sectors` will silently stop reflecting what the controller was actually reacting to at the time — a diagnostic trap for exactly the kind of after-the-fact analysis this node exists for. Both the code comment in Task 2 and the README bullet in Task 3 call this out.
- **Scan topic confirmed live and independently configurable.** `ros2 topic list` on the running container confirms `/scan_raw` is the actual published topic (`/scan` does not exist). The node still declares `scan_topic` as its own ROS param (default `/scan_raw`) rather than hardcoding the string, so it keeps working via `-p scan_topic:=...` if the LiDAR topic name is ever renamed upstream — same override mechanism `path_tracker` already uses for the same topic.
- **Interrupted write leaves at most one incomplete trailing line.** Because each scan is appended as a single `f.write(...)` call ending in `\n`, a process killed mid-write can only ever corrupt the *last* line of the file — every prior line is already complete and flushed by the previous call. Offline analysis should tolerate this: skip a line that fails `json.loads` (expected only ever at end-of-file) rather than treating it as data corruption requiring investigation.
- This is scoped for bounded trial sessions only (~10-35s each); no size cap, rotation, or truncation logic is in scope.
- Container package layout landmine (confirmed this session): the colcon package root inside the container is `~/ros2_ws/src/proximity_alert/`, whose Python module directory is `~/ros2_ws/src/proximity_alert/proximity_alert/` — **do not** create a third nested `proximity_alert/proximity_alert/proximity_alert/` by copying to the wrong depth; `docker cp` silently nests the source inside an existing destination rather than overwriting it. Every container file-sync command below gives the exact, verified-correct destination path.
- Every `docker exec`/`docker cp` interaction with the container **must** use `-u ubuntu` (or immediately `chown ubuntu:ubuntu` afterward) — running as `root` leaves root-owned build artifacts that break subsequent `ubuntu`-user builds with `Permission denied`, and separately breaks FastRTPS shared-memory transport between ROS nodes if it's ever used to run a node rather than just copy/build files.

---

### Task 1: `ScanTraceRecord` — pure JSON serialization, host-tested

**Files:**
- Create: `proximity_alert/proximity_alert/scan_trace_record.py`
- Test: `proximity_alert/test/test_scan_trace_record.py`

**Interfaces:**
- Produces: `ScanTraceRecord` dataclass with fields `scan_number: int, stamp_sec: int, stamp_nanosec: int, angle_min: float, angle_increment: float, range_min: float, range_max: float, ranges: list, sectors: dict`; methods `to_json(self) -> str` and classmethod `from_json(cls, text: str) -> ScanTraceRecord`. Consumed by Task 2's `scan_trace_logger.py`.

- [ ] **Step 1: Write the failing tests**

Create `proximity_alert/test/test_scan_trace_record.py`:

```python
import json
import math

from proximity_alert.scan_trace_record import ScanTraceRecord


def _rec(ranges=None, sectors=None):
    return ScanTraceRecord(
        scan_number=4821,
        stamp_sec=1784824869,
        stamp_nanosec=454205692,
        angle_min=-3.14159,
        angle_increment=0.01745,
        range_min=0.12,
        range_max=25.0,
        ranges=ranges if ranges is not None else [0.45, 0.46, 1.0],
        sectors=sectors if sectors is not None else {
            "front": 0.45, "front_left": 1.2, "front_center": 0.45,
            "front_right": 0.9, "left": 1.4, "right": 0.9, "rear": 2.1,
        },
    )


def test_round_trips_finite_values():
    rec = _rec()
    restored = ScanTraceRecord.from_json(rec.to_json())
    assert restored == rec


def test_round_trips_inf_and_nan_in_ranges():
    # NaN != NaN under Python's normal equality, so this test deliberately
    # does NOT do `restored == rec` (dataclass equality would compare the
    # nan field with `==` and always fail even on a correct round-trip).
    # Every field is asserted individually, using math.isnan() for the nan
    # case specifically.
    rec = _rec(ranges=[0.5, float("inf"), float("-inf"), float("nan")])
    restored = ScanTraceRecord.from_json(rec.to_json())
    assert restored.ranges[0] == 0.5
    assert restored.ranges[1] == float("inf")
    assert restored.ranges[2] == float("-inf")
    assert math.isnan(restored.ranges[3])


def test_round_trips_inf_in_sectors():
    rec = _rec(sectors={
        "front": float("inf"), "front_left": 1.0, "front_center": float("inf"),
        "front_right": 1.0, "left": 1.0, "right": 1.0, "rear": float("inf"),
    })
    restored = ScanTraceRecord.from_json(rec.to_json())
    assert restored.sectors["front"] == float("inf")
    assert restored.sectors["front_center"] == float("inf")
    assert restored.sectors["rear"] == float("inf")
    assert restored.sectors["front_left"] == 1.0


def test_round_trips_realistic_lidar_sized_scan():
    # The LD19 publishes roughly 450 beams per revolution on this hardware
    # (confirmed via ros2 topic echo during development) -- a handful of
    # hand-picked values in earlier tests doesn't prove serialization holds
    # up at real scan width. Build a realistic-sized ranges array with a mix
    # of finite readings and inf gaps (a real scan is never all-finite).
    n = 452
    ranges = [
        float("inf") if i % 37 == 0 else round(0.3 + 0.01 * (i % 50), 4)
        for i in range(n)
    ]
    sectors = {
        "front": 0.45, "front_left": 1.2, "front_center": 0.45,
        "front_right": 0.9, "left": 1.4, "right": 0.9, "rear": 2.1,
    }
    rec = _rec(ranges=ranges, sectors=sectors)
    restored = ScanTraceRecord.from_json(rec.to_json())

    assert len(restored.ranges) == n
    for original, got in zip(ranges, restored.ranges):
        if math.isinf(original):
            assert math.isinf(got) and got > 0
        else:
            assert got == original
    assert restored.sectors == sectors


def test_serializes_as_strict_standard_json_not_python_extension_tokens():
    # json.dumps' default non-standard Infinity/-Infinity/NaN tokens would
    # break strict JSON readers (pandas.read_json, jq, non-Python tooling).
    # The design mandates quoted string sentinels instead -- assert the
    # actual wire format, not just that our own from_json can read it back.
    text = _rec(ranges=[float("inf"), float("nan")]).to_json()
    assert "Infinity" not in text
    assert "NaN" not in text
    assert '"inf"' in text
    assert '"nan"' in text
    # Must still be valid standard JSON (json.loads with default strict
    # allow_nan behavior is fine either way, but this proves no bare
    # unquoted non-finite token slipped through).
    parsed = json.loads(text)
    assert parsed["ranges"] == ["inf", "nan"]
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd proximity_alert && PYTHONPATH=. python3 -m pytest test/test_scan_trace_record.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'proximity_alert.scan_trace_record'`

- [ ] **Step 3: Write the minimal implementation**

Create `proximity_alert/proximity_alert/scan_trace_record.py`:

```python
"""Pure, ROS-free record for one raw LiDAR scan tick, JSON-Lines serialized.

See docs/superpowers/specs/2026-07-24-scan-trace-logger-design.md for the
full design. Non-finite floats (inf/-inf/nan) in `ranges`/`sectors` are
encoded as the literal strings "inf"/"-inf"/"nan" rather than relying on
Python json's non-standard Infinity/NaN extension tokens, so every line
stays valid standard JSON for non-Python readers.
"""
import json
import math
from dataclasses import asdict, dataclass


def _encode_float(v):
    if isinstance(v, float) and not math.isfinite(v):
        if math.isnan(v):
            return "nan"
        return "inf" if v > 0 else "-inf"
    return v


def _decode_float(v):
    if isinstance(v, str) and v in ("inf", "-inf", "nan"):
        return float(v)
    return v


@dataclass
class ScanTraceRecord:
    scan_number: int
    stamp_sec: int
    stamp_nanosec: int
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float
    ranges: list
    sectors: dict

    def to_json(self):
        d = asdict(self)
        d["ranges"] = [_encode_float(r) for r in d["ranges"]]
        d["sectors"] = {k: _encode_float(v) for k, v in d["sectors"].items()}
        return json.dumps(d, allow_nan=False)

    @classmethod
    def from_json(cls, text):
        d = json.loads(text)
        d["ranges"] = [_decode_float(r) for r in d["ranges"]]
        d["sectors"] = {k: _decode_float(v) for k, v in d["sectors"].items()}
        return cls(**d)
```

`allow_nan=False` on the `json.dumps` call is a defense-in-depth check: if `_encode_float` ever missed converting a non-finite value (a future field added without updating the encoder, say), `json.dumps` raises `ValueError` immediately instead of silently emitting a non-standard token that would violate the "valid standard JSON" guarantee.

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd proximity_alert && PYTHONPATH=. python3 -m pytest test/test_scan_trace_record.py -v`
Expected: 5 passed

- [ ] **Step 5: Commit**

```bash
git add proximity_alert/proximity_alert/scan_trace_record.py proximity_alert/test/test_scan_trace_record.py
git commit -m "feat: add ScanTraceRecord, pure JSON-Lines serialization for raw scan traces"
```

---

### Task 2: `scan_trace_logger` ROS node

**Files:**
- Create: `proximity_alert/proximity_alert/scan_trace_logger.py`
- Test: `proximity_alert/test/test_scan_trace_logger.py`
- Modify: `proximity_alert/setup.py`

**Interfaces:**
- Consumes: `ScanTraceRecord(scan_number, stamp_sec, stamp_nanosec, angle_min, angle_increment, range_min, range_max, ranges, sectors)` and `.to_json()` from Task 1 (`proximity_alert.scan_trace_record`); `reduce_to_sectors(msg, front_arc_deg, front_subsector_deg, side_window_deg, rear_window_deg) -> dict` from `proximity_alert.scan_utils` (existing, unchanged).
- Produces: `ScanTraceLogger` class (importable for tests) and a `main()` entry point registered as the `scan_trace_logger` console script.

This task's test needs `rclpy`, which is not installed on the host — it must be copied into the container and run there (this project has no rclpy on the host by design; see `test_decision_logger_csv.py` for the existing precedent of an in-container-only test).

- [ ] **Step 1: Write the failing test**

Create `proximity_alert/test/test_scan_trace_logger.py`:

```python
import json
import math
import os
import tempfile
from types import SimpleNamespace

import rclpy

from proximity_alert.scan_trace_logger import ScanTraceLogger


def _scan(ranges, stamp_sec=1784824869, stamp_nanosec=454205692,
          angle_min=-math.pi, inc_deg=1.0, range_min=0.05, range_max=8.0):
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp_sec, nanosec=stamp_nanosec)),
        angle_min=angle_min, angle_increment=math.radians(inc_deg),
        range_min=range_min, range_max=range_max, ranges=ranges,
    )


def _fresh_logger(csv_path):
    node = ScanTraceLogger()
    node.csv_path = csv_path
    return node


def test_writes_one_jsonl_line_per_scan_with_scan_number_and_stamp():
    rclpy.init()
    fd, path = tempfile.mkstemp(suffix=".jsonl")
    os.close(fd)
    os.remove(path)  # let the node create it fresh on first write

    node = _fresh_logger(path)
    n = 360  # 1-degree increments over 2*pi
    ranges = [1.0] * n
    node.on_scan(_scan(ranges, stamp_sec=100, stamp_nanosec=200))
    node.on_scan(_scan(ranges, stamp_sec=100, stamp_nanosec=300))

    with open(path) as f:
        lines = [json.loads(line) for line in f]

    node.destroy_node()
    rclpy.shutdown()
    os.remove(path)

    assert len(lines) == 2
    assert lines[0]["scan_number"] == 0
    assert lines[1]["scan_number"] == 1          # increments per scan, in receipt order
    assert lines[0]["stamp_sec"] == 100
    assert lines[0]["stamp_nanosec"] == 200
    assert lines[1]["stamp_nanosec"] == 300
    assert lines[0]["range_max"] == 8.0
    assert len(lines[0]["ranges"]) == 360
    assert "front" in lines[0]["sectors"]        # reduce_to_sectors output attached


def test_ranges_and_sectors_use_string_sentinels_for_non_finite():
    rclpy.init()
    fd, path = tempfile.mkstemp(suffix=".jsonl")
    os.close(fd)
    os.remove(path)

    node = _fresh_logger(path)
    n = 360
    ranges = [float("inf")] * n  # nothing in range anywhere -> every sector is inf
    node.on_scan(_scan(ranges))

    with open(path) as f:
        line = json.loads(f.readline())

    node.destroy_node()
    rclpy.shutdown()
    os.remove(path)

    assert line["ranges"][0] == "inf"
    assert line["sectors"]["front"] == "inf"
    assert line["sectors"]["rear"] == "inf"
```

- [ ] **Step 2: Run test to verify it fails**

This requires rclpy — copy the new test and (not-yet-existing) module into the container and run there:

```bash
docker cp proximity_alert/test/test_scan_trace_logger.py MentorPi:/home/ubuntu/ros2_ws/src/proximity_alert/test/test_scan_trace_logger.py
docker exec -u root MentorPi bash -lc "chown ubuntu:ubuntu /home/ubuntu/ros2_ws/src/proximity_alert/test/test_scan_trace_logger.py"
docker exec -u ubuntu MentorPi zsh -lc "source ~/.zshrc >/dev/null 2>&1 && source ~/ros2_ws/install/setup.bash >/dev/null 2>&1 && cd ~/ros2_ws/src/proximity_alert && python3 -m pytest test/test_scan_trace_logger.py -v"
```

Expected: FAIL with `ModuleNotFoundError: No module named 'proximity_alert.scan_trace_logger'`

- [ ] **Step 3: Write the minimal implementation**

Create `proximity_alert/proximity_alert/scan_trace_logger.py`:

```python
#!/usr/bin/env python3
"""Companion node: records the full raw LiDAR scan (every beam) plus a
reduced-sector summary to a continuous JSON-Lines trace file, independent of
path_tracker/trial_logger/decision_logger.

Exists to diagnose a total detection miss (e.g. a thin chair leg the LiDAR
never saw) -- such a miss never triggers path_tracker's AvoidanceController,
so it has no row in either trial_logger's or decision_logger's CSVs. This
node captures raw scan data continuously so a miss can be traced back after
the fact and classified as a sensor-geometry blind spot versus a tunable
threshold miss. See docs/superpowers/specs/2026-07-24-scan-trace-logger-design.md.

Each scan is appended as a single write ending in a newline, so a process
killed mid-write can only ever leave the LAST line incomplete -- every prior
line is already complete. Offline analysis should skip a line that fails to
parse (expected only at end-of-file) rather than treat it as corruption.
"""
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from proximity_alert.scan_trace_record import ScanTraceRecord
from proximity_alert.scan_utils import reduce_to_sectors


class ScanTraceLogger(Node):
    def __init__(self):
        super().__init__("scan_trace_logger")
        self.declare_parameter("csv_path", "/home/ubuntu/shared/trials/scan_trace.jsonl")
        # /scan_raw is the actual, confirmed-live LiDAR topic (verified via
        # `ros2 topic list`; no /scan topic exists) -- kept as an overridable
        # param, not hardcoded, so this still works if the topic is ever
        # renamed upstream, same as path_tracker's own scan_topic param.
        self.declare_parameter("scan_topic", "/scan_raw")
        # These four defaults MUST be kept in sync with AvoidanceConfig's
        # fields of the same name in avoidance.py. There is no shared config
        # between this node and path_tracker (independent by design), so if
        # path_tracker's sector-window tuning ever changes, these must be
        # updated too -- otherwise the logged `sectors` silently stop
        # reflecting what the controller was actually reacting to.
        self.declare_parameter("front_arc_deg", 180.0)
        self.declare_parameter("front_subsector_deg", 60.0)
        self.declare_parameter("side_window_deg", 60.0)
        self.declare_parameter("rear_window_deg", 60.0)

        self.csv_path = self.get_parameter("csv_path").value
        self.front_arc_deg = self.get_parameter("front_arc_deg").value
        self.front_subsector_deg = self.get_parameter("front_subsector_deg").value
        self.side_window_deg = self.get_parameter("side_window_deg").value
        self.rear_window_deg = self.get_parameter("rear_window_deg").value

        self._scan_number = 0

        scan_topic = self.get_parameter("scan_topic").value
        self.create_subscription(LaserScan, scan_topic, self.on_scan, qos_profile_sensor_data)
        self.get_logger().info(f"scan_trace_logger up. Writing to '{self.csv_path}'.")

    def on_scan(self, msg: LaserScan):
        sectors = reduce_to_sectors(
            msg, self.front_arc_deg, self.front_subsector_deg,
            self.side_window_deg, self.rear_window_deg,
        )
        rec = ScanTraceRecord(
            scan_number=self._scan_number,
            stamp_sec=msg.header.stamp.sec,
            stamp_nanosec=msg.header.stamp.nanosec,
            angle_min=msg.angle_min,
            angle_increment=msg.angle_increment,
            range_min=msg.range_min,
            range_max=msg.range_max,
            ranges=list(msg.ranges),
            sectors=sectors,
        )
        self._scan_number += 1

        os.makedirs(os.path.dirname(self.csv_path) or ".", exist_ok=True)
        with open(self.csv_path, "a") as f:
            f.write(rec.to_json() + "\n")


def main(args=None):
    rclpy.init(args=args)
    node = ScanTraceLogger()
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

Note: `os.makedirs(...)` runs on every write (not just once at startup like `decision_logger`'s `_ensure_header`) because there is no header to seed the file with on first creation — `exist_ok=True` makes the repeated call a cheap no-op once the directory exists.

Add the console-script entry point. In `proximity_alert/setup.py`, change:

```python
        'console_scripts': [
            'path_tracker = proximity_alert.path_tracker:main',
            'trial_logger = proximity_alert.trial_logger:main',
            'decision_logger = proximity_alert.decision_logger:main',
            'floor_test_reconcile = proximity_alert.floor_test_reconcile:main',
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
        ],
```

Copy both changed/new files into the container at the **correct, verified** module path (`~/ros2_ws/src/proximity_alert/proximity_alert/`, not the triple-nested one — see Global Constraints) and rebuild:

Also copy Task 1's test file in now — it was only run on the host in Task 1 (pure Python, no rclpy needed there), but the full in-container regression check below needs it present too, exactly like every other pure-module test in this package (`test_avoidance_controller.py`, `test_scan_utils.py`, etc. all live in both places):

```bash
docker cp proximity_alert/proximity_alert/scan_trace_logger.py MentorPi:/home/ubuntu/ros2_ws/src/proximity_alert/proximity_alert/scan_trace_logger.py
docker cp proximity_alert/proximity_alert/scan_trace_record.py MentorPi:/home/ubuntu/ros2_ws/src/proximity_alert/proximity_alert/scan_trace_record.py
docker cp proximity_alert/test/test_scan_trace_record.py MentorPi:/home/ubuntu/ros2_ws/src/proximity_alert/test/test_scan_trace_record.py
docker cp proximity_alert/setup.py MentorPi:/home/ubuntu/ros2_ws/src/proximity_alert/setup.py
docker exec -u root MentorPi bash -lc "chown -R ubuntu:ubuntu /home/ubuntu/ros2_ws/src/proximity_alert/proximity_alert/scan_trace_logger.py /home/ubuntu/ros2_ws/src/proximity_alert/proximity_alert/scan_trace_record.py /home/ubuntu/ros2_ws/src/proximity_alert/test/test_scan_trace_record.py /home/ubuntu/ros2_ws/src/proximity_alert/setup.py /home/ubuntu/ros2_ws/build/proximity_alert /home/ubuntu/ros2_ws/install/proximity_alert 2>/dev/null; echo chowned"
docker exec -u ubuntu MentorPi zsh -lc "source ~/.zshrc >/dev/null 2>&1 && cd ~/ros2_ws && rm -rf build/proximity_alert install/proximity_alert && colcon build --packages-select proximity_alert 2>&1 | tail -5"
```

- [ ] **Step 4: Run test to verify it passes**

```bash
docker exec -u ubuntu MentorPi zsh -lc "source ~/.zshrc >/dev/null 2>&1 && source ~/ros2_ws/install/setup.bash >/dev/null 2>&1 && cd ~/ros2_ws/src/proximity_alert && python3 -m pytest test/test_scan_trace_logger.py -v"
```

Expected: 2 passed

Then run the **full** in-container suite to confirm nothing else regressed:

```bash
docker exec -u ubuntu MentorPi zsh -lc "source ~/.zshrc >/dev/null 2>&1 && source ~/ros2_ws/install/setup.bash >/dev/null 2>&1 && cd ~/ros2_ws/src/proximity_alert && python3 -m pytest test/ -q"
```

Expected: all tests pass (48 existing + 5 Task-1 tests now copied in + 2 new Task-2 tests = 55; exact prior count may differ slightly by the time this runs — the key check is 0 failed).

- [ ] **Step 5: Commit**

```bash
git add proximity_alert/proximity_alert/scan_trace_logger.py proximity_alert/test/test_scan_trace_logger.py proximity_alert/setup.py
git commit -m "feat: add scan_trace_logger node for raw-scan miss diagnosis"
```

---

### Task 3: Documentation and VS Code visibility

**Files:**
- Modify: `README.md`

**Interfaces:**
- None (documentation only).

- [ ] **Step 1: Add the new node to the Repository structure diagram**

In `README.md`, find this block (added in an earlier session):

```
│       ├── decision_logger.py        # Logs each avoidance decision to its own CSV
│       └── floor_test_reconcile.py   # Fills floor_test_log.csv's Time/Stop-clearance from real trial data
├── trials/                     # CSVs, gitignored (not committed) -- see below
│   ├── floor_test_log.csv      # Hand-maintained PID/safety-distance tuning session report
│   ├── granite.csv             # -> symlink to /home/pi/docker/tmp/trials/granite.csv
│   └── decision_log.csv        # -> symlink to /home/pi/docker/tmp/trials/decision_log.csv
└── README.md
```

Replace it with:

```
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

- [ ] **Step 2: Add a Terminal D run command to Quick Start**

In `README.md`, find:

```
   ```bash
   # Terminal C — log the avoidance decisions (separate CSV)
   docker exec -it -u ubuntu MentorPi zsh -lc "source ~/.zshrc && source ~/ros2_ws/install/setup.bash && ros2 run proximity_alert decision_logger --ros-args -p csv_path:=/home/ubuntu/shared/trials/decision_log.csv"
   ```
```

Add immediately after it:

```

   ```bash
   # Terminal D — log every raw LiDAR scan (for diagnosing total detection misses)
   docker exec -it -u ubuntu MentorPi zsh -lc "source ~/.zshrc && source ~/ros2_ws/install/setup.bash && ros2 run proximity_alert scan_trace_logger --ros-args -p csv_path:=/home/ubuntu/shared/trials/scan_trace.jsonl"
   ```
```

- [ ] **Step 3: Document the new log in the "Two separate CSVs" section**

In `README.md`, find the section header:

```
### Two separate CSVs, both on your computer
```

Change the header to:

```
### Three logs, all on your computer
```

Immediately after the existing bullet list describing the trial motion log and decision log (do not remove those bullets), add:

```
- **Scan trace log** (`scan_trace_logger`) — one row per raw LiDAR scan tick, for diagnosing detection misses that never trigger an avoidance encounter at all (e.g. a thin chair leg outside the LiDAR's scan plane): `scan_number, stamp_sec, stamp_nanosec, angle_min, angle_increment, range_min, range_max, ranges, sectors`. JSON Lines (`.jsonl`), not CSV — `ranges` is a variable-length array that doesn't fit CSV's fixed-column shape. `stamp_sec`/`stamp_nanosec` come from the LaserScan message's own `header.stamp`, not wall-clock time, so this log stays on the same clock as `/odom` and every other ROS message for valid cross-message correlation. A process killed mid-write can only ever corrupt the last line of the file — skip a line that fails to parse rather than treating it as corruption. The node's `front_arc_deg`/`front_subsector_deg`/`side_window_deg`/`rear_window_deg` params default to match `path_tracker`'s current `AvoidanceConfig` values; if those get tuned in `avoidance.py`, update this node's defaults too or the logged `sectors` will stop reflecting what the controller actually saw. See `docs/superpowers/specs/2026-07-24-scan-trace-logger-design.md` for the full design and the post-hoc miss-diagnosis workflow.
```

- [ ] **Step 4: Create the symlink so the file is visible in VS Code**

```bash
ln -sf /home/pi/docker/tmp/trials/scan_trace.jsonl trials/scan_trace.jsonl
git check-ignore -v trials/scan_trace.jsonl
```

Expected: the `check-ignore` command prints a match against `.gitignore:11:trials/`, confirming the symlink stays untracked (same as the other two logs) rather than being accidentally committed.

- [ ] **Step 5: Commit the README changes**

```bash
git add README.md
git commit -m "docs: document scan_trace_logger, Terminal D run command, repo structure"
```

(The `trials/scan_trace.jsonl` symlink itself is gitignored and not part of this commit — it's a local convenience, recreated by Step 4's command on any machine that needs it.)

---

## Plan self-review notes

- **Spec coverage:** §4 architecture → Task 2. §5 record schema → Task 1 (encoding) + Task 2 (population from `LaserScan`). §6 file location/naming → Task 2 (default `csv_path`) + Task 3 (symlink, README). §7 post-hoc workflow → Task 3 (README reference to the spec; this is analysis guidance, not code, per the spec's own §7 framing — no task implements it as code because there is no code to implement). §8 testing plan → Tasks 1 and 2. §9 edge cases (bounded sessions, variable `ranges` length, JSON Lines over CSV) → reflected in Global Constraints and Task 1/2 implementations (no fixed-width assumptions anywhere).
- **Placeholder scan:** none found — every step has complete, runnable code or exact commands with expected output.
- **Type consistency:** `ScanTraceRecord` field names/types are identical across Task 1's definition, Task 1's tests, Task 2's construction call, and Task 2's tests (`scan_number: int`, `stamp_sec: int`, `stamp_nanosec: int`, `angle_min/angle_increment/range_min/range_max: float`, `ranges: list`, `sectors: dict`). `reduce_to_sectors(msg, front_arc_deg, front_subsector_deg, side_window_deg, rear_window_deg) -> dict` signature in Task 2 matches the existing, unmodified `scan_utils.py` definition exactly (verified against the current file during planning).
- **Post-brainstorm revision (this pass):** addressed five review points — (1) confirmed `/scan_raw` live via `ros2 topic list` and confirmed `scan_topic` is already an overridable ROS param, not hardcoded; (2) documented the `path_tracker`/`scan_trace_logger` sector-default sync requirement in Global Constraints, the node's own code comment, and the README bullet; (3) the NaN round-trip test already asserted fields individually rather than via dataclass `==`, but added an explicit comment stating why, since `NaN != NaN` makes this a real gotcha worth calling out rather than leaving implicit; (4) documented the partial-last-line-on-interrupted-write behavior in Global Constraints, the node's module docstring, and the README bullet; (5) added `test_round_trips_realistic_lidar_sized_scan` (452 beams, mixed finite/inf) to Task 1.
- **Gap caught during this revision:** Task 2's original file-sync step copied the new module files into the container but never copied Task 1's test file there, so the "full in-container suite" check would have silently excluded it despite the task claiming a specific pass count. Fixed by adding `test_scan_trace_record.py` to the `docker cp`/`chown` commands and correcting the expected total from 52 to 55 (48 existing + 5 Task-1 tests + 2 Task-2 tests).
