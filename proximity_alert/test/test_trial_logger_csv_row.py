import csv
import os
import tempfile
import time

import rclpy
from std_msgs.msg import String

from proximity_alert.decision_record import DecisionRecord
from proximity_alert.trial_logger import TrialLogger


def _fresh_logger(csv_path):
    node = TrialLogger()
    node.csv_path = csv_path
    node.trial_num = 0
    node._ensure_csv_header()
    return node


def test_append_row_includes_lidar_stop_range_and_keeps_existing_columns():
    rclpy.init()
    fd, path = tempfile.mkstemp(suffix=".csv")
    os.close(fd)
    os.remove(path)  # let _ensure_csv_header create it fresh

    node = _fresh_logger(path)
    node._append_row(
        surface="granite",
        transit_time_s=4.2,
        odom_distance_m=2.05,
        ground_truth_m=2.0,
        avoidance_events=0,
        lidar_stop_range_m=0.28,
        notes="clean run",
        battery_level_str="High",
        lateral_offset_m=0.05,
    )

    with open(path, newline="") as f:
        rows = list(csv.reader(f))

    node.destroy_node()
    rclpy.shutdown()
    os.remove(path)
    if os.path.exists("trial_log.csv"):
        os.remove("trial_log.csv")  # side effect of TrialLogger() default param

    header, row = rows[0], rows[1]
    # New column exists, sits after avoidance_events, and
    # ground_truth_lateral_offset_m is last (appended, not inserted).
    assert header == [
        "timestamp",
        "surface",
        "trial_num",
        "transit_time_s",
        "odom_distance_m",
        "ground_truth_distance_m",
        "slippage_error_m",
        "slippage_pct",
        "avoidance_events",
        "lidar_stop_range_m",
        "notes",
        "battery_level",
        "ground_truth_lateral_offset_m",
        "obstacle_count",
        "layout_id",
        "outcome",
        "cause",
    ]
    by_col = dict(zip(header, row))
    assert by_col["lidar_stop_range_m"] == "0.2800"
    assert by_col["avoidance_events"] == "0"
    assert by_col["notes"] == "clean run"
    assert by_col["battery_level"] == "High"
    assert by_col["ground_truth_lateral_offset_m"] == "0.0500"
    # A plain surface trial (no track_obstacle_outcome) leaves these blank,
    # not "0" or fabricated -- genuinely untracked, not "0 obstacles".
    assert by_col["obstacle_count"] == ""
    assert by_col["layout_id"] == ""
    assert by_col["outcome"] == ""
    assert by_col["cause"] == ""


def test_append_row_records_obstacle_outcome_fields_when_provided():
    rclpy.init()
    fd, path = tempfile.mkstemp(suffix=".csv")
    os.close(fd)
    os.remove(path)

    node = _fresh_logger(path)
    node._append_row(
        surface="avoidance",
        transit_time_s=6.1,
        odom_distance_m=2.0,
        ground_truth_m=2.0,
        avoidance_events=1,
        lidar_stop_range_m=0.22,
        notes="",
        battery_level_str="High",
        lateral_offset_m=None,
        obstacle_count=2,
        layout_id="layout_A",
        outcome="success",
        cause="strafed clear on first attempt",
    )

    with open(path, newline="") as f:
        rows = list(csv.reader(f))

    node.destroy_node()
    rclpy.shutdown()
    os.remove(path)
    if os.path.exists("trial_log.csv"):
        os.remove("trial_log.csv")

    by_col = dict(zip(rows[0], rows[1]))
    assert by_col["obstacle_count"] == "2"
    assert by_col["layout_id"] == "layout_A"
    assert by_col["outcome"] == "success"
    assert by_col["cause"] == "strafed clear on first attempt"


def test_ensure_header_migrates_stale_short_header_and_pads_old_rows():
    # Regression: a CSV created before avoidance_events/lidar_stop_range_m/
    # notes existed has an 8-column header and 8-column data rows. If a node
    # pointed at that file later appends an 11-column row (current schema),
    # the stored header never gets migrated -- any strict CSV/pandas reader
    # then sees 8 header names against up to 11 data columns per row, and
    # silently drops or misaligns the newest fields. _ensure_csv_header must
    # migrate the header and right-pad old rows to the canonical width.
    rclpy.init()
    fd, path = tempfile.mkstemp(suffix=".csv")
    os.close(fd)
    old_header = [
        "timestamp", "surface", "trial_num", "transit_time_s",
        "odom_distance_m", "ground_truth_distance_m", "slippage_error_m",
        "slippage_pct",
    ]
    old_row = ["2026-07-21 18:41:43", "granite", "1", "10.732", "1.0961",
               "0.2250", "0.8711", "387.14"]
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(old_header)
        w.writerow(old_row)

    node = TrialLogger()
    node.csv_path = path
    node.trial_num = 1
    node._ensure_csv_header()  # should migrate in place, not treat as fresh
    node._append_row(
        surface="granite",
        transit_time_s=5.497,
        odom_distance_m=1.0916,
        ground_truth_m=23.2,
        avoidance_events=0,
        lidar_stop_range_m=0.256,
        notes="PID tuning",
        battery_level_str="Medium",
    )

    with open(path, newline="") as f:
        rows = list(csv.reader(f))

    node.destroy_node()
    rclpy.shutdown()
    os.remove(path)
    if os.path.exists("trial_log.csv"):
        os.remove("trial_log.csv")

    canonical = [
        "timestamp", "surface", "trial_num", "transit_time_s",
        "odom_distance_m", "ground_truth_distance_m", "slippage_error_m",
        "slippage_pct", "avoidance_events", "lidar_stop_range_m", "notes",
        "battery_level", "ground_truth_lateral_offset_m",
        "obstacle_count", "layout_id", "outcome", "cause",
    ]
    assert rows[0] == canonical
    assert len(rows[1]) == len(canonical)          # old row padded, not left short
    assert rows[1][:8] == old_row                  # original data untouched
    assert rows[1][8:] == [""] * (len(canonical) - 8)  # padded blank, not fabricated
    assert len(rows[2]) == len(canonical)           # new row same width
    by_col = dict(zip(canonical, rows[2]))
    assert by_col["notes"] == "PID tuning"
    assert by_col["battery_level"] == "Medium"


def _decision_json(encounter_id, maneuver, state="STRAFE"):
    return DecisionRecord(
        timestamp="2026-08-11 21:37:29", encounter_id=encounter_id, state=state,
        chosen_maneuver=maneuver, reason="strafe: narrow+side_clear",
        obstacle_span_deg=91.6, front_distance_m=0.195, front_left_m=0.232,
        front_center_m=0.195, front_right_m=0.246, left_clearance_m=2.35,
        right_clearance_m=2.17, rear_clearance_m=0.0, required_clearing_m=0.34,
        cumulative_strafe_m=0.0, consecutive_avoid_count=1,
        recovery_triggered=False, outcome="committed", maneuver_duration_s=0.0,
    ).to_json()


def test_strafe_only_avoidance_is_counted_as_an_event():
    # Regression: avoidance_events used to be inferred solely from a negative
    # linear.x on /cmd_vel. AvoidanceController._strafe commands linear.x == 0.0
    # (avoidance.py:412), so a strafe-only encounter -- the most common one --
    # produced a reverse edge NEVER, and every such trial logged
    # avoidance_events=0 while decision_log.csv showed a real STRAFE. The count
    # must come from the decision stream, which is unambiguous.
    rclpy.init()
    node = TrialLogger()
    node.state = "moving"
    node.avoidance_events = 0
    node._reset_avoidance_tracking()

    # Two ticks of one encounter, then a second encounter -> 2 events, not 3.
    node.decision_callback(String(data=_decision_json(1, "STRAFE")))
    node.decision_callback(String(data=_decision_json(1, "STRAFE")))
    node.decision_callback(String(data=_decision_json(2, "TURN", state="TURN")))
    # A plain "cleared" DRIVE record is not itself a maneuver.
    node.decision_callback(String(data=_decision_json(2, "NONE", state="DRIVE")))

    events = node.avoidance_events
    node.destroy_node()
    rclpy.shutdown()
    if os.path.exists("trial_log.csv"):
        os.remove("trial_log.csv")

    assert events == 2


def test_append_row_uses_trial_end_timestamp_not_write_time():
    # Regression: the timestamp column was generated by time.strftime() at
    # row-write time, which is AFTER the operator answers every interactive
    # prompt. With track_obstacle_outcome that is ~9 prompts, observed at 3-4
    # minutes -- so CSV rows could not be correlated against decision_log.csv
    # or scan_trace.jsonl by time at all.
    rclpy.init()
    fd, path = tempfile.mkstemp(suffix=".csv")
    os.close(fd)
    os.remove(path)

    node = _fresh_logger(path)
    node._append_row(
        surface="avoidance",
        transit_time_s=13.0,
        odom_distance_m=2.0128,
        ground_truth_m=1.85,
        avoidance_events=1,
        lidar_stop_range_m=2.723,
        notes="",
        battery_level_str="Medium",
        trial_end_epoch=1786484251.0,  # 2026-08-11 21:37:31 UTC
    )

    with open(path, newline="") as f:
        rows = list(csv.reader(f))

    node.destroy_node()
    rclpy.shutdown()
    os.remove(path)
    if os.path.exists("trial_log.csv"):
        os.remove("trial_log.csv")

    expected = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(1786484251.0))
    assert dict(zip(rows[0], rows[1]))["timestamp"] == expected


def test_append_row_logs_blank_range_for_infinite_reading():
    rclpy.init()
    fd, path = tempfile.mkstemp(suffix=".csv")
    os.close(fd)
    os.remove(path)

    node = _fresh_logger(path)
    node._append_row(
        surface="wood",
        transit_time_s=3.0,
        odom_distance_m=1.5,
        ground_truth_m=1.5,
        avoidance_events=2,
        lidar_stop_range_m=float("inf"),
        notes="",
        battery_level_str="",
    )

    with open(path, newline="") as f:
        rows = list(csv.reader(f))

    node.destroy_node()
    rclpy.shutdown()
    os.remove(path)
    if os.path.exists("trial_log.csv"):
        os.remove("trial_log.csv")

    by_col = dict(zip(rows[0], rows[1]))
    assert by_col["lidar_stop_range_m"] == ""
