import csv
import os
import tempfile

import rclpy

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
    )

    with open(path, newline="") as f:
        rows = list(csv.reader(f))

    node.destroy_node()
    rclpy.shutdown()
    os.remove(path)
    if os.path.exists("trial_log.csv"):
        os.remove("trial_log.csv")  # side effect of TrialLogger() default param

    header, row = rows[0], rows[1]
    # New column exists, sits after avoidance_events, and battery_level is last.
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
    ]
    by_col = dict(zip(header, row))
    assert by_col["lidar_stop_range_m"] == "0.2800"
    assert by_col["avoidance_events"] == "0"
    assert by_col["notes"] == "clean run"
    assert by_col["battery_level"] == "High"


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
        "battery_level",
    ]
    assert rows[0] == canonical
    assert len(rows[1]) == len(canonical)          # old row padded, not left short
    assert rows[1][:8] == old_row                  # original data untouched
    assert rows[1][8:] == ["", "", "", ""]          # padded blank, not fabricated
    assert len(rows[2]) == len(canonical)           # new row same width
    by_col = dict(zip(canonical, rows[2]))
    assert by_col["notes"] == "PID tuning"
    assert by_col["battery_level"] == "Medium"


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
