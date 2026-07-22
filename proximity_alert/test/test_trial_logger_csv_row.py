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
    )

    with open(path, newline="") as f:
        rows = list(csv.reader(f))

    node.destroy_node()
    rclpy.shutdown()
    os.remove(path)
    if os.path.exists("trial_log.csv"):
        os.remove("trial_log.csv")  # side effect of TrialLogger() default param

    header, row = rows[0], rows[1]
    # New column exists, sits after avoidance_events, and notes stays last.
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
    ]
    by_col = dict(zip(header, row))
    assert by_col["lidar_stop_range_m"] == "0.2800"
    assert by_col["avoidance_events"] == "0"
    assert by_col["notes"] == "clean run"


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
