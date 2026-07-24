import csv
import os
import tempfile

import rclpy
from std_msgs.msg import String

from proximity_alert.decision_logger import DecisionLogger
from proximity_alert.decision_record import DecisionRecord


def _rec():
    return DecisionRecord(
        timestamp="2026-07-23 10:00:00", encounter_id=1, state="ASSESS",
        chosen_maneuver="STRAFE", reason="strafe: narrow+side_clear",
        obstacle_span_deg=12.0, front_distance_m=0.29, front_left_m=0.5,
        front_center_m=0.29, front_right_m=0.6, left_clearance_m=1.2,
        right_clearance_m=0.4, rear_clearance_m=2.0, required_clearing_m=0.22,
        cumulative_strafe_m=0.0, consecutive_avoid_count=1,
        recovery_triggered=False, outcome="cleared", maneuver_duration_s=1.4)


def test_survives_malformed_decision_json():
    # A bad or empty message on the topic (e.g. from a stray publish) must
    # not kill the subscription or crash the node -- mirrors
    # obstacle_audio's test_survives_malformed_decision_json.
    rclpy.init()
    fd, path = tempfile.mkstemp(suffix=".csv")
    os.close(fd)
    os.remove(path)
    node = DecisionLogger()
    node.csv_path = path
    node._ensure_header()
    node.on_decision(String(data=""))          # must not raise
    node.on_decision(String(data="{not valid json"))  # must not raise
    with open(path, newline="") as f:
        rows = list(csv.reader(f))
    node.destroy_node()
    rclpy.shutdown()
    os.remove(path)
    if os.path.exists("decision_log.csv"):
        os.remove("decision_log.csv")
    assert rows == [DecisionRecord.csv_header()]  # header only, no bad rows written


def test_writes_header_and_row_to_separate_csv():
    rclpy.init()
    fd, path = tempfile.mkstemp(suffix=".csv")
    os.close(fd)
    os.remove(path)
    node = DecisionLogger()
    node.csv_path = path
    node._ensure_header()
    node.on_decision(String(data=_rec().to_json()))
    with open(path, newline="") as f:
        rows = list(csv.reader(f))
    node.destroy_node()
    rclpy.shutdown()
    os.remove(path)
    if os.path.exists("decision_log.csv"):
        os.remove("decision_log.csv")
    assert rows[0] == DecisionRecord.csv_header()
    assert dict(zip(rows[0], rows[1]))["chosen_maneuver"] == "STRAFE"
