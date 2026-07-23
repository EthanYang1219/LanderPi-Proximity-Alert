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
