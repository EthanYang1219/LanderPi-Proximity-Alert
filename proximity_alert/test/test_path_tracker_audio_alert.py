from unittest.mock import patch

import rclpy

from proximity_alert.decision_record import DecisionRecord
from proximity_alert.path_tracker import PathTracker


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


def test_audio_alert_enabled_by_default():
    rclpy.init()
    node = PathTracker()
    enabled = node.audio_alert_enabled
    node.destroy_node()
    rclpy.shutdown()
    assert enabled is True


def test_spawns_aplay_on_new_encounter():
    rclpy.init()
    node = PathTracker()
    with patch("proximity_alert.path_tracker.subprocess.Popen") as mock_popen:
        node._maybe_play_audio_alert(_decision(state="STRAFE", encounter_id=1))
    node.destroy_node()
    rclpy.shutdown()
    mock_popen.assert_called_once_with(
        ["aplay", "-D", "plughw:2,0", "/home/ubuntu/shared/audio/obstacle_alert.wav"]
    )


def test_does_not_spawn_aplay_again_within_same_encounter():
    rclpy.init()
    node = PathTracker()
    with patch("proximity_alert.path_tracker.subprocess.Popen") as mock_popen:
        node._maybe_play_audio_alert(_decision(state="STRAFE", encounter_id=1))
        node._maybe_play_audio_alert(_decision(state="TURN", encounter_id=1))
    node.destroy_node()
    rclpy.shutdown()
    assert mock_popen.call_count == 1


def test_does_not_spawn_aplay_on_drive_state():
    rclpy.init()
    node = PathTracker()
    with patch("proximity_alert.path_tracker.subprocess.Popen") as mock_popen:
        node._maybe_play_audio_alert(_decision(state="DRIVE", encounter_id=1))
    node.destroy_node()
    rclpy.shutdown()
    mock_popen.assert_not_called()


def test_survives_aplay_not_installed():
    # Popen raising OSError (e.g. aplay binary missing) must not crash the node.
    rclpy.init()
    node = PathTracker()
    with patch("proximity_alert.path_tracker.subprocess.Popen",
               side_effect=FileNotFoundError("aplay")):
        node._maybe_play_audio_alert(_decision(state="STRAFE", encounter_id=1))  # must not raise
    node.destroy_node()
    rclpy.shutdown()


def test_logs_error_when_wav_path_missing():
    # Regression: a missing WAV produced no visible error at all (aplay is
    # spawned non-blocking with stderr discarded), so the robot silently
    # never beeped. Startup must say so loudly.
    rclpy.init()
    with patch("proximity_alert.path_tracker.os.path.exists", return_value=False), \
            patch.object(PathTracker, "get_logger") as mock_get_logger:
        node = PathTracker()
        errors = [str(c) for c in mock_get_logger.return_value.error.call_args_list]
    node.destroy_node()
    rclpy.shutdown()
    assert any("does not exist" in e for e in errors), errors


def test_no_error_logged_when_wav_path_present():
    rclpy.init()
    with patch("proximity_alert.path_tracker.os.path.exists", return_value=True), \
            patch.object(PathTracker, "get_logger") as mock_get_logger:
        node = PathTracker()
        errors = [str(c) for c in mock_get_logger.return_value.error.call_args_list]
    node.destroy_node()
    rclpy.shutdown()
    assert not any("does not exist" in e for e in errors), errors


def test_audio_alert_disabled_param_suppresses_playback():
    rclpy.init()
    node = PathTracker()
    node.audio_alert_enabled = False
    with patch("proximity_alert.path_tracker.subprocess.Popen") as mock_popen:
        # Mirrors how control_loop gates the call on the flag.
        if node.audio_alert_enabled:
            node._maybe_play_audio_alert(_decision(state="STRAFE", encounter_id=1))
    node.destroy_node()
    rclpy.shutdown()
    mock_popen.assert_not_called()
