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
