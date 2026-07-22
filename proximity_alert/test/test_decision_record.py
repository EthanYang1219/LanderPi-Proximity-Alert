from proximity_alert.decision_record import DecisionRecord


def _rec(**kw):
    base = dict(timestamp="2026-07-23 10:00:00", encounter_id=1, state="ASSESS",
                chosen_maneuver="STRAFE", reason="strafe: narrow+side_clear",
                obstacle_span_deg=12.0, front_distance_m=0.29, front_left_m=0.5,
                front_center_m=0.29, front_right_m=0.6, left_clearance_m=1.2,
                right_clearance_m=0.4, rear_clearance_m=2.0, required_clearing_m=0.22,
                cumulative_strafe_m=0.0, consecutive_avoid_count=1,
                recovery_triggered=False, outcome="cleared", maneuver_duration_s=1.4)
    base.update(kw)
    return DecisionRecord(**base)


def test_json_round_trip():
    r = _rec()
    assert DecisionRecord.from_json(r.to_json()) == r


def test_csv_header_matches_row_length():
    assert len(DecisionRecord.csv_header()) == len(_rec().csv_row())


def test_infinite_range_serializes_blank_in_csv():
    row = dict(zip(DecisionRecord.csv_header(),
                   _rec(rear_clearance_m=float("inf")).csv_row()))
    assert row["rear_clearance_m"] == ""
