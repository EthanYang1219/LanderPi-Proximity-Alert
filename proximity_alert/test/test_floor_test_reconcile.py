import csv
from datetime import date, datetime
from pathlib import Path

import pytest

from proximity_alert.floor_test_reconcile import (
    TrialRecord,
    fill_row,
    find_reconciliation_targets,
    find_unlogged_trial_dates,
    group_trials_by_date,
    load_trial_records,
    parse_session_date,
)

FLOOR_HEADER = [
    "Date and Session #", "Safety Distance (m)", "Time (s)",
    "Stop clearance (m)", "Notes",
]


def _trial(ts, transit=5.0, stop_range=0.29, notes=""):
    return TrialRecord(
        timestamp=datetime.strptime(ts, "%Y-%m-%d %H:%M:%S"),
        transit_time_s=transit, lidar_stop_range_m=stop_range, notes=notes,
    )


@pytest.mark.parametrize("label,expected", [
    ("07/21/2026 - Session 1", date(2026, 7, 21)),
    ("07/22/2026 - Session 2", date(2026, 7, 22)),
    ("", None),
    ("no date here", None),
])
def test_parse_session_date(label, expected):
    assert parse_session_date(label) == expected


def test_load_trial_records_parses_and_sorts(tmp_path: Path):
    p = tmp_path / "granite.csv"
    p.write_text(
        "timestamp,surface,trial_num,transit_time_s,odom_distance_m,"
        "ground_truth_distance_m,slippage_error_m,slippage_pct,"
        "avoidance_events,lidar_stop_range_m,notes\n"
        "2026-07-21 18:44:48,granite,2,10.698,1.0750,0.2300,0.8450,367.38,,,\n"
        "2026-07-21 18:41:43,granite,1,10.732,1.0961,0.2250,0.8711,387.14,0,0.28,ok\n"
    )
    records = load_trial_records(p)
    assert [r.transit_time_s for r in records] == [10.732, 10.698]  # sorted by time
    assert records[0].lidar_stop_range_m == 0.28
    assert records[1].lidar_stop_range_m is None  # blank field -> None, not 0.0


def test_group_trials_by_date():
    trials = [_trial("2026-07-21 18:41:43"), _trial("2026-07-21 18:44:48"),
              _trial("2026-07-22 16:54:30")]
    grouped = group_trials_by_date(trials)
    assert len(grouped[date(2026, 7, 21)]) == 2
    assert len(grouped[date(2026, 7, 22)]) == 1


def test_find_reconciliation_targets_flags_blank_time_and_stop_clearance():
    rows = [["07/21/2026 - Session 1", "0.30", "", "", "some notes"]]
    trials = [_trial("2026-07-21 18:41:43")]
    targets = find_reconciliation_targets(FLOOR_HEADER, rows, trials)
    assert len(targets) == 1
    assert targets[0].missing_columns == ("Time (s)", "Stop clearance (m)")
    assert targets[0].trials == tuple(trials)


def test_find_reconciliation_targets_skips_already_complete_rows():
    rows = [["07/21/2026 - Session 1", "0.30", "5.0", "0.29", "notes"]]
    trials = [_trial("2026-07-21 18:41:43")]
    assert find_reconciliation_targets(FLOOR_HEADER, rows, trials) == []


def test_find_reconciliation_targets_skips_rows_with_no_matching_trial_date():
    rows = [["07/21/2026 - Session 1", "0.30", "", "", ""]]
    trials = [_trial("2026-07-22 16:54:30")]  # different date
    assert find_reconciliation_targets(FLOOR_HEADER, rows, trials) == []


def test_find_reconciliation_targets_refuses_ambiguous_same_day_sessions():
    # Two floor-log rows share 07/21 -- there is no way to tell which real
    # trials belong to "Session 1" vs "Session 2" from the date alone, so
    # auto-filling either from a per-date aggregate would silently write
    # the SAME wrong value into both rows. Must refuse rather than guess.
    rows = [
        ["07/21/2026 - Session 1", "0.30", "", "", ""],
        ["07/21/2026 - Session 2", "0.30", "", "", ""],
    ]
    trials = [_trial("2026-07-21 18:41:43"), _trial("2026-07-21 18:44:48")]
    assert find_reconciliation_targets(FLOOR_HEADER, rows, trials) == []


def test_fill_row_averages_multiple_same_day_trials_and_preserves_other_columns():
    row = ["07/21/2026 - Session 1", "0.30", "", "", "prewritten note"]
    trials = (
        _trial("2026-07-21 18:41:43", transit=10.0, stop_range=0.28),
        _trial("2026-07-21 18:44:48", transit=12.0, stop_range=0.30),
    )
    filled = fill_row(FLOOR_HEADER, row, trials)
    assert filled[FLOOR_HEADER.index("Time (s)")] == "11.000"
    assert filled[FLOOR_HEADER.index("Stop clearance (m)")] == "0.2900"
    assert filled[0] == "07/21/2026 - Session 1"       # untouched
    assert filled[4] == "prewritten note"              # never overwritten


def test_fill_row_never_overwrites_existing_value():
    row = ["07/21/2026 - Session 1", "0.30", "99.0", "", ""]
    trials = (_trial("2026-07-21 18:41:43", transit=10.0),)
    filled = fill_row(FLOOR_HEADER, row, trials)
    assert filled[FLOOR_HEADER.index("Time (s)")] == "99.0"  # left alone


def test_find_unlogged_trial_dates_reports_dates_with_no_row_at_all():
    rows = [["07/21/2026 - Session 1", "0.30", "5.0", "0.29", ""]]
    trials = [_trial("2026-07-21 18:41:43"), _trial("2026-07-22 16:54:30")]
    assert find_unlogged_trial_dates(FLOOR_HEADER, rows, trials) == [date(2026, 7, 22)]
