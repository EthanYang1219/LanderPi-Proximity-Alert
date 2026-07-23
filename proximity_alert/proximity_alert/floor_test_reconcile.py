#!/usr/bin/env python3
"""Reconciles floor_test_log.csv (the hand-maintained Google-Sheet-schema
report of PID/safety-distance tuning sessions) against the real trial data
recorded by trial_logger.py (e.g. granite.csv).

floor_test_log.csv is never written by any ROS node -- after each floor-test
session you currently have to remember to hand-transcribe Time and Stop
clearance from the trial CSV, plus the safety_distance/speed/Kp/Ki/Kd you
ran with, into the spreadsheet. That transcription step is easy to forget
(it was, for the two most recent sessions), silently leaving the report
with blank rows Haotian can't use.

This script does not try to fully automate the row (safety_distance and
Kp/Ki/Kd are node CLI parameters, never persisted anywhere), but it removes
the parts that ARE derivable from the trial CSV -- Time and Stop clearance --
and tells you exactly which sessions still need attention, so nothing gets
silently skipped again.

Pure ROS-free module; run directly on the host, no rclpy needed:
    python3 -m proximity_alert.floor_test_reconcile \\
        --floor-log trials/floor_test_log.csv \\
        --trial-csv /path/to/granite.csv [/path/to/other_surface.csv ...]
"""
from __future__ import annotations

import argparse
import csv
import re
from dataclasses import dataclass
from datetime import date, datetime
from pathlib import Path

SESSION_DATE_RE = re.compile(r"(\d{2})/(\d{2})/(\d{4})")

# Columns in floor_test_log.csv that trial data can actually fill in.
TIME_COLUMN = "Time (s)"
STOP_CLEARANCE_COLUMN = "Stop clearance (m)"
SESSION_COLUMN = "Date and Session #"


@dataclass(frozen=True)
class TrialRecord:
    """One row of a trial_logger.py output CSV (e.g. granite.csv)."""

    timestamp: datetime
    transit_time_s: float
    lidar_stop_range_m: float | None
    notes: str

    @property
    def date(self) -> date:
        return self.timestamp.date()


@dataclass(frozen=True)
class ReconciliationTarget:
    """A floor_test_log.csv session that trial data can help complete."""

    row_index: int  # index into the data rows (0-based, header excluded)
    session_label: str
    trials: tuple[TrialRecord, ...]
    missing_columns: tuple[str, ...]


def load_trial_records(csv_path: Path) -> list[TrialRecord]:
    """Parses a trial_logger.py CSV into TrialRecords, oldest first."""
    records: list[TrialRecord] = []
    with csv_path.open(newline="") as f:
        for row in csv.DictReader(f):
            stop_range_raw = row.get("lidar_stop_range_m", "")
            records.append(
                TrialRecord(
                    timestamp=datetime.strptime(
                        row["timestamp"], "%Y-%m-%d %H:%M:%S"
                    ),
                    transit_time_s=float(row["transit_time_s"]),
                    lidar_stop_range_m=(
                        float(stop_range_raw) if stop_range_raw else None
                    ),
                    notes=row.get("notes", ""),
                )
            )
    return sorted(records, key=lambda r: r.timestamp)


def parse_session_date(session_label: str) -> date | None:
    """Extracts the MM/DD/YYYY date from a "Date and Session #" cell.

    Returns None if the cell doesn't contain a recognizable date (e.g. an
    empty trailing row).
    """
    m = SESSION_DATE_RE.search(session_label)
    if m is None:
        return None
    month, day, year = (int(g) for g in m.groups())
    return date(year, month, day)


def group_trials_by_date(
    trials: list[TrialRecord],
) -> dict[date, list[TrialRecord]]:
    grouped: dict[date, list[TrialRecord]] = {}
    for t in trials:
        grouped.setdefault(t.date, []).append(t)
    return grouped


def _dates_with_multiple_rows(header: list[str], rows: list[list[str]]) -> set[date]:
    """Dates claimed by more than one floor_test_log row (e.g. two sessions
    run the same day) -- a per-date trial match can't tell those rows apart,
    so they must be excluded from auto-fill rather than guessed at."""
    session_col = header.index(SESSION_COLUMN)
    seen: dict[date, int] = {}
    for row in rows:
        if session_col < len(row):
            parsed = parse_session_date(row[session_col])
            if parsed is not None:
                seen[parsed] = seen.get(parsed, 0) + 1
    return {d for d, count in seen.items() if count > 1}


def find_reconciliation_targets(
    header: list[str],
    rows: list[list[str]],
    trials: list[TrialRecord],
) -> list[ReconciliationTarget]:
    """Finds floor_test_log rows whose Time/Stop-clearance are blank but a
    same-day trial exists to fill them from.

    Skips any date shared by more than one floor_test_log row: with only a
    date to match on, there is no way to tell which physical trials belong
    to which of those rows, and auto-filling would silently write the same
    aggregate into all of them.
    """
    by_date = group_trials_by_date(trials)
    ambiguous_dates = _dates_with_multiple_rows(header, rows)
    col = {name: i for i, name in enumerate(header)}
    targets: list[ReconciliationTarget] = []

    for i, row in enumerate(rows):
        session_label = row[col[SESSION_COLUMN]] if col.get(SESSION_COLUMN) is not None else ""
        session_date = parse_session_date(session_label)
        if session_date is None or session_date not in by_date or session_date in ambiguous_dates:
            continue

        missing = tuple(
            name
            for name in (TIME_COLUMN, STOP_CLEARANCE_COLUMN)
            if col.get(name) is not None
            and (col[name] >= len(row) or not row[col[name]].strip())
        )
        if missing:
            targets.append(
                ReconciliationTarget(
                    row_index=i,
                    session_label=session_label,
                    trials=tuple(by_date[session_date]),
                    missing_columns=missing,
                )
            )
    return targets


def find_unlogged_trial_dates(
    header: list[str], rows: list[list[str]], trials: list[TrialRecord]
) -> list[date]:
    """Dates that have real trial data but no floor_test_log row at all."""
    session_col = header.index(SESSION_COLUMN)
    logged_dates: set[date] = set()
    for row in rows:
        if session_col < len(row):
            parsed = parse_session_date(row[session_col])
            if parsed is not None:
                logged_dates.add(parsed)

    trial_dates = {t.date for t in trials}
    return sorted(trial_dates - logged_dates)


def fill_row(
    header: list[str], row: list[str], trials: tuple[TrialRecord, ...]
) -> list[str]:
    """Returns a copy of ``row`` with Time/Stop-clearance derived from
    ``trials`` (averaged if the session covers more than one trial run).
    Never overwrites a column that already has a value.
    """
    filled = list(row) + [""] * (len(header) - len(row))
    col = {name: i for i, name in enumerate(header)}

    if TIME_COLUMN in col and not filled[col[TIME_COLUMN]].strip():
        avg_time = sum(t.transit_time_s for t in trials) / len(trials)
        filled[col[TIME_COLUMN]] = f"{avg_time:.3f}"

    stop_ranges = [t.lidar_stop_range_m for t in trials if t.lidar_stop_range_m is not None]
    if STOP_CLEARANCE_COLUMN in col and not filled[col[STOP_CLEARANCE_COLUMN]].strip() and stop_ranges:
        avg_range = sum(stop_ranges) / len(stop_ranges)
        filled[col[STOP_CLEARANCE_COLUMN]] = f"{avg_range:.4f}"

    return filled


def _read_floor_log(path: Path) -> tuple[list[str], list[list[str]]]:
    with path.open(newline="") as f:
        rows = list(csv.reader(f))
    if not rows:
        raise ValueError(f"{path} is empty")
    return rows[0], rows[1:]


def _write_floor_log(path: Path, header: list[str], rows: list[list[str]]) -> None:
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(rows)


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--floor-log", type=Path, required=True)
    parser.add_argument("--trial-csv", type=Path, nargs="+", required=True)
    args = parser.parse_args(argv)

    trials: list[TrialRecord] = []
    for p in args.trial_csv:
        trials.extend(load_trial_records(p))

    header, rows = _read_floor_log(args.floor_log)

    targets = find_reconciliation_targets(header, rows, trials)
    for target in targets:
        print(
            f"\nSession '{target.session_label}': missing "
            f"{', '.join(target.missing_columns)}, filling from "
            f"{len(target.trials)} trial(s) on {target.trials[0].date}."
        )
        rows[target.row_index] = fill_row(header, rows[target.row_index], target.trials)

    if targets:
        _write_floor_log(args.floor_log, header, rows)
        print(f"\nUpdated {len(targets)} row(s) in {args.floor_log}.")
    else:
        print("\nNo incomplete rows had matching trial data.")

    ambiguous_dates = sorted(_dates_with_multiple_rows(header, rows))
    if ambiguous_dates:
        print(
            f"\nWARNING: {len(ambiguous_dates)} date(s) have more than one "
            "floor_test_log session row -- can't tell which trials belong "
            "to which row from the date alone, so these were left "
            "untouched. Fill Time/Stop clearance by hand for: "
            + ", ".join(d.strftime("%m/%d/%Y") for d in ambiguous_dates)
        )

    missing_dates = find_unlogged_trial_dates(header, rows, trials)
    if missing_dates:
        print(
            f"\nWARNING: trial data exists for {len(missing_dates)} date(s) "
            "with NO row in floor_test_log.csv at all -- these sessions "
            "were never entered and this script won't fabricate the "
            "Safety Distance/Speed/Kp/Ki/Kd for you. Add a row by hand for: "
            + ", ".join(d.strftime("%m/%d/%Y") for d in missing_dates)
        )


if __name__ == "__main__":
    main()
