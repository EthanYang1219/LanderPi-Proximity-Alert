# Processed / hand-maintained session logs

Unlike the `raw/` logs, these files are never written by a ROS node — they're
manually transcribed, per-session summary spreadsheets (Google-Sheet-schema)
of PID/safety-distance tuning runs. `Safety Distance`/`Speed`/`Kp`/`Ki`/`Kd`
are ROS CLI parameters that aren't persisted anywhere else, so they only exist
here. `Time`/`Stop clearance` can be partially auto-filled from the matching
`raw/trial_logs/` CSV via `floor_test_reconcile.py` (see main
[README.md](../../README.md#reconciling-floor_test_logcsv)) — run that after a
session instead of hand-copying those two columns.

## Files

- **`floor_test_log.csv`** — the original log, predates the multi-surface
  schema below (no `Surface Type`, `Turning Speed`, `Battery level`, `Trail`,
  or `Ambient Light` columns). Sessions 1-2 shown (placeholder/early data).
- **`wood_floor_test_log.csv`** — a newer, separate per-surface log (wood
  only) using an expanded column set. This is a distinct file, not a
  replacement for `floor_test_log.csv` — the two currently use different
  schemas and are not meant to be merged. Sessions 1-14 plus one trailing
  wood-only summary row. A duplicated/stacked header found mid-paste when this
  file was created was dropped, keeping one canonical header.

## Columns

| Column | Meaning | In which file(s) |
|---|---|---|
| `Date and Session #` | Session date (`MM/DD/YYYY`) and sequence number for that day | both |
| `Safety Distance (m)` | `path_tracker`'s `safety_distance` parameter for the session | both |
| `± Safety Distance Uncertainity (m)` | Measurement uncertainty on the above (spelling as entered in the original sheet) | both |
| `Forward Speed (m/s)` | `path_tracker`'s `forward_speed` parameter | both |
| `Turning Speed (m/s)` | `path_tracker`'s `turn_speed` parameter | wood only |
| `± Forward/Speed Uncertainity (m/s)` | Measurement uncertainty on forward speed | both |
| `Time (s)` | Transit time for the session — auto-fillable from `raw/trial_logs/` via `floor_test_reconcile.py` | both |
| `Stop clearance (m)` | Real stop clearance — auto-fillable from `raw/trial_logs/` via `floor_test_reconcile.py` | both |
| `± Stop Clearance Uncertainity (m)` | Measurement uncertainty on stop clearance | both |
| `Measurement Method` | How ground-truth distance/clearance was measured (e.g. "Measurement Tape (± 0.1 cm)") | both |
| `Surface Type` | Test surface (e.g. Wood) | wood only |
| `Obstacle Type` | Obstacle used (e.g. Box) | both |
| `Notes` | Freeform session notes | both |
| `Bugs/Issues` | Freeform bug/issue notes for the session | both |
| `Kp` / `Ki` / `Kd` | PID gains used for the session | both |
| `Battery level` | Rough High/Medium/Low estimate for the session | wood only |
| `Trail` | Trial/run identifier within the session (as entered in the original sheet) | wood only |
| `Ambient Light` | Ambient lighting condition during the session | wood only |

**Note:** as of 2026-07-29, most `Battery level`/`Trail`/`Ambient Light` values
in `wood_floor_test_log.csv` are placeholders from the original data paste,
per the explicit instruction to ignore placeholder values when this file was
created.
