"""Structured per-decision research record. Pure (no rclpy)."""
import json
import math
from dataclasses import dataclass, asdict, fields


@dataclass
class DecisionRecord:
    timestamp: str
    encounter_id: int
    state: str
    chosen_maneuver: str
    reason: str
    obstacle_span_deg: float
    front_distance_m: float
    front_left_m: float
    front_center_m: float
    front_right_m: float
    left_clearance_m: float
    right_clearance_m: float
    rear_clearance_m: float
    required_clearing_m: float
    cumulative_strafe_m: float
    consecutive_avoid_count: int
    recovery_triggered: bool
    outcome: str
    maneuver_duration_s: float

    def to_json(self):
        return json.dumps(asdict(self))

    @classmethod
    def from_json(cls, text):
        return cls(**json.loads(text))

    @staticmethod
    def csv_header():
        return [f.name for f in fields(DecisionRecord)]

    def csv_row(self):
        row = []
        for f in fields(DecisionRecord):
            v = getattr(self, f.name)
            if isinstance(v, float) and not math.isfinite(v):
                row.append("")
            else:
                row.append(v)
        return row
