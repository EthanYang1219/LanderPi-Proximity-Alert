"""Pure, ROS-free record for one raw LiDAR scan tick, JSON-Lines serialized.

See docs/superpowers/specs/2026-07-24-scan-trace-logger-design.md for the
full design. Non-finite floats (inf/-inf/nan) in `ranges`/`sectors` are
encoded as the literal strings "inf"/"-inf"/"nan" rather than relying on
Python json's non-standard Infinity/NaN extension tokens, so every line
stays valid standard JSON for non-Python readers.
"""
import json
import math
from dataclasses import asdict, dataclass


def _encode_float(v):
    if isinstance(v, float) and not math.isfinite(v):
        if math.isnan(v):
            return "nan"
        return "inf" if v > 0 else "-inf"
    return v


def _decode_float(v):
    if isinstance(v, str) and v in ("inf", "-inf", "nan"):
        return float(v)
    return v


@dataclass
class ScanTraceRecord:
    scan_number: int
    stamp_sec: int
    stamp_nanosec: int
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float
    ranges: list
    sectors: dict

    def to_json(self):
        d = asdict(self)
        d["ranges"] = [_encode_float(r) for r in d["ranges"]]
        d["sectors"] = {k: _encode_float(v) for k, v in d["sectors"].items()}
        return json.dumps(d, allow_nan=False)

    @classmethod
    def from_json(cls, text):
        d = json.loads(text)
        d["ranges"] = [_decode_float(r) for r in d["ranges"]]
        d["sectors"] = {k: _decode_float(v) for k, v in d["sectors"].items()}
        return cls(**d)
