import json
import math

from proximity_alert.scan_trace_record import ScanTraceRecord


def _rec(ranges=None, sectors=None):
    return ScanTraceRecord(
        scan_number=4821,
        stamp_sec=1784824869,
        stamp_nanosec=454205692,
        angle_min=-3.14159,
        angle_increment=0.01745,
        range_min=0.12,
        range_max=25.0,
        ranges=ranges if ranges is not None else [0.45, 0.46, 1.0],
        sectors=sectors if sectors is not None else {
            "front": 0.45, "front_left": 1.2, "front_center": 0.45,
            "front_right": 0.9, "left": 1.4, "right": 0.9, "rear": 2.1,
        },
    )


def test_round_trips_finite_values():
    rec = _rec()
    restored = ScanTraceRecord.from_json(rec.to_json())
    assert restored == rec


def test_round_trips_inf_and_nan_in_ranges():
    # NaN != NaN under Python's normal equality, so this test deliberately
    # does NOT do `restored == rec` (dataclass equality would compare the
    # nan field with `==` and always fail even on a correct round-trip).
    # Every field is asserted individually, using math.isnan() for the nan
    # case specifically.
    rec = _rec(ranges=[0.5, float("inf"), float("-inf"), float("nan")])
    restored = ScanTraceRecord.from_json(rec.to_json())
    assert restored.ranges[0] == 0.5
    assert restored.ranges[1] == float("inf")
    assert restored.ranges[2] == float("-inf")
    assert math.isnan(restored.ranges[3])


def test_round_trips_inf_in_sectors():
    rec = _rec(sectors={
        "front": float("inf"), "front_left": 1.0, "front_center": float("inf"),
        "front_right": 1.0, "left": 1.0, "right": 1.0, "rear": float("inf"),
    })
    restored = ScanTraceRecord.from_json(rec.to_json())
    assert restored.sectors["front"] == float("inf")
    assert restored.sectors["front_center"] == float("inf")
    assert restored.sectors["rear"] == float("inf")
    assert restored.sectors["front_left"] == 1.0


def test_round_trips_realistic_lidar_sized_scan():
    # The LD19 publishes roughly 450 beams per revolution on this hardware
    # (confirmed via ros2 topic echo during development) -- a handful of
    # hand-picked values in earlier tests doesn't prove serialization holds
    # up at real scan width. Build a realistic-sized ranges array with a mix
    # of finite readings and inf gaps (a real scan is never all-finite).
    n = 452
    ranges = [
        float("inf") if i % 37 == 0 else round(0.3 + 0.01 * (i % 50), 4)
        for i in range(n)
    ]
    sectors = {
        "front": 0.45, "front_left": 1.2, "front_center": 0.45,
        "front_right": 0.9, "left": 1.4, "right": 0.9, "rear": 2.1,
    }
    rec = _rec(ranges=ranges, sectors=sectors)
    restored = ScanTraceRecord.from_json(rec.to_json())

    assert len(restored.ranges) == n
    for original, got in zip(ranges, restored.ranges):
        if math.isinf(original):
            assert math.isinf(got) and got > 0
        else:
            assert got == original
    assert restored.sectors == sectors


def test_serializes_as_strict_standard_json_not_python_extension_tokens():
    # json.dumps' default non-standard Infinity/-Infinity/NaN tokens would
    # break strict JSON readers (pandas.read_json, jq, non-Python tooling).
    # The design mandates quoted string sentinels instead -- assert the
    # actual wire format, not just that our own from_json can read it back.
    text = _rec(ranges=[float("inf"), float("nan")]).to_json()
    assert "Infinity" not in text
    assert "NaN" not in text
    assert '"inf"' in text
    assert '"nan"' in text
    # Must still be valid standard JSON (json.loads with default strict
    # allow_nan behavior is fine either way, but this proves no bare
    # unquoted non-finite token slipped through).
    parsed = json.loads(text)
    assert parsed["ranges"] == ["inf", "nan"]
