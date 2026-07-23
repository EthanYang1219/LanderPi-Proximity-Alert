import json
import math
import os
import tempfile
from types import SimpleNamespace

import rclpy

from proximity_alert.scan_trace_logger import ScanTraceLogger


def _scan(ranges, stamp_sec=1784824869, stamp_nanosec=454205692,
          angle_min=-math.pi, inc_deg=1.0, range_min=0.05, range_max=8.0):
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp_sec, nanosec=stamp_nanosec)),
        angle_min=angle_min, angle_increment=math.radians(inc_deg),
        range_min=range_min, range_max=range_max, ranges=ranges,
    )


def _fresh_logger(csv_path):
    node = ScanTraceLogger()
    node.csv_path = csv_path
    return node


def test_writes_one_jsonl_line_per_scan_with_scan_number_and_stamp():
    rclpy.init()
    fd, path = tempfile.mkstemp(suffix=".jsonl")
    os.close(fd)
    os.remove(path)  # let the node create it fresh on first write

    node = _fresh_logger(path)
    n = 360  # 1-degree increments over 2*pi
    ranges = [1.0] * n
    node.on_scan(_scan(ranges, stamp_sec=100, stamp_nanosec=200))
    node.on_scan(_scan(ranges, stamp_sec=100, stamp_nanosec=300))

    with open(path) as f:
        lines = [json.loads(line) for line in f]

    node.destroy_node()
    rclpy.shutdown()
    os.remove(path)

    assert len(lines) == 2
    assert lines[0]["scan_number"] == 0
    assert lines[1]["scan_number"] == 1          # increments per scan, in receipt order
    assert lines[0]["stamp_sec"] == 100
    assert lines[0]["stamp_nanosec"] == 200
    assert lines[1]["stamp_nanosec"] == 300
    assert lines[0]["range_max"] == 8.0
    assert len(lines[0]["ranges"]) == 360
    assert "front" in lines[0]["sectors"]        # reduce_to_sectors output attached


def test_ranges_and_sectors_use_string_sentinels_for_non_finite():
    rclpy.init()
    fd, path = tempfile.mkstemp(suffix=".jsonl")
    os.close(fd)
    os.remove(path)

    node = _fresh_logger(path)
    n = 360
    ranges = [float("inf")] * n  # nothing in range anywhere -> every sector is inf
    node.on_scan(_scan(ranges))

    with open(path) as f:
        line = json.loads(f.readline())

    node.destroy_node()
    rclpy.shutdown()
    os.remove(path)

    assert line["ranges"][0] == "inf"
    assert line["sectors"]["front"] == "inf"
    assert line["sectors"]["rear"] == "inf"
