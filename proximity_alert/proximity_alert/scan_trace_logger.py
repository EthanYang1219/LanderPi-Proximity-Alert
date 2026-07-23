#!/usr/bin/env python3
"""Companion node: records the full raw LiDAR scan (every beam) plus a
reduced-sector summary to a continuous JSON-Lines trace file, independent of
path_tracker/trial_logger/decision_logger.

Exists to diagnose a total detection miss (e.g. a thin chair leg the LiDAR
never saw) -- such a miss never triggers path_tracker's AvoidanceController,
so it has no row in either trial_logger's or decision_logger's CSVs. This
node captures raw scan data continuously so a miss can be traced back after
the fact and classified as a sensor-geometry blind spot versus a tunable
threshold miss. See docs/superpowers/specs/2026-07-24-scan-trace-logger-design.md.

Each scan is appended as a single write ending in a newline, so a process
killed mid-write can only ever leave the LAST line incomplete -- every prior
line is already complete. Offline analysis should skip a line that fails to
parse (expected only at end-of-file) rather than treat it as corruption.
"""
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from proximity_alert.scan_trace_record import ScanTraceRecord
from proximity_alert.scan_utils import reduce_to_sectors


class ScanTraceLogger(Node):
    def __init__(self):
        super().__init__("scan_trace_logger")
        self.declare_parameter("csv_path", "/home/ubuntu/shared/trials/scan_trace.jsonl")
        # /scan_raw is the actual, confirmed-live LiDAR topic (verified via
        # `ros2 topic list`; no /scan topic exists) -- kept as an overridable
        # param, not hardcoded, so this still works if the topic is ever
        # renamed upstream, same as path_tracker's own scan_topic param.
        self.declare_parameter("scan_topic", "/scan_raw")
        # These four defaults MUST be kept in sync with AvoidanceConfig's
        # fields of the same name in avoidance.py. There is no shared config
        # between this node and path_tracker (independent by design), so if
        # path_tracker's sector-window tuning ever changes, these must be
        # updated too -- otherwise the logged `sectors` silently stop
        # reflecting what the controller was actually reacting to.
        self.declare_parameter("front_arc_deg", 180.0)
        self.declare_parameter("front_subsector_deg", 60.0)
        self.declare_parameter("side_window_deg", 60.0)
        self.declare_parameter("rear_window_deg", 60.0)

        self.csv_path = self.get_parameter("csv_path").value
        self.front_arc_deg = self.get_parameter("front_arc_deg").value
        self.front_subsector_deg = self.get_parameter("front_subsector_deg").value
        self.side_window_deg = self.get_parameter("side_window_deg").value
        self.rear_window_deg = self.get_parameter("rear_window_deg").value

        self._scan_number = 0

        scan_topic = self.get_parameter("scan_topic").value
        self.create_subscription(LaserScan, scan_topic, self.on_scan, qos_profile_sensor_data)
        self.get_logger().info(f"scan_trace_logger up. Writing to '{self.csv_path}'.")

    def on_scan(self, msg: LaserScan):
        sectors = reduce_to_sectors(
            msg, self.front_arc_deg, self.front_subsector_deg,
            self.side_window_deg, self.rear_window_deg,
        )
        rec = ScanTraceRecord(
            scan_number=self._scan_number,
            stamp_sec=msg.header.stamp.sec,
            stamp_nanosec=msg.header.stamp.nanosec,
            angle_min=msg.angle_min,
            angle_increment=msg.angle_increment,
            range_min=msg.range_min,
            range_max=msg.range_max,
            ranges=list(msg.ranges),
            sectors=sectors,
        )
        self._scan_number += 1

        os.makedirs(os.path.dirname(self.csv_path) or ".", exist_ok=True)
        with open(self.csv_path, "a") as f:
            f.write(rec.to_json() + "\n")


def main(args=None):
    rclpy.init(args=args)
    node = ScanTraceLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
