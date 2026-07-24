#!/usr/bin/env python3
"""Companion node: writes AvoidanceController decision records to their own CSV,
separate from trial_logger's motion CSV, on the host-mounted shared path."""
import csv
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from proximity_alert.decision_record import DecisionRecord


class DecisionLogger(Node):
    def __init__(self):
        super().__init__("decision_logger")
        self.declare_parameter("csv_path", "/home/ubuntu/shared/trials/decision_log.csv")
        self.csv_path = self.get_parameter("csv_path").value
        self._ensure_header()
        self.create_subscription(String, "/avoidance_decision", self.on_decision, 50)
        self.get_logger().info(f"decision_logger up. Writing to '{self.csv_path}'.")

    def _ensure_header(self):
        if not os.path.exists(self.csv_path):
            os.makedirs(os.path.dirname(self.csv_path) or ".", exist_ok=True)
            with open(self.csv_path, "w", newline="") as f:
                csv.writer(f).writerow(DecisionRecord.csv_header())

    def on_decision(self, msg):
        # A companion node must never die from bad input it doesn't control:
        # a malformed message on the topic is logged and skipped, not fatal.
        try:
            rec = DecisionRecord.from_json(msg.data)
        except (ValueError, TypeError) as e:
            self.get_logger().warn(f"Ignoring unparseable /avoidance_decision message: {e}")
            return
        with open(self.csv_path, "a", newline="") as f:
            csv.writer(f).writerow(rec.csv_row())


def main(args=None):
    rclpy.init(args=args)
    node = DecisionLogger()
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
