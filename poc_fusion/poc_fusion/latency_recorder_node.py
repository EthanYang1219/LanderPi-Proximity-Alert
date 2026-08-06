"""Placeholder node — real implementation lands in Task 12.

Records end-to-end sensor-to-costmap latency for the fusion pipeline. This
stub only proves the entry point imports and runs; no measurement logic
here.
"""

import rclpy
from rclpy.node import Node


class LatencyRecorderNode(Node):
    def __init__(self):
        super().__init__('latency_recorder_node')


def main(args=None):
    rclpy.init(args=args)
    node = LatencyRecorderNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
