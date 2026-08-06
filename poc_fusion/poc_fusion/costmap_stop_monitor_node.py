"""Placeholder node — real implementation lands in Task 7 / Task 9.

Watches the fused local costmap and raises a stop signal when an obstacle
enters the safety window. This stub only proves the entry point imports and
runs; no detection or stop logic here.
"""

import rclpy
from rclpy.node import Node


class CostmapStopMonitorNode(Node):
    def __init__(self):
        super().__init__('costmap_stop_monitor_node')


def main(args=None):
    rclpy.init(args=args)
    node = CostmapStopMonitorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
