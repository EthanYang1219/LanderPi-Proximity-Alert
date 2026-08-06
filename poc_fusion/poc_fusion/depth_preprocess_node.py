"""Placeholder node — real implementation lands in Task 4.

Subscribes to the raw depth image / camera_info, republishes a preprocessed
depth stream for the depth_image_proc / obstacle-detection pipeline. This
stub only proves the entry point imports and runs; no pipeline logic here.
"""

import rclpy
from rclpy.node import Node


class DepthPreprocessNode(Node):
    def __init__(self):
        super().__init__('depth_preprocess_node')


def main(args=None):
    rclpy.init(args=args)
    node = DepthPreprocessNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
