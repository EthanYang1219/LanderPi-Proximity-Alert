#!/usr/bin/env python3
"""Depth preprocessing node (Task 4).

Subscribes to the raw depth image from the Aurora depth camera driver
(`mono16`, 640x400, 14.7 Hz), cleans it, and republishes on
`/poc_fusion/depth_cleaned` for Task 5's `depth_image_proc::PointCloudXyzNode`
to consume. Pipeline, per the Task 4 brief:

  1. Subscribe / cv_bridge to numpy / republish, header preserved verbatim.
  2. Re-encode `mono16` -> `16UC1` (identical buffer, different label --
     `depth_image_proc` only accepts `16UC1` or `32FC1`).
  3. Mask invalid pixels (exactly 0 for this unsigned encoding -- no NaN
     branch; see lib/depth_preprocess.py) and log the invalid fraction.
  4. Zero out the self-arm ROI box read from
     config/depth_preprocess_params.yaml.
  5. Median-filter for spatial denoising, kernel size a parameter, then
     re-apply the ROI mask (the median filter can otherwise pull valid
     neighbour values across the ROI boundary and partially un-mask the
     arm's own box -- see lib/depth_preprocess.clean_depth()).
  6. Publish a debug overlay image showing the masked ROI.

All array logic lives in poc_fusion.lib.depth_preprocess (pure, dependency-
free, unit-tested); this module is ROS plumbing only.
"""
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from poc_fusion.lib.camera_info_watchdog import should_warn
from poc_fusion.lib.depth_preprocess import (
    clean_depth,
    invalid_fraction,
)

EXPECTED_ENCODING = 'mono16'
OUTGOING_ENCODING = '16UC1'
CAMERA_INFO_TOPIC = '/ascamera/camera_publisher/depth0/camera_info'
CAMERA_INFO_WARN_TIMEOUT_SEC = 5.0


class DepthPreprocessNode(Node):
    def __init__(self):
        super().__init__('depth_preprocess_node')

        self.declare_parameter(
            'depth_image_topic', '/ascamera/camera_publisher/depth0/image_raw')
        self.declare_parameter('depth_cleaned_topic', '/poc_fusion/depth_cleaned')
        self.declare_parameter('debug_image_topic', '/poc_fusion/debug_image')
        self.declare_parameter('expected_encoding', EXPECTED_ENCODING)
        self.declare_parameter('median_kernel_size', 3)
        self.declare_parameter('roi_profile', 'default')
        self.declare_parameter('roi_profiles.default.row_min', 0)
        self.declare_parameter('roi_profiles.default.row_max', 0)
        self.declare_parameter('roi_profiles.default.col_min', 0)
        self.declare_parameter('roi_profiles.default.col_max', 0)

        depth_image_topic = self.get_parameter('depth_image_topic').value
        depth_cleaned_topic = self.get_parameter('depth_cleaned_topic').value
        debug_image_topic = self.get_parameter('debug_image_topic').value
        self._expected_encoding = self.get_parameter('expected_encoding').value
        self._median_kernel_size = self.get_parameter('median_kernel_size').value

        # Only `roi_profiles.default.*` is declared above. Selecting any
        # other `roi_profile` value will raise ParameterNotDeclaredException
        # here -- adding a second profile (Task 10a) requires adding four
        # more declare_parameter() calls above, it is not YAML-only. See
        # config/depth_preprocess_params.yaml's comment.
        profile = self.get_parameter('roi_profile').value
        prefix = f'roi_profiles.{profile}'
        self._roi_row_min = self.get_parameter(f'{prefix}.row_min').value
        self._roi_row_max = self.get_parameter(f'{prefix}.row_max').value
        self._roi_col_min = self.get_parameter(f'{prefix}.col_min').value
        self._roi_col_max = self.get_parameter(f'{prefix}.col_max').value

        self._bridge = CvBridge()
        self._encoding_checked = False

        self._cleaned_pub = self.create_publisher(Image, depth_cleaned_topic, 10)
        self._debug_pub = self.create_publisher(Image, debug_image_topic, 10)
        self.create_subscription(Image, depth_image_topic, self._on_depth, 10)

        # Step 4 (Task 5): camera_info watchdog. point_cloud_xyz's failure
        # mode when camera_info never arrives is silent -- no points, no
        # error, costmap quietly runs LiDAR-only -- so this is the only
        # place left to surface it. One-shot timer, cancelled the moment a
        # camera_info message arrives; see lib/camera_info_watchdog.py for
        # the pure decision this delegates to.
        self._camera_info_received = False
        self._camera_info_start_time = self.get_clock().now()
        self.create_subscription(
            CameraInfo, CAMERA_INFO_TOPIC, self._on_camera_info, 10)
        self._camera_info_watchdog_timer = self.create_timer(
            CAMERA_INFO_WARN_TIMEOUT_SEC, self._check_camera_info_watchdog)

        self.get_logger().info(
            f'depth_preprocess_node started: {depth_image_topic} -> '
            f'{depth_cleaned_topic} (expected encoding={self._expected_encoding}, '
            f'roi_profile={profile}, median_kernel_size={self._median_kernel_size})'
        )

    def _check_encoding(self, msg):
        """Step 2: assert the encoding once at startup; fail loudly if wrong.

        mono16 is unsigned -- invalid pixels are exactly 0, never NaN. If
        the driver is ever reconfigured to 32FC1 this assumption breaks, so
        refuse to process rather than silently mis-masking.
        """
        if msg.encoding != self._expected_encoding:
            self.get_logger().fatal(
                f'depth_preprocess_node expected encoding '
                f'"{self._expected_encoding}" but got "{msg.encoding}"; '
                f'refusing to process (invalid-pixel masking assumes an '
                f'unsigned integer encoding with 0 as the only invalid '
                f'value).'
            )
            raise RuntimeError(
                f'Unsupported depth encoding: {msg.encoding!r} '
                f'(expected {self._expected_encoding!r})')
        self._encoding_checked = True

    def _on_depth(self, msg):
        if not self._encoding_checked:
            self._check_encoding(msg)

        depth = self._bridge.imgmsg_to_cv2(
            msg, desired_encoding='passthrough')

        frac = invalid_fraction(depth)
        self.get_logger().info(
            f'invalid pixel fraction: {frac:.3f}', throttle_duration_sec=5.0)

        # clean_depth() re-applies the ROI mask after the median filter, not
        # just before it -- see lib/depth_preprocess.py's docstring. Doing
        # the mask-then-filter-then-remask as one pure call keeps the
        # ordering guarantee in one place rather than risking a future edit
        # here re-introducing the review's Important 2 bug.
        cleaned = clean_depth(
            depth,
            row_min=self._roi_row_min, row_max=self._roi_row_max,
            col_min=self._roi_col_min, col_max=self._roi_col_max,
            kernel_size=self._median_kernel_size,
        )

        out_msg = self._bridge.cv2_to_imgmsg(cleaned, encoding=OUTGOING_ENCODING)
        out_msg.header = msg.header  # stamp and frame_id preserved verbatim
        self._cleaned_pub.publish(out_msg)

        self._publish_debug_image(cleaned, msg.header)

    def _on_camera_info(self, msg):
        self._camera_info_received = True
        # No further work needed here beyond flagging arrival; the timer
        # callback checks this flag and cancels itself once it observes
        # `received=True` (belt-and-braces even though the flag alone
        # already makes should_warn() return False from then on).
        if self._camera_info_watchdog_timer is not None:
            self._camera_info_watchdog_timer.cancel()
            self._camera_info_watchdog_timer = None

    def _check_camera_info_watchdog(self):
        elapsed_sec = (
            (self.get_clock().now() - self._camera_info_start_time).nanoseconds
            / 1e9
        )
        if should_warn(
            received=self._camera_info_received,
            elapsed_sec=elapsed_sec,
            timeout_sec=CAMERA_INFO_WARN_TIMEOUT_SEC,
        ):
            self.get_logger().warn(
                f'No message received on {CAMERA_INFO_TOPIC} within '
                f'{CAMERA_INFO_WARN_TIMEOUT_SEC:.0f}s of startup. '
                f'depth_image_proc::PointCloudXyzNode requires camera_info '
                f'to produce points -- without it, the fused costmap will '
                f'silently run LiDAR-only with no other error.'
            )
        if self._camera_info_watchdog_timer is not None:
            self._camera_info_watchdog_timer.cancel()
            self._camera_info_watchdog_timer = None

    def _publish_debug_image(self, cleaned, header):
        """Step 5: 8-bit visualisation with the masked ROI box drawn on it."""
        normalized = cv2.normalize(cleaned, None, 0, 255, cv2.NORM_MINMAX)
        debug = normalized.astype(np.uint8)
        debug_bgr = cv2.cvtColor(debug, cv2.COLOR_GRAY2BGR)
        if self._roi_row_max > self._roi_row_min and self._roi_col_max > self._roi_col_min:
            cv2.rectangle(
                debug_bgr,
                (self._roi_col_min, self._roi_row_min),
                (self._roi_col_max - 1, self._roi_row_max - 1),
                (0, 0, 255), 1,
            )
        debug_msg = self._bridge.cv2_to_imgmsg(debug_bgr, encoding='bgr8')
        debug_msg.header = header
        self._debug_pub.publish(debug_msg)


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
