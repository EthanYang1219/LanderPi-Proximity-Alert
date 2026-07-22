#!/usr/bin/env python3
"""Drives the robot from A to B, holding the goal heading and running the
refined obstacle-avoidance state machine (AvoidanceController): strafe-first,
then turn-and-drive, then one bounded recovery, then halt.

This node is a thin ROS wrapper -- it reduces each LaserScan into sector
clearances, sizes the obstacle, finds an escape gap, and feeds those to the
pure AvoidanceController every control tick. It publishes the commanded Twist,
the live forward-arc range (/forward_min_range) for trial_logger, and a
structured JSON decision record (/avoidance_decision) for decision_logger.
"""
import math
import signal
import time
from dataclasses import fields as dc_fields

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String

from proximity_alert.scan_utils import reduce_to_sectors, size_obstacle, select_gap
from proximity_alert.avoidance import AvoidanceController, AvoidanceConfig


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


class PathTracker(Node):
    def __init__(self):
        super().__init__("path_tracker")

        # Every AvoidanceConfig field is a ROS parameter (defaults from the
        # dataclass), so the whole behavior is tunable from the launch/CLI.
        defaults = AvoidanceConfig()
        for f in dc_fields(AvoidanceConfig):
            self.declare_parameter(f.name, getattr(defaults, f.name))
        self.config = AvoidanceConfig(**{
            f.name: self.get_parameter(f.name).value for f in dc_fields(AvoidanceConfig)
        })

        # Node-level (non-controller) parameters.
        self.declare_parameter("scan_topic", "/scan_raw")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("scan_timeout", 0.5)
        scan_topic = self.get_parameter("scan_topic").value
        odom_topic = self.get_parameter("odom_topic").value
        self.scan_timeout = self.get_parameter("scan_timeout").value

        self.controller = AvoidanceController(self.config)

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.range_pub = self.create_publisher(Float32, "/forward_min_range", 10)
        self.decision_pub = self.create_publisher(String, "/avoidance_decision", 50)
        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self.scan_callback, qos_profile_sensor_data
        )
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10
        )

        self._sectors = None
        self._obstacle = None
        self._gap = None
        self.last_scan_time = None
        self.current_yaw = None
        self.goal_heading_abs = None
        self._goal_set = False

        self.control_dt = 1.0 / self.config.control_rate_hz
        self.control_timer = self.create_timer(self.control_dt, self.control_loop)

        self.get_logger().info(
            f"path_tracker started: safety_distance={self.config.safety_distance}m "
            f"forward_speed={self.config.forward_speed}m/s "
            f"strafe_speed={self.config.strafe_speed}m/s "
            f"disable_avoidance={self.config.disable_avoidance}"
        )

    def publish_stop(self):
        """Command a hard stop, delivered reliably.

        The STM32 holds the last commanded velocity with no watchdog, so a
        single zero Twist published as the process exits can be dropped before
        it reaches the wire. Publish zero several times with a brief pause so at
        least one is delivered. Called from the signal handler while spin is
        suspended -- must NOT spin re-entrantly.
        """
        stop = Twist()
        for _ in range(5):
            self.cmd_vel_pub.publish(stop)
            time.sleep(0.03)

    def scan_callback(self, msg: LaserScan):
        cfg = self.config
        sectors = reduce_to_sectors(msg, cfg.front_arc_deg, cfg.front_subsector_deg,
                                    cfg.side_window_deg, cfg.rear_window_deg)
        obstacle = size_obstacle(msg, cfg.front_arc_deg, cfg.obstacle_detect_range)
        # Gap bearings are relative to the robot's current heading; pass the
        # goal as a relative bearing so the recovery gap tie-break prefers the
        # opening closest to the goal direction.
        goal_rel = 0.0
        if self._goal_set and self.current_yaw is not None:
            goal_rel = normalize_angle(self.goal_heading_abs - self.current_yaw)
        gap = select_gap(msg, cfg.min_gap_clearance, cfg.min_gap_width_deg, goal_rel)

        self._sectors, self._obstacle, self._gap = sectors, obstacle, gap
        self.last_scan_time = self.get_clock().now()

        range_msg = Float32()
        range_msg.data = sectors["front"] if math.isfinite(sectors["front"]) else float("inf")
        self.range_pub.publish(range_msg)

    def odom_callback(self, msg: Odometry):
        self.current_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        if not self._goal_set:
            self.goal_heading_abs = self.current_yaw
            self.controller.set_goal_heading(self.current_yaw)
            self._goal_set = True

    def scan_is_stale(self):
        if self.last_scan_time is None:
            return True
        age = (self.get_clock().now() - self.last_scan_time).nanoseconds / 1e9
        return age > self.scan_timeout

    def control_loop(self):
        if self.scan_is_stale() or self._sectors is None:
            # No fresh obstacle data -> hold still rather than drive blind.
            self.get_logger().warn(
                "No fresh scan within scan_timeout; holding.",
                throttle_duration_sec=1.0,
            )
            self.cmd_vel_pub.publish(Twist())
            return

        now = self.get_clock().now().nanoseconds / 1e9
        yaw = self.current_yaw if self.current_yaw is not None else 0.0
        out = self.controller.step(self._sectors, self._obstacle, self._gap, yaw, now)

        cmd = Twist()
        cmd.linear.x = float(out.linear_x)
        cmd.linear.y = float(out.linear_y)
        cmd.angular.z = float(out.angular_z)
        self.cmd_vel_pub.publish(cmd)

        if out.decision is not None:
            self.decision_pub.publish(String(data=out.decision.to_json()))


def main(args=None):
    # rclpy's default signal handling tears down the context before our own
    # cleanup runs, so a final safety-stop publish in a `finally` block would
    # silently fail on SIGINT/SIGTERM. The handler must ONLY set a flag, never
    # call publish_stop()/shutdown() directly: a second SIGINT arriving while
    # the first call is still running re-enters the handler and calls
    # rclpy.shutdown() a second time while spin_once() is still using the
    # context -- confirmed on real hardware to crash the in-flight executor
    # call. Doing the actual stop/shutdown from the main loop, only after
    # spin_once() has returned, means a repeated signal just re-sets an
    # already-true flag -- harmless.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = PathTracker()

    stop_requested = False

    def request_stop(signum, frame):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    try:
        while rclpy.ok() and not stop_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
