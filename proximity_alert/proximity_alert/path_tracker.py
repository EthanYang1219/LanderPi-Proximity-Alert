#!/usr/bin/env python3
"""Drives the robot from point A to point B, stopping and turning away from obstacles seen on /scan."""
import math
import signal
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

DRIVE = "drive"
AVOIDING = "avoiding"
HALTED = "halted"


class PathTracker(Node):
    def __init__(self):
        super().__init__("path_tracker")

        self.declare_parameter("safety_distance", 0.30)
        self.declare_parameter("forward_speed", 0.15)
        self.declare_parameter("turn_speed", 0.6)
        self.declare_parameter("scan_arc_deg", 180.0)
        self.declare_parameter("avoid_turn_duration", 1.0)
        self.declare_parameter("control_rate_hz", 10.0)
        self.declare_parameter("scan_topic", "/scan_raw")
        self.declare_parameter("scan_timeout", 0.5)

        self.safety_distance = self.get_parameter("safety_distance").value
        self.forward_speed = self.get_parameter("forward_speed").value
        self.turn_speed = self.get_parameter("turn_speed").value
        self.scan_arc_deg = self.get_parameter("scan_arc_deg").value
        self.avoid_turn_duration = self.get_parameter("avoid_turn_duration").value
        control_rate_hz = self.get_parameter("control_rate_hz").value
        scan_topic = self.get_parameter("scan_topic").value
        self.scan_timeout = self.get_parameter("scan_timeout").value

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self.scan_callback, qos_profile_sensor_data
        )

        self.min_range = None
        self.last_scan_time = None
        self.turn_away_direction = 0.0
        self.state = DRIVE
        self.avoid_start_time = None

        self.control_timer = self.create_timer(1.0 / control_rate_hz, self.control_loop)

        self.get_logger().info(
            f"path_tracker started: safety_distance={self.safety_distance}m "
            f"forward_speed={self.forward_speed}m/s scan_arc_deg={self.scan_arc_deg}deg"
        )

    def publish_stop(self):
        """Command a hard stop, delivered reliably.

        The motor controller (STM32) holds the last commanded velocity with no
        watchdog auto-stop, so a single zero Twist published as the process
        exits can be dropped before it reaches the wire, leaving the robot
        driving. Publish zero several times with a brief pause and spin between
        sends so at least one is delivered before shutdown.
        """
        # Called from the signal handler while rclpy.spin() is suspended on the
        # stack, so must NOT spin re-entrantly. Publishing is asynchronous; the
        # short sleeps let the DDS layer flush each message before the process
        # exits.
        stop = Twist()
        for _ in range(5):
            self.cmd_vel_pub.publish(stop)
            time.sleep(0.03)

    def scan_callback(self, msg: LaserScan):
        half_arc = math.radians(self.scan_arc_deg) / 2.0

        closest = None
        closest_angle = 0.0
        angle = msg.angle_min
        for r in msg.ranges:
            # This LiDAR (LD19) reports angles as 0..2pi, so "forward" (0 rad)
            # sits at both ends of the sweep. Wrap each beam into [-pi, pi] so
            # the forward arc is measured symmetrically around straight-ahead
            # instead of only catching the 0..+half_arc (front-right) side.
            rel = math.atan2(math.sin(angle), math.cos(angle))
            if -half_arc <= rel <= half_arc:
                if msg.range_min <= r <= msg.range_max:
                    if closest is None or r < closest:
                        closest = r
                        closest_angle = rel
            angle += msg.angle_increment

        self.min_range = closest
        self.last_scan_time = self.get_clock().now()
        # Obstacle left of center (positive angle) -> turn right (negative), and vice versa.
        self.turn_away_direction = -1.0 if closest_angle >= 0 else 1.0

    def scan_is_stale(self):
        """True if we have no scan yet, or the last one is older than scan_timeout.

        Guards against driving forward on stale data if the LiDAR stops
        publishing mid-run (observed to happen with this LD19), and against
        lurching forward at startup before the first scan arrives.
        """
        if self.last_scan_time is None:
            return True
        age = (self.get_clock().now() - self.last_scan_time).nanoseconds / 1e9
        return age > self.scan_timeout

    def control_loop(self):
        cmd = Twist()

        if self.state == DRIVE:
            if self.scan_is_stale():
                # No fresh obstacle data -> hold still rather than drive blind.
                self.get_logger().warn(
                    "No fresh scan within scan_timeout; holding.",
                    throttle_duration_sec=1.0,
                )
                # cmd stays zeroed for this tick
            elif self.min_range is not None and self.min_range <= self.safety_distance:
                self.get_logger().info(f"Obstacle at {self.min_range:.2f}m, stopping")
                self.state = AVOIDING
                self.avoid_start_time = self.get_clock().now()
                # cmd stays zeroed for this tick
            else:
                cmd.linear.x = self.forward_speed

        elif self.state == AVOIDING:
            elapsed = (self.get_clock().now() - self.avoid_start_time).nanoseconds / 1e9
            if elapsed < self.avoid_turn_duration:
                cmd.angular.z = self.turn_speed * self.turn_away_direction
            elif self.min_range is not None and self.min_range <= self.safety_distance:
                # Still blocked after one turn-away attempt (e.g. a fixed
                # endpoint obstacle) -> hold a final stop instead of spinning
                # forever, so trial_logger sees a stable, confirmable stop.
                self.get_logger().info("Still blocked after turn-away, halting")
                self.state = HALTED
            else:
                self.state = DRIVE

        elif self.state == HALTED:
            pass  # cmd stays zeroed

        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    # rclpy's default signal handling tears down the context before our own
    # cleanup runs, so a final safety-stop publish in a `finally` block would
    # silently fail on SIGINT/SIGTERM. Publish the stop from our own handler
    # instead, while the context is still valid.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = PathTracker()

    def stop_and_shutdown(signum, frame):
        node.publish_stop()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, stop_and_shutdown)
    signal.signal(signal.SIGTERM, stop_and_shutdown)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
