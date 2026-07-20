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
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

DRIVE = "drive"
AVOIDING = "avoiding"
HALTED = "halted"


def normalize_angle(angle):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


class HeadingPID:
    """PID on heading error (rad) -> corrective angular.z (rad/s)."""

    def __init__(self, kp, ki, kd, output_limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.prev_error = None

    def update(self, error, dt):
        self.integral += error * dt
        # Anti-windup: keep the integral term from growing past what could
        # alone saturate the output.
        max_integral = self.output_limit / self.ki if self.ki else float("inf")
        self.integral = max(-max_integral, min(max_integral, self.integral))
        derivative = 0.0 if self.prev_error is None else (error - self.prev_error) / dt
        self.prev_error = error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return max(-self.output_limit, min(self.output_limit, output))


class PathTracker(Node):
    def __init__(self):
        super().__init__("path_tracker")

        self.declare_parameter("safety_distance", 0.30) # The maximum allowed distance from the obstacle to the robot in meters (0.3 meters)
        self.declare_parameter("forward_speed", 0.50) # How fast the robot moves forwards
        self.declare_parameter("turn_speed", 0.6) 
        self.declare_parameter("scan_arc_deg", 180.0)
        self.declare_parameter("avoid_turn_duration", 1.0) # How long the robot avoids the obstacle by turning
        self.declare_parameter("control_rate_hz", 10.0)
        self.declare_parameter("scan_topic", "/scan_raw")
        self.declare_parameter("scan_timeout", 0.5)
        self.declare_parameter("avoid_reverse_speed", 0.1)
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("heading_kp", 1.0)
        self.declare_parameter("heading_ki", 0.0)
        self.declare_parameter("heading_kd", 0.1)
        self.declare_parameter("heading_max_correction", 0.3)
        self.declare_parameter("max_avoid_attempts", 3)
        self.declare_parameter("clear_drive_duration", 3.0)

        self.safety_distance = self.get_parameter("safety_distance").value
        self.forward_speed = self.get_parameter("forward_speed").value
        self.turn_speed = self.get_parameter("turn_speed").value
        self.scan_arc_deg = self.get_parameter("scan_arc_deg").value
        self.avoid_turn_duration = self.get_parameter("avoid_turn_duration").value
        control_rate_hz = self.get_parameter("control_rate_hz").value
        scan_topic = self.get_parameter("scan_topic").value
        self.scan_timeout = self.get_parameter("scan_timeout").value
        self.avoid_reverse_speed = self.get_parameter("avoid_reverse_speed").value
        odom_topic = self.get_parameter("odom_topic").value
        self.max_avoid_attempts = self.get_parameter("max_avoid_attempts").value
        self.clear_drive_duration = self.get_parameter("clear_drive_duration").value

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self.scan_callback, qos_profile_sensor_data
        )
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10
        )

        self.min_range = None
        self.last_scan_time = None
        self.turn_away_direction = 0.0
        self.state = DRIVE
        self.avoid_start_time = None
        self.locked_turn_direction = 0.0
        self.consecutive_avoid_count = 0
        self.drive_since = self.get_clock().now()

        self.current_yaw = None
        self.target_heading = None
        self.heading_pid = HeadingPID(
            kp=self.get_parameter("heading_kp").value,
            ki=self.get_parameter("heading_ki").value,
            kd=self.get_parameter("heading_kd").value,
            output_limit=self.get_parameter("heading_max_correction").value,
        )
        self.control_dt = 1.0 / control_rate_hz

        self.control_timer = self.create_timer(self.control_dt, self.control_loop)

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

    def odom_callback(self, msg: Odometry):
        self.current_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

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
                # Track repeated avoidance attempts without an intervening
                # clean drive. A wide/complex obstacle (e.g. a desk and chair
                # spanning most of the forward arc) can make each turn "fix"
                # being blocked on one side only by revealing the obstacle on
                # the other side -- confirmed by direct log evidence: 13
                # consecutive episodes alternating -0.6/+0.6 with no net
                # progress. Give up and halt instead of oscillating forever.
                self.consecutive_avoid_count += 1
                if self.consecutive_avoid_count > self.max_avoid_attempts:
                    self.get_logger().warn(
                        f"{self.consecutive_avoid_count} avoidance attempts in a "
                        "row without a clear drive -- likely too wide/complex to "
                        "route around. Halting instead of oscillating."
                    )
                    self.state = HALTED
                else:
                    self.get_logger().info(f"Obstacle at {self.min_range:.2f}m, stopping")
                    self.state = AVOIDING
                    self.avoid_start_time = self.get_clock().now()
                    # Lock the turn direction in now. turn_away_direction is
                    # recomputed on every incoming scan regardless of state,
                    # so reading it live during the turn (below) let the
                    # obstacle's shifting apparent angle flip the commanded
                    # direction mid-maneuver, cancelling the turn to ~0 net
                    # rotation.
                    self.locked_turn_direction = self.turn_away_direction
                    # Clear the heading target: once we resume DRIVE after
                    # this turn, "straight" means whatever new direction
                    # we're then facing, not the pre-turn heading.
                    self.target_heading = None
                    self.heading_pid.reset()
                # cmd stays zeroed for this tick
            else:
                drive_elapsed = (self.get_clock().now() - self.drive_since).nanoseconds / 1e9
                if drive_elapsed >= self.clear_drive_duration:
                    # Sustained clean driving -> the obstacle is actually
                    # behind us, not just a lull between oscillation attempts.
                    self.consecutive_avoid_count = 0
                cmd.linear.x = self.forward_speed
                if self.current_yaw is not None:
                    if self.target_heading is None:
                        # Capture the heading to hold the moment straight
                        # driving (re)starts.
                        self.target_heading = self.current_yaw
                    error = normalize_angle(self.target_heading - self.current_yaw)
                    cmd.angular.z = self.heading_pid.update(error, self.control_dt)
                # else: no odom yet -- drive straight open-loop until it arrives.

        elif self.state == AVOIDING:
            elapsed = (self.get_clock().now() - self.avoid_start_time).nanoseconds / 1e9
            if elapsed < self.avoid_turn_duration:
                # The robot's own mecanum driver (controller/mecanum.py, not
                # part of this package) computes all four wheel speeds equal
                # for a pure angular-only command -- confirmed by reading
                # /ros_robot_controller/set_motor directly, all 4 IDs got the
                # identical rps for angular.z alone. That can't produce a
                # clean in-place rotation. Adding a small reverse component
                # breaks the degenerate case (and backs further from the
                # obstacle instead of creeping toward it) without touching
                # the vendor driver.
                cmd.linear.x = -self.avoid_reverse_speed
                cmd.angular.z = self.turn_speed * self.locked_turn_direction
            elif self.min_range is not None and self.min_range <= self.safety_distance:
                # Still blocked after one turn-away attempt (e.g. a fixed
                # endpoint obstacle) -> hold a stop, but keep re-checking in
                # HALTED below rather than freezing permanently.
                self.get_logger().info("Still blocked after turn-away, halting")
                self.state = HALTED
            else:
                self.state = DRIVE
                self.drive_since = self.get_clock().now()

        elif self.state == HALTED:
            # Re-check each tick rather than freezing forever: at the real
            # experiment's endpoint the obstacle never clears, so this stays
            # halted exactly as before. But it recovers if the obstacle is
            # moved away (floor testing) or was a transient false trigger.
            if not self.scan_is_stale() and (
                self.min_range is None or self.min_range > self.safety_distance
            ):
                self.get_logger().info("Obstacle cleared, resuming drive.")
                self.state = DRIVE
                self.drive_since = self.get_clock().now()
                # Deliberately NOT resetting consecutive_avoid_count here:
                # this recovery can fire within ~0.3s of halting (seen on
                # real hardware), too fast to trust as a genuine clear. Only
                # clear_drive_duration of sustained clean driving (checked in
                # the DRIVE branch) resets the give-up counter, regardless of
                # which path led back to DRIVE.
            # else: cmd stays zeroed for this tick

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
