#!/usr/bin/env python3
"""Drives the robot from A to B, holding the goal heading and running the
refined obstacle-avoidance state machine (AvoidanceController): strafe-first,
then turn-and-drive, then one bounded recovery, then halt.

This node is a thin ROS wrapper -- it reduces each LaserScan into sector
clearances, sizes the obstacle, finds an escape gap, and feeds those to the
pure AvoidanceController every control tick. It publishes the commanded Twist,
the live forward-arc range (/forward_min_range) for trial_logger, and a
structured JSON decision record (/avoidance_decision) for decision_logger.

It also plays an audible alert through the robot's USB speaker on by
default -- once per obstacle encounter, not on every escalation step within
one -- via the same trigger logic as the (now folded-in) standalone
obstacle_audio node; see the `audio_alert_enabled`/`wav_path`/`alsa_device`
parameters. Set `audio_alert_enabled:=false` to disable it.
"""
import math
import os
import signal
import subprocess
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

from proximity_alert.audio_trigger import should_play
from proximity_alert.scan_utils import reduce_to_sectors, size_obstacle, select_gap
from proximity_alert.avoidance import AvoidanceController, AvoidanceConfig
from proximity_alert.nav_utils import (
    ARRIVED_TARGET, DRIVING, STOPPED_ODOM_FAULT,
    decide_arrival, distance_from_start, should_skip_controller,
)


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

        # Fixed-distance A-to-B stop. 0.0 disables it entirely, so a node
        # launched without this parameter behaves exactly as it did before.
        self.declare_parameter("target_distance", 0.0)
        self.declare_parameter("odom_timeout_sec", 1.0)
        self.target_distance = self.get_parameter("target_distance").value
        self.odom_timeout_sec = self.get_parameter("odom_timeout_sec").value

        # Obstacle audio alert -- on by default (formerly a standalone
        # obstacle_audio node you had to launch separately). Same
        # once-per-encounter trigger logic (proximity_alert.audio_trigger),
        # just fed directly from the decision this control tick already
        # produced instead of round-tripping through /avoidance_decision.
        self.declare_parameter("audio_alert_enabled", True)
        self.declare_parameter("wav_path", "/home/ubuntu/shared/audio/obstacle_alert.wav")
        self.declare_parameter("alsa_device", "plughw:2,0")
        self.audio_alert_enabled = self.get_parameter("audio_alert_enabled").value
        self.wav_path = self.get_parameter("wav_path").value
        self.alsa_device = self.get_parameter("alsa_device").value
        self._last_played_encounter_id = None

        # Fail loudly at startup, not silently at the first obstacle. aplay is
        # spawned non-blocking with its stderr discarded, so a missing WAV
        # produces NO visible error at all -- the robot just never beeps and
        # you find out by not hearing it (this happened: the audio/ folder had
        # never been created). Checked once here rather than per-encounter.
        if self.audio_alert_enabled and not os.path.exists(self.wav_path):
            self.get_logger().error(
                f"audio_alert_enabled is true but wav_path '{self.wav_path}' "
                "does not exist -- the robot will NOT beep on obstacles. Drop a "
                "WAV there (host side: /home/pi/docker/tmp/audio/), or pass "
                "-p audio_alert_enabled:=false to silence this."
            )

        self.controller = AvoidanceController(self.config)

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.range_pub = self.create_publisher(Float32, "/forward_min_range", 10)
        self.decision_pub = self.create_publisher(String, "/avoidance_decision", 50)
        self.status_pub = self.create_publisher(String, "/path_tracker/status", 10)
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
        self.start_pos = None
        self.distance_traveled = 0.0
        self.last_odom_time = None
        self.arrived = False

        self._last_cmd = Twist()
        self._last_status = DRIVING

        self.control_dt = 1.0 / self.config.control_rate_hz
        self.control_timer = self.create_timer(self.control_dt, self.control_loop)

        self.get_logger().info(
            f"path_tracker started: safety_distance={self.config.safety_distance}m "
            f"forward_speed={self.config.forward_speed}m/s "
            f"strafe_speed={self.config.strafe_speed}m/s "
            f"disable_avoidance={self.config.disable_avoidance} "
            f"audio_alert_enabled={self.audio_alert_enabled} "
            f"target_distance={self.target_distance}"
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

        # Straight-line distance from wherever the node started. Stored as
        # plain floats rather than the message's position object, which is
        # reused/overwritten by the middleware.
        pos = msg.pose.pose.position
        if self.start_pos is None:
            self.start_pos = (pos.x, pos.y)
        self.distance_traveled = distance_from_start(
            self.start_pos[0], self.start_pos[1], pos.x, pos.y
        )
        self.last_odom_time = self.get_clock().now()

    def scan_is_stale(self):
        if self.last_scan_time is None:
            return True
        age = (self.get_clock().now() - self.last_scan_time).nanoseconds / 1e9
        return age > self.scan_timeout

    def odom_is_stale(self):
        if self.last_odom_time is None:
            return True
        age = (self.get_clock().now() - self.last_odom_time).nanoseconds / 1e9
        return age > self.odom_timeout_sec

    def control_loop(self):
        if self.scan_is_stale() or self._sectors is None:
            # No fresh obstacle data -> hold still rather than drive blind.
            self.get_logger().warn(
                "No fresh scan within scan_timeout; holding.",
                throttle_duration_sec=1.0,
            )
            self._publish(Twist(), DRIVING)
            return

        # Two stops are decided without the state machine. Ticking it
        # through them would keep advancing its maneuver timers against
        # motion that is not happening. Note HALT is NOT one of them -- it
        # is recoverable, and needs stepping to notice the path has cleared.
        if should_skip_controller(
            target_distance=self.target_distance,
            odom_stale=self.odom_is_stale(),
            already_arrived=self.arrived,
        ):
            if self.arrived:
                self._publish(Twist(), ARRIVED_TARGET)
            else:
                self.get_logger().warn(
                    "No fresh odom within odom_timeout_sec but target_distance "
                    "is set; stopping rather than driving on a stale distance.",
                    throttle_duration_sec=1.0,
                )
                self._publish(Twist(), STOPPED_ODOM_FAULT)
            return

        now = self.get_clock().now().nanoseconds / 1e9
        yaw = self.current_yaw if self.current_yaw is not None else 0.0
        out = self.controller.step(self._sectors, self._obstacle, self._gap, yaw, now)

        # Arrival is decided AFTER the controller runs and only gates whether
        # its output is forwarded. The state machine is never bypassed or
        # altered, so avoidance behaves exactly as it did before.
        status = decide_arrival(
            controller_state=self.controller.state,
            target_distance=self.target_distance,
            distance_traveled=self.distance_traveled,
            odom_stale=self.odom_is_stale(),
            already_arrived=self.arrived,
        )

        if status == ARRIVED_TARGET:
            self.arrived = True
            self.get_logger().info(
                f"Target distance {self.target_distance:.2f} m reached "
                f"({self.distance_traveled:.2f} m travelled) -- stopping."
            )
            self._publish(Twist(), status)
            return

        cmd = Twist()
        cmd.linear.x = float(out.linear_x)
        cmd.linear.y = float(out.linear_y)
        cmd.angular.z = float(out.angular_z)
        self._publish(cmd, status)

        if out.decision is not None:
            self.decision_pub.publish(String(data=out.decision.to_json()))
            if self.audio_alert_enabled:
                self._maybe_play_audio_alert(out.decision)

    def _publish(self, cmd, status):
        """Single exit point for every tick, so /cmd_vel and the status topic
        can never disagree about what the robot is doing."""
        self._last_cmd, self._last_status = cmd, status
        self.cmd_vel_pub.publish(cmd)
        self.status_pub.publish(String(data=status))

    def _maybe_play_audio_alert(self, decision):
        if not should_play(decision, self._last_played_encounter_id):
            return
        # Mark as played BEFORE launching so a failed launch doesn't retry
        # within this encounter (retry within one encounter is suppressed by
        # the once-per-encounter gate anyway).
        self._last_played_encounter_id = decision.encounter_id
        # aplay is spawned non-blocking so a slow/failed playback can never
        # stall the control loop. Overlapping playback (two encounters in
        # quick succession) is left unguarded intentionally: cosmetic doubled
        # sound only. Finished aplay processes also aren't reaped -- they
        # linger as <defunct> until this node exits; negligible for a
        # session's worth of obstacles. Both are deliberate prototype
        # trade-offs carried over from the original obstacle_audio node.
        try:
            subprocess.Popen(["aplay", "-D", self.alsa_device, self.wav_path])
        except OSError as e:
            self.get_logger().warn(
                f"Could not launch aplay ({e}); is ALSA's aplay installed in this container?"
            )


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
