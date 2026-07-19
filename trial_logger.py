#!/usr/bin/env python3
"""
trial_logger.py

Companion ROS 2 (Humble) node for the LanderPi surface-material trials.
Runs alongside path_tracker.py WITHOUT modifying it. Watches /odom to
detect when a trial starts (robot begins moving) and ends (robot has
been stopped for stop_confirm_duration seconds, which happens naturally
when path_tracker.py's obstacle-avoidance stop kicks in at point B).

For each trial it records:
    - transit_time_s      : end_time - start_time
    - odom_distance_m     : straight-line distance from start to end
                             odometry position (drift-affected estimate)

Then, at the terminal, it prompts you for:
    - surface material    (granite / concrete / wood / metal)
    - ground_truth_distance_m (read off your tape-measure marks by eye)

...and appends one row to a CSV so Haotian can run stats without any
manual spreadsheet wrangling.

CSV columns:
    timestamp, surface, trial_num, transit_time_s, odom_distance_m,
    ground_truth_distance_m, slippage_error_m, slippage_pct

Topics:
    Subscribes: /odom (nav_msgs/Odometry)

Parameters:
    csv_path                (str,   default "trial_log.csv")
    move_velocity_threshold (float, default 0.03) m/s -- above this = "moving"
    stop_velocity_threshold (float, default 0.02) m/s -- below this = "stopped"
    stop_confirm_duration   (float, default 1.0)  seconds of continuous
                                                    stopped-ness before a
                                                    trial is finalized

Run (in a second terminal, alongside path_tracker.py):
    ros2 run <your_package> trial_logger
    # or directly:
    python3 trial_logger.py --ros-args -p csv_path:=trials/granite.csv
"""

import csv
import math
import os
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class TrialLogger(Node):
    def __init__(self):
        super().__init__("trial_logger")

        self.declare_parameter("csv_path", "trial_log.csv")
        self.declare_parameter("move_velocity_threshold", 0.03)
        self.declare_parameter("stop_velocity_threshold", 0.02)
        self.declare_parameter("stop_confirm_duration", 1.0)

        self.csv_path = self.get_parameter("csv_path").value
        self.move_threshold = self.get_parameter("move_velocity_threshold").value
        self.stop_threshold = self.get_parameter("stop_velocity_threshold").value
        self.stop_confirm_duration = self.get_parameter("stop_confirm_duration").value

        self._ensure_csv_header()

        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )

        # State machine: "idle" -> "moving" -> trial finalized -> "idle"
        self.state = "idle"
        self.start_pos = None
        self.start_time = None
        self.last_pos = None
        self.stopped_since = None
        self.trial_num = self._count_existing_trials()

        # Set by odom_callback when a trial has just finished; consumed by
        # the main loop so we can safely call blocking input() outside of
        # the subscription callback.
        self.pending_trial = None

        self.get_logger().info(
            f"trial_logger up. Writing to '{self.csv_path}'. "
            "Waiting for the robot to start moving to begin a trial."
        )

    # ---------- CSV helpers ----------

    def _ensure_csv_header(self):
        new_file = not os.path.exists(self.csv_path)
        if new_file:
            os.makedirs(os.path.dirname(self.csv_path) or ".", exist_ok=True)
            with open(self.csv_path, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(
                    [
                        "timestamp",
                        "surface",
                        "trial_num",
                        "transit_time_s",
                        "odom_distance_m",
                        "ground_truth_distance_m",
                        "slippage_error_m",
                        "slippage_pct",
                    ]
                )

    def _count_existing_trials(self):
        if not os.path.exists(self.csv_path):
            return 0
        with open(self.csv_path, "r", newline="") as f:
            return max(0, sum(1 for _ in csv.reader(f)) - 1)

    def _append_row(self, surface, transit_time_s, odom_distance_m, ground_truth_m):
        self.trial_num += 1
        error = odom_distance_m - ground_truth_m
        slippage_pct = (error / ground_truth_m * 100.0) if ground_truth_m else 0.0
        with open(self.csv_path, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    time.strftime("%Y-%m-%d %H:%M:%S"),
                    surface,
                    self.trial_num,
                    f"{transit_time_s:.3f}",
                    f"{odom_distance_m:.4f}",
                    f"{ground_truth_m:.4f}",
                    f"{error:.4f}",
                    f"{slippage_pct:.2f}",
                ]
            )
        self.get_logger().info(f"Trial {self.trial_num} logged to {self.csv_path}")

    # ---------- Odometry / state machine ----------

    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        speed = math.hypot(vx, vy)
        now = time.time()

        if self.state == "idle":
            if speed > self.move_threshold:
                self.state = "moving"
                self.start_pos = pos
                self.start_time = now
                self.stopped_since = None
                self.get_logger().info("Trial started (robot began moving).")

        elif self.state == "moving":
            self.last_pos = pos
            if speed < self.stop_threshold:
                if self.stopped_since is None:
                    self.stopped_since = now
                elif now - self.stopped_since >= self.stop_confirm_duration:
                    transit_time_s = self.stopped_since - self.start_time
                    odom_distance_m = math.hypot(
                        self.last_pos.x - self.start_pos.x,
                        self.last_pos.y - self.start_pos.y,
                    )
                    self.pending_trial = (transit_time_s, odom_distance_m)
                    self.state = "idle"
                    self.get_logger().info(
                        f"Trial ended: transit_time={transit_time_s:.2f}s, "
                        f"odom_distance={odom_distance_m:.3f}m. "
                        "Waiting for ground-truth entry..."
                    )
            else:
                self.stopped_since = None


def main(args=None):
    rclpy.init(args=args)
    node = TrialLogger()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.pending_trial is not None:
                transit_time_s, odom_distance_m = node.pending_trial
                node.pending_trial = None
                print(
                    f"\n--- Trial ready: transit_time={transit_time_s:.2f}s, "
                    f"odom_distance={odom_distance_m:.3f}m ---"
                )
                surface = input(
                    "Surface material (granite/concrete/wood/metal): "
                ).strip()
                gt_raw = input(
                    "Ground-truth stop distance from your tape measure (m): "
                ).strip()
                try:
                    ground_truth_m = float(gt_raw)
                except ValueError:
                    node.get_logger().warn(
                        f"Could not parse '{gt_raw}' as a number -- logging as 0.0"
                    )
                    ground_truth_m = 0.0
                node._append_row(
                    surface, transit_time_s, odom_distance_m, ground_truth_m
                )
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
