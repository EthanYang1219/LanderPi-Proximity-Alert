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

It also watches /cmd_vel during the trial to detect whether path_tracker's
obstacle-avoidance state (AVOIDING) ever triggered -- that state is the
only place a negative linear.x is ever commanded, so a reverse command
during the trial window is an unambiguous signal the robot swerved
around something instead of driving a clean A->B line. Trials with
avoidance_events > 0 measure something different (avoidance-affected
slippage) than a clean run and must not be silently pooled with clean
trials in the stats -- see avoidance_events below.

Then, at the terminal, it prompts you for:
    - surface material    (granite / concrete / wood / metal / hpl)
    - ground_truth_distance_m (read off your tape-measure marks by eye)
    - notes                (freeform, optional -- e.g. "motors fought each
                             other on the turn", "oscillated near desk")

...and appends one row to a CSV so Haotian can run stats without any
manual spreadsheet wrangling.

CSV columns:
    timestamp, surface, trial_num, transit_time_s, odom_distance_m,
    ground_truth_distance_m, slippage_error_m, slippage_pct,
    avoidance_events, notes

Topics:
    Subscribes: /odom (nav_msgs/Odometry)
    Subscribes: /cmd_vel (geometry_msgs/Twist)

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
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32


CSV_HEADER = [
    "timestamp",
    "surface",
    "trial_num",
    "transit_time_s",
    "odom_distance_m",
    "ground_truth_distance_m",
    "slippage_error_m",
    "slippage_pct",
    "avoidance_events",
    "lidar_stop_range_m",
    "notes",
]


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
        self.cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, 10
        )
        self.range_sub = self.create_subscription(
            Float32, "/forward_min_range", self.range_callback, 10
        )
        # Latest forward-arc LiDAR range from path_tracker; captured at the
        # moment a trial finalizes to record the actual stop distance.
        self.last_min_range = None

        # State machine: "idle" -> "moving" -> trial finalized -> "idle"
        self.state = "idle"
        self.start_pos = None
        self.start_time = None
        self.last_pos = None
        self.stopped_since = None
        self.trial_num = self._count_existing_trials()

        # Avoidance detection: path_tracker only ever commands a negative
        # linear.x while in its AVOIDING state, so a reverse command seen
        # during "moving" is an unambiguous obstacle-avoidance signal.
        # was_reversing tracks edge transitions so multiple ticks of the
        # same maneuver count as one event, not one per control-loop tick.
        self.avoidance_events = 0
        self.was_reversing = False

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
        if not os.path.exists(self.csv_path):
            os.makedirs(os.path.dirname(self.csv_path) or ".", exist_ok=True)
            with open(self.csv_path, "w", newline="") as f:
                csv.writer(f).writerow(CSV_HEADER)
            return

        with open(self.csv_path, newline="") as f:
            rows = list(csv.reader(f))
        if not rows or rows[0] == CSV_HEADER:
            return

        # Stale header from before a schema addition (e.g. avoidance_events,
        # lidar_stop_range_m, notes were appended later). New columns are only
        # ever appended at the end, so old data rows are a strict prefix of
        # the canonical header -- right-pad them rather than fabricate values,
        # and rewrite the file with the canonical header so every row lines
        # up under it (a stale short header against wider new rows silently
        # misaligns or drops columns for any strict CSV/pandas reader).
        self.get_logger().warn(
            f"'{self.csv_path}' has a stale CSV header "
            f"({len(rows[0])} cols vs canonical {len(CSV_HEADER)}); migrating."
        )
        migrated = [row + [""] * (len(CSV_HEADER) - len(row)) for row in rows[1:]]
        with open(self.csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(CSV_HEADER)
            writer.writerows(migrated)

    def _count_existing_trials(self):
        if not os.path.exists(self.csv_path):
            return 0
        with open(self.csv_path, "r", newline="") as f:
            return max(0, sum(1 for _ in csv.reader(f)) - 1)

    def _append_row(
        self,
        surface,
        transit_time_s,
        odom_distance_m,
        ground_truth_m,
        avoidance_events,
        lidar_stop_range_m,
        notes,
    ):
        self.trial_num += 1
        error = odom_distance_m - ground_truth_m
        slippage_pct = (error / ground_truth_m * 100.0) if ground_truth_m else 0.0
        # inf (nothing in the forward arc at stop) has no meaningful distance;
        # log it blank rather than the literal "inf".
        range_str = (
            f"{lidar_stop_range_m:.4f}"
            if lidar_stop_range_m is not None and math.isfinite(lidar_stop_range_m)
            else ""
        )
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
                    avoidance_events,
                    range_str,
                    notes,
                ]
            )
        if avoidance_events:
            self.get_logger().warn(
                f"Trial {self.trial_num} logged with {avoidance_events} "
                "avoidance event(s) -- this run is NOT a clean A->B line, "
                "flag it before pooling with clean-run slippage stats."
            )
        else:
            self.get_logger().info(f"Trial {self.trial_num} logged to {self.csv_path}")

    # ---------- Odometry / state machine ----------

    def cmd_vel_callback(self, msg: Twist):
        if self.state != "moving":
            self.was_reversing = False
            return
        is_reversing = msg.linear.x < 0.0
        if is_reversing and not self.was_reversing:
            self.avoidance_events += 1
        self.was_reversing = is_reversing

    def range_callback(self, msg: Float32):
        self.last_min_range = msg.data

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
                self.avoidance_events = 0
                self.was_reversing = False
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
                    self.pending_trial = (
                        transit_time_s,
                        odom_distance_m,
                        self.avoidance_events,
                        self.last_min_range,
                    )
                    self.state = "idle"
                    avoid_note = (
                        f" ({self.avoidance_events} avoidance event(s) -- "
                        "not a clean run)"
                        if self.avoidance_events
                        else ""
                    )
                    self.get_logger().info(
                        f"Trial ended: transit_time={transit_time_s:.2f}s, "
                        f"odom_distance={odom_distance_m:.3f}m{avoid_note}. "
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
                (
                    transit_time_s,
                    odom_distance_m,
                    avoidance_events,
                    lidar_stop_range_m,
                ) = node.pending_trial
                node.pending_trial = None
                avoid_note = (
                    f", avoidance_events={avoidance_events} (NOT a clean run)"
                    if avoidance_events
                    else ""
                )
                print(
                    f"\n--- Trial ready: transit_time={transit_time_s:.2f}s, "
                    f"odom_distance={odom_distance_m:.3f}m{avoid_note} ---"
                )
                surface = input(
                    "Surface material (granite/concrete/wood/metal/hpl): "
                ).strip()
                # Re-prompt until a valid positive number, or let the user
                # discard the trial. Never write a bogus ground-truth value:
                # a single wrong row silently corrupts the slippage dataset.
                ground_truth_m = None
                while ground_truth_m is None:
                    gt_raw = input(
                        "Ground-truth stop distance in m "
                        "(or 's' to skip/discard this trial): "
                    ).strip()
                    if gt_raw.lower() in ("s", "skip"):
                        node.get_logger().warn("Trial discarded, not logged.")
                        break
                    try:
                        value = float(gt_raw)
                    except ValueError:
                        print(f"  '{gt_raw}' is not a number -- try again.")
                        continue
                    if value <= 0.0:
                        print("  Distance must be positive -- try again.")
                        continue
                    ground_truth_m = value

                if ground_truth_m is not None:
                    notes = input(
                        "Notes -- anything unusual? e.g. motor conflict, "
                        "oscillation, false stop (blank if none): "
                    ).strip()
                    node._append_row(
                        surface,
                        transit_time_s,
                        odom_distance_m,
                        ground_truth_m,
                        avoidance_events,
                        lidar_stop_range_m,
                        notes,
                    )
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
