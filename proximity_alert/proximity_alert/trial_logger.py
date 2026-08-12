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

It also watches /avoidance_decision during the trial and counts how many
distinct avoidance encounters the controller actually committed to, which is
an unambiguous signal the robot swerved around something instead of driving
a clean A->B line. (It does NOT infer this from /cmd_vel: a STRAFE commands
linear.x == 0.0, so reverse-detection missed strafe-only encounters entirely,
and a crab correction is indistinguishable from a strafe on /cmd_vel.)
Trials with
avoidance_events > 0 measure something different (avoidance-affected
slippage) than a clean run and must not be silently pooled with clean
trials in the stats -- see avoidance_events below.

Surface material is inferred automatically from the CSV filename (e.g.
csv_path=".../granite.csv" -> surface="granite") -- override with the
`surface` parameter if the filename doesn't match (e.g. a shared/misc log).

Then, at the terminal, it prompts you for:
    - ground_truth_distance_m        (read off your tape-measure marks by eye)
    - ground_truth_lateral_offset_m  (tape-measured lateral offset from the
                                       A->B line, blank if not measured;
                                       + = right, - = left)
    - notes                (freeform, optional -- e.g. "motors fought each
                             other on the turn", "oscillated near desk")

...and appends one row to a CSV so Haotian can run stats without any
manual spreadsheet wrangling.

CSV columns:
    timestamp, surface, trial_num, transit_time_s, odom_distance_m,
    ground_truth_distance_m, slippage_error_m, slippage_pct,
    avoidance_events, lidar_stop_range_m, notes, battery_level,
    ground_truth_lateral_offset_m, obstacle_count, layout_id, outcome, cause

The last four columns (obstacle_count, layout_id, outcome, cause) are only
ever populated when track_obstacle_outcome:=true (see Parameters below) --
plain surface trials (granite.csv etc.) leave them blank and are prompted
exactly as before. layout_id is a short label you assign per obstacle
arrangement (e.g. "layout_A"); the actual obstacle geometry for that ID
lives in your own separate layout sheet, not in this CSV.

Topics:
    Subscribes: /odom (nav_msgs/Odometry)
    Subscribes: /avoidance_decision (std_msgs/String)
    Subscribes: /ros_robot_controller/battery (std_msgs/UInt16, raw mV) --
                converted to a rough High/Medium/Low estimate assuming a 2S
                Li-ion pack (6.0V empty - 8.4V full); not a precise SoC.

Parameters:
    csv_path                (str,   default "trial_log.csv")
    surface                 (str,   default "" -- inferred from csv_path's
                                     filename, e.g. "granite.csv" -> "granite";
                                     set explicitly to override)
    move_velocity_threshold (float, default 0.03) m/s -- above this = "moving"
    stop_velocity_threshold (float, default 0.02) m/s -- below this = "stopped"
    stop_confirm_duration   (float, default 1.0)  seconds of continuous
                                                    stopped-ness before a
                                                    trial is finalized
    track_obstacle_outcome  (bool,  default False) -- when true, also prompts
                                                    for obstacle_count,
                                                    layout_id, outcome
                                                    (success/failure) and
                                                    cause per trial. Use for
                                                    csv_path=avoidance_trials.csv;
                                                    leave false for plain
                                                    surface trials.

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
from std_msgs.msg import Float32, String, UInt16

from proximity_alert.decision_record import DecisionRecord


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
    "battery_level",
    "ground_truth_lateral_offset_m",
    "obstacle_count",
    "layout_id",
    "outcome",
    "cause",
]

# /ros_robot_controller/battery publishes raw millivolts (std_msgs/UInt16),
# not a percentage. This robot's pack is a 2S Li-ion (nominal 7.4V), so the
# range below is a rough estimate, not a manufacturer-specified curve --
# good enough for a High/Medium/Low bucket, not for precise SoC.
BATTERY_MIN_MV = 6000
BATTERY_MAX_MV = 8400

# Low/Medium boundary confirmed against real hardware: the robot's own
# low-battery beep was observed at 6952 mV, which the original 6800 mV cutoff
# missed (classified it as "Medium"). Raised past the observed beep point so
# that reading -- and anything at or below it -- lands in "Low".
BATTERY_LOW_MAX_MV = 7100
BATTERY_MEDIUM_MAX_MV = 7600


def battery_percent(mv):
    pct = (mv - BATTERY_MIN_MV) / (BATTERY_MAX_MV - BATTERY_MIN_MV) * 100.0
    return max(0.0, min(100.0, pct))


def battery_level(mv):
    if mv < BATTERY_LOW_MAX_MV:
        return "Low"
    elif mv < BATTERY_MEDIUM_MAX_MV:
        return "Medium"
    return "High"


class TrialLogger(Node):
    def __init__(self):
        super().__init__("trial_logger")

        self.declare_parameter("csv_path", "trial_log.csv")
        self.declare_parameter("surface", "")
        self.declare_parameter("move_velocity_threshold", 0.03)
        self.declare_parameter("stop_velocity_threshold", 0.02)
        self.declare_parameter("stop_confirm_duration", 1.0)
        self.declare_parameter("track_obstacle_outcome", False)

        self.csv_path = self.get_parameter("csv_path").value
        self.move_threshold = self.get_parameter("move_velocity_threshold").value
        self.stop_threshold = self.get_parameter("stop_velocity_threshold").value
        self.stop_confirm_duration = self.get_parameter("stop_confirm_duration").value
        self.track_obstacle_outcome = self.get_parameter("track_obstacle_outcome").value

        # Surface defaults to the CSV filename (e.g. ".../granite.csv" ->
        # "granite") -- one CSV per surface is already this project's
        # convention, so the filename already says what's being logged;
        # override with the `surface` param if a file doesn't follow it.
        surface_param = self.get_parameter("surface").value
        self.surface = surface_param or os.path.splitext(
            os.path.basename(self.csv_path)
        )[0]

        self._ensure_csv_header()

        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )
        self.decision_sub = self.create_subscription(
            String, "/avoidance_decision", self.decision_callback, 10
        )
        self.range_sub = self.create_subscription(
            Float32, "/forward_min_range", self.range_callback, 10
        )
        # Latest forward-arc LiDAR range from path_tracker; captured at the
        # moment a trial finalizes to record the actual stop distance.
        self.last_min_range = None

        self.battery_sub = self.create_subscription(
            UInt16, "/ros_robot_controller/battery", self.battery_callback, 10
        )
        # Latest raw battery reading (millivolts); captured at trial-finalize
        # time, same pattern as last_min_range.
        self.last_battery_mv = None

        # State machine: "idle" -> "moving" -> trial finalized -> "idle"
        self.state = "idle"
        self.start_pos = None
        self.start_time = None
        self.last_pos = None
        self.stopped_since = None
        self.trial_num = self._count_existing_trials()

        # Avoidance detection: counted from the /avoidance_decision stream,
        # NOT from /cmd_vel. The old heuristic keyed on a negative linear.x,
        # on the assumption that path_tracker only reverses while avoiding.
        # That assumption is false: AvoidanceController._strafe commands
        # linear.x == 0.0 (avoidance.py:412), so a strafe-only encounter --
        # the most common kind -- never produced a reverse edge and silently
        # logged avoidance_events=0 despite a real STRAFE in decision_log.csv.
        # cmd_vel cannot distinguish an avoidance strafe from the cross-track
        # crab correction either (both are linear.y with linear.x >= 0), so
        # the decision stream is the only unambiguous source.
        #
        # Counted as DISTINCT encounter_id values carrying a real maneuver, so
        # multiple decision records within one encounter count once, and a
        # "cleared" DRIVE/NONE record does not count at all.
        self.avoidance_events = 0
        self._maneuver_encounters = set()

        # Set by odom_callback when a trial has just finished; consumed by
        # the main loop so we can safely call blocking input() outside of
        # the subscription callback.
        self.pending_trial = None

        self.get_logger().info(
            f"trial_logger up. Writing to '{self.csv_path}' "
            f"(surface='{self.surface}'). "
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
        battery_level_str,
        lateral_offset_m=None,
        obstacle_count=None,
        layout_id="",
        outcome="",
        cause="",
        trial_end_epoch=None,
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
        # Not every trial measures a lateral offset (only offset trials do) --
        # blank rather than fabricate a 0.0 that would misread as "measured
        # and on-line".
        lateral_str = (
            f"{lateral_offset_m:.4f}" if lateral_offset_m is not None else ""
        )
        # Blank rather than "0", "None" or a fabricated value -- these four
        # columns are only ever populated for track_obstacle_outcome runs;
        # a plain surface trial must leave them genuinely empty, not "0".
        obstacle_count_str = (
            str(obstacle_count) if obstacle_count is not None else ""
        )
        # The moment the ROBOT stopped, not the moment this row was written.
        # Writing happens only after the operator answers every interactive
        # prompt (~9 of them under track_obstacle_outcome), observed at 3-4
        # minutes -- stamping write time made rows impossible to correlate
        # against decision_log.csv or scan_trace.jsonl by time.
        stamp = time.strftime(
            "%Y-%m-%d %H:%M:%S",
            time.localtime(trial_end_epoch) if trial_end_epoch is not None
            else time.localtime(),
        )
        with open(self.csv_path, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    stamp,
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
                    battery_level_str,
                    lateral_str,
                    obstacle_count_str,
                    layout_id,
                    outcome,
                    cause,
                ]
            )
        if avoidance_events:
            self.get_logger().warn(
                f"Trial {self.trial_num} logged with {avoidance_events} "
                "avoidance event(s)"
            )
        else:
            self.get_logger().info(f"Trial {self.trial_num} logged to {self.csv_path}")

    # ---------- Odometry / state machine ----------

    def _reset_avoidance_tracking(self):
        self.avoidance_events = 0
        self._maneuver_encounters = set()

    def decision_callback(self, msg: String):
        """Count distinct avoidance encounters seen during a trial.

        A malformed or schema-drifted message must never take the logger down
        mid-trial -- the trial's own measurements stay valid, we just cannot
        attribute this one record, so it is logged and skipped (same policy as
        decision_logger).
        """
        if self.state != "moving":
            return
        try:
            rec = DecisionRecord.from_json(msg.data)
        except Exception as exc:  # noqa: BLE001 - never fatal to a live trial
            self.get_logger().warning(f"Unparseable decision record skipped: {exc}")
            return
        if rec.chosen_maneuver in ("", "NONE", None):
            return
        self._maneuver_encounters.add(rec.encounter_id)
        self.avoidance_events = len(self._maneuver_encounters)

    def range_callback(self, msg: Float32):
        self.last_min_range = msg.data

    def battery_callback(self, msg: UInt16):
        self.last_battery_mv = msg.data

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
                self._reset_avoidance_tracking()
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
                        self.last_battery_mv,
                        self.stopped_since,
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
                    battery_mv,
                    trial_end_epoch,
                ) = node.pending_trial
                node.pending_trial = None
                avoid_note = (
                    f", avoidance_events={avoidance_events} (NOT a clean run)"
                    if avoidance_events
                    else ""
                )
                if battery_mv is not None:
                    battery_level_str = battery_level(battery_mv)
                    battery_note = (
                        f", battery=~{battery_percent(battery_mv):.0f}% "
                        f"({battery_level_str})"
                    )
                else:
                    battery_level_str = ""
                    battery_note = ", battery=unknown (no reading yet)"
                print(
                    f"\n--- Trial ready: transit_time={transit_time_s:.2f}s, "
                    f"odom_distance={odom_distance_m:.3f}m{avoid_note}"
                    f"{battery_note} ---"
                )
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
                    # Optional -- only offset trials measure this. Blank means
                    # "not measured", not "measured as zero", so it stays
                    # None rather than being coerced to 0.0.
                    lateral_offset_m = None
                    lateral_entered = False
                    while not lateral_entered:
                        lat_raw = input(
                            "Ground-truth lateral offset in m, + = right / "
                            "- = left (blank if not measured): "
                        ).strip()
                        if lat_raw == "":
                            lateral_entered = True
                            continue
                        try:
                            lateral_offset_m = float(lat_raw)
                        except ValueError:
                            print(f"  '{lat_raw}' is not a number -- try again.")
                            continue
                        lateral_entered = True

                    notes = input(
                        "Notes -- anything unusual? e.g. motor conflict, "
                        "oscillation, false stop (blank if none): "
                    ).strip()

                    obstacle_count = None
                    layout_id = ""
                    outcome = ""
                    cause = ""
                    if node.track_obstacle_outcome:
                        while obstacle_count is None:
                            oc_raw = input(
                                "Obstacle count for this trial: "
                            ).strip()
                            try:
                                value = int(oc_raw)
                            except ValueError:
                                print(f"  '{oc_raw}' is not an integer -- try again.")
                                continue
                            if value < 0:
                                print("  Obstacle count must be >= 0 -- try again.")
                                continue
                            obstacle_count = value

                        layout_id = input(
                            "Layout ID for this obstacle arrangement "
                            "(e.g. 'layout_A', blank if untracked): "
                        ).strip()

                        while outcome not in ("success", "failure"):
                            outcome = input(
                                "Outcome -- 'success' or 'failure': "
                            ).strip().lower()
                            if outcome not in ("success", "failure"):
                                print("  Enter exactly 'success' or 'failure'.")

                        cause = input(
                            f"Cause of {outcome} -- freeform "
                            "(blank if none): "
                        ).strip()

                    node._append_row(
                        node.surface,
                        transit_time_s,
                        odom_distance_m,
                        ground_truth_m,
                        avoidance_events,
                        lidar_stop_range_m,
                        notes,
                        battery_level_str,
                        lateral_offset_m,
                        obstacle_count,
                        layout_id,
                        outcome,
                        cause,
                        trial_end_epoch,
                    )
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
