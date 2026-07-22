"""Pure, ROS-free obstacle-avoidance state machine.

Governing invariant: always take the maneuver that minimizes deviation from
the goal heading while maintaining safety (strafe -> turn -> bounded recovery
-> halt). Time is injected (`now`, seconds) so this is unit-testable without a
ROS clock; no rclpy/sensor_msgs imports.
"""
import math
import time
from dataclasses import dataclass

from proximity_alert.decision_record import DecisionRecord

DRIVE, ASSESS, STRAFE, TURN, DRIVE_PAST, RECOVER, HALT = (
    "DRIVE", "ASSESS", "STRAFE", "TURN", "DRIVE_PAST", "RECOVER", "HALT")


def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class HeadingPID:
    def __init__(self, kp, ki, kd, output_limit):
        self.kp, self.ki, self.kd, self.output_limit = kp, ki, kd, output_limit
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.prev_error = None

    def update(self, error, dt):
        self.integral += error * dt
        max_i = self.output_limit / self.ki if self.ki else float("inf")
        self.integral = max(-max_i, min(max_i, self.integral))
        deriv = 0.0 if self.prev_error is None else (error - self.prev_error) / dt
        self.prev_error = error
        out = self.kp * error + self.ki * self.integral + self.kd * deriv
        return max(-self.output_limit, min(self.output_limit, out))


@dataclass
class AvoidanceConfig:
    safety_distance: float = 0.30
    clear_margin: float = 0.10
    clear_confirm_time: float = 0.5
    obstacle_detect_range: float = 0.40
    obstacle_confirm_scans: int = 2
    front_arc_deg: float = 180.0
    front_subsector_deg: float = 60.0
    side_window_deg: float = 60.0
    rear_window_deg: float = 60.0
    robot_half_width: float = 0.11
    corridor_margin: float = 0.05
    forward_speed: float = 0.50
    strafe_speed: float = 0.25
    strafe_timeout: float = 1.5
    strafe_side_clearance_min: float = 0.30
    max_obstacle_width: float = 0.50
    max_cumulative_strafe: float = 0.60
    turn_speed: float = 0.6
    turn_step_deg: float = 30.0
    turn_timeout: float = 1.5
    reverse_trigger_range: float = 0.20
    avoid_reverse_speed: float = 0.10
    rear_clearance_min: float = 0.25
    pass_clearance: float = 0.35
    max_drive_past_distance: float = 0.80
    heading_kp: float = 1.0
    heading_ki: float = 0.0
    heading_kd: float = 0.1
    heading_max_correction: float = 0.3
    heading_tol_deg: float = 5.0
    max_avoid_attempts: int = 3
    clear_drive_duration: float = 3.0
    recover_backup_clearance: float = 0.50
    recover_commit_distance: float = 0.50
    min_gap_clearance: float = 0.60
    min_gap_width_deg: float = 40.0
    control_rate_hz: float = 10.0
    disable_avoidance: bool = False

    @property
    def max_strafe_distance(self):
        return self.strafe_speed * self.strafe_timeout

    @property
    def corridor_half(self):
        return self.robot_half_width + self.corridor_margin

    @property
    def clear_threshold(self):
        return self.safety_distance + self.clear_margin


@dataclass
class ControllerOutput:
    linear_x: float
    linear_y: float
    angular_z: float
    state: str
    decision: object  # DecisionRecord | None


class AvoidanceController:
    def __init__(self, config):
        self.config = config
        self.state = DRIVE
        self.goal_heading = None
        self.target_heading = None
        self.consecutive_avoid_count = 0
        self.cumulative_strafe = 0.0
        self.has_recovered_this_encounter = False
        self.encounter_id = 0
        self._in_encounter = False
        self._confirm_count = 0
        self._clean_drive_since = None
        self._locked_dir = 0.0
        self._maneuver_start = None
        self._turn_target = 0.0
        self._drive_past_dist = 0.0
        self._clear_since = None
        self._recover_phase = "orient"
        self._recover_target = 0.0
        self._commit_dist = 0.0
        self._prev_now = None
        self._pid = HeadingPID(config.heading_kp, config.heading_ki,
                               config.heading_kd, config.heading_max_correction)
        # per-tick inputs
        self._s = None
        self._o = None
        self._gap = None
        self._yaw = 0.0
        self._now = 0.0
        self._dt = 1.0 / config.control_rate_hz

    def set_goal_heading(self, yaw):
        self.goal_heading = yaw
        self.target_heading = yaw

    # ---------- tick ----------

    def step(self, sectors, obstacle, gap_bearing, current_yaw, now):
        self._advance_clock(now)
        self._s, self._o, self._gap = sectors, obstacle, gap_bearing
        self._yaw, self._now = current_yaw, now
        handler = {DRIVE: self._drive, STRAFE: self._strafe, TURN: self._turn,
                   DRIVE_PAST: self._drive_past, RECOVER: self._recover,
                   HALT: self._halt}[self.state]
        return handler()

    def _advance_clock(self, now):
        if self._prev_now is None:
            self._dt = 1.0 / self.config.control_rate_hz
        else:
            dt = now - self._prev_now
            self._dt = dt if dt > 0 else 1.0 / self.config.control_rate_hz
        self._prev_now = now

    def _hold(self, target):
        if self.goal_heading is None:
            return 0.0
        return self._pid.update(normalize_angle(target - self._yaw), self._dt)

    def _tol(self):
        return math.radians(self.config.heading_tol_deg)

    # ---------- states ----------

    def _drive(self):
        s, now, cfg = self._s, self._now, self.config
        blocked = self._o is not None and s["front"] <= cfg.safety_distance
        if blocked:
            self._clean_drive_since = None
            self._confirm_count += 1
            if self._confirm_count >= cfg.obstacle_confirm_scans:
                self._confirm_count = 0
                return self._assess()
            return ControllerOutput(0.0, 0.0, 0.0, DRIVE, None)  # confirming
        self._confirm_count = 0
        if self._clean_drive_since is None:
            self._clean_drive_since = now
        elif self._in_encounter and (now - self._clean_drive_since >= cfg.clear_drive_duration):
            self._in_encounter = False
            self.consecutive_avoid_count = 0
            self.cumulative_strafe = 0.0
            self.has_recovered_this_encounter = False
        return ControllerOutput(cfg.forward_speed, 0.0, self._hold(self.goal_heading), DRIVE, None)

    def _assess(self):
        s, o, cfg = self._s, self._o, self.config
        if not self._in_encounter:
            self.encounter_id += 1
            self._in_encounter = True
            self.consecutive_avoid_count = 0
            self.cumulative_strafe = 0.0
            self.has_recovered_this_encounter = False
        self.consecutive_avoid_count += 1

        if cfg.disable_avoidance:
            return self._enter_halt("halt: avoidance_disabled")

        if self.consecutive_avoid_count > cfg.max_avoid_attempts:
            if not self.has_recovered_this_encounter:
                self.has_recovered_this_encounter = True
                return self._enter_recover("recover: attempts_exceeded")
            return self._enter_halt("halt: attempts_exhausted")

        ps = o["preferred_side"]
        # Gate strafing on the obstacle's PHYSICAL lateral width (metres), not
        # its angular span. Angular span grows as the obstacle nears, so at the
        # trigger distance (front <= safety_distance) any real object subtends a
        # large angle -- an angular-span gate is unsatisfiable there and forces
        # TURN every time. Physical width is distance-robust and is what "can I
        # strafe past this vs. is it a wall" actually depends on.
        width = o["y_hi"] - o["y_lo"]
        if ps > 0:
            required = o["y_hi"] + cfg.corridor_half
            side_clear = s["left"]
        else:
            required = cfg.corridor_half - o["y_lo"]
            side_clear = s["right"]
        strafe_ok = (side_clear >= cfg.strafe_side_clearance_min
                     and width <= cfg.max_obstacle_width
                     and required <= cfg.max_strafe_distance
                     and self.cumulative_strafe + cfg.max_strafe_distance <= cfg.max_cumulative_strafe)

        if strafe_ok:
            self._locked_dir = 1.0 if ps > 0 else -1.0
            self._maneuver_start = self._now
            self.state = STRAFE
            rec = self._record(STRAFE, "strafe: narrow+side_clear", required, "committed")
            out = self._strafe()
            out.decision = rec
            return out

        self._locked_dir = 1.0 if s["left"] >= s["right"] else -1.0
        self._maneuver_start = self._now
        self._turn_target = self._yaw + math.radians(cfg.turn_step_deg) * self._locked_dir
        self.state = TURN
        reason = "turn: too_wide" if width > cfg.max_obstacle_width else "turn: side_blocked"
        rec = self._record(TURN, reason, 0.0, "committed")
        out = self._turn()
        out.decision = rec
        return out

    def _strafe(self):
        s, cfg = self._s, self.config
        elapsed = self._now - self._maneuver_start
        if s["front"] >= cfg.clear_threshold:
            return self._complete_to_drive("cleared", elapsed)
        if elapsed >= cfg.strafe_timeout or self.cumulative_strafe >= cfg.max_cumulative_strafe:
            return self._assess()
        self.cumulative_strafe += cfg.strafe_speed * self._dt
        ly = cfg.strafe_speed * self._locked_dir
        return ControllerOutput(0.0, ly, self._hold(self.goal_heading), STRAFE, None)

    def _turn(self):
        s, cfg = self._s, self.config
        elapsed = self._now - self._maneuver_start
        reached = abs(normalize_angle(self._turn_target - self._yaw)) <= self._tol()
        if reached or elapsed >= cfg.turn_timeout:
            self.state = DRIVE_PAST
            self.target_heading = self._yaw
            self._drive_past_dist = 0.0
            self._clear_since = None
            self._maneuver_start = self._now
            return self._drive_past()
        lx = -cfg.avoid_reverse_speed if s["rear"] >= cfg.rear_clearance_min else 0.0
        return ControllerOutput(lx, 0.0, cfg.turn_speed * self._locked_dir, TURN, None)

    def _drive_past(self):
        s, now, cfg = self._s, self._now, self.config
        if s["front"] <= cfg.safety_distance:
            return self._assess()
        self._drive_past_dist += cfg.forward_speed * self._dt
        inside = s["right"] if self._locked_dir > 0 else s["left"]
        if s["front"] >= cfg.clear_threshold and inside >= cfg.pass_clearance:
            if self._clear_since is None:
                self._clear_since = now
            elif now - self._clear_since >= cfg.clear_confirm_time:
                return self._complete_to_drive("cleared", now - self._maneuver_start)
        else:
            self._clear_since = None
        if self._drive_past_dist >= cfg.max_drive_past_distance:
            return self._assess()
        return ControllerOutput(cfg.forward_speed, 0.0, self._hold(self.target_heading), DRIVE_PAST, None)

    def _enter_recover(self, reason):
        self.state = RECOVER
        self._maneuver_start = self._now
        self._recover_phase = "orient"
        rec = self._record(RECOVER, reason, 0.0, "escalated")
        if self._gap is None:
            return ControllerOutput(0.0, 0.0, 0.0, RECOVER, rec)  # halts next tick
        self._recover_target = self._yaw + self._gap
        out = self._recover()
        out.decision = rec
        return out

    def _recover(self):
        s, now, cfg = self._s, self._now, self.config
        if self._gap is None:
            return self._enter_halt("halt: no_gap")
        if self._recover_phase == "orient":
            err = normalize_angle(self._recover_target - self._yaw)
            if abs(err) <= self._tol():
                self._recover_phase = "commit"
                self._commit_dist = 0.0
                self._maneuver_start = now
            else:
                lx = -cfg.avoid_reverse_speed if s["rear"] >= cfg.rear_clearance_min else 0.0
                az = cfg.turn_speed * (1.0 if err > 0 else -1.0)
                return ControllerOutput(lx, 0.0, az, RECOVER, None)
        if s["front"] >= cfg.clear_threshold:
            return self._complete_to_drive("cleared", now - self._maneuver_start)
        self._commit_dist += cfg.forward_speed * self._dt
        if self._commit_dist >= cfg.recover_commit_distance:
            return self._enter_halt("halt: recover_failed")
        return ControllerOutput(cfg.forward_speed, 0.0, self._hold(self.goal_heading), RECOVER, None)

    def _halt(self):
        s, now, cfg = self._s, self._now, self.config
        if s["front"] >= cfg.clear_threshold:
            if self._clear_since is None:
                self._clear_since = now
            elif now - self._clear_since >= cfg.clear_confirm_time:
                return self._complete_to_drive("cleared", 0.0)
        else:
            self._clear_since = None
        return ControllerOutput(0.0, 0.0, 0.0, HALT, None)

    def _enter_halt(self, reason):
        self.state = HALT
        self._clear_since = None
        dur = (self._now - self._maneuver_start) if self._maneuver_start is not None else 0.0
        rec = self._record(HALT, reason, 0.0, "halted", dur)
        return ControllerOutput(0.0, 0.0, 0.0, HALT, rec)

    def _complete_to_drive(self, outcome, duration):
        self.state = DRIVE
        self._clean_drive_since = None
        self._confirm_count = 0
        self._clear_since = None
        self.target_heading = self.goal_heading
        self._pid.reset()
        rec = self._record(DRIVE, outcome, 0.0, outcome, duration)
        return ControllerOutput(self.config.forward_speed, 0.0, self._hold(self.goal_heading), DRIVE, rec)

    # ---------- record ----------

    def _record(self, state, reason, required, outcome, duration=0.0):
        s, o = self._s, self._o
        return DecisionRecord(
            timestamp=time.strftime("%Y-%m-%d %H:%M:%S"),
            encounter_id=self.encounter_id,
            state=state,
            chosen_maneuver=state if state in (STRAFE, TURN, RECOVER, HALT) else "NONE",
            reason=reason,
            obstacle_span_deg=(o["span_deg"] if o else 0.0),
            front_distance_m=s["front"],
            front_left_m=s["front_left"],
            front_center_m=s["front_center"],
            front_right_m=s["front_right"],
            left_clearance_m=s["left"],
            right_clearance_m=s["right"],
            rear_clearance_m=s["rear"],
            required_clearing_m=required,
            cumulative_strafe_m=self.cumulative_strafe,
            consecutive_avoid_count=self.consecutive_avoid_count,
            recovery_triggered=self.has_recovered_this_encounter,
            outcome=outcome,
            maneuver_duration_s=duration,
        )
