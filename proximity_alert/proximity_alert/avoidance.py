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


def _distance(pos_a, pos_b):
    return math.hypot(pos_b[0] - pos_a[0], pos_b[1] - pos_a[1])


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
class AvoidanceConfig: # All measurements are in meters or m/s for the speed. All timeouts/time durations are in seconds
    safety_distance: float = 0.20          # FRONT range that triggers a maneuver. Lower = drives closer before reacting; too low risks contact given LiDAR/stop latency
    clear_margin: float = 0.10             # Added to safety_distance for "clear" (clear_threshold). Bigger = more hysteresis before declaring an obstacle passed, less re-trigger flicker
    clear_confirm_time: float = 0.5        # Seconds front must stay clear before DRIVE_PAST/RECOVER/HALT call it cleared. Higher = fewer false "cleared" on a jittery reading
    obstacle_detect_range: float = 0.40    # Range within which a FRONT beam counts toward sizing the obstacle. Bigger = obstacle width/span measured from farther out
    obstacle_confirm_scans: int = 2        # Consecutive blocked scans required before committing to ASSESS. Higher = more debounce against a single noisy reading, slower to react
    front_arc_deg: float = 180.0           # Total width of the forward sensing arc. Wider = sees obstacles further off-center as "front", narrower = only reacts dead ahead
    front_subsector_deg: float = 60.0      # Width of each of the 3 front sub-sectors (left/center/right) used for logging/diagnostics
    side_window_deg: float = 65.0          # Angular width of the LEFT/RIGHT clearance windows (centered at +/-90 deg). Wider = clearance check considers more of the flank
    rear_window_deg: float = 65.0          # Angular width of the REAR clearance window (centered behind). Wider = more of the back considered when reversing/recovering
    robot_half_width: float = 0.085        # Physical half-width of the chassis. Sets the required strafe/pass clearance; too small risks clipping an obstacle, too big blocks strafes that would actually fit
    corridor_margin: float = 0.05          # Extra clearance added beyond robot_half_width (-> corridor_half). Bigger = wider safety buffer on every strafe/pass, but rejects more strafes as "too far"
    forward_speed: float = 0.20            # Constant driving speed, m/s. MUST match the speed actually delivered, not just the value commanded: path_tracker publishes to /cmd_vel, which the vendor's app-control node clamps to +/-0.20 m/s before it ever reaches the motors or /odom -- see README's "Speed clamp" note. Every distance-based timeout below (max_drive_past_distance, recover_commit_distance, cumulative_strafe accounting) integrates this value against real elapsed time, so setting it higher than what's delivered makes those counters overestimate real distance travelled and give up early. Higher = faster runs but less reaction time before safety_distance is reached
    strafe_speed: float = 0.20             # Lateral (sideways) speed during STRAFE, m/s. Same delivered-speed caveat as forward_speed -- this also passes through the /cmd_vel clamp. Higher = clears an obstacle faster but overshoots more before the next scan reacts
    strafe_timeout: float = 2            # Max seconds to hold a STRAFE before giving up and re-assessing. Also sets max_strafe_distance = strafe_speed * strafe_timeout
    strafe_side_clearance_min: float = 0.20  # Minimum LEFT/RIGHT clearance required to permit a strafe that direction. Higher = more conservative, refuses strafes into tight gaps
    max_obstacle_width: float = 0.75      # Max physical lateral width (y_hi - y_lo) still considered "narrow enough to strafe past". Above this it's treated as a wall -> TURN instead
    max_cumulative_strafe: float = 1.25     # Hard cap on total lateral distance strafed within one encounter (guards against creeping sideways along a long wall). Must stay above strafe_speed * strafe_timeout (max_strafe_distance, currently 0.40) or the strafe_ok cap check fails on the very first attempt and STRAFE becomes unreachable. Lower = escalates to TURN sooner, but never below max_strafe_distance
    turn_speed: float = 0.75                # Angular speed while turning, rad/s. Higher = faster turns but more overshoot past turn_step_deg
    turn_step_deg: float = 30.0            # Heading change commanded per TURN attempt. Bigger = clears wider obstacles in one attempt but deviates further from goal heading
    turn_timeout: float = 1.5              # Max seconds to hold a TURN before moving on to DRIVE_PAST regardless of whether turn_step_deg was reached
    reverse_trigger_range: float = 0.20    # (Currently unused by _turn/_recover, which key off rear_clearance_min instead) Intended FRONT range below which a reverse nudge is warranted
    avoid_reverse_speed: float = 0.10      # Speed of the small reverse nudge during TURN/RECOVER when rear is clear. Higher = backs off faster but eats into rear_clearance_min sooner. Ignored when turn_radius > 0 (which derives this instead)
    rear_clearance_min: float = 0.25       # Minimum REAR clearance required to allow the reverse nudge during TURN/RECOVER. Lower = reverses even with less room behind
    turn_radius: float = 0.0               # Reverse-arc radius during TURN/RECOVER, meters. TURN already commands reverse + rotation together, so it traces an arc of radius (reverse_speed / turn_speed) -- at the legacy defaults a tight ~0.13m. Set > 0 to control that geometry directly (reverse speed becomes turn_radius * turn_speed): bigger = a wider, longer sweeping arc instead of an almost-in-place pivot. 0 (default) keeps the legacy fixed avoid_reverse_speed
    rear_taper_zone: float = 0.0           # Distance above rear_clearance_min over which the reverse component fades out linearly instead of snapping to 0, meters. The hard cutoff makes a turn lurch from arc to pure pivot the instant clearance runs low, which reads as the "repetitive little movements" -- a taper degrades smoothly instead. 0 (default) keeps the legacy hard cutoff
    pass_clearance: float = 0.35           # Inside-flank clearance required (alongside FRONT clear) before DRIVE_PAST starts confirming it has cleared the obstacle
    max_drive_past_distance: float = 0.80  # Max distance to drive in DRIVE_PAST before giving up and re-assessing. Higher = more patient with a wide obstacle, but risks driving further off-line
    heading_kp: float = 1.0                # Heading-hold PID proportional gain. Higher = snappier correction toward goal/target heading, more prone to overshoot/oscillation
    heading_ki: float = 0.0                # Heading-hold PID integral gain. Nonzero corrects small steady-state heading bias, but risks windup/overshoot if too high
    heading_kd: float = 0.2               # Heading-hold PID derivative gain. Higher = damps oscillation from kp, but amplifies noise in the heading error, used to tune overshooting
    heading_max_correction: float = 0.3    # Clamp on the PID's angular_z output, rad/s. Lower = gentler heading correction, may not keep up with a large heading error
    heading_tol_deg: float = 5.0           # Heading error considered "on target" (used by TURN/RECOVER completion checks). Smaller = stricter alignment before proceeding, may hunt near the tolerance edge
    # --- Cross-track (return-to-line) correction. Applied as a mecanum crab in DRIVE only; see nav_utils.lateral_correction. Every default below is DERIVED, not guessed -- the formula is given so they can be re-derived rather than fiddled with. All assume forward_speed's actual delivered value of 0.20 m/s above -- RESCALE these together with forward_speed if that ever changes (e.g. once the /cmd_vel clamp is lifted), per the formula in each comment
    cross_track_kp: float = 0.6            # Gain from cross-track error (m) to crab velocity (m/s). Derived from the distance you want re-centering to take, not a time: e_dot = -kp*e settles 95% in 3 time constants, so kp = 3*v/D. At v=0.20 m/s and D=1.0 m of travel, kp = 0.6. RESCALE THIS IF forward_speed CHANGES -- holding kp fixed while raising speed re-centers over a proportionally longer distance. 0.0 disables the correction entirely (exact pre-feature behavior)
    cross_track_max_speed: float = 0.10    # Clamp on the crab velocity, m/s. Derived from the largest crab angle you will accept: v_lat = tan(angle) * forward_speed, and past ~27 deg the LiDAR's forward arc no longer covers the direction the robot is actually travelling. At v=0.20 m/s, 27 deg gives 0.10. Higher = re-centers sooner but drives increasingly sideways-on
    cross_track_deadband: float = 0.03     # Cross-track error below which the correction is exactly zero, m. Set from the smallest offset you can actually measure on the floor (~1 cm ruler-tip) times a small factor -- correcting below your own measurement resolution just chatters. Higher = settles to a wider band around the line and holds it more quietly
    cross_track_ramp_time: float = 0.3     # Seconds to fade the crab in after a maneuver hands back to DRIVE. Derived from the chassis acceleration limit in the platform's own ekf.yaml (1.3 m/s^2): reaching 0.10 m/s needs >= 0.077 s, rounded up for margin. Prevents a step from 0 to full crab in a single tick right as the robot clears an obstacle. 0 = engage at full strength immediately
    cross_track_tolerance: float = 0.05    # How close to the line counts as "on it" for the arrival centering phase, m. After covering target_distance the robot stops driving forward and keeps crabbing until within this band. Larger = accepts a bigger final offset and finishes sooner. 0.0 disables centering, so arrival latches the instant the distance is covered
    centering_timeout: float = 5.0         # Max seconds to spend centering before declaring arrival regardless of remaining offset. Guarantees the run terminates when the correction is gated off or the flank is blocked; the leftover offset is then measured on the floor instead
    max_avoid_attempts: int = 3            # Consecutive failed STRAFE/TURN cycles before escalating to RECOVER. Lower = escalates sooner, higher = keeps retrying the normal ladder longer
    clear_drive_distance: float = 0.3        # Meters of confirmed-clear straight-line displacement before an encounter is considered over and its counters (cumulative_strafe, attempt count) reset. Physically meaningful (unlike the old time-based clear_drive_duration), tunable directly against real obstacle spacing; needs on-hardware validation. Measures straight-line odometric displacement, not integrated path length -- conservative (never closes an encounter early), consistent with distance tracking elsewhere in this project.
    encounter_close_confirm_scans: int = 3   # Consecutive clear scans required before clear_drive_distance even starts accumulating -- mirrors obstacle_confirm_scans on the exit side, so a single noisy clear reading can't start (or falsely advance) the measurement.
    odom_jump_threshold: float = 0.15        # Meters. A per-tick position delta larger than this during an in-progress clear-distance measurement is treated as a discontinuous odometry jump (e.g. a localization reset), not real motion -- the measurement is abandoned and restarts after the next confirmed-clear streak. Intentionally much larger than the robot's max expected per-tick displacement: confirmed hardware speed cap is 0.2 m/s, so even a sluggish 5 Hz control loop only covers ~0.04m/tick -- 0.15m is roughly 4x that, a reasonable buffer for timing jitter and odometry noise without being triggered by normal motion. This assumes a reasonably stable control loop; it does not scale with actual elapsed dt (a stalled loop that legitimately covers more distance in one delayed tick could in principle exceed this), which is an acceptable simplification for this project's scope, not a hidden gap -- a delta-t-aware version (`expected = forward_speed * dt`, jump if `actual > expected + margin`) is a documented future option if long scheduling stalls ever become a real concern. Needs on-hardware validation against real odometry noise.
    recover_backup_clearance: float = 0.50 # (Currently unused by _recover, which keys off rear_clearance_min) Intended rear clearance required before backing up during recovery
    recover_commit_distance: float = 0.50  # Max distance to drive during RECOVER's commit phase before giving up and halting. Higher = more patient attempt to power through the gap
    min_gap_clearance: float = 0.50        # Minimum range a beam must have to count as part of a usable gap for RECOVER. Higher = only wider-open gaps are considered viable
    min_gap_width_deg: float = 40.0        # Minimum angular width a clear run of beams must span to count as a usable gap. Bigger = only wide enough gaps are chosen, small ones ignored
    control_rate_hz: float = 20.0          # Control loop frequency. Higher = finer-grained reaction and PID stepping, but must stay under actual scan/odom publish rate to be meaningful
    disable_avoidance: bool = False        # If true, ASSESS always halts instead of maneuvering (used for clean go-and-stop distance/PID-tuning runs, no turn/strafe)

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
        self._clear_scan_streak = 0
        self._clean_drive_start_pos = None
        self._last_clear_tick_pos = None
        self._pos = None
        self._locked_dir = 0.0
        self._maneuver_start = None
        self._turn_target = 0.0
        self._drive_past_dist = 0.0
        self._clear_since = None
        self._recover_phase = "orient"
        self._recover_target = 0.0
        self._commit_dist = 0.0
        self._lateral_correction = 0.0
        self._drive_since = None
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

    def set_lateral_correction(self, v_lat):
        """Crab velocity requested by path_tracker to return to the A->B line.

        Pushed in every tick BEFORE step(); consulted only while DRIVE is
        actually driving. Deliberately a separate channel from goal_heading:
        the maneuver states (STRAFE/TURN/DRIVE_PAST/RECOVER) must never see
        it, because a line-correction firing mid-strafe would fight the very
        maneuver that is trying to get around the obstacle.

        Defaults to 0.0 and stays there unless something calls this, so a
        controller driven without it -- every test in
        test_avoidance_controller.py, and any caller predating this feature
        -- behaves exactly as before.
        """
        self._lateral_correction = v_lat

    def _gated_lateral(self):
        """The requested crab, suppressed when it would close on the obstacle
        we just avoided.

        A completed STRAFE is the dangerous case, and it is not an edge
        case -- it is the normal one. _strafe commands linear_x = 0.0, so a
        strafe makes NO forward progress: it ends the instant the front arc
        clears, with the robot exactly level with the obstacle and offset by
        the bare minimum that uncovered its front. Crabbing straight back
        then drives into the obstacle's flank. Worse, it is a stable limit
        cycle -- strafe out, front clears, crab back, front blocks, strafe
        out -- so the run hangs rather than merely bumping.

        The gate is asymmetric, because the two directions are not equally
        risky. Moving AWAY from the avoided side can only increase
        clearance, so it is never blocked. Moving TOWARD it requires that
        side's live LiDAR clearance to exceed pass_clearance -- the same
        threshold DRIVE_PAST already uses to decide it is safely past an
        obstacle.

        Gating on measured flank clearance rather than a fixed "drive
        forward N metres first" is what makes this self-timing: it waits
        exactly as long as the obstacle actually needs, which is longer for
        a wall than for a cone, where any fixed N would be wrong for one of
        them. Once the encounter closes (clear_drive_distance of confirmed-clear
        driving) there is no remembered obstacle left to protect, and the
        correction runs unrestricted.
        """
        v = self._lateral_correction
        if v == 0.0 or not self._in_encounter or self._locked_dir == 0.0:
            return v
        # We dodged toward _locked_dir, so the obstacle is on the far side.
        obstacle_is_left = self._locked_dir < 0.0
        closing = (v > 0.0) if obstacle_is_left else (v < 0.0)
        if not closing:
            return v
        flank = self._s["left"] if obstacle_is_left else self._s["right"]
        return v if flank >= self.config.pass_clearance else 0.0

    def _drive_lateral(self):
        """Gated crab, faded in over cross_track_ramp_time after entering DRIVE."""
        v = self._gated_lateral()
        ramp = self.config.cross_track_ramp_time
        if v == 0.0 or ramp <= 0.0 or self._drive_since is None:
            return v
        return v * min(1.0, (self._now - self._drive_since) / ramp)

    # ---------- tick ----------

    def step(self, sectors, obstacle, gap_bearing, current_yaw, current_pos, now):
        self._advance_clock(now)
        self._s, self._o, self._gap = sectors, obstacle, gap_bearing
        self._yaw, self._now, self._pos = current_yaw, now, current_pos
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
        if self._drive_since is None:
            self._drive_since = now
        blocked = self._o is not None and s["front"] <= cfg.safety_distance
        if blocked:
            self._reset_clear_tracking()
            self._confirm_count += 1
            if self._confirm_count >= cfg.obstacle_confirm_scans:
                self._confirm_count = 0
                return self._assess()
            # Confirming a possible obstacle -- hold still, and do NOT crab:
            # the whole point of this tick is to not move while deciding.
            return ControllerOutput(0.0, 0.0, 0.0, DRIVE, None)  # confirming
        self._confirm_count = 0
        self._advance_clear_tracking(self._pos)
        return ControllerOutput(cfg.forward_speed, self._drive_lateral(),
                                self._hold(self.goal_heading), DRIVE, None)

    def _reset_clear_tracking(self):
        self._clear_scan_streak = 0
        self._clean_drive_start_pos = None
        self._last_clear_tick_pos = None

    def _advance_clear_tracking(self, pos):
        if not self._in_encounter or pos is None:
            return
        cfg = self.config
        # Jump detection only matters once a measurement is actually active
        # (clean_drive_start_pos is set) -- checking it during the
        # confirm-scan streak, before there is anything to protect, would
        # just restart the streak-building phase for no benefit.
        if self._clean_drive_start_pos is not None and self._last_clear_tick_pos is not None:
            if _distance(self._last_clear_tick_pos, pos) > cfg.odom_jump_threshold:
                # Discontinuous odometry jump (e.g. a localization reset) --
                # not real motion. Abandon the in-progress measurement rather
                # than compute a meaningless displacement.
                self._reset_clear_tracking()
                self._last_clear_tick_pos = pos
                return
        self._last_clear_tick_pos = pos

        self._clear_scan_streak += 1
        if self._clear_scan_streak < cfg.encounter_close_confirm_scans:
            return
        if self._clean_drive_start_pos is None:
            self._clean_drive_start_pos = pos
            return
        if _distance(self._clean_drive_start_pos, pos) >= cfg.clear_drive_distance:
            self._in_encounter = False
            self.consecutive_avoid_count = 0
            self.cumulative_strafe = 0.0
            self.has_recovered_this_encounter = False
            self._reset_clear_tracking()

    def _assess(self):
        s, o, cfg = self._s, self._o, self.config
        self._drive_since = None   # leaving DRIVE; the crab ramp restarts on return
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

    def _reverse_component(self):
        """Reverse (linear_x) part of the TURN/RECOVER arc, <= 0.

        TURN/RECOVER command reverse and rotation on the same tick, so the
        robot traces an arc of radius reverse_speed / turn_speed rather than
        pivoting in place. turn_radius makes that geometry the thing you set
        (reverse_speed = turn_radius * turn_speed) instead of an emergent
        ratio of two speeds tuned for other reasons; turn_radius = 0 keeps
        the legacy fixed avoid_reverse_speed.

        Backing into something is worse than a tight turn, so the reverse
        component still yields to rear clearance -- but rear_taper_zone lets
        it fade out linearly instead of snapping to 0, which is what made a
        turn lurch from arc to pure pivot mid-maneuver.
        """
        s, cfg = self._s, self.config
        rear = s["rear"]
        if rear < cfg.rear_clearance_min:   # legacy gate was rear >= min
            return 0.0

        speed = (cfg.turn_radius * cfg.turn_speed if cfg.turn_radius > 0.0
                 else cfg.avoid_reverse_speed)
        if cfg.rear_taper_zone > 0.0:
            # rear may be inf (nothing behind at all) -- inf/zone -> inf, and
            # min() clamps it back to full speed, so no special-casing needed.
            speed *= min(1.0, (rear - cfg.rear_clearance_min) / cfg.rear_taper_zone)
        return -speed

    def _turn(self):
        cfg = self.config
        elapsed = self._now - self._maneuver_start
        reached = abs(normalize_angle(self._turn_target - self._yaw)) <= self._tol()
        if reached or elapsed >= cfg.turn_timeout:
            self.state = DRIVE_PAST
            self.target_heading = self._yaw
            self._drive_past_dist = 0.0
            self._clear_since = None
            self._maneuver_start = self._now
            return self._drive_past()
        return ControllerOutput(self._reverse_component(), 0.0,
                                cfg.turn_speed * self._locked_dir, TURN, None)

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
                az = cfg.turn_speed * (1.0 if err > 0 else -1.0)
                return ControllerOutput(self._reverse_component(), 0.0, az, RECOVER, None)
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
        self._reset_clear_tracking()
        self._confirm_count = 0
        self._clear_since = None
        self.target_heading = self.goal_heading
        self._pid.reset()
        # Restart the crab ramp here, not on the next _drive() tick: this is
        # the exact moment the robot is furthest off the line and least
        # clear of the obstacle, so the correction must fade in from zero
        # rather than step to full strength.
        self._drive_since = self._now
        rec = self._record(DRIVE, outcome, 0.0, outcome, duration)
        return ControllerOutput(self.config.forward_speed, self._drive_lateral(),
                                self._hold(self.goal_heading), DRIVE, rec)

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
