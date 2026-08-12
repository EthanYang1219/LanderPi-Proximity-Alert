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
    safety_distance: float = 0.20          # How close something in front can get before the robot reacts. Turn it up and it starts dodging sooner, from farther away; turn it down and it waits, driving closer before doing anything
    clear_margin: float = 0.10             # Extra buffer past safety_distance before the robot calls the obstacle "passed". Turn it up and it waits for more breathing room before deciding it's clear, so it won't immediately flip back to "blocked"
    clear_confirm_time: float = 0.5        # How many seconds the front has to stay clear before it's trusted. Turn it up and one lucky clear reading won't fool it, but it takes longer to declare the coast clear
    obstacle_detect_range: float = 0.40    # How far out a LiDAR point counts toward measuring the obstacle. Turn it up and it sizes up obstacles from farther away
    obstacle_confirm_scans: int = 2        # How many bad readings in a row before it commits to dodging. Turn it up and a single noisy reading won't trigger a dodge, but it reacts a beat slower
    front_arc_deg: float = 180.0           # How wide a cone counts as "in front". Turn it up and it reacts to things off to the side too, not just straight ahead
    front_subsector_deg: float = 60.0      # Width of the left/center/right zones used for logging only -- doesn't change driving behavior
    side_window_deg: float = 70.0          # How wide a slice to each side it checks before strafing that way. Turn it up and it checks more space before trusting a strafe is safe
    rear_window_deg: float = 100.0         # Same as side_window_deg but behind the robot, used when backing up
    robot_half_width: float = 0.085        # Half the robot's actual width -- how much room it assumes it needs. Keep this matched to the real chassis; too small risks clipping things, too big blocks strafes that would've fit
    corridor_margin: float = 0.025          # Extra padding added on top of robot_half_width. Turn it up and the robot demands more elbow room before attempting a strafe, refusing tighter gaps it could've fit through
    forward_speed: float = 0.20            # Normal driving speed. Turn it up to drive faster -- but this MUST match what the robot actually delivers (the vendor controller caps real speed at 0.20 m/s), or every distance/timeout number below starts lying to itself
    strafe_speed: float = 0.20             # Sideways speed while dodging. Turn it up and it clears an obstacle quicker but overshoots more before it can double-check
    strafe_timeout: float = 2.0            # How long it'll keep sliding sideways before giving up and re-checking the situation
    strafe_side_clearance_min: float = 0.20  # How much open space to the side is required before it's allowed to strafe that way. Turn it up and it plays it safer, refusing tight gaps
    max_obstacle_width: float = 0.75      # How wide something can be before the robot gives up trying to slide past it and turns around it instead
    max_cumulative_strafe: float = 1.25     # Total sideways distance it'll allow itself within one dodge before giving up on strafing and trying something else
    turn_speed: float = 0.50                # How fast it spins while turning. Turn it up and it turns faster, but overshoots past the intended angle more
    turn_step_deg: float = 35.0            # How many degrees it turns per attempt. Turn it up and it clears wider obstacles in one go, but drifts further off its original heading
    turn_timeout: float = 2.0              # How long it'll hold a turn before moving on regardless of whether it finished turning
    # reverse_trigger_range: float = 0.20    # (Not currently used -- rear_clearance_min governs this instead)
    avoid_reverse_speed: float = 0.10      # Speed of the small backup nudge while turning/recovering, when there's room behind. Turn it up and it backs off faster, but eats into its rear safety margin sooner
    rear_clearance_min: float = 0.10       # How much room is required behind before it's allowed to back up. Turn it down and it'll reverse with less space behind it
    turn_radius: float = 0.15               # Shape of the backup-while-turning arc. 0 = a tight near-in-place pivot; turn it up for a wider, longer sweeping arc instead
    rear_taper_zone: float = 0.15           # Smooths out the backup motion as rear space runs low, instead of cutting it off abruptly. 0 = hard cutoff, which can look like a little stutter; turn it up to fade out smoothly instead
    pass_clearance: float = 0.40           # Room required on the near side before the robot considers itself past the obstacle
    max_drive_past_distance: float = 0.80  # How far it'll drive past an obstacle before giving up and re-checking. Turn it up to be more patient with wide obstacles, at the cost of drifting further off-line
    heading_kp: float = 5.0               # How hard it steers back toward the target direction. Turn it up and it corrects faster, but too high makes it swing back and forth instead of settling
    heading_ki: float = 0.05                # Corrects small lingering aim error that heading_kp alone won't fully clear. Turn it up to squeeze out drift more aggressively, but too high causes overshoot after a turn
    heading_kd: float = 0.2               # Smooths out the correction from heading_kp so it doesn't overshoot. Turn it up to reduce wobble, but too high makes it jumpy since it reacts to sensor noise
    heading_max_correction: float = 0.3    # Caps how hard it's allowed to steer at once. Turn it down for gentler, calmer corrections, but it may not keep up with a big aim error
    heading_tol_deg: float = 5.0           # How close to the target heading counts as "good enough". Turn it down for stricter aim, but too strict makes it hunt back and forth right at the edge
    # --- Cross-track (return-to-line) correction: a sideways "crab" that nudges the robot back onto its straight path while driving. All the defaults below assume the robot is actually moving at 0.20 m/s (forward_speed above) -- if that changes, these should be re-tuned together with it
    cross_track_kp: float = 5.0           # How hard it crabs sideways back onto the line. Turn it up and it snaps back to the line faster, but too high makes it swerve side to side instead of settling. 0 turns this off entirely
    cross_track_max_speed: float = 0.10    # Top sideways speed while re-centering. Turn it up and it re-centers quicker, but it also drives more sideways-on, which can turn the LiDAR away from what's actually ahead
    cross_track_deadband: float = 0.01     # Below this much sideways drift, it won't bother correcting. Turn it up and it tolerates being off-line by more before reacting, so it holds steadier instead of constantly fidgeting
    cross_track_ramp_time: float = 0.01     # How long it takes to ease into full sideways correction right after a dodge. Turn it up for a gentler, gradual start; 0 means it engages at full strength immediately
    cross_track_tolerance: float = 0.05    # How close to the line counts as "centered" once the run's target distance is reached. Turn it up and it accepts a bigger final offset and finishes sooner
    centering_timeout: float = 7.5         # Max time it'll spend trying to center itself at the finish before giving up and stopping anyway
    max_avoid_attempts: int = 3            # How many failed dodge attempts in a row before it tries a bigger recovery maneuver instead
    clear_drive_distance: float = 0.20        # How far it has to drive with a confirmed-clear path before it considers an obstacle fully behind it and resets its dodge counters
    encounter_close_confirm_scans: int = 3   # How many clear readings in a row before it starts counting toward clear_drive_distance, so one noisy clear reading can't fake it out
    odom_jump_threshold: float = 0.15        # If the robot's tracked position jumps more than this in a single tick, it's treated as bad position data (not real motion) and ignored
    # recover_backup_clearance: float = 0.50 # (Not currently used -- rear_clearance_min governs this instead)
    recover_commit_distance: float = 0.50  # How far it'll drive during a recovery attempt before giving up. Turn it up to be more patient trying to power through a gap
    min_gap_clearance: float = 0.50        # How much open space a gap needs to have before recovery considers it usable. Turn it up to only trust more clearly open gaps
    min_gap_width_deg: float = 40.0        # How wide (in degrees) a gap needs to be before recovery considers it usable. Turn it up to only accept wider gaps
    control_rate_hz: float = 20.0          # How many times per second the control loop updates. Turn it up to react more often, but it's pointless past the sensor's own update rate
    disable_avoidance: bool = False        # If on, obstacles just make the robot stop -- no turning or strafing at all. Handy for plain drive-and-stop test runs

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
        self._maneuver_abort_count = 0
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

        # Try BOTH directions, preferred first. size_obstacle picks
        # preferred_side purely from far-field openness (which side has more
        # room BEYOND the obstacle) -- it never considers how far you would have
        # to travel sideways to get past the obstacle's edge. Those two can
        # disagree badly: an obstacle sitting mostly to one side of the robot is
        # cheap to clear on its near edge and expensive on its far edge, while
        # the open space is usually beyond the FAR edge. Checking only the
        # preferred side then rejects a perfectly good strafe and drops to TURN.
        # Every one of the six TURN commits in the 2026-08-05 five-obstacle run
        # was this case: preferred side needed 0.26-0.52m (limit 0.40) while the
        # opposite side needed -0.06 to 0.22m with 0.44-0.65m of clearance.
        for direction in (ps, -ps):
            if direction > 0:
                required = o["y_hi"] + cfg.corridor_half
                side_clear = s["left"]
            else:
                required = cfg.corridor_half - o["y_lo"]
                side_clear = s["right"]
            # An obstacle entirely on the far side needs no lateral travel at
            # all; clamp so the logged required_clearing_m stays physical.
            required = max(0.0, required)
            strafe_ok = (side_clear >= cfg.strafe_side_clearance_min
                         and width <= cfg.max_obstacle_width
                         and required <= cfg.max_strafe_distance
                         and self.cumulative_strafe + cfg.max_strafe_distance <= cfg.max_cumulative_strafe)
            if strafe_ok:
                self._locked_dir = 1.0 if direction > 0 else -1.0
                self._maneuver_start = self._now
                self._maneuver_abort_count = 0
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
        # DRIVE asks "is there an obstacle?"; a maneuver asks "is the maneuver I
        # chose still valid?" -- different questions. We already KNOW the front is
        # blocked; that is why we are strafing, and front <= safety_distance stays
        # true for most of a healthy strafe. So front is deliberately NOT checked
        # here: doing so would abort every strafe ~1 tick after it started. What is
        # genuinely new information is whether the gap we committed to using has
        # closed off -- three obstacle contacts on 2026-08-03/04 came from exactly
        # that going unnoticed (flank 0.369m -> 0.150m mid-strafe, unwatched).
        #
        # Known limitation, out of scope here: obstacle_confirm_scans debounces
        # CONTROL TICKS, not distinct LiDAR scans. control_rate_hz (20) exceeds the
        # LD19's ~10Hz publish rate, so two consecutive "ticks" may be the same scan
        # counted twice. Affects every debounce in this controller; a future change
        # should key debounce off new-scan arrival instead of tick count.
        flank = s["left"] if self._locked_dir > 0 else s["right"]
        if flank <= cfg.strafe_side_clearance_min:
            self._maneuver_abort_count += 1
            if self._maneuver_abort_count >= cfg.obstacle_confirm_scans:
                return self._assess()
        else:
            self._maneuver_abort_count = 0
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
