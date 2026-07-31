"""Pure, ROS-free arrival and path-return logic for the fixed-distance A-to-B run.

Kept separate from path_tracker (and from the AvoidanceController it wraps)
so the priority rules are unit-testable without a live node, following the
same pattern as scan_utils.py.

SCOPE LIMIT -- read before trusting any number out of this module.
The cross-track functions here describe the robot's position relative to the
A->B line using /odom, and on this platform /odom's x/y is an OPEN-LOOP
integral of COMMANDED velocity:

  * there are no wheel encoders;
  * the EKF's only configured exteroceptive x/y source (odom1: odom_rf2o,
    laser odometry) is NOT running -- 0 publishers;
  * so odom0 contributes velocities only, and position is dead reckoning.

Consequence: the correction built on these functions undoes lateral
displacement the robot COMMANDED (which is what a strafe-based avoidance
maneuver produces, and is the bug this was written for). It CANNOT see
wheel slip, or dead-reckoning drift -- if a strafe slips, odom still reports
a perfect strafe and the correction will happily "re-center" onto a line
that has itself drifted. Yaw is genuinely EKF-fused (IMU) and is trustworthy;
position is not. True path following needs an external position source --
start the rf2o laser-odometry node, or equivalent.
"""
import math

from proximity_alert.avoidance import DRIVE, HALT

DRIVING = "driving"
CENTERING = "centering"
ARRIVED_TARGET = "arrived_target_distance"
ARRIVED_OBSTACLE = "arrived_obstacle"
STOPPED_ODOM_FAULT = "stopped_odom_fault"
STOPPED_SCAN_FAULT = "stopped_scan_fault"


def distance_from_start(start_x, start_y, current_x, current_y):
    """Straight-line displacement from the start pose.

    Deliberately the same math.hypot(dx, dy) trial_logger uses for
    odom_distance_m, so the two nodes agree on "distance travelled".

    The FORMULA agrees but the two nodes' ORIGINS do not: trial_logger
    latches its start position when motion is first detected, while
    path_tracker latches it on the first /odom message after node start.
    Don't assume the two numbers are interchangeable -- they can differ by
    however far the robot moved (or was moved) between node launch and the
    trial actually starting.

    Note this is DISPLACEMENT, not path length -- a sideways strafe counts,
    and driving backwards increases it. With heading hold engaged the two
    are close, but they diverge after an avoidance maneuver.

    NOT used for the arrival decision -- see along_track_distance, which is
    the projection onto the A->B line and is what target_distance is
    measured against. This function remains because trial_logger's
    odom_distance_m column is defined as this same displacement.
    """
    return math.hypot(current_x - start_x, current_y - start_y)


def along_track_distance(start_x, start_y, path_heading, current_x, current_y):
    """Progress ALONG the A->B line, in meters -- the projection of the
    displacement onto the line's own direction.

    This, not distance_from_start, is what target_distance is compared
    against. The difference matters exactly when the robot is off the line:
    distance_from_start is a RADIUS, so lateral offset inflates it and the
    run stops EARLY. A robot 1.20 m down the line but 0.40 m off to the side
    reads hypot = 1.26 m -- it would call a 1.25 m target reached having
    actually advanced only 1.20 m. The projection reports 1.20 m regardless
    of how far off to the side the robot is.

    Decreases if the robot backs up past the start, and can go negative --
    that is intentional, it is signed progress, not a magnitude.
    """
    dx = current_x - start_x
    dy = current_y - start_y
    return dx * math.cos(path_heading) + dy * math.sin(path_heading)


def cross_track_error(start_x, start_y, path_heading, current_x, current_y):
    """Signed perpendicular offset from the A->B line, in meters.

    POSITIVE means the robot has drifted to the LEFT of the line, negative
    to the right, zero means exactly on it. Left-positive matches every
    other side convention in this codebase (a positive Twist.linear.y is a
    leftward strafe; size_obstacle's preferred_side > 0 means left), so the
    sign can be used to pick a strafe direction without a flip.

    The companion of along_track_distance: together they are just the
    displacement vector rewritten in the path's own frame (along, across).

    See this module's docstring for why this number reflects COMMANDED
    displacement only and cannot observe slip.
    """
    dx = current_x - start_x
    dy = current_y - start_y
    return -dx * math.sin(path_heading) + dy * math.cos(path_heading)


def lateral_correction(cross_track_err, kp, max_speed, deadband, ramp_scale=1.0):
    """Sideways velocity (m/s, +ve = left) that crabs the robot back onto the
    A->B line. Feeds Twist.linear.y directly.

    This is a crab, not a steer, because the chassis is mecanum: it can move
    sideways directly, and lateral strafe is the one motion this platform
    does cleanly (pure rotation hits the documented vendor kinematics quirk).
    Steering-based cross-track controllers -- Stanley, Pure Pursuit,
    line-of-sight -- exist to solve this problem on car-like bases that
    CANNOT move sideways; all their machinery (lookahead distance, the
    atan(k*e/v) term, curvature limits) is there to convert lateral error
    into a steering angle without going unstable. None of that is needed
    here, and using it would route the correction through the one actuator
    axis this robot is bad at.

    Crabbing also keeps the LiDAR pointed straight down the path, so the
    forward-arc sectors keep meaning what the avoidance state machine was
    tuned to expect -- a heading bias would rotate the sensor frame and
    quietly change which obstacles read as "in front".

    Stability: the closed loop is a plain first-order lag, e_dot = -kp * e,
    which decays exponentially and never overshoots for any kp > 0. There is
    no cascade (steering would add a second integrator, heading -> position,
    which is what makes naive proportional steering oscillate). Saturating
    at max_speed only slows convergence; it cannot destabilize it. Purely
    proportional, recomputed from live position every tick, so there is no
    integral term to wind up and nothing to reset between encounters.

    Below `deadband` the output is exactly zero, so the robot stops fighting
    for the last centimeter and does not chatter around the line.
    """
    if abs(cross_track_err) <= deadband:
        return 0.0
    # Negative sign: drifted left (+err) needs rightward (-y) motion.
    v = -kp * cross_track_err
    return max(-max_speed, min(max_speed, v)) * ramp_scale


def decide_arrival(*, controller_state, target_distance, along_track,
                   odom_stale, already_arrived, cross_track_err=0.0,
                   cross_track_tolerance=0.0, centering_elapsed=0.0,
                   centering_timeout=0.0):
    """Which stop condition (if any) governs this control tick.

    Priority, highest first:

    1. Already arrived -- latched, so the robot stays stopped. Arrival can
       only latch from DRIVE (rule 3), so this can never mask an obstacle
       halt that was already in progress.
    2. Stale odom while target_distance is set -- along_track is derived
       from odom, so stale odom means the distance is untrustworthy.
       Stop rather than drive blind or declare a bogus arrival.
    3. Target reached AND controller is in DRIVE. `along_track` is the
       projection onto the A->B line, NOT straight-line displacement, so a
       laterally-offset robot no longer stops early (see
       along_track_distance). Deferring until DRIVE lets a strafe/turn
       finish, so the robot ends square and on-heading instead of frozen
       sideways mid-maneuver. The overshoot this costs is not always small:
       if a full maneuver ladder runs (strafe, then turn, then drive-past)
       before DRIVE is reached again, it can total roughly a metre
       (max_drive_past_distance plus strafe travel).

       Having covered the distance, the run finishes centering before it
       calls itself arrived: while the robot is still further than
       cross_track_tolerance off the line it reports CENTERING, which stops
       forward motion but keeps crabbing sideways. It gives up and declares
       arrival anyway after centering_timeout, so a robot that cannot close
       the gap (blocked flank, correction gated off) still terminates
       instead of hanging -- the residual offset is then measured on the
       floor rather than asserted by the robot.
    4. Controller in HALT -- it exhausted its avoidance options, so an
       obstacle is what stopped this run.
    5. Otherwise, driving.

    target_distance <= 0.0 disables rules 2 and 3 entirely, preserving
    today's behavior exactly. cross_track_tolerance <= 0.0 disables the
    centering phase alone, so arrival latches the moment the distance is
    covered, as it did before centering existed.
    """
    if already_arrived:
        return ARRIVED_TARGET

    if target_distance > 0.0:
        if odom_stale:
            return STOPPED_ODOM_FAULT
        if controller_state == DRIVE and along_track >= target_distance:
            still_off_line = (cross_track_tolerance > 0.0
                              and abs(cross_track_err) > cross_track_tolerance)
            if still_off_line and centering_elapsed < centering_timeout:
                return CENTERING
            return ARRIVED_TARGET

    if controller_state == HALT:
        return ARRIVED_OBSTACLE

    return DRIVING


def should_skip_controller(*, target_distance, odom_stale, already_arrived):
    """True when this tick is decided without consulting the state machine.

    Both cases are terminal stops, and ticking the controller through them
    would keep advancing its maneuver timers against motion that is not
    happening. Every other case -- including HALT, which is recoverable and
    needs stepping to notice the path has cleared -- must still step.

    Mirrors rules 1 and 2 of decide_arrival; keep the two in sync.

    Skipping controller.step() freezes its injected clock. For the arrival
    latch (case 1) that freeze is permanent and harmless -- the controller
    is never stepped again this run. For the odom-fault case (case 2) it is
    NOT harmless in the same way: odom can recover, and
    AvoidanceController._advance_clock derives dt from the caller's `now`,
    so a skipped stretch (e.g. a 5s odom dropout) makes the next dt on
    resume equal to that whole gap. If that satisfies clear_confirm_time in
    one jump, the state machine can treat clearance as "sustained" when it
    was only ever sampled once, eroding the debounce. This is judged
    acceptable because the resulting forward command is still gated on a
    fresh scan showing a clear front -- but it means the debounce guarantee
    is weaker across a resume than it is in steady-state operation.
    """
    return already_arrived or (target_distance > 0.0 and odom_stale)
