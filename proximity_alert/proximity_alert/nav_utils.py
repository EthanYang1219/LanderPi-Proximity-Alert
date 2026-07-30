"""Pure, ROS-free arrival logic for the fixed-distance A-to-B stop.

Kept separate from path_tracker (and from the AvoidanceController it wraps)
so the priority rules are unit-testable without a live node, following the
same pattern as scan_utils.py.
"""
import math

from proximity_alert.avoidance import DRIVE, HALT

DRIVING = "driving"
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
    """
    return math.hypot(current_x - start_x, current_y - start_y)


def decide_arrival(*, controller_state, target_distance, distance_traveled,
                   odom_stale, already_arrived):
    """Which stop condition (if any) governs this control tick.

    Priority, highest first:

    1. Already arrived -- latched, so the robot stays stopped. Arrival can
       only latch from DRIVE (rule 3), so this can never mask an obstacle
       halt that was already in progress.
    2. Stale odom while target_distance is set -- distance_traveled is
       derived from odom, so stale odom means the distance is untrustworthy.
       Stop rather than drive blind or declare a bogus arrival.
    3. Target reached AND controller is in DRIVE -- deferring until DRIVE
       lets a strafe/turn finish, so the robot ends square and on-heading
       instead of frozen sideways mid-maneuver. The overshoot this costs is
       not always small: if a full maneuver ladder runs (strafe, then turn,
       then drive-past) before DRIVE is reached again, it can total roughly
       a metre (max_drive_past_distance plus strafe travel).
    4. Controller in HALT -- it exhausted its avoidance options, so an
       obstacle is what stopped this run.
    5. Otherwise, driving.

    target_distance <= 0.0 disables rules 2 and 3 entirely, preserving
    today's behavior exactly.
    """
    if already_arrived:
        return ARRIVED_TARGET

    if target_distance > 0.0:
        if odom_stale:
            return STOPPED_ODOM_FAULT
        if controller_state == DRIVE and distance_traveled >= target_distance:
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
