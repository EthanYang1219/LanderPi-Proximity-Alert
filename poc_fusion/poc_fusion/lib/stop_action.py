"""Pure decision logic for the Task 9 stop action.

No ROS, no hardware: the Bool reading and its age are plain values so every
rule here is unit-testable off-robot.

WHY THIS NEEDS ITS OWN FAIL-SAFE RULES
--------------------------------------
The stop action consumes the monitor's DERIVED `std_msgs/Bool`, not its
authoritative tri-state. `monitor_state.should_publish_bool()` deliberately
keeps that Bool SILENT while the monitor is UNKNOWN rather than publishing
False, because a False there would read as "clear" to every consumer.

That is right for the monitor and it puts the burden here: on the Bool topic,
UNKNOWN is *a gap in the stream*. At the message level a gap is
indistinguishable from a dead monitor, a dropped message, or a system that has
not started yet. None of those is permission to drive.

So absence is never treated as clearance:

    obstacle is None (nothing ever received)  -> STOP, reason NO_SIGNAL
    reading older than the staleness bound    -> STOP, reason SIGNAL_STALE
    fresh True                                -> STOP, reason OBSTACLE
    fresh False                               -> drive, reason CLEAR

This is the same fail-safe direction `monitor_state.evaluate_state()` takes
with `costmap_age_s is None`, applied one layer downstream.

WHY THE ALERT IS EDGE-TRIGGERED, AND WHY STALENESS DOES NOT TRIGGER IT
----------------------------------------------------------------------
`fire_alert` is true only when this tick's reason becomes OBSTACLE having not
been OBSTACLE last tick. The alert means "an obstacle was seen": firing it
every tick for as long as the obstacle sits there would be noise, and firing
it on a stale reading would announce a detection the system no longer has
live evidence for. A stale or missing signal still stops the robot -- it just
does so without claiming a sighting. The degradation is logged instead.

A stop held across a signal gap and then re-confirmed DOES re-fire the alert
(previous_reason was SIGNAL_STALE, not OBSTACLE). That is a deliberate
trade-off: a fresh determinate detection is worth announcing, and the
alternative would silently suppress a real sighting.

THIS MODULE DOES NOT HOLD THE STOP
----------------------------------
It decides, per tick, whether a zero Twist should be published. A zero Twist
published at costmap rate does NOT hold a stop on this platform -- the STM32
latches the last commanded velocity forever, which is what allowed a
previously recorded 8-minute uncommanded spin. Holding the stop is
`proximity_alert`'s existing `motion_watchdog` node's job, and Task 9 Step 3
requires it in the loop. See stop_action_node.py's module docstring for the
required topology.
"""

from collections import namedtuple

REASON_NO_SIGNAL = 'no_obstacle_signal'
REASON_SIGNAL_STALE = 'obstacle_signal_stale'
REASON_OBSTACLE = 'obstacle_detected'
REASON_CLEAR = 'clear'

# Every reason except CLEAR commands a stop. Kept explicit so that adding a
# reason forces a deliberate classification instead of silently defaulting to
# the drive side.
STOPPING_REASONS = (REASON_NO_SIGNAL, REASON_SIGNAL_STALE, REASON_OBSTACLE)

StopAction = namedtuple('StopAction', ['stop', 'fire_alert', 'reason'])


def evaluate_stop_action(obstacle, signal_age_s, staleness_bound_s,
                         previous_reason):
    """Decide this tick's stop action from the latest Bool reading.

    Args:
        obstacle: the most recent `obstacle_detected` payload, or None if no
            message has ever been received. None is the fail-safe startup
            input and yields a STOP, never a drive.
        signal_age_s: seconds since that message arrived, or None. None with a
            present reading is defensive: a reading with no arrival time is
            not evidence of anything and fails to the stop side.
        staleness_bound_s: the maximum age at which the reading still counts
            as fresh. Inclusive, matching monitor_state.evaluate_state().
        previous_reason: the reason this function returned last tick, or None
            on the first call. Only used to make the alert edge-triggered.

    The checks are ordered so that `reason` names the root condition: a stale
    True reports staleness rather than an obstacle, because the obstacle claim
    is exactly what staleness invalidates.
    """
    if obstacle is None or signal_age_s is None:
        reason = REASON_NO_SIGNAL
    elif signal_age_s > staleness_bound_s:
        reason = REASON_SIGNAL_STALE
    elif obstacle:
        reason = REASON_OBSTACLE
    else:
        reason = REASON_CLEAR

    return StopAction(
        stop=reason in STOPPING_REASONS,
        fire_alert=(reason == REASON_OBSTACLE
                    and previous_reason != REASON_OBSTACLE),
        reason=reason,
    )
