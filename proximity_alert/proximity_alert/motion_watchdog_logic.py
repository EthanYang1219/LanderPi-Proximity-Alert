"""Pure, ROS-free decision logic for motion_watchdog.

The STM32 on this platform holds the last commanded velocity FOREVER -- there
is no motion watchdog anywhere else on the stack. Every stop today depends on
some upstream process successfully publishing a final zero Twist as it exits
(path_tracker's own publish_stop(), for instance). That is fine as long as
the process exits cleanly and its last message is delivered -- but if it is
orphaned, SIGKILLed, or its last message is dropped, nothing ever corrects
the latched command and the robot keeps moving indefinitely. Confirmed twice
on hardware.

motion_watchdog is the fix: it sits between the real command source and the
topic that actually reaches the motors, and forces a stop the moment its
input goes stale, regardless of why. This module is the part of that logic
that has nothing to do with ROS -- it is a pure function of "what did we last
hear, how old is it," so it is unit-testable without a live node.
"""

ZERO = (0.0, 0.0, 0.0)


def decide_watchdog_output(last_cmd, age_s, timeout_s):
    """The (linear_x, linear_y, angular_z) tuple to publish this tick.

    Passes `last_cmd` through unchanged as long as it is fresh (age_s within
    timeout_s); forces ZERO otherwise. Also ZERO before anything has ever
    been received (last_cmd is None) -- a watchdog that has not yet heard
    from its input must default to safe, not to whatever motion happened to
    be latched before it started.

    age_s is ignored (and may be None) when last_cmd is None -- there is
    nothing to be stale.
    """
    if last_cmd is None:
        return ZERO
    if age_s is None or age_s > timeout_s:
        return ZERO
    return last_cmd
