"""Series gate: decides what reaches the wheels, given a stop decision.

No ROS, so every rule is unit-testable off-robot.

WHY A GATE AND NOT A PARALLEL PUBLISHER
---------------------------------------
Task 9 Step 3 measured the parallel design failing on real hardware. With
`stop_action_node` publishing zeros onto the same topic as a live motion source,
ROS 2's last-write-wins produced a **51.6% forward duty cycle for 9.5 s after a
stop engaged** — the robot kept being commanded forward while the system logged
that it was holding a stop.

ROS 2 does not arbitrate publishers. Two publishers on one topic is not a race
that can be tuned down; `zero_twist_burst: 3` did not mitigate it, because three
zeros are simply overwritten 50 ms later by the next forward message.

So the motion command flows THROUGH this decision:

    motion source -> cmd_vel_in -> [gate] -> cmd_vel_out -> motion_watchdog

A stop now REMOVES the forward command rather than competing with it. There is
exactly one publisher on the output topic, so there is nothing left to race.

THE THREE OUTCOMES
------------------
  GATE_ZERO     publish a zero Twist. Stop is active.
  GATE_FORWARD  publish the motion source's Twist verbatim.
  GATE_SILENT   publish NOTHING.

`GATE_SILENT` is deliberately distinct from `GATE_ZERO`. Silence means "this gate
has no command to pass on", and `motion_watchdog`'s own staleness timeout then
owns the stop. Manufacturing a zero instead would put two mechanisms in charge of
the same behaviour, which is the shape of the bug this module exists to fix.

Stop is checked FIRST and unconditionally, so no combination of input freshness,
rate or absence can produce a forward command while stopped.
"""

GATE_ZERO = 'zero'
GATE_FORWARD = 'forward'
GATE_SILENT = 'silent'


def evaluate_gate(stop, has_input, input_age_s, input_timeout_s):
    """What the gate should put on its output topic this tick.

    `stop` comes from `stop_action.evaluate_stop_action`, which is already
    fail-safe: it is True when the obstacle signal is missing or stale, not only
    when an obstacle was seen.

    Staleness is `age > timeout`, not `>=`, so a command that arrived exactly on
    time is still forwarded.
    """
    if stop:
        return GATE_ZERO
    if not has_input or input_age_s is None or input_age_s > input_timeout_s:
        return GATE_SILENT
    return GATE_FORWARD
