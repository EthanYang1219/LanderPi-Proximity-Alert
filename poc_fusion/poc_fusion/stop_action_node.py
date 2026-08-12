"""Task 9 stop action: a series gate on cmd_vel, plus a buzzer on detection.

Consumes the stop monitor's derived `std_msgs/Bool` and turns it into the
POC's only response to a detection: a full stop plus an audible alert. There
is deliberately NO avoidance maneuver here -- design doc section 11 defers
consuming the costmap in `avoidance.py` to protect the surface-trial dataset,
and Task 9 Step 4 says stop only.

All decision rules live in `poc_fusion.lib.stop_action` (when to stop) and
`poc_fusion.lib.cmd_vel_gate` (what reaches the wheels), both unit-tested
off-robot. This file is the ROS shell: parameters, subscriptions, timer,
publisher, and the alert subprocess.


REQUIRED TOPOLOGY -- READ BEFORE RUNNING WITH A MOVING ROBOT
-----------------------------------------------------------
A zero Twist published at costmap rate does NOT hold a stop on this platform.
The STM32 latches the last commanded velocity indefinitely; that is what
allowed a previously recorded 8-minute uncommanded spin. This node therefore
REQUIRES `proximity_alert`'s existing `motion_watchdog` in the loop rather than
reimplementing stop-hold here.

This node is a SERIES GATE. The motion source flows THROUGH it:

    <motion source> --> /cmd_vel_raw --> [stop_action_node] --> /cmd_vel_unsafe
                                                                      |
                                          motion_watchdog <-----------/
                                                 |
                                                 v
                                             /cmd_vel --> motors

    ros2 run proximity_alert motion_watchdog        # /cmd_vel_unsafe -> /cmd_vel
    ros2 run proximity_alert path_tracker --ros-args -r /cmd_vel:=/cmd_vel_raw
    ros2 run poc_fusion stop_action_node --ros-args \\
        -p cmd_vel_in_topic:=/cmd_vel_raw -p cmd_vel_out_topic:=/cmd_vel_unsafe

`path_tracker` publishes to a hardcoded `/cmd_vel`, so it is REMAPPED at launch.
That is the idiom already used by `motion_watchdog`'s own docstring, and it
means `proximity_alert` -- frozen for the surface trials -- needs no edit.

WHY A GATE AND NOT A PARALLEL PUBLISHER. An earlier version of this node
published zeros onto the same topic as the motion source and relied on
`zero_twist_burst` to win the resulting race. Task 9 Step 3 measured that design
failing on hardware: a **51.6% forward duty cycle for 9.5 s after a stop
engaged**, with the robot commanded forward roughly half the time while the log
said it was holding a stop. ROS 2 does not arbitrate publishers, and three zeros
are simply overwritten 50 ms later by the next forward message. `zero_twist_burst`
has been REMOVED rather than retuned: no value of it fixes a topology defect, and
leaving it would imply the race is still something to manage.

With the gate there is exactly ONE publisher on the output topic, so there is
nothing left to race. A stop REMOVES the forward command instead of competing
with it.

`cmd_vel_out_topic` is NEVER `/controller/cmd_vel`: that topic already has five
vendor publishers (lidar_app, line_following, object_tracking, hand_gesture,
joystick_control) and publishing into it would join a five-way race. The node
refuses to start if either topic is set to it, or if input and output are equal
(which would feed the gate its own output).


VERIFYING WITHOUT MOVING THE ROBOT
----------------------------------
`publish_cmd_vel: false` runs every decision, log line and alert exactly as
normal but suppresses the Twist publisher entirely, so the full chain can be
verified with no possibility of commanding a wheel. The node logs loudly that
it is in this mode so an observe-only run can never be mistaken for a live
safety test. Note that in this mode the gate is also not passing motion
through, so nothing downstream can move either.


ON THE ALERT
------------
The alert mechanism is deliberately identical to the existing path in
`proximity_alert`'s `path_tracker`: a non-blocking `aplay -D <device> <wav>`,
same default WAV and ALSA device. The code is not imported because that
playback is a private method on `path_tracker`'s Node class and
`proximity_alert` is frozen for the surface trials -- extracting it would mean
editing a package this task must not touch. Same trade-offs are inherited:
playback failures are non-fatal, overlapping playback is unguarded, and
finished `aplay` processes are not reaped.
"""

import os
import subprocess

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool

from poc_fusion.lib.cmd_vel_gate import (
    GATE_FORWARD,
    GATE_ZERO,
    evaluate_gate,
)
from poc_fusion.lib.stop_action import REASON_OBSTACLE, evaluate_stop_action


class StopActionNode(Node):
    def __init__(self):
        super().__init__('stop_action_node')

        self.declare_parameter('obstacle_detected_topic',
                               '/costmap_app/obstacle_detected')
        # Series gate: motion arrives on _in and leaves on _out. Never
        # /controller/cmd_vel. See the topology note in the module docstring.
        self.declare_parameter('cmd_vel_in_topic', '/cmd_vel_raw')
        self.declare_parameter('cmd_vel_out_topic', '/cmd_vel_unsafe')
        # How long a motion command is forwarded after it arrives. Beyond this
        # the gate falls SILENT rather than repeating a stale command, and
        # motion_watchdog's own timeout becomes the stop holder.
        self.declare_parameter('cmd_vel_input_timeout_s', 0.5)
        self.declare_parameter('stop_active_topic', '/costmap_app/stop_active')
        # Bounds how long a Bool reading is trusted. Sized against the
        # monitor's own publish period, not the costmap's: the monitor
        # publishes the Bool on its status timer whenever the state is
        # determinate, so a gap here means UNKNOWN or a dead monitor.
        self.declare_parameter('signal_staleness_bound_s', 0.75)
        self.declare_parameter('tick_rate_hz', 10.0)
        self.declare_parameter('publish_cmd_vel', True)
        self.declare_parameter('alert_enabled', True)
        self.declare_parameter('wav_path',
                               '/home/ubuntu/shared/audio/obstacle_alert.wav')
        self.declare_parameter('alsa_device', 'plughw:2,0')

        obstacle_topic = self.get_parameter('obstacle_detected_topic').value
        self.cmd_vel_in_topic = self.get_parameter('cmd_vel_in_topic').value
        self.cmd_vel_out_topic = self.get_parameter('cmd_vel_out_topic').value
        self.input_timeout_s = self.get_parameter(
            'cmd_vel_input_timeout_s').value
        stop_active_topic = self.get_parameter('stop_active_topic').value
        self.staleness_bound_s = self.get_parameter(
            'signal_staleness_bound_s').value
        tick_rate = self.get_parameter('tick_rate_hz').value
        self.publish_cmd_vel = self.get_parameter('publish_cmd_vel').value
        self.alert_enabled = self.get_parameter('alert_enabled').value
        self.wav_path = self.get_parameter('wav_path').value
        self.alsa_device = self.get_parameter('alsa_device').value

        for name, topic in (('cmd_vel_in_topic', self.cmd_vel_in_topic),
                            ('cmd_vel_out_topic', self.cmd_vel_out_topic)):
            if topic == '/controller/cmd_vel':
                raise ValueError(
                    f'{name} must never be /controller/cmd_vel -- that topic '
                    f'already has five vendor publishers and ROS 2 will not '
                    f'arbitrate between them (design doc section 5).')

        if self.cmd_vel_in_topic == self.cmd_vel_out_topic:
            raise ValueError(
                f'cmd_vel_in_topic and cmd_vel_out_topic are both '
                f"'{self.cmd_vel_in_topic}'. The gate would feed on its own "
                f'output, and a stop would race the motion source instead of '
                f'removing it -- the exact defect measured in Task 9 Step 3.')

        # Same reasoning as path_tracker: fail loudly at startup rather than
        # silently at the first obstacle. A missing WAV is otherwise invisible
        # because aplay is spawned non-blocking with its output discarded.
        if self.alert_enabled and not os.path.exists(self.wav_path):
            self.get_logger().error(
                f"alert_enabled is true but wav_path '{self.wav_path}' does "
                f'not exist; no alert will be audible. Put the WAV there or '
                f'pass -p alert_enabled:=false to silence this.')

        self._obstacle = None
        self._last_msg_time = None
        self._previous_reason = None
        self._input_twist = None
        self._input_time = None
        self._previous_outcome = None

        self.stop_active_pub = self.create_publisher(Bool, stop_active_topic, 10)
        self.cmd_vel_pub = (
            self.create_publisher(Twist, self.cmd_vel_out_topic, 10)
            if self.publish_cmd_vel else None)

        self.create_subscription(Bool, obstacle_topic, self._on_obstacle, 10)
        self.create_subscription(Twist, self.cmd_vel_in_topic,
                                 self._on_cmd_vel_in, 10)
        self.create_timer(1.0 / tick_rate, self._tick)

        if not self.publish_cmd_vel:
            self.get_logger().warn(
                'publish_cmd_vel is FALSE -- this is an OBSERVE-ONLY run. No '
                'Twist publisher exists and no wheel can be commanded, and no '
                'motion is passed through the gate. This run does NOT verify '
                'the stop path.')
        self.get_logger().info(
            f'stop_action_node started as a SERIES GATE: '
            f'{self.cmd_vel_in_topic} -> {self.cmd_vel_out_topic} '
            f'(publishing={self.publish_cmd_vel}), gated by {obstacle_topic}, '
            f'staleness_bound_s={self.staleness_bound_s}, '
            f'input_timeout_s={self.input_timeout_s}, '
            f'alert_enabled={self.alert_enabled}. motion_watchdog is REQUIRED '
            f'in the loop to hold the stop; see this node\'s docstring.')

    def _on_obstacle(self, msg: Bool):
        self._obstacle = msg.data
        self._last_msg_time = self.get_clock().now()

    def _on_cmd_vel_in(self, msg: Twist):
        self._input_twist = msg
        self._input_time = self.get_clock().now()

    def _signal_age_s(self):
        if self._last_msg_time is None:
            return None
        return (self.get_clock().now() - self._last_msg_time).nanoseconds / 1e9

    def _input_age_s(self):
        if self._input_time is None:
            return None
        return (self.get_clock().now() - self._input_time).nanoseconds / 1e9

    def _tick(self):
        action = evaluate_stop_action(
            obstacle=self._obstacle,
            signal_age_s=self._signal_age_s(),
            staleness_bound_s=self.staleness_bound_s,
            previous_reason=self._previous_reason,
        )

        if action.reason != self._previous_reason:
            self.get_logger().info(
                f'stop action: {self._previous_reason} -> {action.reason} '
                f'(stop={action.stop})')
            if action.stop and action.reason != REASON_OBSTACLE:
                # Degraded stop: the robot is being held without a sighting.
                # Loud, because this is the case a silent system would hide.
                self.get_logger().warn(
                    f'holding a stop with NO live obstacle signal '
                    f'({action.reason}). This is fail-safe, not a detection.')

        if action.fire_alert:
            self._fire_alert()

        outcome = evaluate_gate(
            stop=action.stop,
            has_input=self._input_twist is not None,
            input_age_s=self._input_age_s(),
            input_timeout_s=self.input_timeout_s,
        )
        self._drive_output(outcome)

        self.stop_active_pub.publish(Bool(data=bool(action.stop)))
        self._previous_reason = action.reason
        self._previous_outcome = outcome

    def _drive_output(self, outcome):
        if outcome != self._previous_outcome:
            self.get_logger().info(f'gate: {self._previous_outcome} -> {outcome}')

        if self.cmd_vel_pub is None:
            return
        if outcome == GATE_ZERO:
            self.cmd_vel_pub.publish(Twist())
        elif outcome == GATE_FORWARD:
            # Forwarded verbatim: the gate decides WHETHER a command passes,
            # never what it contains. Rewriting it here would make the gate a
            # second motion source with no test covering what it emits.
            self.cmd_vel_pub.publish(self._input_twist)
        # GATE_SILENT: publish nothing, and let motion_watchdog's own staleness
        # timeout own the stop. Manufacturing a zero here would put two
        # mechanisms in charge of one behaviour.

    def _fire_alert(self):
        if not self.alert_enabled:
            return
        try:
            subprocess.Popen(['aplay', '-D', self.alsa_device, self.wav_path],
                             stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL)
        except OSError as e:
            self.get_logger().warn(
                f'could not launch aplay ({e}); is ALSA installed in this '
                f'container?')


def main(args=None):
    # Same signal-handling shape as proximity_alert's motion_watchdog and
    # path_tracker, and for the same reason: rclpy's default handler tears
    # down the context before our own cleanup runs, so a final safety-stop
    # publish in a `finally` block would silently fail on SIGINT/SIGTERM.
    import signal

    from rclpy.executors import ExternalShutdownException
    from rclpy.signals import SignalHandlerOptions

    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = StopActionNode()

    stop_requested = False

    def request_stop(signum, frame):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    try:
        while rclpy.ok() and not stop_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Leave the robot commanded to zero, not to whatever was last sent.
        # Published several times with a pause: a single zero on the way out
        # can be dropped before it reaches the wire.
        if node.cmd_vel_pub is not None:
            import time
            for _ in range(5):
                node.cmd_vel_pub.publish(Twist())
                time.sleep(0.03)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
