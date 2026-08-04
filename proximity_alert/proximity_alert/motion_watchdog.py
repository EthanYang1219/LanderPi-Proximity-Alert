#!/usr/bin/env python3
"""Fail-safe cmd_vel watchdog -- forces a stop when its input goes stale.

Sits between the real Twist source (path_tracker, or anything else) and the
topic that actually reaches the motors. Remap the source to publish to this
node's `input_topic` instead of directly to the motor-facing topic, and let
THIS node own `output_topic`:

    ros2 run proximity_alert path_tracker --ros-args -r /cmd_vel:=/cmd_vel_unsafe
    ros2 run proximity_alert motion_watchdog

As long as fresh commands keep arriving on `input_topic`, they are forwarded
to `output_topic` unchanged, republished at `check_rate_hz` regardless of the
source's own publish rate. The moment `input_topic` goes stale for longer
than `timeout_sec` -- source died, was orphaned, was SIGKILLed, or its final
message was simply dropped -- this node starts publishing zero on
`output_topic` itself, every tick, until fresh input resumes. No other node
on this platform provides this: the STM32 otherwise holds the last commanded
velocity forever.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from proximity_alert.motion_watchdog_logic import ZERO, decide_watchdog_output


class MotionWatchdog(Node):
    def __init__(self):
        super().__init__("motion_watchdog")
        self.declare_parameter("input_topic", "/cmd_vel_unsafe")
        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("timeout_sec", 0.5)
        self.declare_parameter("check_rate_hz", 20.0)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.timeout_sec = self.get_parameter("timeout_sec").value
        rate = self.get_parameter("check_rate_hz").value

        self._last_cmd = None
        self._last_time = None

        self.pub = self.create_publisher(Twist, output_topic, 10)
        self.create_subscription(Twist, input_topic, self._on_cmd, 10)
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f"motion_watchdog started: {input_topic} -> {output_topic}, "
            f"timeout_sec={self.timeout_sec}"
        )

    def _on_cmd(self, msg: Twist):
        self._last_cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
        self._last_time = self.get_clock().now()

    def _age(self):
        if self._last_time is None:
            return None
        return (self.get_clock().now() - self._last_time).nanoseconds / 1e9

    def _tick(self):
        age = self._age()
        out = decide_watchdog_output(self._last_cmd, age, self.timeout_sec)
        if out == ZERO and self._last_cmd != ZERO:
            self.get_logger().warn(
                "cmd_vel input stale or never received; forcing stop.",
                throttle_duration_sec=1.0,
            )
        cmd = Twist()
        cmd.linear.x, cmd.linear.y, cmd.angular.z = out
        self.pub.publish(cmd)

    def publish_stop(self):
        """Same STM32-latching concern as path_tracker's own publish_stop:
        a single zero published as the process exits can be dropped before
        it reaches the wire. Publish several times with a brief pause."""
        import time
        stop = Twist()
        for _ in range(5):
            self.pub.publish(stop)
            time.sleep(0.03)


def main(args=None):
    # Same signal-handling shape as path_tracker's main(), and for the same
    # reason: rclpy's default handler tears down the context before our own
    # cleanup runs, so a final safety-stop publish in a `finally` block would
    # silently fail on SIGINT/SIGTERM otherwise.
    from rclpy.signals import SignalHandlerOptions
    from rclpy.executors import ExternalShutdownException
    import signal

    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = MotionWatchdog()

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
        node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
