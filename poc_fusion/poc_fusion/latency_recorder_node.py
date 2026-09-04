"""Records sensor-to-stop latency for the fusion pipeline.

On each False->True transition of `/costmap_app/obstacle_detected`, records

    now  -  (stamp of the newest depth frame that had ARRIVED before the
             transition)

`/poc_fusion/depth_cleaned` carries the camera's stamp verbatim (Task 4
Step 1), which is why it -- and not the rectified or projected topic -- is the
measurement origin.

All decision rules live in `poc_fusion.lib.latency_stats` and are unit-tested
off-robot, including the two errors that survived a first round of mutation
testing here: selecting the reference frame by stamp instead of by arrival,
and truncating rather than rounding up the p95 rank.

THIS NODE COMMANDS NOTHING. It only subscribes and records, so it is safe to
run during any test, including motion tests.

WHAT THE REPORTED FIGURE IS
---------------------------
A LOWER BOUND on true physical-entry-to-stop latency (Task 12 Step 3).

  Covers:     depth preprocessing, rectification, projection, costmap update,
              the costmap publish interval, and monitor evaluation --
              everything this POC adds.
  Does NOT
  cover:      the camera's own exposure and internal processing latency,
              which is unrecoverable from message stamps and would need
              external high-frame-rate video to capture.

Do not describe the result as end-to-end. It is also NOT stop-completion
latency: it ends when the Bool flips, not when the wheels have stopped.

Task 12 Step 2 requires at least 20 edges and reports MEDIAN and P95, never a
single sample. The node logs a running summary and prints a final one on
shutdown, and appends every edge to a CSV so the sample set is auditable
rather than existing only in a log line.

The CSV path is deliberately its own file under a poc_fusion-specific
default. It is NOT part of the surface-trial logging pipeline and must never
be pointed at it.
"""

import csv
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from poc_fusion.lib.latency_stats import (
    Frame,
    is_rising_edge,
    newest_preceding_frame,
    summarize,
)


class LatencyRecorderNode(Node):
    def __init__(self):
        super().__init__('latency_recorder_node')

        self.declare_parameter('depth_topic', '/poc_fusion/depth_cleaned')
        self.declare_parameter('obstacle_detected_topic',
                               '/costmap_app/obstacle_detected')
        self.declare_parameter('csv_path', '/tmp/poc_fusion_latency.csv')
        # ~14 s of history at the depth topic's measured ~14.2 Hz. Only needs
        # to outlast the gap between a frame arriving and the edge it caused.
        self.declare_parameter('frame_buffer_size', 200)
        self.declare_parameter('summary_log_period_s', 10.0)

        depth_topic = self.get_parameter('depth_topic').value
        obstacle_topic = self.get_parameter('obstacle_detected_topic').value
        self.csv_path = self.get_parameter('csv_path').value
        self.buffer_size = int(self.get_parameter('frame_buffer_size').value)
        summary_period = self.get_parameter('summary_log_period_s').value

        self._frames = []
        self._previous = None
        self._samples = []
        # Edges that arrived with no preceding depth frame are DROPPED, never
        # measured against a later frame. Counted so a run that silently
        # dropped most of its edges cannot look like a clean one.
        self._dropped_edges = 0

        self._init_csv()

        self.create_subscription(Image, depth_topic, self._on_depth, 10)
        self.create_subscription(Bool, obstacle_topic, self._on_obstacle, 10)
        self.create_timer(summary_period, self._log_summary)

        self.get_logger().info(
            f'latency_recorder_node started: origin={depth_topic}, '
            f'edge={obstacle_topic}, csv={self.csv_path}. This node commands '
            f'nothing. Reported latency is a LOWER BOUND -- it excludes '
            f'camera exposure and internal processing.')

    def _init_csv(self):
        directory = os.path.dirname(self.csv_path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        new_file = not os.path.exists(self.csv_path)
        self._csv_file = open(self.csv_path, 'a', newline='')
        self._csv = csv.writer(self._csv_file)
        if new_file:
            self._csv.writerow([
                'edge_index', 'transition_time_s', 'depth_stamp_s',
                'depth_received_s', 'latency_s',
            ])
            self._csv_file.flush()

    def _now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _on_depth(self, msg: Image):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self._frames.append(Frame(stamp=stamp, received=self._now_s()))
        if len(self._frames) > self.buffer_size:
            del self._frames[:len(self._frames) - self.buffer_size]

    def _on_obstacle(self, msg: Bool):
        current = bool(msg.data)
        if is_rising_edge(self._previous, current):
            self._record_edge(self._now_s())
        self._previous = current

    def _record_edge(self, transition_time_s):
        frame = newest_preceding_frame(self._frames, transition_time_s)
        if frame is None:
            self._dropped_edges += 1
            self.get_logger().warn(
                'rising edge with no preceding depth frame; DROPPED rather '
                'than measured against a later frame '
                f'(dropped so far: {self._dropped_edges}).')
            return

        latency = transition_time_s - frame.stamp
        self._samples.append(latency)
        self._csv.writerow([
            len(self._samples), f'{transition_time_s:.6f}',
            f'{frame.stamp:.6f}', f'{frame.received:.6f}', f'{latency:.6f}',
        ])
        self._csv_file.flush()
        self.get_logger().info(
            f'edge {len(self._samples)}: latency {latency * 1000:.1f} ms')

    def _log_summary(self):
        s = summarize(self._samples)
        if s.n == 0:
            self.get_logger().info(
                f'no rising edges captured yet '
                f'(dropped: {self._dropped_edges}).')
            return
        self.get_logger().info(
            f'latency over {s.n} edges (LOWER BOUND): '
            f'median {s.median * 1000:.1f} ms, p95 {s.p95 * 1000:.1f} ms, '
            f'min {s.min * 1000:.1f} ms, max {s.max * 1000:.1f} ms, '
            f'dropped {self._dropped_edges}')

    def final_report(self):
        s = summarize(self._samples)
        if s.n == 0:
            self.get_logger().warn(
                f'FINAL: no rising edges captured. Nothing is being claimed '
                f'about latency. (dropped: {self._dropped_edges})')
            return
        if s.n < 20:
            self.get_logger().warn(
                f'FINAL: only {s.n} edges captured. Task 12 Step 2 requires '
                f'at least 20 before median/p95 may be reported as a result.')
        self.get_logger().info(
            f'FINAL latency over {s.n} edges (LOWER BOUND, excludes camera '
            f'exposure): median {s.median * 1000:.1f} ms, '
            f'p95 {s.p95 * 1000:.1f} ms, min {s.min * 1000:.1f} ms, '
            f'max {s.max * 1000:.1f} ms, dropped {self._dropped_edges}. '
            f'Samples: {self.csv_path}')

    def close(self):
        try:
            self._csv_file.close()
        except Exception:
            pass


def main(args=None):
    import signal

    from rclpy.executors import ExternalShutdownException
    from rclpy.signals import SignalHandlerOptions

    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = LatencyRecorderNode()

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
        node.final_report()
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
