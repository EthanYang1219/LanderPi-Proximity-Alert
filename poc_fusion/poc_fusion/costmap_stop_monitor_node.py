#!/usr/bin/env python3
"""Costmap stop monitor node (Task 7).

Watches the fused local costmap and reports whether the body-aligned safety
window ahead of the robot is clear. It does NOT publish velocity and never
commands motion -- acting on the signal is Task 9's scope.

THE GOVERNING SAFETY SEMANTIC
-----------------------------
"'Costmap node exists' does not equal 'costmap is operational.' The stop
monitor must not interpret silence from an inactive or broken costmap as
'there are no obstacles.'"

So the authoritative output is a TRI-STATE, not a boolean:

    CLEAR    fresh, valid, ACTIVE costmap; window holds no lethal cells
    OBSTACLE fresh, valid, ACTIVE costmap; window does
    UNKNOWN  no costmap yet / stale costmap / costmap not ACTIVE /
             TF lookup failed for the latest costmap

published on `status_topic` as a `diagnostic_msgs/DiagnosticArray` on a
fixed timer, whether or not costmaps are arriving -- silence must never be
the only symptom of a blind monitor. The decision itself lives in
`lib/monitor_state.py` (pure, unit-tested off-robot); this module is ROS
plumbing plus the loud logging around it.

`obstacle_detected_topic` (`std_msgs/Bool`) is RETAINED as a derived
downstream-compatibility output for Tasks 9 and 12. In UNKNOWN this node
publishes NOTHING on it -- it does not publish False. A False there would
hand every consumer exactly the CLEAR/UNKNOWN conflation the tri-state
exists to prevent.

WHY DiagnosticArray AND NOT A CUSTOM .msg
-----------------------------------------
Everything a trial audit needs (state, reason, costmap age, lifecycle
label, camera axis) fits `DiagnosticStatus.values` as key/value pairs, the
array carries a header stamp for rosbag replay, and `rqt_robot_monitor`
renders it with no extra code. A custom .msg would also force `poc_fusion`
from `ament_python` to `ament_cmake` (rosidl generation is unavailable in a
pure-Python package), which is a large, review-visible change to Task 2's
scaffolding for no capability gain.

WHY /costmap/costmap_raw AND NOT /costmap/costmap
-------------------------------------------------
A CORRECTION to the Task 7 brief's Step 1, forced by two live-measured
facts (both pasted in docs/poc_fusion_verification.md "Task 7"):

  1. COST SCALE. `nav_msgs/OccupancyGrid` carries costs rescaled to 0..100
     by Costmap2DPublisher's translation table; `nav2_msgs/Costmap` carries
     the raw 0..255 costs. `lethal_threshold: 253` is a RAW value -- against
     the rescaled OccupancyGrid nothing could ever reach it and this node
     would have reported CLEAR forever with no error anywhere.
  2. PUBLICATION. With nav2's default `always_send_full_costmap: false`,
     /costmap/costmap publishes on new-subscriber only; /costmap/costmap_raw
     publishes the full grid every cycle.

Consuming costmap_raw keeps the threshold meaningful and leaves Task 6's
reviewed costmap configuration untouched.
"""
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point
from lifecycle_msgs.msg import TransitionEvent
from lifecycle_msgs.srv import GetState
from nav2_msgs.msg import Costmap
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker

from poc_fusion.lib.costmap_raw import decode_raw_costmap
from poc_fusion.lib.monitor_state import (
    CAMERA_FUSION_ACTIVE,
    STATE_OBSTACLE,
    STATE_UNKNOWN,
    bool_value,
    camera_health,
    evaluate_state,
    is_identity_quaternion,
    should_publish_bool,
)
from poc_fusion.lib.obstacle_detection import should_stop
from poc_fusion.lib.window_geometry import (
    window_corners,
    window_mask,
    yaw_from_quaternion,
)

#: Seconds between repeats of the throttled warn/error logs. Long enough
#: not to flood the console at the costmap rate, short enough that a
#: degraded run is unmissable in the scrollback.
LOG_THROTTLE_SEC = 2.0

DIAG_COSTMAP_NAME = 'costmap_stop_monitor: costmap'
DIAG_CAMERA_NAME = 'costmap_stop_monitor: fusion'


class CostmapStopMonitorNode(Node):
    def __init__(self):
        super().__init__('costmap_stop_monitor_node')

        self.declare_parameter('costmap_topic', '/costmap/costmap_raw')
        self.declare_parameter('costmap_lifecycle_node', '/costmap/costmap')
        self.declare_parameter('status_topic', '/costmap_app/monitor_status')
        self.declare_parameter('obstacle_detected_topic',
                               '/costmap_app/obstacle_detected')
        self.declare_parameter('marker_topic', '/costmap_app/window_marker')
        self.declare_parameter('depth_cleaned_topic', '/poc_fusion/depth_cleaned')
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('window_forward_m', 1.0)
        self.declare_parameter('window_half_width_m', 0.3)
        self.declare_parameter('lethal_threshold', 253)
        self.declare_parameter('min_occupancy_fraction', 0.30)
        self.declare_parameter('min_cluster_area_cm2', 100.0)
        self.declare_parameter('costmap_staleness_bound_s', 0.75)
        self.declare_parameter('lifecycle_query_period_s', 1.0)
        self.declare_parameter('lifecycle_query_timeout_s', 0.5)
        self.declare_parameter('status_publish_period_s', 0.2)
        self.declare_parameter('camera_timeout_s', 2.0)
        self.declare_parameter('camera_coverage_ceiling_m', 0.94)
        self.declare_parameter('origin_orientation_tol', 0.001)
        self.declare_parameter('tf_retry_period_s', 0.05)

        self._costmap_topic = self.get_parameter('costmap_topic').value
        lifecycle_node = self.get_parameter('costmap_lifecycle_node').value
        status_topic = self.get_parameter('status_topic').value
        obstacle_topic = self.get_parameter('obstacle_detected_topic').value
        marker_topic = self.get_parameter('marker_topic').value
        depth_topic = self.get_parameter('depth_cleaned_topic').value
        self._robot_base_frame = self.get_parameter('robot_base_frame').value
        self._forward_m = self.get_parameter('window_forward_m').value
        self._half_width_m = self.get_parameter('window_half_width_m').value
        self._lethal_threshold = self.get_parameter('lethal_threshold').value
        self._min_occupancy_fraction = self.get_parameter(
            'min_occupancy_fraction').value
        self._min_cluster_area_cm2 = self.get_parameter(
            'min_cluster_area_cm2').value
        self._staleness_bound_s = self.get_parameter(
            'costmap_staleness_bound_s').value
        lifecycle_period_s = self.get_parameter('lifecycle_query_period_s').value
        self._lifecycle_timeout_s = self.get_parameter(
            'lifecycle_query_timeout_s').value
        status_period_s = self.get_parameter('status_publish_period_s').value
        self._camera_timeout_s = self.get_parameter('camera_timeout_s').value
        self._camera_ceiling_m = self.get_parameter(
            'camera_coverage_ceiling_m').value
        self._origin_orientation_tol = self.get_parameter(
            'origin_orientation_tol').value
        tf_retry_period_s = self.get_parameter('tf_retry_period_s').value

        # --- Requirement 4: fail-safe startup ------------------------------
        #
        # Every one of these initial values means "no evidence yet", and
        # evaluate_state() maps that to UNKNOWN. There is deliberately no
        # `self._obstacle_detected = False` anywhere in this class: a bare
        # boolean initialised to False IS the bug this task exists to
        # prevent, because it renders identically to a real CLEAR.
        self._last_costmap_recv = None
        self._last_depth_recv = None
        self._lifecycle_label = None
        self._tf_ok = False
        self._obstacle_in_window = False
        self._costmap_frame = ''
        self._last_reason = None
        self._last_state = None
        self._last_camera_axis = None
        self._pending_lifecycle_call = None
        self._pending_lifecycle_since = None

        self._pending_costmap = None

        self._tf_buffer = Buffer()
        # spin_thread stays at its default False on purpose: the buffer is
        # filled by this node's own executor and every lookup below is
        # NON-BLOCKING (zero timeout), so there is nothing to deadlock and
        # no second thread to starve. Measured live, both alternatives were
        # worse -- see the _try_process() comment.
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # The costmap publishes RELIABLE/TRANSIENT_LOCAL (verified live with
        # `ros2 topic info -v`, pasted in the verification doc). This
        # subscription asks for VOLATILE on purpose: a TRANSIENT_LOCAL
        # subscription would be handed the publisher's retained last message
        # the instant it connects, and this node timestamps freshness by
        # RECEIPT, so a costmap that died minutes ago would be delivered and
        # counted as fresh for one staleness window. VOLATILE is compatible
        # with a TRANSIENT_LOCAL publisher and only ever yields genuinely
        # new publications.
        costmap_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._status_pub = self.create_publisher(
            DiagnosticArray, status_topic, 10)
        self._obstacle_pub = self.create_publisher(Bool, obstacle_topic, 10)
        self._marker_pub = self.create_publisher(Marker, marker_topic, 10)

        self.create_subscription(
            Costmap, self._costmap_topic, self._on_costmap, costmap_qos)
        self.create_subscription(Image, depth_topic, self._on_depth, 10)

        # --- Requirement 2: POSITIVE liveness, not inferred ----------------
        #
        # Subscription existence is not evidence of anything: Task 6's review
        # established that `ros2 node info` proves subscription, not
        # consumption. Both mechanisms below are used together and neither
        # alone would do:
        #   * transition_event gives immediate notification WHEN the node
        #     announces a transition, but a node that dies, hangs, or is
        #     SIGKILLed announces nothing at all;
        #   * the polled get_state service is what makes the confirmation
        #     POSITIVE -- a dead node stops answering, and no answer is
        #     recorded as no confirmation (label None => UNKNOWN), never as
        #     an assumed 'active'.
        self._lifecycle_node_name = lifecycle_node
        self._get_state_client = self.create_client(
            GetState, f'{lifecycle_node}/get_state')
        self.create_subscription(
            TransitionEvent, f'{lifecycle_node}/transition_event',
            self._on_transition_event, 10)
        self.create_timer(lifecycle_period_s, self._poll_lifecycle_state)

        # Retries the pending costmap's TF lookup; see _try_process().
        self.create_timer(tf_retry_period_s, self._try_process)

        # Requirement 5: the status topic ticks on its own clock so that a
        # monitor receiving nothing still SAYS so, loudly and on the wire,
        # for the whole duration of a trial recording.
        self.create_timer(status_period_s, self._publish_status)

        self.get_logger().info(
            f'costmap_stop_monitor_node started: costmap={self._costmap_topic} '
            f'(nav2_msgs/Costmap, raw 0-255 cost scale), lifecycle='
            f'{lifecycle_node}, status={status_topic}, bool={obstacle_topic}, '
            f'marker={marker_topic}, window={self._forward_m}x'
            f'+/-{self._half_width_m} m, lethal_threshold='
            f'{self._lethal_threshold}, staleness_bound='
            f'{self._staleness_bound_s} s. Initial state is UNKNOWN until a '
            f'fresh costmap from an ACTIVE lifecycle node is received.')

    # --- Requirement 2: liveness ------------------------------------------

    def _on_transition_event(self, msg):
        self._lifecycle_label = msg.goal_state.label
        self.get_logger().info(
            f'{self._lifecycle_node_name} lifecycle transition: '
            f'{msg.start_state.label} -> {msg.goal_state.label}')

    def _poll_lifecycle_state(self):
        now = self.get_clock().now()

        if self._pending_lifecycle_call is not None:
            elapsed = (now - self._pending_lifecycle_since).nanoseconds / 1e9
            if elapsed > self._lifecycle_timeout_s:
                # No answer inside the bound. Absence of an answer is NOT an
                # affirmative answer: drop the confirmation entirely rather
                # than letting the previous 'active' label stand.
                self._pending_lifecycle_call = None
                self._lifecycle_label = None
                self.get_logger().warn(
                    f'get_state on {self._lifecycle_node_name} did not answer '
                    f'within {self._lifecycle_timeout_s:.2f} s; costmap '
                    f'liveness is now UNCONFIRMED.',
                    throttle_duration_sec=LOG_THROTTLE_SEC)
            else:
                return

        if not self._get_state_client.service_is_ready():
            self._lifecycle_label = None
            self.get_logger().warn(
                f'{self._lifecycle_node_name}/get_state is not available; '
                f'costmap liveness is UNCONFIRMED.',
                throttle_duration_sec=LOG_THROTTLE_SEC)
            return

        self._pending_lifecycle_since = now
        self._pending_lifecycle_call = self._get_state_client.call_async(
            GetState.Request())
        self._pending_lifecycle_call.add_done_callback(
            self._on_lifecycle_response)

    def _on_lifecycle_response(self, future):
        if self._pending_lifecycle_call is not future:
            # A timed-out call that answered late. Ignore it: it describes a
            # moment already recorded as unconfirmed.
            return
        self._pending_lifecycle_call = None
        try:
            self._lifecycle_label = future.result().current_state.label
        except Exception as exc:  # noqa: BLE001 - any failure is "no answer"
            self._lifecycle_label = None
            self.get_logger().warn(
                f'get_state on {self._lifecycle_node_name} failed: {exc!r}; '
                f'costmap liveness is UNCONFIRMED.',
                throttle_duration_sec=LOG_THROTTLE_SEC)

    # --- Step 7: camera health, the SECOND and independent axis ------------

    def _on_depth(self, msg):
        self._last_depth_recv = self.get_clock().now()

    def _camera_axis(self):
        return camera_health(self._age_s(self._last_depth_recv),
                             self._camera_timeout_s)

    # --- Steps 1-6: the costmap cycle --------------------------------------

    def _on_costmap(self, msg):
        self._last_costmap_recv = self.get_clock().now()
        self._costmap_frame = msg.header.frame_id

        # A message still pending when its successor arrives never got a TF
        # at its stamp within a whole costmap period. That is a REAL failure,
        # not a transient: record it in the tri-state rather than letting the
        # previous cycle's CLEAR stand unchallenged.
        if self._pending_costmap is not None:
            self._tf_ok = False
            self.get_logger().warn(
                f'no TF {self._costmap_frame} -> {self._robot_base_frame} at '
                f'the previous costmap message\'s stamp before the next '
                f'message arrived; that cycle was dropped unevaluated. State '
                f'is UNKNOWN, NOT clear.',
                throttle_duration_sec=LOG_THROTTLE_SEC)

        # Step 3: the one explicit assumption in window_mask().
        orientation = msg.metadata.origin.orientation
        if not is_identity_quaternion(orientation.x, orientation.y,
                                      orientation.z, orientation.w,
                                      self._origin_orientation_tol):
            self.get_logger().warn(
                f'costmap origin orientation is not identity '
                f'(x={orientation.x}, y={orientation.y}, z={orientation.z}, '
                f'w={orientation.w}); window_mask() maps grid indices to '
                f'world coordinates by pure translation and will mask the '
                f'WRONG CELLS for a rotated grid.',
                throttle_duration_sec=LOG_THROTTLE_SEC)

        self._pending_costmap = msg
        self._try_process()

    def _try_process(self):
        """Step 2 + Steps 4-6 for the pending costmap, if its TF exists yet.

        WHY A RETRY INSTEAD OF A DIRECT LOOKUP
        --------------------------------------
        The lookup is ALWAYS made at the pending message's OWN stamp. There
        is no fallback to the latest available pose, which would evaluate
        the window at the wrong place. But the costmap stamps a message at
        publish time, and that stamp is routinely a few milliseconds AHEAD
        of the newest odom->base_link transform, so a lookup attempted the
        instant the message arrives raises ExtrapolationException ("Lookup
        would require extrapolation into the future") almost every cycle.
        Measured live: with the lookup done directly in the subscription
        callback the monitor stayed in UNKNOWN(tf_lookup_failed) for a whole
        30 s run and NEVER reached CLEAR, at observed stamp-ahead-of-TF
        margins of 8.7, 7.7 and 18.1 ms.

        Blocking inside the callback for that transform was tried and is
        WORSE, also measured live: with `TransformListener(spin_thread=True)`
        and a 0.15 s blocking timeout, the buffer fell ~2.4 s behind and the
        node still never left UNKNOWN. Not isolated to a single cause; it was
        not pursued further because the non-blocking retry below removes the
        need for either mechanism.

        So: keep the message, return, and let a `tf_retry_period_s` timer try
        the SAME stamp again until the transform arrives. Every lookup is
        zero-timeout, so nothing blocks the executor that fills the buffer.
        The cost is at most one retry period of added latency when TF is not
        already ahead of the costmap, which bounds Task 12's rising-edge
        measurement rather than biasing it invisibly. If TF never catches up
        the message is dropped unevaluated by the next arrival, which sets
        tf_ok False and drives the tri-state to UNKNOWN(tf_lookup_failed) --
        the skip is bounded and reported, never silent.
        """
        msg = self._pending_costmap
        if msg is None:
            return
        try:
            tf = self._tf_buffer.lookup_transform(
                msg.header.frame_id, self._robot_base_frame,
                msg.header.stamp, timeout=Duration(seconds=0.0))
        except Exception as exc:  # noqa: BLE001 - tf2 raises several types
            self.get_logger().debug(
                f'TF {msg.header.frame_id} -> {self._robot_base_frame} at the '
                f'costmap stamp not available yet, will retry: {exc!r}')
            return

        self._pending_costmap = None
        self._tf_ok = True
        meta = msg.metadata
        rx = tf.transform.translation.x
        ry = tf.transform.translation.y
        q = tf.transform.rotation
        ryaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

        # Step 4. decode_raw_costmap() remaps NO_INFORMATION (255) to -1;
        # left raw it sits ABOVE LETHAL_OBSTACLE (254) and every unobserved
        # cell would read as an obstacle. See lib/costmap_raw.py.
        grid = decode_raw_costmap(msg.data, size_x=meta.size_x,
                                  size_y=meta.size_y)
        mask = window_mask(
            meta.size_x, meta.size_y, meta.resolution,
            meta.origin.position.x, meta.origin.position.y,
            rx, ry, ryaw, self._forward_m, self._half_width_m)
        self._obstacle_in_window = should_stop(
            grid, mask, meta.resolution, self._lethal_threshold,
            self._min_occupancy_fraction, self._min_cluster_area_cm2)

        evaluation = self._evaluate()

        # Step 6: the window as drawn must be the window as evaluated --
        # window_corners() and window_mask() share their transform, and a
        # unit test pins that they agree.
        self._publish_marker(msg.header.frame_id, rx, ry, ryaw, evaluation.state)

        # Step 5, as constrained by the tri-state ruling: publish the derived
        # Bool on the costmap cycle (so Task 12's rising-edge latency
        # measurement is unchanged), and ONLY for a determinate state.
        if should_publish_bool(evaluation.state):
            self._obstacle_pub.publish(Bool(data=bool_value(evaluation.state)))

    def _publish_marker(self, frame_id, rx, ry, ryaw, state):
        corners = window_corners(rx, ry, ryaw, self._forward_m,
                                 self._half_width_m)
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'costmap_stop_monitor'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.02
        marker.pose.orientation.w = 1.0
        marker.color.a = 1.0
        if state == STATE_OBSTACLE:
            marker.color.r = 1.0
        else:
            marker.color.g = 1.0
        for x, y in corners + [corners[0]]:  # closed loop
            marker.points.append(Point(x=float(x), y=float(y), z=0.0))
        self._marker_pub.publish(marker)

    # --- The authoritative tri-state and its publication -------------------

    def _age_s(self, stamp):
        if stamp is None:
            return None
        return (self.get_clock().now() - stamp).nanoseconds / 1e9

    def _evaluate(self):
        return evaluate_state(
            costmap_age_s=self._age_s(self._last_costmap_recv),
            staleness_bound_s=self._staleness_bound_s,
            lifecycle_state=self._lifecycle_label,
            tf_ok=self._tf_ok,
            obstacle_in_window=self._obstacle_in_window,
        )

    def _publish_status(self):
        evaluation = self._evaluate()
        costmap_age_s = self._age_s(self._last_costmap_recv)
        depth_age_s = self._age_s(self._last_depth_recv)
        camera_axis = self._camera_axis()

        self._log_transitions(evaluation, camera_axis, costmap_age_s)

        costmap_status = DiagnosticStatus()
        costmap_status.name = DIAG_COSTMAP_NAME
        costmap_status.hardware_id = self._costmap_topic
        # Requirement 5: UNKNOWN is ERROR-level on the wire, so it is
        # unmissable in rqt_robot_monitor and greppable in a rosbag when a
        # trial is audited afterwards for whether the monitor ever ran blind.
        costmap_status.level = (DiagnosticStatus.ERROR
                                if evaluation.state == STATE_UNKNOWN
                                else DiagnosticStatus.OK)
        costmap_status.message = evaluation.state
        costmap_status.values = [
            KeyValue(key='state', value=evaluation.state),
            KeyValue(key='reason', value=evaluation.reason),
            KeyValue(key='costmap_age_s',
                     value=('none' if costmap_age_s is None
                            else f'{costmap_age_s:.3f}')),
            KeyValue(key='costmap_staleness_bound_s',
                     value=f'{self._staleness_bound_s:.3f}'),
            KeyValue(key='costmap_lifecycle_node',
                     value=self._lifecycle_node_name),
            KeyValue(key='costmap_lifecycle_state',
                     value=('unconfirmed' if self._lifecycle_label is None
                            else self._lifecycle_label)),
            KeyValue(key='costmap_frame', value=self._costmap_frame),
            KeyValue(key='tf_ok', value=str(self._tf_ok)),
            KeyValue(key='obstacle_in_window',
                     value=str(self._obstacle_in_window)),
            KeyValue(key='bool_published',
                     value=str(should_publish_bool(evaluation.state))),
            KeyValue(key='window_forward_m', value=f'{self._forward_m:.3f}'),
            KeyValue(key='window_half_width_m',
                     value=f'{self._half_width_m:.3f}'),
            KeyValue(key='lethal_threshold', value=str(self._lethal_threshold)),
        ]

        # The camera axis is reported as its OWN DiagnosticStatus, not folded
        # into the costmap one. A dead depth camera leaves a perfectly valid
        # LiDAR-ONLY costmap: that is a legitimate CLEAR/OBSTACLE input, and
        # collapsing the two axes would either blind a working monitor or
        # hide the loss of the fusion contribution -- which is the whole
        # thing this project set out to measure.
        camera_status = DiagnosticStatus()
        camera_status.name = DIAG_CAMERA_NAME
        camera_status.hardware_id = self._costmap_topic
        camera_status.level = (DiagnosticStatus.OK
                               if camera_axis == CAMERA_FUSION_ACTIVE
                               else DiagnosticStatus.WARN)
        camera_status.message = camera_axis
        camera_status.values = [
            KeyValue(key='camera_axis', value=camera_axis),
            KeyValue(key='depth_age_s',
                     value=('none' if depth_age_s is None
                            else f'{depth_age_s:.3f}')),
            KeyValue(key='camera_timeout_s',
                     value=f'{self._camera_timeout_s:.3f}'),
            # window_forward_m (1.0 m) exceeds the camera's measured 0.94 m
            # coverage ceiling, so the far slice of the window is LiDAR-only
            # even while fusion is healthy. Reported here so a trigger near
            # the far edge is never written up as a camera detection.
            KeyValue(key='fusion_coverage_m',
                     value=f'{min(self._forward_m, self._camera_ceiling_m):.3f}'),
            KeyValue(key='lidar_only_beyond_m',
                     value=f'{self._camera_ceiling_m:.3f}'),
        ]

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [costmap_status, camera_status]
        self._status_pub.publish(array)

    def _log_transitions(self, evaluation, camera_axis, costmap_age_s):
        """Requirement 5: make degradation loud in the log, not just on the wire."""
        if evaluation.state == STATE_UNKNOWN:
            self.get_logger().error(
                f'MONITOR STATE UNKNOWN ({evaluation.reason}): this node is '
                f'NOT a valid safety input right now and this is NOT "clear". '
                f'costmap_age_s='
                f'{"none" if costmap_age_s is None else f"{costmap_age_s:.3f}"}'
                f' bound={self._staleness_bound_s:.3f} lifecycle='
                f'{self._lifecycle_label!r} tf_ok={self._tf_ok}. No '
                f'std_msgs/Bool is being published while degraded.',
                throttle_duration_sec=LOG_THROTTLE_SEC)

        if (evaluation.state, evaluation.reason) != (self._last_state,
                                                     self._last_reason):
            self.get_logger().info(
                f'monitor state: {self._last_state} -> {evaluation.state} '
                f'({evaluation.reason})')
            self._last_state = evaluation.state
            self._last_reason = evaluation.reason

        if camera_axis != self._last_camera_axis:
            if camera_axis == CAMERA_FUSION_ACTIVE:
                self.get_logger().info(
                    f'depth frames resumed on the camera axis: fusion is '
                    f'ACTIVE again (was {self._last_camera_axis}).')
            else:
                self.get_logger().error(
                    f'no depth frame within {self._camera_timeout_s:.1f} s: '
                    f'FUSION IS NO LONGER ACTIVE and the costmap is now '
                    f'LiDAR-ONLY. Low-profile obstacles the LiDAR plane '
                    f'misses will NOT be seen. Not halting -- degrading to '
                    f'the proven sensor is correct here; degrading silently '
                    f'is not.')
            self._last_camera_axis = camera_axis


def main(args=None):
    rclpy.init(args=args)
    node = CostmapStopMonitorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
