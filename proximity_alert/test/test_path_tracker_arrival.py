import math

import pytest
import rclpy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from proximity_alert.avoidance import DRIVE, HALT, STRAFE, ControllerOutput
from proximity_alert.decision_record import DecisionRecord
from proximity_alert.path_tracker import PathTracker


@pytest.fixture(autouse=True)
def _ensure_rclpy_shutdown():
    """Guarantee rclpy.shutdown() runs even if a test body raises.

    Every test in this file calls rclpy.init() at the top and
    rclpy.shutdown() before its asserts -- deliberate, so a failed assert
    never masks a shutdown bug. But if the test BODY itself raises before
    reaching that shutdown call, shutdown() is skipped and every subsequent
    test in the run dies with "rcl_init called while already initialized",
    turning one real failure into a misleading cascade. This fixture is the
    safety net: it never runs shutdown() itself when a test's own call
    already succeeded (rclpy.ok() is False by then), so normal passing/
    failing-on-assert tests are unaffected.
    """
    yield
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass


def _decision(state="STRAFE", encounter_id=1):
    return DecisionRecord(
        timestamp="2026-07-24 10:00:00", encounter_id=encounter_id, state=state,
        chosen_maneuver=state if state != "DRIVE" else "NONE", reason="test",
        obstacle_span_deg=10.0, front_distance_m=0.29, front_left_m=0.5,
        front_center_m=0.29, front_right_m=0.6, left_clearance_m=1.2,
        right_clearance_m=0.4, rear_clearance_m=2.0, required_clearing_m=0.22,
        cumulative_strafe_m=0.0, consecutive_avoid_count=1,
        recovery_triggered=False, outcome="committed", maneuver_duration_s=0.0,
    )


class _FakeController:
    """Stands in for AvoidanceController so these tests exercise the gating
    logic, not the state machine.

    Forcing the real controller's `state` attribute directly raises
    TypeError -- `_maneuver_start` stays None until `_assess()` runs -- and
    driving it into STRAFE/HALT legitimately would take a fixture far
    larger than the behavior under test. The real controller is covered by
    test_avoidance_controller.py; what needs proving here is that
    path_tracker forwards or replaces its output correctly.
    """

    def __init__(self, state=DRIVE, linear_x=0.5, decision=None, linear_y=0.0):
        self.state = state
        self.goal_heading = None
        self.lateral_correction = 0.0
        self.step_calls = 0
        self.last_pos = None
        self._out = ControllerOutput(linear_x, linear_y, 0.0, state, decision)

    def set_goal_heading(self, yaw):
        self.goal_heading = yaw

    def set_lateral_correction(self, v_lat):
        # odom_callback pushes this every tick; the real gating/ramping is
        # covered in test_avoidance_controller.py. Recorded so the
        # cross-track tests below can assert what path_tracker computed.
        self.lateral_correction = v_lat

    def step(self, sectors, obstacle, gap, yaw, pos, now):
        self.step_calls += 1
        # Recorded so the tests below can assert path_tracker threads the
        # odometry position through; the real controller measures clear-drive
        # displacement from it.
        self.last_pos = pos
        return self._out


def _clear_scan():
    scan = LaserScan()
    scan.angle_min = -math.pi
    scan.angle_max = math.pi
    scan.angle_increment = math.pi / 90
    scan.range_min = 0.05
    scan.range_max = 12.0
    scan.ranges = [5.0] * 180
    return scan


def _odom_at(x, y):
    msg = Odometry()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.w = 1.0
    return msg


def _primed(node, target_distance, controller=None):
    """Node with a fresh scan and a start odom fix, ready to drive."""
    node.target_distance = target_distance
    if controller is not None:
        node.controller = controller
    node.odom_callback(_odom_at(0.0, 0.0))
    node.scan_callback(_clear_scan())
    return node


# --- backward compatibility, real controller ---

def test_disabled_target_drives_forward_unchanged():
    rclpy.init()
    node = _primed(PathTracker(), 0.0)
    node.odom_callback(_odom_at(99.0, 0.0))
    node.control_loop()
    cmd, status = node._last_cmd, node._last_status
    node.destroy_node()
    rclpy.shutdown()
    assert cmd.linear.x > 0.0          # still driving after 99 m
    assert status == "driving"


def test_stale_odom_ignored_when_target_disabled():
    # No odom watchdog existed before this feature.
    rclpy.init()
    node = PathTracker()
    node.target_distance = 0.0
    node.scan_callback(_clear_scan())   # fresh scan, but no odom ever
    node.control_loop()
    cmd = node._last_cmd
    node.destroy_node()
    rclpy.shutdown()
    assert cmd.linear.x > 0.0


# --- target arrival, fake controller ---

def test_stops_and_reports_arrival_at_target():
    rclpy.init()
    node = _primed(PathTracker(), 2.0, _FakeController(state=DRIVE))
    node.odom_callback(_odom_at(2.5, 0.0))
    node.control_loop()
    cmd, status = node._last_cmd, node._last_status
    node.destroy_node()
    rclpy.shutdown()
    assert cmd.linear.x == 0.0
    assert cmd.angular.z == 0.0
    assert status == "arrived_target_distance"


def test_arrival_deferred_until_controller_returns_to_drive():
    rclpy.init()
    fake = _FakeController(state=STRAFE)
    node = _primed(PathTracker(), 2.0, fake)
    node.odom_callback(_odom_at(2.5, 0.0))
    node.control_loop()
    mid_status, mid_cmd = node._last_status, node._last_cmd
    fake.state = DRIVE                      # maneuver finished
    node.control_loop()
    after = node._last_status
    node.destroy_node()
    rclpy.shutdown()
    assert mid_status == "driving"          # did not abort the strafe
    assert mid_cmd.linear.x > 0.0           # controller output forwarded
    assert after == "arrived_target_distance"


def test_decision_on_arrival_tick_still_published():
    # Regression: the decision-publish block used to sit AFTER the arrival
    # check, which `return`ed before it ran -- dropping any DecisionRecord
    # produced on the same tick arrival is decided. This fires systematically
    # when arrival is deferred through a maneuver: the first DRIVE tick IS
    # the _complete_to_drive tick carrying the "cleared" record.
    rclpy.init()
    decision = _decision(state="DRIVE", encounter_id=7)
    fake = _FakeController(state=DRIVE, decision=decision)
    node = _primed(PathTracker(), 2.0, fake)
    node.odom_callback(_odom_at(2.5, 0.0))
    published = []
    node.decision_pub.publish = lambda msg: published.append(msg)
    node.control_loop()
    status = node._last_status
    node.destroy_node()
    rclpy.shutdown()
    assert status == "arrived_target_distance"
    assert len(published) == 1
    assert '"encounter_id": 7' in published[0].data


def test_arrival_latches_and_stops_stepping_controller():
    rclpy.init()
    fake = _FakeController(state=DRIVE)
    node = _primed(PathTracker(), 2.0, fake)
    node.odom_callback(_odom_at(2.5, 0.0))
    node.control_loop()
    calls_at_arrival = fake.step_calls
    node.control_loop()
    node.control_loop()
    cmd, status, calls_after = node._last_cmd, node._last_status, fake.step_calls
    node.destroy_node()
    rclpy.shutdown()
    assert cmd.linear.x == 0.0
    assert status == "arrived_target_distance"
    # Latched: the state machine is not ticked against motion that is not
    # happening, so its maneuver timers cannot drift.
    assert calls_after == calls_at_arrival


# --- obstacle halt ---

def test_controller_halt_reports_arrived_obstacle():
    rclpy.init()
    node = _primed(PathTracker(), 5.0, _FakeController(state=HALT, linear_x=0.0))
    node.control_loop()
    status = node._last_status
    node.destroy_node()
    rclpy.shutdown()
    assert status == "arrived_obstacle"


def test_halt_is_still_stepped_so_it_can_recover():
    # HALT is recoverable -- _halt() returns to DRIVE once the front stays
    # clear. Skipping step() while halted would freeze the robot forever.
    rclpy.init()
    fake = _FakeController(state=HALT, linear_x=0.0)
    node = _primed(PathTracker(), 5.0, fake)
    node.control_loop()
    node.control_loop()
    calls = fake.step_calls
    node.destroy_node()
    rclpy.shutdown()
    assert calls == 2


# --- stale-odom watchdog ---

def test_stale_odom_stops_when_target_set():
    rclpy.init()
    fake = _FakeController(state=DRIVE)
    node = PathTracker()
    node.target_distance = 5.0
    node.controller = fake
    node.scan_callback(_clear_scan())   # fresh scan, but no odom ever
    node.control_loop()
    cmd, status = node._last_cmd, node._last_status
    node.destroy_node()
    rclpy.shutdown()
    assert cmd.linear.x == 0.0
    assert status == "stopped_odom_fault"
    assert fake.step_calls == 0         # not stepped on a fault stop


# --- stale scan still wins, unchanged ---

def test_stale_scan_still_holds():
    rclpy.init()
    node = PathTracker()
    node.target_distance = 2.0
    node.odom_callback(_odom_at(0.0, 0.0))   # odom fresh, scan never arrived
    node.control_loop()
    cmd, status = node._last_cmd, node._last_status
    node.destroy_node()
    rclpy.shutdown()
    assert cmd.linear.x == 0.0
    assert status == "stopped_scan_fault"


def test_stale_scan_after_arrival_still_reports_arrived():
    # Regression: the stale-scan hold branch used to run before the
    # arrival/skip check and always published DRIVING, so a latched arrival
    # followed by a LiDAR dropout (or the operator stopping the lidar node
    # at end of run) flipped the published status back to "driving" even
    # though self.arrived stayed latched and the robot never moved.
    rclpy.init()
    fake = _FakeController(state=DRIVE)
    node = _primed(PathTracker(), 2.0, fake)
    node.odom_callback(_odom_at(2.5, 0.0))
    node.control_loop()
    assert node._last_status == "arrived_target_distance"   # sanity check
    node.last_scan_time = None   # simulate the scan going stale
    node.control_loop()
    cmd, status = node._last_cmd, node._last_status
    node.destroy_node()
    rclpy.shutdown()
    assert cmd.linear.x == 0.0
    assert status == "arrived_target_distance"


# --- arrival measured along the line, not as displacement ---

def test_arrival_uses_along_track_not_displacement():
    # Regression for the early-stop bug: 1.20 m down the line but 0.40 m off
    # to the side reads hypot = 1.265 m, which would have tripped a 1.25 m
    # target having actually advanced only 1.20 m.
    rclpy.init()
    node = _primed(PathTracker(), 1.25, _FakeController(state=DRIVE))
    node.odom_callback(_odom_at(1.20, 0.40))
    node.control_loop()
    status, along = node._last_status, node.along_track
    node.destroy_node()
    rclpy.shutdown()
    assert math.hypot(1.20, 0.40) > 1.25      # displacement would have stopped
    assert along == pytest.approx(1.20)
    assert status == "driving"


def test_arrives_once_along_track_actually_reaches_target():
    rclpy.init()
    node = _primed(PathTracker(), 1.25, _FakeController(state=DRIVE))
    node.odom_callback(_odom_at(1.30, 0.02))   # offset inside tolerance
    node.control_loop()
    status = node._last_status
    node.destroy_node()
    rclpy.shutdown()
    assert status == "arrived_target_distance"


# --- arrival centering phase ---

def test_centering_stops_forward_motion_but_keeps_crabbing():
    rclpy.init()
    node = _primed(PathTracker(), 2.0,
                   _FakeController(state=DRIVE, linear_y=-0.10))
    node.odom_callback(_odom_at(2.5, 0.30))     # distance covered, still off line
    node.control_loop()
    cmd, status = node._last_cmd, node._last_status
    arrived = node.arrived
    node.destroy_node()
    rclpy.shutdown()
    assert status == "centering"
    assert cmd.linear.x == 0.0                  # run length already decided
    assert cmd.linear.y == -0.10                # still sliding onto the line
    assert arrived is False                     # not latched yet


def test_centering_finishes_once_within_tolerance():
    rclpy.init()
    node = _primed(PathTracker(), 2.0, _FakeController(state=DRIVE))
    node.odom_callback(_odom_at(2.5, 0.30))
    node.control_loop()
    assert node._last_status == "centering"
    node.odom_callback(_odom_at(2.5, 0.01))     # crabbed onto the line
    node.control_loop()
    cmd, status = node._last_cmd, node._last_status
    node.destroy_node()
    rclpy.shutdown()
    assert status == "arrived_target_distance"
    assert cmd.linear.x == 0.0 and cmd.linear.y == 0.0


def test_centering_gives_up_at_timeout():
    # A blocked flank can make the offset uncloseable; the run must still
    # terminate. centering_timeout=0 is the degenerate "never centre" case.
    rclpy.init()
    node = PathTracker()
    node.config.centering_timeout = 0.0
    node = _primed(node, 2.0, _FakeController(state=DRIVE))
    node.odom_callback(_odom_at(2.5, 0.90))     # far off line
    node.control_loop()
    status = node._last_status
    node.destroy_node()
    rclpy.shutdown()
    assert status == "arrived_target_distance"


def test_centering_disabled_by_zero_tolerance():
    rclpy.init()
    node = PathTracker()
    node.config.cross_track_tolerance = 0.0
    node = _primed(node, 2.0, _FakeController(state=DRIVE))
    node.odom_callback(_odom_at(2.5, 0.90))
    node.control_loop()
    status = node._last_status
    node.destroy_node()
    rclpy.shutdown()
    assert status == "arrived_target_distance"


def test_obstacle_halt_during_centering_reports_obstacle_stop():
    rclpy.init()
    node = _primed(PathTracker(), 2.0, _FakeController(state=HALT))
    node.odom_callback(_odom_at(2.5, 0.30))
    node.control_loop()
    status = node._last_status
    node.destroy_node()
    rclpy.shutdown()
    assert status == "arrived_obstacle"


def test_odometry_position_is_threaded_into_the_controller():
    # The controller measures clear-drive displacement from this position to
    # decide when an obstacle encounter is over, so path_tracker must hand it
    # the live odometry fix -- not None, and not a stale first fix.
    rclpy.init()
    fake = _FakeController(state=DRIVE)
    node = _primed(PathTracker(), 0.0, fake)
    node.odom_callback(_odom_at(1.25, -0.5))
    node.control_loop()
    node.destroy_node()
    rclpy.shutdown()
    assert fake.last_pos == (1.25, -0.5)


def test_controller_position_is_none_before_any_odom():
    # No odometry yet must mean no fabricated coordinates -- the controller
    # relies on None to keep the clear-distance measurement from starting.
    rclpy.init()
    fake = _FakeController(state=DRIVE)
    node = PathTracker()
    node.target_distance = 0.0
    node.controller = fake
    node.scan_callback(_clear_scan())     # fresh scan, but no odom ever
    node.control_loop()
    node.destroy_node()
    rclpy.shutdown()
    assert fake.last_pos is None
