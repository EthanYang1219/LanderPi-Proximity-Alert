import math

import rclpy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from proximity_alert.avoidance import DRIVE, HALT, STRAFE, ControllerOutput
from proximity_alert.path_tracker import PathTracker


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

    def __init__(self, state=DRIVE, linear_x=0.5):
        self.state = state
        self.goal_heading = None
        self.step_calls = 0
        self._out = ControllerOutput(linear_x, 0.0, 0.0, state, None)

    def set_goal_heading(self, yaw):
        self.goal_heading = yaw

    def step(self, sectors, obstacle, gap, yaw, now):
        self.step_calls += 1
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
    cmd = node._last_cmd
    node.destroy_node()
    rclpy.shutdown()
    assert cmd.linear.x == 0.0
