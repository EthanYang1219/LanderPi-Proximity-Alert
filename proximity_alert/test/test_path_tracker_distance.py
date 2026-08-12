import pytest
import rclpy
from nav_msgs.msg import Odometry

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


def _odom_at(x, y):
    msg = Odometry()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.w = 1.0
    return msg


def test_target_distance_defaults_to_disabled():
    rclpy.init()
    node = PathTracker()
    target, timeout = node.target_distance, node.odom_timeout_sec
    node.destroy_node()
    rclpy.shutdown()
    assert target == 0.0
    assert timeout == 1.0


def test_distance_starts_at_zero():
    rclpy.init()
    node = PathTracker()
    d, arrived = node.distance_traveled, node.arrived
    node.destroy_node()
    rclpy.shutdown()
    assert d == 0.0
    assert arrived is False


def test_tracks_distance_from_first_odom():
    rclpy.init()
    node = PathTracker()
    node.odom_callback(_odom_at(10.0, 10.0))   # start is wherever it boots
    node.odom_callback(_odom_at(13.0, 14.0))
    d = node.distance_traveled
    node.destroy_node()
    rclpy.shutdown()
    assert d == 5.0


def test_start_position_does_not_drift():
    # start_pos must be latched once, on the FIRST /odom message, and never
    # re-stamped on subsequent ones -- not a test of middleware buffer reuse
    # (these Odometry objects are freshly constructed, so buffer reuse is
    # never exercised here).
    rclpy.init()
    node = PathTracker()
    node.odom_callback(_odom_at(1.0, 0.0))
    node.odom_callback(_odom_at(4.0, 0.0))
    node.odom_callback(_odom_at(7.0, 0.0))
    d = node.distance_traveled
    node.destroy_node()
    rclpy.shutdown()
    assert d == 6.0     # 7 - 1, not 7 - 4


def test_odom_callback_still_sets_goal_heading():
    # Regression guard: distance tracking must not displace the existing
    # heading-hold latch, which is what keeps the robot driving straight.
    rclpy.init()
    node = PathTracker()
    node.odom_callback(_odom_at(0.0, 0.0))
    goal_set, yaw = node._goal_set, node.current_yaw
    controller_goal = node.controller.goal_heading
    node.destroy_node()
    rclpy.shutdown()
    assert goal_set is True
    assert yaw is not None
    assert controller_goal is not None


def test_odom_freshness_tracked():
    rclpy.init()
    node = PathTracker()
    stale_before = node.odom_is_stale()
    node.odom_callback(_odom_at(0.0, 0.0))
    stale_after = node.odom_is_stale()
    node.destroy_node()
    rclpy.shutdown()
    assert stale_before is True     # nothing received yet
    assert stale_after is False


# --- path-frame tracking and the crab correction it feeds ---

def _at_heading_zero(node):
    """Latch the A->B line as +x, so along-track is x and cross-track is y."""
    node.odom_callback(_odom_at(0.0, 0.0))
    return node


def test_tracks_along_and_cross_track_from_odom():
    rclpy.init()
    node = _at_heading_zero(PathTracker())
    node.odom_callback(_odom_at(1.2, 0.4))
    along, cross = node.along_track, node.cross_track
    node.destroy_node()
    rclpy.shutdown()
    assert along == pytest.approx(1.2)
    assert cross == pytest.approx(0.4)      # +y is left of the line


def test_along_track_unaffected_by_lateral_drift():
    rclpy.init()
    node = _at_heading_zero(PathTracker())
    node.odom_callback(_odom_at(1.2, 0.0))
    straight = node.along_track
    node.odom_callback(_odom_at(1.2, -0.7))
    drifted = node.along_track
    node.destroy_node()
    rclpy.shutdown()
    assert straight == pytest.approx(drifted)


def test_correction_pushed_to_controller_with_correcting_sign():
    rclpy.init()
    node = _at_heading_zero(PathTracker())
    node.odom_callback(_odom_at(1.0, 0.4))       # drifted left
    left_drift = node.controller._lateral_correction
    node.odom_callback(_odom_at(1.0, -0.4))      # drifted right
    right_drift = node.controller._lateral_correction
    node.destroy_node()
    rclpy.shutdown()
    assert left_drift < 0.0                      # crab right, back to the line
    assert right_drift > 0.0                     # crab left


def test_no_correction_requested_while_on_the_line():
    rclpy.init()
    node = _at_heading_zero(PathTracker())
    node.odom_callback(_odom_at(3.0, 0.0))
    v = node.controller._lateral_correction
    node.destroy_node()
    rclpy.shutdown()
    assert v == 0.0


def test_correction_respects_the_configured_clamp():
    rclpy.init()
    node = _at_heading_zero(PathTracker())
    node.odom_callback(_odom_at(1.0, 5.0))       # absurdly far off line
    v, limit = node.controller._lateral_correction, node.config.cross_track_max_speed
    node.destroy_node()
    rclpy.shutdown()
    assert abs(v) == pytest.approx(limit)


def test_start_position_latches_the_line_not_just_the_origin():
    # The line is (start_pos, goal_heading). Starting away from the odom
    # origin must not make the robot think it is already off the line.
    rclpy.init()
    node = PathTracker()
    node.odom_callback(_odom_at(10.0, -4.0))     # first fix: this is point A
    node.odom_callback(_odom_at(11.0, -4.0))     # 1 m straight down +x
    along, cross = node.along_track, node.cross_track
    node.destroy_node()
    rclpy.shutdown()
    assert along == pytest.approx(1.0)
    assert cross == pytest.approx(0.0)
