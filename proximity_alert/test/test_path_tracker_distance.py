import rclpy
from nav_msgs.msg import Odometry

from proximity_alert.path_tracker import PathTracker


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
    # start_pos must be a snapshot, not a reference into the last message.
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
