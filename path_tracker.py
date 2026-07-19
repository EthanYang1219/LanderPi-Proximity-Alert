#!/usr/bin/env python3
"""
path_tracker.py

ROS 2 (Humble) node for the LanderPi ADAS proximity-alert experiment.

Drives the robot forward from point A toward point B and reactively
stops + turns away whenever the MS200 TOF LiDAR detects an obstacle
inside `safety_distance` within a forward arc centered on the robot's
heading.

Topics:
    Subscribes: /scan      (sensor_msgs/LaserScan)
    Publishes:  /cmd_vel   (geometry_msgs/Twist)

Parameters (all overridable via --ros-args -p name:=value or a launch file):
    safety_distance   (float, default 0.30) meters, stop threshold
    forward_speed     (float, default 0.15) m/s, constant forward speed
    turn_speed        (float, default 0.6)  rad/s, turn-away angular speed
    scan_arc_deg      (float, default 180.0) total forward arc width in degrees
                       (90 = looks 45 deg either side of straight ahead)

Run:
    ros2 run <your_package> path_tracker
    # or directly:
    python3 path_tracker.py --ros-args -p safety_distance:=0.30
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class PathTracker(Node):
    def __init__(self):
        super().__init__("path_tracker")

        self.declare_parameter("safety_distance", 0.30)
        self.declare_parameter("forward_speed", 0.15)
        self.declare_parameter("turn_speed", 0.6)
        self.declare_parameter("scan_arc_deg", 180.0)

        self.safety_distance = self.get_parameter("safety_distance").value
        self.forward_speed = self.get_parameter("forward_speed").value
        self.turn_speed = self.get_parameter("turn_speed").value
        self.scan_arc_deg = self.get_parameter("scan_arc_deg").value

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )

        self.obstacle_detected = False

        self.get_logger().info(
            f"path_tracker up: safety_distance={self.safety_distance} m, "
            f"forward_speed={self.forward_speed} m/s, "
            f"scan_arc_deg={self.scan_arc_deg} deg"
        )

    def scan_callback(self, msg: LaserScan):
        min_range = self._min_range_in_forward_arc(msg)

        cmd = Twist()

        if min_range is not None and min_range < self.safety_distance:
            if not self.obstacle_detected:
                self.get_logger().warn(
                    f"Obstacle at {min_range:.2f} m (< {self.safety_distance} m) "
                    "-- stopping and turning."
                )
            self.obstacle_detected = True
            cmd.linear.x = 0.0
            cmd.angular.z = self.turn_speed
        else:
            if self.obstacle_detected:
                self.get_logger().info("Path clear -- resuming forward travel.")
            self.obstacle_detected = False
            cmd.linear.x = self.forward_speed
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

    def _min_range_in_forward_arc(self, msg: LaserScan):
        """Return the closest valid range reading within the forward arc
        centered on 0 rad (straight ahead), or None if no valid readings."""
        half_arc_rad = math.radians(self.scan_arc_deg) / 2.0

        closest = None
        angle = msg.angle_min
        for r in msg.ranges:
            if -half_arc_rad <= angle <= half_arc_rad:
                if msg.range_min <= r <= msg.range_max and not math.isnan(r):
                    if closest is None or r < closest:
                        closest = r
            angle += msg.angle_increment

        return closest


def main(args=None):
    rclpy.init(args=args)
    node = PathTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_cmd = Twist()
        node.cmd_pub.publish(stop_cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
