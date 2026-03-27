#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped


class CircleHandPosePublisher(Node):
    def __init__(self):
        super().__init__("circle_hand_pose_pub")

        self.hand_pub = self.create_publisher(PoseStamped, "/unity/hand_pose", 10)
        self.teleop_pub = self.create_publisher(Bool, "/unity/teleop_enabled", 10)

        self.rate_hz = 60.0
        self.dt = 1.0 / self.rate_hz

        # Hold still first so the filter can latch the anchor cleanly
        self.hold_time_sec = 2.0

        # Small gentle circle first
        self.radius = 0.015          # 1.5 cm
        self.period_sec = 12.0       # slow circle
        self.omega = 2.0 * math.pi / self.period_sec

        # Unity frame center
        # Mapping is:
        #   ROS x = Unity z
        #   ROS y = -Unity x
        #   ROS z = Unity y
        #
        # So varying Unity x and z makes a horizontal circle in ROS x-y.
        self.cx = 0.0
        self.cy = 0.0
        self.cz = 0.0

        self.t = 0.0
        self.start_time = self.get_clock().now()

        self.timer = self.create_timer(self.dt, self.on_timer)

        self.get_logger().info("circle_hand_pose_pub started")
        self.get_logger().info(f"radius={self.radius} m, period={self.period_sec} s")

    def on_timer(self):
        teleop = Bool()
        teleop.data = True
        self.teleop_pub.publish(teleop)

        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds * 1e-9

        msg = PoseStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "unity_world"

        if elapsed < self.hold_time_sec:
            x = self.cx
            y = self.cy
            z = self.cz
        else:
            tau = elapsed - self.hold_time_sec
            # Circle in Unity x-z plane
            x = self.cx + self.radius * math.cos(self.omega * tau)
            y = self.cy
            z = self.cz + self.radius * math.sin(self.omega * tau)

        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z

        # Keep orientation fixed
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        self.hand_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CircleHandPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()