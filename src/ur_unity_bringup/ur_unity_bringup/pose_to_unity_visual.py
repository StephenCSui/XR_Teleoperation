#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


def q_normalize(x, y, z, w):
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if n < 1e-12:
        return 0.0, 0.0, 0.0, 1.0
    return x / n, y / n, z / n, w / n


def ros_to_unity_position(px, py, pz):
    return -py, pz, px


def ros_to_unity_quat(qx, qy, qz, qw):
    # Exact basis remap for:
    # ROS  : x forward, y left,  z up
    # Unity: x right,   y up,    z forward
    return q_normalize(
        -qy,
         qz,
         qx,
        -qw
    )


class PoseToUnityVisual(Node):
    def __init__(self):
        super().__init__("pose_to_unity_visual")

        self.input_topic = self.declare_parameter("input_topic", "/robot/ee_pose").value
        self.output_topic = self.declare_parameter("output_topic", "/unity_vis/pose").value
        self.rotation_mode = self.declare_parameter("rotation_mode", "full").value

        self.sub = self.create_subscription(
            PoseStamped,
            self.input_topic,
            self.on_pose,
            10,
        )

        self.pub = self.create_publisher(
            PoseStamped,
            self.output_topic,
            10,
        )

        self.get_logger().info(
            f"pose_to_unity_visual: {self.input_topic} -> {self.output_topic} mode={self.rotation_mode}"
        )

    def on_pose(self, msg: PoseStamped):
        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "unity_world"

        ux, uy, uz = ros_to_unity_position(
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        )
        out.pose.position.x = ux
        out.pose.position.y = uy
        out.pose.position.z = uz

        if self.rotation_mode == "position_only":
            out.pose.orientation.x = 0.0
            out.pose.orientation.y = 0.0
            out.pose.orientation.z = 0.0
            out.pose.orientation.w = 1.0
        else:
            qx, qy, qz, qw = ros_to_unity_quat(
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            )
            out.pose.orientation.x = qx
            out.pose.orientation.y = qy
            out.pose.orientation.z = qz
            out.pose.orientation.w = qw

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PoseToUnityVisual()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()