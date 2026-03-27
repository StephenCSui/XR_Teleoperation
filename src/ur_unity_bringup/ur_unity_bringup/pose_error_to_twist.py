#!/usr/bin/env python3

import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped


Quat = Tuple[float, float, float, float]  # x, y, z, w


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def vec_norm(x: float, y: float, z: float) -> float:
    return math.sqrt(x * x + y * y + z * z)


def clamp_vec_norm(x: float, y: float, z: float, max_norm: float) -> Tuple[float, float, float]:
    n = vec_norm(x, y, z)
    if n < 1e-12 or n <= max_norm:
        return x, y, z
    s = max_norm / n
    return x * s, y * s, z * s


def quat_normalize(q: Quat) -> Quat:
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / n, y / n, z / n, w / n)


def quat_conjugate(q: Quat) -> Quat:
    x, y, z, w = q
    return (-x, -y, -z, w)


def quat_multiply(q1: Quat, q2: Quat) -> Quat:
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2

    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    return (x, y, z, w)


def quat_inverse(q: Quat) -> Quat:
    return quat_conjugate(quat_normalize(q))


def quat_to_axis_angle(q: Quat) -> Tuple[Tuple[float, float, float], float]:
    x, y, z, w = quat_normalize(q)

    if w < 0.0:
        x = -x
        y = -y
        z = -z
        w = -w

    sin_half = math.sqrt(x * x + y * y + z * z)
    if sin_half < 1e-12:
        return (0.0, 0.0, 0.0), 0.0

    axis = (x / sin_half, y / sin_half, z / sin_half)
    angle = 2.0 * math.atan2(sin_half, w)

    if angle > math.pi:
        angle -= 2.0 * math.pi

    return axis, angle


class PoseErrorToTwist(Node):
    def __init__(self) -> None:
        super().__init__("pose_error_to_twist")

        self.desired_pose_topic = self.declare_parameter(
            "desired_pose_topic", "/unity/command_pose"
        ).value
        self.current_pose_topic = self.declare_parameter(
            "current_pose_topic", "/robot/ee_pose"
        ).value
        self.twist_topic = self.declare_parameter(
            "twist_topic", "/servo_node/delta_twist_cmds"
        ).value
        self.base_frame = self.declare_parameter(
            "base_frame", "base_link"
        ).value

        self.linear_gain = float(self.declare_parameter("linear_gain", 4.0).value)
        self.angular_gain = float(self.declare_parameter("angular_gain", 3.0).value)

        self.max_linear_speed = float(self.declare_parameter("max_linear_speed", 0.15).value)
        self.max_angular_speed = float(self.declare_parameter("max_angular_speed", 0.8).value)

        self.position_deadband_m = float(self.declare_parameter("position_deadband_m", 0.001).value)
        self.angle_deadband_rad = float(self.declare_parameter("angle_deadband_rad", 0.01).value)

        self.publish_rate_hz = float(self.declare_parameter("publish_rate_hz", 60.0).value)
        self.debug_verbose = bool(self.declare_parameter("debug_verbose", False).value)

        self.desired_pose: Optional[PoseStamped] = None
        self.current_pose: Optional[PoseStamped] = None

        self.sub_desired = self.create_subscription(
            PoseStamped,
            self.desired_pose_topic,
            self.on_desired_pose,
            10,
        )

        self.sub_current = self.create_subscription(
            PoseStamped,
            self.current_pose_topic,
            self.on_current_pose,
            10,
        )

        self.pub_twist = self.create_publisher(
            TwistStamped,
            self.twist_topic,
            10,
        )

        period = 1.0 / max(self.publish_rate_hz, 1.0)
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info("pose_error_to_twist started")
        self.get_logger().info(f"desired_pose_topic: {self.desired_pose_topic}")
        self.get_logger().info(f"current_pose_topic: {self.current_pose_topic}")
        self.get_logger().info(f"twist_topic: {self.twist_topic}")

    def on_desired_pose(self, msg: PoseStamped) -> None:
        self.desired_pose = msg

    def on_current_pose(self, msg: PoseStamped) -> None:
        self.current_pose = msg

    def on_timer(self) -> None:
        if self.desired_pose is None or self.current_pose is None:
            return

        dp_x = self.desired_pose.pose.position.x - self.current_pose.pose.position.x
        dp_y = self.desired_pose.pose.position.y - self.current_pose.pose.position.y
        dp_z = self.desired_pose.pose.position.z - self.current_pose.pose.position.z

        pos_err_norm = vec_norm(dp_x, dp_y, dp_z)
        if pos_err_norm < self.position_deadband_m:
            lin_x, lin_y, lin_z = 0.0, 0.0, 0.0
        else:
            lin_x = self.linear_gain * dp_x
            lin_y = self.linear_gain * dp_y
            lin_z = self.linear_gain * dp_z
            lin_x, lin_y, lin_z = clamp_vec_norm(
                lin_x, lin_y, lin_z, self.max_linear_speed
            )

        q_des = quat_normalize((
            self.desired_pose.pose.orientation.x,
            self.desired_pose.pose.orientation.y,
            self.desired_pose.pose.orientation.z,
            self.desired_pose.pose.orientation.w,
        ))

        q_cur = quat_normalize((
            self.current_pose.pose.orientation.x,
            self.current_pose.pose.orientation.y,
            self.current_pose.pose.orientation.z,
            self.current_pose.pose.orientation.w,
        ))

        q_err = quat_multiply(q_des, quat_inverse(q_cur))
        axis, angle = quat_to_axis_angle(q_err)

        if abs(angle) < self.angle_deadband_rad:
            ang_x, ang_y, ang_z = 0.0, 0.0, 0.0
        else:
            ang_x = self.angular_gain * angle * axis[0]
            ang_y = self.angular_gain * angle * axis[1]
            ang_z = self.angular_gain * angle * axis[2]
            ang_x, ang_y, ang_z = clamp_vec_norm(
                ang_x, ang_y, ang_z, self.max_angular_speed
            )

        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = self.base_frame

        twist.twist.linear.x = lin_x
        twist.twist.linear.y = lin_y
        twist.twist.linear.z = lin_z

        twist.twist.angular.x = ang_x
        twist.twist.angular.y = ang_y
        twist.twist.angular.z = ang_z

        self.pub_twist.publish(twist)

        if self.debug_verbose:
            self.get_logger().info(
                f"pos_err=({dp_x:.4f}, {dp_y:.4f}, {dp_z:.4f}) "
                f"lin=({lin_x:.4f}, {lin_y:.4f}, {lin_z:.4f}) "
                f"ang=({ang_x:.4f}, {ang_y:.4f}, {ang_z:.4f})"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PoseErrorToTwist()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()