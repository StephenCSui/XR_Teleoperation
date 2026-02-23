import math
import rclpy

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Bool


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def quat_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class UnityPoseToServoTwist(Node):
    def __init__(self):
        super().__init__("unity_pose_to_servo_twist")

        self.declare_parameter("unity_pose_topic", "/unity/target_pose")
        self.declare_parameter("enable_topic", "/unity/teleop_enable")
        self.declare_parameter("servo_twist_topic", "/servo_node/delta_twist_cmds")
        self.declare_parameter("output_frame", "base_link")

        self.declare_parameter("rate_hz", 60.0)
        self.declare_parameter("pos_gain", 2.0)
        self.declare_parameter("yaw_gain", 2.0)

        self.declare_parameter("max_lin_vel", 0.15)
        self.declare_parameter("max_ang_vel", 0.8)

        self.unity_pose_topic = self.get_parameter("unity_pose_topic").value
        self.enable_topic = self.get_parameter("enable_topic").value
        self.servo_twist_topic = self.get_parameter("servo_twist_topic").value
        self.output_frame = self.get_parameter("output_frame").value

        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.pos_gain = float(self.get_parameter("pos_gain").value)
        self.yaw_gain = float(self.get_parameter("yaw_gain").value)

        self.max_lin_vel = float(self.get_parameter("max_lin_vel").value)
        self.max_ang_vel = float(self.get_parameter("max_ang_vel").value)

        self.latest_target = None

        self.teleop_enabled = False
        self.zero_set = False
        self.zero_x = 0.0
        self.zero_y = 0.0
        self.zero_z = 0.0
        self.zero_yaw = 0.0

        self.sub = self.create_subscription(
            PoseStamped,
            self.unity_pose_topic,
            self.on_target,
            10,
        )

        self.enable_sub = self.create_subscription(
            Bool,
            self.enable_topic,
            self.on_enable,
            10,
        )

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE

        self.pub = self.create_publisher(
            TwistStamped,
            self.servo_twist_topic,
            qos,
        )

        self.timer = self.create_timer(1.0 / self.rate_hz, self.on_timer)

        self.get_logger().info(f"Subscribing pose:   {self.unity_pose_topic}")
        self.get_logger().info(f"Subscribing enable: {self.enable_topic}")
        self.get_logger().info(f"Publishing twist:   {self.servo_twist_topic}")
        self.get_logger().info(f"Output frame:       {self.output_frame}")

    def on_enable(self, msg: Bool):
        if msg.data and not self.teleop_enabled:
            self.zero_set = False
        self.teleop_enabled = bool(msg.data)

    def on_target(self, msg: PoseStamped):
        self.latest_target = msg

        if self.teleop_enabled and not self.zero_set:
            p = msg.pose.position
            self.zero_x = float(p.x)
            self.zero_y = float(p.y)
            self.zero_z = float(p.z)
            self.zero_yaw = quat_to_yaw(msg.pose.orientation)
            self.zero_set = True
            self.get_logger().info("Latched zero pose")

    def on_timer(self):
        if not self.teleop_enabled:
            return
        if self.latest_target is None:
            return
        if not self.zero_set:
            return

        p = self.latest_target.pose.position
        q = self.latest_target.pose.orientation

        dx = float(p.x) - self.zero_x
        dy = float(p.y) - self.zero_y
        dz = float(p.z) - self.zero_z
        dyaw = wrap_pi(quat_to_yaw(q) - self.zero_yaw)

        vx = clamp(self.pos_gain * dx, -self.max_lin_vel, self.max_lin_vel)
        vy = clamp(self.pos_gain * dy, -self.max_lin_vel, self.max_lin_vel)
        vz = clamp(self.pos_gain * dz, -self.max_lin_vel, self.max_lin_vel)

        wz = clamp(self.yaw_gain * dyaw, -self.max_ang_vel, self.max_ang_vel)

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.output_frame

        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = vz

        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = wz

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = UnityPoseToServoTwist()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()