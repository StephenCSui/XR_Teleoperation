import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener

class EeTfToPose(Node):
    def __init__(self):
        super().__init__("ee_tf_to_pose")
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.ee_frame = self.declare_parameter("ee_frame", "tool0").value
        self.out_topic = self.declare_parameter("topic_name", "/robot/ee_pose").value
        self.rate_hz = float(self.declare_parameter("rate_hz", 60.0).value)

        self.pub = self.create_publisher(PoseStamped, self.out_topic, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0 / self.rate_hz, self.tick)

        self.get_logger().info(f"Publishing {self.base_frame}->{self.ee_frame} as {self.out_topic}")

    def tick(self):
        try:
            t = self.tf_buffer.lookup_transform(self.base_frame, self.ee_frame, rclpy.time.Time())
        except Exception:
            return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.pose.position.x = t.transform.translation.x
        msg.pose.position.y = t.transform.translation.y
        msg.pose.position.z = t.transform.translation.z
        msg.pose.orientation = t.transform.rotation
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = EeTfToPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
