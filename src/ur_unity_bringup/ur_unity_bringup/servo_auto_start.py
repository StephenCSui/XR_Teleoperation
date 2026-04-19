#!/usr/bin/env python3

import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from builtin_interfaces.msg import Duration
from std_msgs.msg import Bool
from std_srvs.srv import Empty, Trigger
from controller_manager_msgs.srv import SwitchController


class ServoAutoStart(Node):
    def __init__(self) -> None:
        super().__init__("servo_auto_start")

        self.robot_program_topic = self.declare_parameter(
            "robot_program_running_topic",
            "/io_and_status_controller/robot_program_running",
        ).value
        self.switch_service = self.declare_parameter(
            "switch_service", "/controller_manager/switch_controller"
        ).value
        self.forward_controller = self.declare_parameter(
            "forward_controller", "forward_velocity_controller"
        ).value
        self.deactivate_controllers = list(
            self.declare_parameter(
                "deactivate_controllers",
                ["scaled_joint_trajectory_controller", "joint_trajectory_controller"],
            ).value
        )
        self.reset_service = self.declare_parameter(
            "reset_service", "/servo_node/reset_servo_status"
        ).value
        self.start_service = self.declare_parameter(
            "start_service", "/servo_node/start_servo"
        ).value
        self.unpause_service = self.declare_parameter(
            "unpause_service", "/servo_node/unpause_servo"
        ).value
        self.wait_timeout_sec = float(
            self.declare_parameter("wait_timeout_sec", 5.0).value
        )
        self.max_startup_wait_sec = float(
            self.declare_parameter("max_startup_wait_sec", 30.0).value
        )

        self._robot_running = False

        # Transient local to catch the latched state published before we subscribed.
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._sub = self.create_subscription(
            Bool, self.robot_program_topic, self._on_robot_program, qos
        )

        self._switch_client = self.create_client(SwitchController, self.switch_service)
        self._reset_client = self.create_client(Empty, self.reset_service)
        self._start_client = self.create_client(Trigger, self.start_service)
        self._unpause_client = self.create_client(Trigger, self.unpause_service)

    def _on_robot_program(self, msg: Bool) -> None:
        self._robot_running = msg.data

    def _wait_for_service(self, client, name: str) -> bool:
        if not client.wait_for_service(timeout_sec=self.wait_timeout_sec):
            self.get_logger().error(f"[servo_auto_start] Timeout waiting for {name}")
            return False
        return True

    def _make_timeout(self) -> Duration:
        d = Duration()
        d.sec = int(self.wait_timeout_sec)
        d.nanosec = 0
        return d

    def _wait_robot_running(self) -> bool:
        self.get_logger().info(
            f"[servo_auto_start] Waiting for robot program on {self.robot_program_topic}"
        )
        deadline = self.get_clock().now() + rclpy.duration.Duration(
            seconds=self.max_startup_wait_sec
        )
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
            if self._robot_running:
                self.get_logger().info("[servo_auto_start] Robot program running — continuing")
                return True
            if self.get_clock().now() >= deadline:
                self.get_logger().error(
                    "[servo_auto_start] Timed out waiting for robot_program_running=true. "
                    "Check that URSim is running and External Control program is active."
                )
                return False
        return False

    def _switch_controller(self) -> bool:
        if not self._wait_for_service(self._switch_client, self.switch_service):
            return False

        req = SwitchController.Request()
        req.activate_controllers = [self.forward_controller]
        req.deactivate_controllers = self.deactivate_controllers
        req.strictness = SwitchController.Request.BEST_EFFORT
        req.activate_asap = True
        req.timeout = self._make_timeout()

        self.get_logger().info(
            f"[servo_auto_start] Activating [{self.forward_controller}], "
            f"deactivating {self.deactivate_controllers} (BEST_EFFORT)"
        )

        future = self._switch_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.wait_timeout_sec)

        if not future.done() or future.exception() is not None:
            self.get_logger().error(
                f"[servo_auto_start] switch_controller call error: {future.exception()}"
            )
            return False

        resp = future.result()
        if resp is None or not resp.ok:
            self.get_logger().error(
                f"[servo_auto_start] switch_controller ok=False — "
                f"[{self.forward_controller}] could not be activated. "
                "Check that the controller is loaded in controller_manager."
            )
            return False

        self.get_logger().info(
            f"[servo_auto_start] [{self.forward_controller}] active"
        )
        return True

    def _servo_bootstrap(self) -> bool:
        for client, name in [
            (self._reset_client, self.reset_service),
            (self._start_client, self.start_service),
            (self._unpause_client, self.unpause_service),
        ]:
            if not self._wait_for_service(client, name):
                return False

        f = self._reset_client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, f, timeout_sec=self.wait_timeout_sec)
        if not f.done() or f.exception() is not None:
            self.get_logger().error(f"[servo_auto_start] reset_servo_status error: {f.exception()}")
            return False

        for client, label in [
            (self._start_client, "start_servo"),
            (self._unpause_client, "unpause_servo"),
        ]:
            f = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, f, timeout_sec=self.wait_timeout_sec)
            if not f.done() or f.exception() is not None:
                self.get_logger().error(f"[servo_auto_start] {label} error: {f.exception()}")
                return False
            resp = f.result()
            if not resp.success:
                self.get_logger().error(
                    f"[servo_auto_start] {label} returned failure: {resp.message}"
                )
                return False

        return True

    def run(self) -> bool:
        if not self._wait_robot_running():
            return False
        if not self._switch_controller():
            return False
        if not self._servo_bootstrap():
            return False
        self.get_logger().info(
            "[servo_auto_start] Bootstrap complete — "
            f"[{self.forward_controller}] active, Servo running"
        )
        return True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ServoAutoStart()
    ok = False
    try:
        ok = node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if not ok:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
