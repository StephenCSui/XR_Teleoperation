#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger, Empty


class ServoAutoStart(Node):
    def __init__(self) -> None:
        super().__init__("servo_auto_start")

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
            self.declare_parameter("wait_timeout_sec", 2.0).value
        )
        self.max_startup_wait_sec = float(
            self.declare_parameter("max_startup_wait_sec", 30.0).value
        )

        self.reset_client = self.create_client(Empty, self.reset_service)
        self.start_client = self.create_client(Trigger, self.start_service)
        self.unpause_client = self.create_client(Trigger, self.unpause_service)

        self.timer = self.create_timer(0.5, self.try_bootstrap)
        self.started = False
        self.start_time = self.get_clock().now()

        self.get_logger().info("servo_auto_start started")

    def elapsed_sec(self) -> float:
        return (self.get_clock().now() - self.start_time).nanoseconds / 1e9

    def wait_for_client(self, client, name: str) -> bool:
        if client.wait_for_service(timeout_sec=self.wait_timeout_sec):
            return True
        self.get_logger().warn(f"waiting for service: {name}")
        return False

    def call_empty(self, client, name: str) -> bool:
        req = Empty.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.wait_timeout_sec)

        if not future.done():
            self.get_logger().warn(f"timeout calling {name}")
            return False

        exc = future.exception()
        if exc is not None:
            self.get_logger().error(f"{name} failed: {exc}")
            return False

        self.get_logger().info(f"{name}: success")
        return True

    def call_trigger(self, client, name: str) -> bool:
        req = Trigger.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.wait_timeout_sec)

        if not future.done():
            self.get_logger().warn(f"timeout calling {name}")
            return False

        exc = future.exception()
        if exc is not None:
            self.get_logger().error(f"{name} failed: {exc}")
            return False

        resp = future.result()
        if resp is None:
            self.get_logger().error(f"{name} returned no response")
            return False

        if resp.success:
            self.get_logger().info(f"{name}: success - {resp.message}")
        else:
            self.get_logger().warn(f"{name}: failed - {resp.message}")

        return resp.success

    def try_bootstrap(self) -> None:
        if self.started:
            return

        if self.elapsed_sec() > self.max_startup_wait_sec:
            self.get_logger().error("servo_auto_start timed out waiting for services")
            self.timer.cancel()
            return

        if not self.wait_for_client(self.reset_client, self.reset_service):
            return
        if not self.wait_for_client(self.start_client, self.start_service):
            return
        if not self.wait_for_client(self.unpause_client, self.unpause_service):
            return

        ok_reset = self.call_empty(self.reset_client, self.reset_service)
        ok_start = self.call_trigger(self.start_client, self.start_service)
        ok_unpause = self.call_trigger(self.unpause_client, self.unpause_service)

        if ok_reset and ok_start and ok_unpause:
            self.get_logger().info("servo bootstrap complete")
            self.started = True
            self.timer.cancel()
        else:
            self.get_logger().warn("servo bootstrap incomplete, will retry")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ServoAutoStart()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
