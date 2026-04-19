#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import SwitchController
from std_srvs.srv import Empty, Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class StartupPoseAndServoBootstrap(Node):
    def __init__(self) -> None:
        super().__init__("startup_pose_and_servo_bootstrap")

        self.startup_controller = self.declare_parameter(
            "startup_controller", "joint_trajectory_controller"
        ).value
        self.forward_controller = self.declare_parameter(
            "forward_controller", "forward_velocity_controller"
        ).value

        self.startup_action_name = self.declare_parameter(
            "startup_action_name",
            f"/{self.startup_controller}/follow_joint_trajectory",
        ).value

        self.switch_service = self.declare_parameter(
            "switch_service", "/controller_manager/switch_controller"
        ).value

        self.reset_service = self.declare_parameter(
            "reset_service", "/servo_node/reset_servo_status"
        ).value
        self.start_service = self.declare_parameter(
            "start_service", "/servo_node/start_servo"
        ).value
        self.unpause_service = self.declare_parameter(
            "unpause_service", "/servo_node/unpause_servo"
        ).value

        self.startup_joint_names = self.declare_parameter(
            "startup_joint_names",
            [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ],
        ).value

        self.startup_positions = self.declare_parameter(
            "startup_positions",
            [0.0, -1.1, 1.6, -2.0, -1.57, 0.0],
        ).value

        self.startup_time_sec = float(
            self.declare_parameter("startup_time_sec", 3.0).value
        )
        self.settle_time_sec = float(
            self.declare_parameter("settle_time_sec", 0.75).value
        )
        self.wait_timeout_sec = float(
            self.declare_parameter("wait_timeout_sec", 30.0).value
        )

        self.traj_client = ActionClient(
            self, FollowJointTrajectory, self.startup_action_name
        )

        self.switch_client = self.create_client(
            SwitchController, self.switch_service
        )

        self.reset_client = self.create_client(
            Empty, self.reset_service
        )
        self.start_client = self.create_client(
            Trigger, self.start_service
        )
        self.unpause_client = self.create_client(
            Trigger, self.unpause_service
        )

    def _duration_msg(self, seconds: float) -> Duration:
        sec = int(math.floor(seconds))
        nanosec = int((seconds - sec) * 1e9)
        msg = Duration()
        msg.sec = sec
        msg.nanosec = nanosec
        return msg

    def wait_for_action_server(self) -> bool:
        self.get_logger().info(f"Waiting for action server: {self.startup_action_name}")
        ok = self.traj_client.wait_for_server(timeout_sec=self.wait_timeout_sec)
        if not ok:
            self.get_logger().error(f"Timeout waiting for {self.startup_action_name}")
            return False
        return True

    def wait_for_service(self, client, name: str) -> bool:
        self.get_logger().info(f"Waiting for service: {name}")
        ok = client.wait_for_service(timeout_sec=self.wait_timeout_sec)
        if not ok:
            self.get_logger().error(f"Timeout waiting for service {name}")
            return False
        return True

    def send_startup_goal(self) -> bool:
        traj = JointTrajectory()
        traj.joint_names = list(self.startup_joint_names)

        point = JointTrajectoryPoint()
        point.positions = [float(x) for x in self.startup_positions]
        point.time_from_start = self._duration_msg(self.startup_time_sec)
        traj.points.append(point)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        goal.goal_time_tolerance = self._duration_msg(1.0)

        self.get_logger().info(
            f"Sending startup joint pose via {self.startup_action_name}"
        )
        self.get_logger().info(
            f"joint_names={traj.joint_names} positions={point.positions}"
        )

        send_goal_future = self.traj_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=self.wait_timeout_sec)

        if not send_goal_future.done():
            self.get_logger().error("Timed out waiting for trajectory goal response")
            return False

        exc = send_goal_future.exception()
        if exc is not None:
            self.get_logger().error(f"Trajectory goal send failed: {exc}")
            return False

        goal_handle = send_goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Trajectory goal was rejected")
            return False

        self.get_logger().info("Trajectory goal accepted, waiting for result")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=self.wait_timeout_sec)

        if not result_future.done():
            self.get_logger().error("Timed out waiting for trajectory result")
            return False

        exc = result_future.exception()
        if exc is not None:
            self.get_logger().error(f"Trajectory result failed: {exc}")
            return False

        wrapped = result_future.result()
        if wrapped is None:
            self.get_logger().error("Trajectory returned no result")
            return False

        status = wrapped.status
        result = wrapped.result

        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f"Startup trajectory failed: action_status={status} "
                f"error_code={getattr(result, 'error_code', 'n/a')} "
                f"error_string={getattr(result, 'error_string', '')}"
            )
            return False

        if getattr(result, "error_code", 0) != 0:
            self.get_logger().error(
                f"Startup trajectory returned controller error: "
                f"{result.error_code} {result.error_string}"
            )
            return False

        self.get_logger().info("Startup trajectory completed successfully")
        return True

    def switch_to_forward_controller(self) -> bool:
        if self.startup_controller == self.forward_controller:
            self.get_logger().info("startup_controller == forward_controller, skipping switch")
            return True

        if not self.wait_for_service(self.switch_client, self.switch_service):
            return False

        req = SwitchController.Request()
        req.activate_controllers = [self.forward_controller]
        req.deactivate_controllers = [self.startup_controller]
        req.strictness = SwitchController.Request.BEST_EFFORT
        req.activate_asap = True
        req.timeout = self._duration_msg(5.0)

        self.get_logger().info(
            f"Switching controllers: deactivate={req.deactivate_controllers} "
            f"activate={req.activate_controllers}"
        )

        future = self.switch_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.wait_timeout_sec)

        if not future.done():
            self.get_logger().error("Timed out waiting for switch_controller response")
            return False

        exc = future.exception()
        if exc is not None:
            self.get_logger().error(f"switch_controller failed: {exc}")
            return False

        resp = future.result()
        if resp is None or not resp.ok:
            self.get_logger().error("switch_controller returned failure")
            return False

        self.get_logger().info("Controller switch succeeded")
        return True

    def reset_start_unpause_servo(self) -> bool:
        if not self.wait_for_service(self.reset_client, self.reset_service):
            return False
        if not self.wait_for_service(self.start_client, self.start_service):
            return False
        if not self.wait_for_service(self.unpause_client, self.unpause_service):
            return False

        reset_future = self.reset_client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, reset_future, timeout_sec=self.wait_timeout_sec)
        if not reset_future.done() or reset_future.exception() is not None:
            self.get_logger().error(f"reset_servo_status failed: {reset_future.exception()}")
            return False
        self.get_logger().info("reset_servo_status succeeded")

        start_future = self.start_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, start_future, timeout_sec=self.wait_timeout_sec)
        if not start_future.done() or start_future.exception() is not None:
            self.get_logger().error(f"start_servo failed: {start_future.exception()}")
            return False
        start_resp = start_future.result()
        if start_resp is None or not start_resp.success:
            self.get_logger().error(f"start_servo returned failure: {getattr(start_resp, 'message', '')}")
            return False
        self.get_logger().info("start_servo succeeded")

        unpause_future = self.unpause_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, unpause_future, timeout_sec=self.wait_timeout_sec)
        if not unpause_future.done() or unpause_future.exception() is not None:
            self.get_logger().error(f"unpause_servo failed: {unpause_future.exception()}")
            return False
        unpause_resp = unpause_future.result()
        if unpause_resp is None or not unpause_resp.success:
            self.get_logger().error(f"unpause_servo returned failure: {getattr(unpause_resp, 'message', '')}")
            return False
        self.get_logger().info("unpause_servo succeeded")

        return True

    def run_sequence(self) -> bool:
        if len(self.startup_joint_names) != len(self.startup_positions):
            self.get_logger().error("startup_joint_names and startup_positions length mismatch")
            return False

        if not self.wait_for_action_server():
            return False

        if not self.send_startup_goal():
            return False

        self.get_logger().info(f"Settling for {self.settle_time_sec:.2f}s")
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=self.settle_time_sec))

        if not self.switch_to_forward_controller():
            return False

        if not self.reset_start_unpause_servo():
            return False

        self.get_logger().info("Startup pose + controller switch + servo bootstrap complete")
        return True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StartupPoseAndServoBootstrap()
    ok = False
    try:
        ok = node.run_sequence()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if not ok:
        raise SystemExit(1)


if __name__ == "__main__":
    main()