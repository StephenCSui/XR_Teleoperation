#!/usr/bin/env python3
"""
canvas_pose_follower.py  --  Commands UR3e EEF to follow /canvas/pose.

Uses the MoveGroup action client (moveit_msgs) -- no moveit_py required.
Fully executor-driven with exception handling and timeout recovery.

Parameters (live-tunable via ros2 param set)
----------
  pose_topic            str    default: /canvas/pose
  planning_group        str    default: ur_manipulator
  eef_link              str    default: tool0
  planning_frame        str    default: base_link
  standoff_m            float  EEF standoff in front of canvas (metres)  default: 0.05
  approach_from_normal  bool   Flip tool-Z to point into canvas           default: True
  pos_deadband_m        float  Min XYZ change to trigger replan           default: 0.010
  rot_deadband_deg      float  Min rotation change to trigger replan      default: 3.0
  velocity_scaling      float  MoveIt velocity scale 0-1                  default: 0.3
  accel_scaling         float  MoveIt accel scale 0-1                     default: 0.3
  planning_time_s       float  MoveIt planning timeout per attempt        default: 5.0
  pos_tolerance_m       float  Position IK tolerance (metres)             default: 0.005
  ori_tolerance_rad     float  Orientation IK tolerance (radians)         default: 0.05
  enabled               bool   False = pause without killing node          default: True
  tf_timeout_s          float  tf2 lookup timeout                         default: 0.5
"""

import math
import threading
import time

import numpy as np
import rclpy
import rclpy.duration
import rclpy.time
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, PoseStamped
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from visualization_msgs.msg import Marker, MarkerArray

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    MotionPlanRequest,
    OrientationConstraint,
    PlanningOptions,
    PositionConstraint,
)


# =============================================================================
# Geometry helpers
# =============================================================================

def quat_to_rotmat(qx, qy, qz, qw):
    return np.array([
        [1 - 2*(qy*qy + qz*qz),     2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [    2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz),     2*(qy*qz - qx*qw)],
        [    2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)],
    ])


def rotmat_to_quat(R):
    trace = R[0,0] + R[1,1] + R[2,2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        return ((R[2,1]-R[1,2])*s, (R[0,2]-R[2,0])*s,
                (R[1,0]-R[0,1])*s, 0.25/s)
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = 2.0 * math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
        return (0.25*s, (R[0,1]+R[1,0])/s,
                (R[0,2]+R[2,0])/s, (R[2,1]-R[1,2])/s)
    elif R[1,1] > R[2,2]:
        s = 2.0 * math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
        return ((R[0,1]+R[1,0])/s, 0.25*s,
                (R[1,2]+R[2,1])/s, (R[0,2]-R[2,0])/s)
    else:
        s = 2.0 * math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
        return ((R[0,2]+R[2,0])/s, (R[1,2]+R[2,1])/s,
                0.25*s,            (R[1,0]-R[0,1])/s)


def quat_angular_distance_deg(q1, q2):
    dot = abs(q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3])
    return math.degrees(2.0 * math.acos(min(1.0, dot)))


def compute_approach_pose(canvas_pos, canvas_quat, standoff_m, approach_from_normal):
    R = quat_to_rotmat(*canvas_quat)
    canvas_normal = R[:, 2]
    if approach_from_normal:
        goal_pos  = canvas_pos + standoff_m * canvas_normal
        Rx180     = np.array([[1,0,0],[0,-1,0],[0,0,-1]], dtype=float)
        goal_quat = rotmat_to_quat(R @ Rx180)
    else:
        goal_pos  = canvas_pos.copy()
        goal_quat = tuple(canvas_quat)
    return goal_pos, goal_quat


# =============================================================================
# MoveGroup goal builder
# =============================================================================

def build_move_group_goal(frame_id, eef_link, group_name,
                          goal_pos, goal_quat,
                          pos_tol, ori_tol,
                          velocity_scaling, accel_scaling,
                          planning_time, plan_only=False):
    pos_c                        = PositionConstraint()
    pos_c.header.frame_id        = frame_id
    pos_c.link_name              = eef_link
    pos_c.weight                 = 1.0

    sphere                       = SolidPrimitive()
    sphere.type                  = SolidPrimitive.SPHERE
    sphere.dimensions            = [pos_tol]

    prim_pose                    = PoseStamped()
    prim_pose.header.frame_id    = frame_id
    prim_pose.pose.position.x    = float(goal_pos[0])
    prim_pose.pose.position.y    = float(goal_pos[1])
    prim_pose.pose.position.z    = float(goal_pos[2])
    prim_pose.pose.orientation.w = 1.0

    region                       = BoundingVolume()
    region.primitives.append(sphere)
    region.primitive_poses.append(prim_pose.pose)
    pos_c.constraint_region      = region

    ori_c                              = OrientationConstraint()
    ori_c.header.frame_id              = frame_id
    ori_c.link_name                    = eef_link
    ori_c.orientation.x                = float(goal_quat[0])
    ori_c.orientation.y                = float(goal_quat[1])
    ori_c.orientation.z                = float(goal_quat[2])
    ori_c.orientation.w                = float(goal_quat[3])
    ori_c.absolute_x_axis_tolerance    = ori_tol
    ori_c.absolute_y_axis_tolerance    = ori_tol
    ori_c.absolute_z_axis_tolerance    = ori_tol
    ori_c.weight                       = 1.0

    constraints = Constraints()
    constraints.position_constraints.append(pos_c)
    constraints.orientation_constraints.append(ori_c)

    req                                     = MotionPlanRequest()
    req.group_name                          = group_name
    req.goal_constraints.append(constraints)
    req.num_planning_attempts               = 5
    req.allowed_planning_time               = float(planning_time)
    req.max_velocity_scaling_factor         = float(velocity_scaling)
    req.max_acceleration_scaling_factor     = float(accel_scaling)

    opts           = PlanningOptions()
    opts.plan_only = plan_only

    goal                  = MoveGroup.Goal()
    goal.request          = req
    goal.planning_options = opts
    return goal


# =============================================================================
# Follower node
# =============================================================================

class CanvasPoseFollower(Node):

    IDLE      = 'IDLE'
    PLANNING  = 'PLANNING'
    EXECUTING = 'EXECUTING'

    def __init__(self):
        super().__init__('canvas_pose_follower')

        self.declare_parameter('pose_topic',            '/canvas/pose')
        self.declare_parameter('planning_group',        'ur_manipulator')
        self.declare_parameter('eef_link',              'tool0')
        self.declare_parameter('planning_frame',        'base_link')
        self.declare_parameter('standoff_m',            0.05)
        self.declare_parameter('approach_from_normal',  True)
        self.declare_parameter('pos_deadband_m',        0.010)
        self.declare_parameter('rot_deadband_deg',      3.0)
        self.declare_parameter('velocity_scaling',      0.3)
        self.declare_parameter('accel_scaling',         0.3)
        self.declare_parameter('planning_time_s',       5.0)
        self.declare_parameter('pos_tolerance_m',       0.005)
        self.declare_parameter('ori_tolerance_rad',     0.05)
        self.declare_parameter('enabled',               True)
        self.declare_parameter('tf_timeout_s',          0.5)

        self._action_client   = ActionClient(self, MoveGroup, '/move_action')
        self._tf_buffer       = Buffer()
        self._tf_listener     = TransformListener(self._tf_buffer, self)
        self._goal_marker_pub = self.create_publisher(
            MarkerArray, '/follower/goal_marker', 5)

        self._lock              = threading.Lock()
        self._state             = self.IDLE
        self._state_entered_at  = time.monotonic()
        self._pending           = None
        self._last_exec_pos     = None
        self._last_exec_quat    = None
        self._inflight_canvas_pos  = None
        self._inflight_canvas_quat = None

        # Kick timer: fires from executor, sends goals when IDLE+pending
        self._plan_kick_timer = self.create_timer(0.2, self._kick_cb)

        # Watchdog timer: resets state if stuck in PLANNING/EXECUTING too long
        self._watchdog_timer = self.create_timer(1.0, self._watchdog_cb)

        topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        self._sub = self.create_subscription(
            PoseStamped, topic, self._pose_cb, 10)

        self.get_logger().info(
            '\ncanvas_pose_follower ready.\n'
            f'  pose_topic:     {topic}\n'
            f'  planning_frame: '
            f'{self.get_parameter("planning_frame").get_parameter_value().string_value}\n'
            f'  group:          '
            f'{self.get_parameter("planning_group").get_parameter_value().string_value}\n'
            f'  eef_link:       '
            f'{self.get_parameter("eef_link").get_parameter_value().string_value}\n'
            f'  standoff:       '
            f'{self.get_parameter("standoff_m").get_parameter_value().double_value*100:.0f} cm\n'
        )

    # ── Watchdog: resets state if callbacks never fire ─────────────────────────

    def _watchdog_cb(self):
        t_plan = self.get_parameter('planning_time_s').get_parameter_value().double_value
        timeout = t_plan + 10.0  # generous buffer for execution

        with self._lock:
            state    = self._state
            elapsed  = time.monotonic() - self._state_entered_at

        if state != self.IDLE and elapsed > timeout:
            self.get_logger().warn(
                f'Watchdog: stuck in {state} for {elapsed:.0f}s (timeout={timeout:.0f}s).\n'
                '  MoveGroup callbacks never fired -- resetting to IDLE.\n'
                '  This usually means the action server is not responding.\n'
                '  Check: ros2 action send_goal /move_group moveit_msgs/action/MoveGroup '
                '"{request: {group_name: \'ur_manipulator\', allowed_planning_time: 3.0}, '
                'planning_options: {plan_only: true}}"')
            with self._lock:
                self._state            = self.IDLE
                self._state_entered_at = time.monotonic()
                # Reset deadband so the next pose triggers a fresh attempt
                self._last_exec_pos    = None
                self._last_exec_quat   = None

    # ── Subscriber callback ────────────────────────────────────────────────────

    def _pose_cb(self, msg: PoseStamped):
        if not self.get_parameter('enabled').get_parameter_value().bool_value:
            return

        planning_frame = self.get_parameter(
            'planning_frame').get_parameter_value().string_value
        tf_timeout_s   = self.get_parameter(
            'tf_timeout_s').get_parameter_value().double_value

        if msg.header.frame_id != planning_frame:
            try:
                tf = self._tf_buffer.lookup_transform(
                    planning_frame, msg.header.frame_id,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=tf_timeout_s))
                from tf2_geometry_msgs import do_transform_pose_stamped
                msg = do_transform_pose_stamped(msg, tf)
            except (LookupException, ConnectivityException,
                    ExtrapolationException) as e:
                self.get_logger().warn(
                    f'TF lookup failed: {e}', throttle_duration_sec=2.0)
                return

        pos  = np.array([msg.pose.position.x,
                         msg.pose.position.y,
                         msg.pose.position.z])
        quat = (msg.pose.orientation.x, msg.pose.orientation.y,
                msg.pose.orientation.z, msg.pose.orientation.w)

        if not self._exceeds_deadband(pos, quat):
            return

        with self._lock:
            self._pending = (pos, quat)

    def _exceeds_deadband(self, pos, quat):
        pos_db = self.get_parameter('pos_deadband_m').get_parameter_value().double_value
        rot_db = self.get_parameter('rot_deadband_deg').get_parameter_value().double_value
        with self._lock:
            if self._last_exec_pos is None:
                return True
            return (np.linalg.norm(pos - self._last_exec_pos) > pos_db or
                    quat_angular_distance_deg(quat, self._last_exec_quat) > rot_db)

    # ── Kick timer ─────────────────────────────────────────────────────────────

    def _kick_cb(self):
        with self._lock:
            if self._state != self.IDLE:
                return
            if self._pending is None:
                return
            canvas_pos, canvas_quat = self._pending
            self._pending           = None
            self._state             = self.PLANNING
            self._state_entered_at  = time.monotonic()

        self._send_goal(canvas_pos, canvas_quat)

    # ── Goal send ──────────────────────────────────────────────────────────────

    def _send_goal(self, canvas_pos, canvas_quat):
        standoff_m           = self.get_parameter('standoff_m').get_parameter_value().double_value
        approach_from_normal = self.get_parameter('approach_from_normal').get_parameter_value().bool_value
        vel                  = self.get_parameter('velocity_scaling').get_parameter_value().double_value
        acc                  = self.get_parameter('accel_scaling').get_parameter_value().double_value
        t_plan               = self.get_parameter('planning_time_s').get_parameter_value().double_value
        pos_tol              = self.get_parameter('pos_tolerance_m').get_parameter_value().double_value
        ori_tol              = self.get_parameter('ori_tolerance_rad').get_parameter_value().double_value
        frame                = self.get_parameter('planning_frame').get_parameter_value().string_value
        eef_link             = self.get_parameter('eef_link').get_parameter_value().string_value
        group                = self.get_parameter('planning_group').get_parameter_value().string_value

        goal_pos, goal_quat = compute_approach_pose(
            canvas_pos, canvas_quat, standoff_m, approach_from_normal)

        self.get_logger().info(
            f'Planning -> ({goal_pos[0]:+.3f}, {goal_pos[1]:+.3f}, '
            f'{goal_pos[2]:+.3f}) m  standoff={standoff_m*100:.0f}cm')

        self._publish_goal_marker(frame, goal_pos, goal_quat)

        self._inflight_canvas_pos  = canvas_pos
        self._inflight_canvas_quat = canvas_quat

        goal = build_move_group_goal(
            frame_id=frame, eef_link=eef_link, group_name=group,
            goal_pos=goal_pos, goal_quat=goal_quat,
            pos_tol=pos_tol, ori_tol=ori_tol,
            velocity_scaling=vel, accel_scaling=acc,
            planning_time=t_plan, plan_only=False,
        )

        try:
            self.get_logger().debug('Calling send_goal_async...')
            future = self._action_client.send_goal_async(goal)
            future.add_done_callback(self._goal_response_cb)
            self.get_logger().debug('send_goal_async returned -- waiting for goal response callback.')
        except Exception as e:
            self.get_logger().error(f'send_goal_async raised: {e}')
            with self._lock:
                self._state            = self.IDLE
                self._state_entered_at = time.monotonic()

    # ── Action callbacks ───────────────────────────────────────────────────────

    def _goal_response_cb(self, future):
        try:
            self.get_logger().info('_goal_response_cb fired.')
            gh = future.result()
            if not gh.accepted:
                self.get_logger().warn(
                    'MoveGroup goal REJECTED by server.\n'
                    '  Possible causes:\n'
                    '    - Planning group "ur_manipulator" not found\n'
                    '    - move_group busy or in error state\n'
                    '  Try: ros2 param set /canvas_pose_follower enabled false\n'
                    '       then re-enable to reset.')
                with self._lock:
                    self._state            = self.IDLE
                    self._state_entered_at = time.monotonic()
                return

            self.get_logger().info('Goal ACCEPTED -- waiting for execution result...')
            with self._lock:
                self._state            = self.EXECUTING
                self._state_entered_at = time.monotonic()
            gh.get_result_async().add_done_callback(self._result_cb)

        except Exception as e:
            self.get_logger().error(f'_goal_response_cb exception: {e}')
            with self._lock:
                self._state            = self.IDLE
                self._state_entered_at = time.monotonic()

    def _result_cb(self, future):
        try:
            self.get_logger().info('_result_cb fired.')
            ec = future.result().result.error_code.val
            if ec == 1:
                self.get_logger().info('Execution SUCCEEDED.')
                with self._lock:
                    self._last_exec_pos  = self._inflight_canvas_pos.copy()
                    self._last_exec_quat = self._inflight_canvas_quat
            else:
                self.get_logger().warn(
                    f'MoveGroup FAILED  error_code={ec}\n'
                    '  -1  PLANNING_FAILED  -- pose unreachable or IK failed\n'
                    '  -4  INVALIDATED      -- environment changed mid-plan\n'
                    '  -6  CONTROL_FAILED   -- controller rejected trajectory\n'
                    '  Adjust canvas position or reduce standoff:\n'
                    '    ros2 param set /canvas_pose_injector canvas_z 0.35\n'
                    '    ros2 param set /canvas_pose_injector canvas_x 0.2\n'
                    '    ros2 param set /canvas_pose_follower approach_from_normal false')
        except Exception as e:
            self.get_logger().error(f'_result_cb exception: {e}')
        finally:
            with self._lock:
                self._state            = self.IDLE
                self._state_entered_at = time.monotonic()

    # ── RViz goal marker ───────────────────────────────────────────────────────

    def _publish_goal_marker(self, frame, pos, quat):
        stamp = self.get_clock().now().to_msg()
        life  = Duration(sec=2, nanosec=0)
        arr   = MarkerArray()

        def base(mid, mtype):
            m                 = Marker()
            m.header.frame_id = frame
            m.header.stamp    = stamp
            m.ns              = 'follower_goal'
            m.id              = mid
            m.type            = mtype
            m.action          = Marker.ADD
            m.lifetime        = life
            return m

        sph = base(0, Marker.SPHERE)
        sph.pose.position.x    = float(pos[0])
        sph.pose.position.y    = float(pos[1])
        sph.pose.position.z    = float(pos[2])
        sph.pose.orientation.x = float(quat[0])
        sph.pose.orientation.y = float(quat[1])
        sph.pose.orientation.z = float(quat[2])
        sph.pose.orientation.w = float(quat[3])
        sph.scale.x = sph.scale.y = sph.scale.z = 0.025
        sph.color.r = 1.0; sph.color.g = 1.0
        sph.color.b = 0.0; sph.color.a = 0.9
        arr.markers.append(sph)

        R   = quat_to_rotmat(*quat)
        tip = pos + 0.07 * R[:, 2]
        arrow = base(1, Marker.ARROW)
        p0 = Point(); p0.Xx, p0.y, p0.z = float(pos[0]), float(pos[1]), float(pos[2])
        p1 = Point(); p1.x, p1.y, p1.z = float(tip[0]), float(tip[1]), float(tip[2])
        arrow.points = [p0, p1]
        arrow.scale.x = 0.005; arrow.scale.y = 0.012
        arrow.color.r = 1.0; arrow.color.g = 0.5
        arrow.color.b = 0.0; arrow.color.a = 1.0
        arr.markers.append(arrow)

        lbl = base(2, Marker.TEXT_VIEW_FACING)
        lbl.pose.position.x    = float(pos[0])
        lbl.pose.position.y    = float(pos[1]) - 0.04
        lbl.pose.position.z    = float(pos[2])
        lbl.pose.orientation.w = 1.0
        lbl.scale.z = 0.022
        lbl.text    = f'EEF goal\n({pos[0]:+.3f}, {pos[1]:+.3f}, {pos[2]:+.3f})'
        lbl.color.r = 1.0; lbl.color.g = 1.0
        lbl.color.b = 0.5; lbl.color.a = 1.0
        arr.markers.append(lbl)

        self._goal_marker_pub.publish(arr)


# =============================================================================
# Entry point
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = CanvasPoseFollower()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()