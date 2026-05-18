#!/usr/bin/env python3
"""
canvas_tf_bridge.py  --  Generic PoseStamped frame transformer.

Subscribes to a PoseStamped topic published in one TF frame, transforms it
into a configurable target frame using the live TF tree, and republishes.
Optionally broadcasts the result as a TF child frame.

In this project
---------------
  source : /canvas/pose              (camera_color_optical_frame)
  target : base_link                 (MoveIt Servo planning frame)
  output : /canvas/pose_base         (primary -- feed into corrector)
           /canvas/pose_tool0        (secondary -- verification only)

Requires
--------
  Static TF  flange -> camera_color_optical_frame  must be published before
  this node starts, otherwise lookups will fail until TF is available.

  Run the static TF publisher from canvas_tf_bridge.launch.py or separately:
    ros2 run tf2_ros static_transform_publisher \\
      --frame-id flange --child-frame-id camera_color_optical_frame \\
      --x 0.0 --y 0.09 --z 0.0 --roll -0.785 --pitch 0.0 --yaw 0.0

Parameters (all live-tunable)
------------------------------
  source_topic      str   Input PoseStamped topic          /canvas/pose
  target_frame      str   Primary output TF frame          base_link
  output_topic      str   Primary output topic             /canvas/pose_base
  secondary_frame   str   Secondary output frame           tool0
  secondary_topic   str   Secondary output topic           /canvas/pose_tool0
  broadcast_tf      bool  Broadcast target pose as TF      true
  tf_child_frame    str   TF child frame name              canvas_centre_base
  fallback_frame    str   Frame if source msg has none     camera_color_optical_frame
  tf_timeout_s      float TF lookup timeout                0.10
  queue_size        int   Subscriber/publisher queue       10
"""

import rclpy
import rclpy.duration
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from tf2_ros import (
    Buffer,
    LookupException,
    ConnectivityException,
    ExtrapolationException,
    TransformBroadcaster,
    TransformListener,
)
import tf2_geometry_msgs  # noqa: F401  registers PoseStamped type support for Buffer.transform()


# =============================================================================
# Node
# =============================================================================

class CanvasTFBridge(Node):
    """
    Transforms a PoseStamped from its source TF frame into one or two target
    frames using the live TF tree.

    Design
    ------
    - _setup_params()   : declare + read all ROS parameters
    - _setup_tf()       : Buffer + TransformListener + optional broadcaster
    - _setup_pubsub()   : subscriber + publishers
    - _on_pose()        : subscriber callback; calls _transform() + _publish()
    - _transform()      : pure TF lookup; returns transformed PoseStamped or None
    - _publish()        : publishes result and optional TF broadcast
    """

    def __init__(self):
        super().__init__('canvas_tf_bridge')
        self._setup_params()
        self._setup_tf()
        self._setup_pubsub()
        self._log_startup()

    # ── Setup ─────────────────────────────────────────────────────────────────

    def _setup_params(self):
        self.declare_parameter('source_topic',    '/canvas/pose')
        self.declare_parameter('target_frame',    'base_link')
        self.declare_parameter('output_topic',    '/canvas/pose_base')
        self.declare_parameter('secondary_frame', 'tool0')
        self.declare_parameter('secondary_topic', '/canvas/pose_tool0')
        self.declare_parameter('broadcast_tf',    True)
        self.declare_parameter('tf_child_frame',  'canvas_centre_base')
        self.declare_parameter('fallback_frame',  'camera_color_optical_frame')
        self.declare_parameter('tf_timeout_s',    0.10)
        self.declare_parameter('queue_size',      10)

        # Cache frequently-accessed params (others read live in callbacks
        # so they stay tunable without restart)
        self._source_topic    = self._get_str('source_topic')
        self._output_topic    = self._get_str('output_topic')
        self._secondary_topic = self._get_str('secondary_topic')
        self._queue_size      = self._get_int('queue_size')

    def _setup_tf(self):
        self._tf_buffer      = Buffer()
        self._tf_listener    = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self)

    def _setup_pubsub(self):
        q = self._queue_size
        self._primary_pub   = self.create_publisher(PoseStamped, self._output_topic,    q)
        self._secondary_pub = self.create_publisher(PoseStamped, self._secondary_topic, q)
        self.create_subscription(PoseStamped, self._source_topic, self._on_pose, q)

    def _log_startup(self):
        target    = self._get_str('target_frame')
        secondary = self._get_str('secondary_frame')
        fallback  = self._get_str('fallback_frame')
        self.get_logger().info(
            f'\ncanvas_tf_bridge ready.\n'
            f'  source   : {self._source_topic}  (frame from msg, fallback: {fallback})\n'
            f'  primary  : {self._output_topic}  (frame: {target})\n'
            f'  secondary: {self._secondary_topic}  (frame: {secondary})\n'
            f'  TF child : {self._get_str("tf_child_frame")}  (child of {target})\n'
            f'  Waiting for TF tree: flange -> camera_color_optical_frame\n'
        )

    # ── Subscriber callback ────────────────────────────────────────────────────

    def _on_pose(self, msg: PoseStamped):
        """Receive source pose, transform to both target frames, publish."""
        # Ensure the message has a frame_id; fall back to param if empty
        if not msg.header.frame_id:
            msg.header.frame_id = self._get_str('fallback_frame')

        target    = self._get_str('target_frame')
        secondary = self._get_str('secondary_frame')

        primary_pose   = self._transform(msg, target)
        secondary_pose = self._transform(msg, secondary)

        if primary_pose is not None:
            self._publish_primary(primary_pose)

        if secondary_pose is not None:
            self._publish_secondary(secondary_pose)

    # ── Transform ─────────────────────────────────────────────────────────────

    def _transform(self, msg: PoseStamped, target_frame: str) -> PoseStamped | None:
        """
        Look up the transform from msg.header.frame_id to target_frame at the
        message timestamp, apply it, and return a new PoseStamped.

        Returns None on any TF failure; logs a throttled warning.
        """
        timeout_s = self._get_float('tf_timeout_s')
        try:
            return self._tf_buffer.transform(
                msg,
                target_frame,
                timeout=rclpy.duration.Duration(seconds=timeout_s),
            )
        except LookupException as e:
            self.get_logger().warn(
                f'[TF] Lookup failed ({msg.header.frame_id} -> {target_frame}): {e}',
                throttle_duration_sec=3.0,
            )
        except ConnectivityException as e:
            self.get_logger().warn(
                f'[TF] No connection ({msg.header.frame_id} -> {target_frame}): {e}',
                throttle_duration_sec=3.0,
            )
        except ExtrapolationException as e:
            self.get_logger().warn(
                f'[TF] Extrapolation ({msg.header.frame_id} -> {target_frame}): {e}',
                throttle_duration_sec=3.0,
            )
        return None

    # ── Publish ───────────────────────────────────────────────────────────────

    def _publish_primary(self, pose: PoseStamped):
        self._primary_pub.publish(pose)
        if self._get_bool('broadcast_tf'):
            self._broadcast_tf(pose, self._get_str('tf_child_frame'))

    def _publish_secondary(self, pose: PoseStamped):
        self._secondary_pub.publish(pose)

    def _broadcast_tf(self, pose: PoseStamped, child_frame: str):
        """Broadcast the transformed pose as a TF frame."""
        ts                         = TransformStamped()
        ts.header.stamp            = pose.header.stamp
        ts.header.frame_id         = pose.header.frame_id
        ts.child_frame_id          = child_frame
        ts.transform.translation.x = pose.pose.position.x
        ts.transform.translation.y = pose.pose.position.y
        ts.transform.translation.z = pose.pose.position.z
        ts.transform.rotation      = pose.pose.orientation
        self._tf_broadcaster.sendTransform(ts)

    # ── Parameter helpers (read live for tunability) ───────────────────────────

    def _get_str(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _get_bool(self, name: str) -> bool:
        return self.get_parameter(name).get_parameter_value().bool_value

    def _get_float(self, name: str) -> float:
        return self.get_parameter(name).get_parameter_value().double_value

    def _get_int(self, name: str) -> int:
        return self.get_parameter(name).get_parameter_value().integer_value


# =============================================================================
# Entry point
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = CanvasTFBridge()
    executor = MultiThreadedExecutor(num_threads=2)
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