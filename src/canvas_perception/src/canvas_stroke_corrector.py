#!/usr/bin/env python3
"""
canvas_stroke_corrector.py  --  SE3 pose error correction node.

Latch behaviour (improved):
  Instead of latching the FIRST detection (which may be noisy), this node
  collects canvas poses for `latch_window_s` seconds (default 10s), averages
  all translations and quaternions over that window, then freezes the result
  as T_assumed. This gives a stable, low-noise reference.

  During the collection window:
    status = "COLLECTING  X.Xs / 10.0s  N frames"
  After latch:
    status = "OK"

  To re-latch: restart the node, or publish to /canvas/pose_assumed directly.

SE3 error (in assumed canvas frame):
  R_err = R_assumed^T * R_real
  t_err = R_assumed^T * (t_real - t_assumed)
    x = lateral error   (canvas X, right = +)  metres
    y = vertical error  (canvas Y, up    = +)  metres
    z = depth error     (canvas Z, toward camera = +)  metres

Publishes:
  /canvas/pose_error       PoseStamped   SE3 error (position=t_err, ori=R_err)
  /canvas/pose_assumed     PoseStamped   Averaged reference pose (after latch)
  /canvas/correction_data  String        JSON with errors, latency, latch status

Parameters (live-tunable)
----------
  latch_window_s   float  Seconds to collect before latching assumed pose  default: 10.0
  pose_timeout_s   float  Max canvas pose age before PAUSED                default: 5.0
  publish_rate_hz  float  Output rate                                       default: 50.0
  enabled          bool   False = freeze output                             default: True
"""

import json
import math
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster, Buffer, TransformListener


# =============================================================================
# SE3 helpers
# =============================================================================

def quat_to_rotmat(qx, qy, qz, qw) -> np.ndarray:
    return np.array([
        [1-2*(qy*qy+qz*qz),   2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
        [  2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz),   2*(qy*qz-qx*qw)],
        [  2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)],
    ])


def rotmat_to_quat(R: np.ndarray):
    trace = R[0,0]+R[1,1]+R[2,2]
    if trace > 0:
        s = 0.5/math.sqrt(trace+1.0)
        return ((R[2,1]-R[1,2])*s,(R[0,2]-R[2,0])*s,(R[1,0]-R[0,1])*s,0.25/s)
    elif R[0,0]>R[1,1] and R[0,0]>R[2,2]:
        s = 2.0*math.sqrt(1.0+R[0,0]-R[1,1]-R[2,2])
        return (0.25*s,(R[0,1]+R[1,0])/s,(R[0,2]+R[2,0])/s,(R[2,1]-R[1,2])/s)
    elif R[1,1]>R[2,2]:
        s = 2.0*math.sqrt(1.0+R[1,1]-R[0,0]-R[2,2])
        return ((R[0,1]+R[1,0])/s,0.25*s,(R[1,2]+R[2,1])/s,(R[0,2]-R[2,0])/s)
    else:
        s = 2.0*math.sqrt(1.0+R[2,2]-R[0,0]-R[1,1])
        return ((R[0,2]+R[2,0])/s,(R[1,2]+R[2,1])/s,0.25*s,(R[1,0]-R[0,1])/s)


def rotation_angle_deg(R_err: np.ndarray) -> float:
    trace = float(np.clip(R_err[0,0]+R_err[1,1]+R_err[2,2], -1.0, 3.0))
    return math.degrees(math.acos(max(-1.0, min(1.0, (trace-1.0)/2.0))))


def pose_to_Rt(pose):
    p = pose.position; o = pose.orientation
    return quat_to_rotmat(o.x, o.y, o.z, o.w), np.array([p.x, p.y, p.z])


def average_quats(quats: list) -> np.ndarray:
    """
    Averaging quaternions via eigenvector of the outer-product sum (Markley 2007).
    More accurate than naive element-wise average, handles sign ambiguity.
    Returns (qx, qy, qz, qw) as numpy array.
    """
    M = np.zeros((4, 4))
    for qx, qy, qz, qw in quats:
        q = np.array([qx, qy, qz, qw])
        M += np.outer(q, q)
    M /= len(quats)
    eigenvalues, eigenvectors = np.linalg.eigh(M)
    # Eigenvector with largest eigenvalue = average quaternion
    avg_q = eigenvectors[:, -1]
    if avg_q[3] < 0:   # ensure w > 0 convention
        avg_q = -avg_q
    return avg_q   # (qx, qy, qz, qw)


# =============================================================================
# Node
# =============================================================================

class CanvasStrokeCorrector(Node):

    def __init__(self):
        super().__init__('canvas_stroke_corrector')

        self.declare_parameter('latch_window_s',   10.0)
        self.declare_parameter('pose_timeout_s',    5.0)
        self.declare_parameter('publish_rate_hz',  50.0)
        self.declare_parameter('enabled',           True)
        # Standoff above canvas surface for the Z constraint (metres).
        # 0.0 = brush tip touching surface. 0.005 = 5mm hover (safe default).
        self.declare_parameter('brush_standoff_m',  0.005)

        self._tf_broadcaster = TransformBroadcaster(self)
        self._tf_buffer      = Buffer()
        self._tf_listener    = TransformListener(self._tf_buffer, self)

        # Publishers
        self._error_pub      = self.create_publisher(
            PoseStamped, '/canvas/pose_error',      10)
        self._assumed_pub    = self.create_publisher(
            PoseStamped, '/canvas/pose_assumed',    10)
        self._data_pub       = self.create_publisher(
            String,      '/canvas/correction_data', 10)
        # Z depth constraint for MoveIt Servo.
        # PoseStamped in camera_color_optical_frame:
        #   position.z = canvas surface depth + standoff (metres from camera)
        #                this is the Z the brush tip must maintain
        #   position.x/y = canvas centre lateral position
        #   orientation  = canvas face normal (tool must approach perpendicular)
        # Member 2 uses position.z as the Z-lock value for servo streaming.
        self._z_constraint_pub = self.create_publisher(
            PoseStamped, '/canvas/z_constraint',   10)

        # Subscribers
        self.create_subscription(
            PoseStamped, '/canvas/pose',         self._real_pose_cb,    10)
        self.create_subscription(
            PoseStamped, '/canvas/pose_assumed', self._assumed_pose_cb, 10)

        # ── State ─────────────────────────────────────────────────────────────
        self._lock             = threading.Lock()
        self._real_pose        = None
        self._real_recv_time   = None   # wall time of last received pose

        # Latch accumulation
        self._assumed_pose     = None   # frozen once latched
        self._assumed_latched  = False
        self._collect_start    = None   # wall time when first pose arrived
        self._collect_ts       = []     # list of (wall_time, pose) tuples

        rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self.create_timer(1.0/rate, self._tick)

        latch_s = self.get_parameter('latch_window_s').get_parameter_value().double_value
        self.get_logger().info(
            f'\ncanvas_stroke_corrector ready.\n'
            f'  Will collect poses for {latch_s:.0f}s then latch as T_assumed.\n'
            f'  Keep canvas STILL during collection window.\n'
            f'  Publishes: /canvas/pose_error  /canvas/pose_assumed  /canvas/correction_data\n'
            f'  TF:        canvas_centre_assumed\n'
            f'  To override reference: ros2 topic pub /canvas/pose_assumed ...\n'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _real_pose_cb(self, msg: PoseStamped):
        recv = time.monotonic()
        with self._lock:
            self._real_pose      = msg
            self._real_recv_time = recv

            if self._assumed_latched:
                return   # already locked -- ignore for collection

            # Start collection timer on first pose
            if self._collect_start is None:
                self._collect_start = recv
                self.get_logger().info(
                    '\n[CORRECTOR] First detection -- starting reference collection.\n'
                    '  Keep canvas STILL for the collection window.\n'
                    f'  latch_window_s = '
                    f'{self.get_parameter("latch_window_s").get_parameter_value().double_value:.0f}s'
                )

            self._collect_ts.append((recv, msg))

    def _assumed_pose_cb(self, msg: PoseStamped):
        """External override -- immediately latches regardless of window."""
        with self._lock:
            self._assumed_pose    = msg
            self._assumed_latched = True
        self._broadcast_assumed_tf(msg)
        self.get_logger().info(
            f'[CORRECTOR] Assumed pose OVERRIDDEN externally:\n'
            f'  ({msg.pose.position.x:+.4f}, '
            f'{msg.pose.position.y:+.4f}, {msg.pose.position.z:+.4f}) m')

    def _try_latch(self, now_wall: float):
        """
        Called from tick. Checks if collection window has elapsed.
        If yes, averages all collected poses and freezes T_assumed.
        Thread note: called with self._lock held.
        """
        latch_window = self.get_parameter(
            'latch_window_s').get_parameter_value().double_value

        if self._collect_start is None:
            return   # no poses yet
        elapsed = now_wall - self._collect_start
        if elapsed < latch_window:
            return   # still collecting

        # Window elapsed -- compute averaged pose
        translations = []
        quats        = []
        frame_id     = 'camera_color_optical_frame'
        for _, msg in self._collect_ts:
            o = msg.pose.orientation
            p = msg.pose.position
            translations.append([p.x, p.y, p.z])
            quats.append((o.x, o.y, o.z, o.w))
            frame_id = msg.header.frame_id or frame_id

        t_avg = np.mean(translations, axis=0)
        q_avg = average_quats(quats)   # (qx, qy, qz, qw)

        # Build the assumed PoseStamped
        ps = PoseStamped()
        ps.header.frame_id    = frame_id
        ps.header.stamp       = self.get_clock().now().to_msg()
        ps.pose.position.x    = float(t_avg[0])
        ps.pose.position.y    = float(t_avg[1])
        ps.pose.position.z    = float(t_avg[2])
        ps.pose.orientation.x = float(q_avg[0])
        ps.pose.orientation.y = float(q_avg[1])
        ps.pose.orientation.z = float(q_avg[2])
        ps.pose.orientation.w = float(q_avg[3])

        self._assumed_pose    = ps
        self._assumed_latched = True
        n = len(self._collect_ts)
        self._collect_ts      = []   # free memory

        self.get_logger().info(
            f'\n[CORRECTOR] T_assumed LATCHED from {n} frames over {elapsed:.1f}s:\n'
            f'  position : ({t_avg[0]:+.4f}, {t_avg[1]:+.4f}, {t_avg[2]:+.4f}) m\n'
            f'  std dev  : {np.std(translations, axis=0)} m\n'
            f'  Canvas must stay in this position for zero error.\n'
            f'  Move canvas now -- error will reflect displacement.'
        )

    def _broadcast_assumed_tf(self, pose_msg: PoseStamped):
        ts                         = TransformStamped()
        ts.header.stamp            = self.get_clock().now().to_msg()
        ts.header.frame_id         = pose_msg.header.frame_id or 'camera_color_optical_frame'
        ts.child_frame_id          = 'canvas_centre_assumed'
        ts.transform.translation.x = pose_msg.pose.position.x
        ts.transform.translation.y = pose_msg.pose.position.y
        ts.transform.translation.z = pose_msg.pose.position.z
        ts.transform.rotation      = pose_msg.pose.orientation
        self._tf_broadcaster.sendTransform(ts)

    # ── Main tick ─────────────────────────────────────────────────────────────

    def _tick(self):
        if not self.get_parameter('enabled').get_parameter_value().bool_value:
            return

        now_wall = time.monotonic()

        with self._lock:
            real_pose      = self._real_pose
            real_recv_time = self._real_recv_time
            assumed_latched = self._assumed_latched
            assumed_pose   = self._assumed_pose
            collect_start  = self._collect_start
            n_collected    = len(self._collect_ts)

            # Try to latch if window elapsed
            if not assumed_latched:
                self._try_latch(now_wall)
                assumed_latched = self._assumed_latched
                assumed_pose    = self._assumed_pose

        # ── Guard: waiting for first pose ─────────────────────────────────────
        if real_pose is None:
            self._publish_data({'status': 'WAITING', 'reason': 'no /canvas/pose received'})
            return

        # ── Guard: still collecting reference ─────────────────────────────────
        if not assumed_latched:
            latch_window = self.get_parameter(
                'latch_window_s').get_parameter_value().double_value
            elapsed = (now_wall - collect_start) if collect_start else 0.0
            remaining = max(0.0, latch_window - elapsed)
            self._publish_data({
                'status':    'COLLECTING',
                'elapsed_s': round(elapsed, 1),
                'window_s':  latch_window,
                'remaining_s': round(remaining, 1),
                'n_frames':  n_collected,
                'reason':    f'collecting reference -- {remaining:.1f}s remaining, keep canvas still',
            })
            return

        # ── Guard: pose freshness ─────────────────────────────────────────────
        timeout  = self.get_parameter('pose_timeout_s').get_parameter_value().double_value
        age_wall = now_wall - real_recv_time
        if age_wall > timeout:
            self._publish_data({
                'status': 'PAUSED',
                'reason': f'pose stale {age_wall:.2f}s > {timeout:.1f}s'
            })
            return

        # ── SE3 error ─────────────────────────────────────────────────────────
        R_real,    t_real    = pose_to_Rt(real_pose.pose)
        R_assumed, t_assumed = pose_to_Rt(assumed_pose.pose)

        R_err   = R_assumed.T @ R_real
        t_err   = R_assumed.T @ (t_real - t_assumed)
        rot_deg = rotation_angle_deg(R_err)
        latency_ms = (now_wall - real_recv_time) * 1000.0

        stamp = self.get_clock().now().to_msg()
        self._publish_error(stamp, t_err, R_err)
        self._broadcast_assumed_tf(assumed_pose)

        # Echo assumed pose with fresh stamp
        ap = PoseStamped()
        ap.header.frame_id = assumed_pose.header.frame_id
        ap.header.stamp    = stamp
        ap.pose            = assumed_pose.pose
        self._assumed_pub.publish(ap)

        self._publish_data({
            'status':       'OK',
            'pose_age_ms':  round(age_wall * 1000.0, 1),
            'latency_ms':   round(latency_ms, 1),
            'error': {
                'lateral_mm':   round(float(t_err[0]) * 1000.0, 3),
                'vertical_mm':  round(float(t_err[1]) * 1000.0, 3),
                'depth_mm':     round(float(t_err[2]) * 1000.0, 3),
                'rotation_deg': round(rot_deg, 3),
                'total_3d_mm':  round(float(np.linalg.norm(t_err)) * 1000.0, 3),
            },
            'T_real':    {'x': round(float(t_real[0]),6),
                          'y': round(float(t_real[1]),6),
                          'z': round(float(t_real[2]),6)},
            'T_assumed': {'x': round(float(t_assumed[0]),6),
                          'y': round(float(t_assumed[1]),6),
                          'z': round(float(t_assumed[2]),6)},
            'z_constraint': {
                'canvas_depth_m':   round(float(t_real[2]), 6),
                'assumed_depth_m':  round(float(t_assumed[2]), 6),
                'standoff_m':       round(self.get_parameter('brush_standoff_m')
                                         .get_parameter_value().double_value, 6),
                'brush_z_m':        round(float(t_real[2]) - self.get_parameter(
                                         'brush_standoff_m')
                                         .get_parameter_value().double_value, 6),
            },
        })
        self._publish_z_constraint(stamp, t_real, R_real)

    def _publish_z_constraint(self, stamp, t_real, R_real):
        """
        Z depth constraint for MoveIt Servo (Member 2).

        Published as PoseStamped in camera_color_optical_frame:
          position.x/y = live canvas centre lateral position
          position.z   = canvas surface depth MINUS standoff
                         (i.e. where brush tip must sit in camera Z)
          orientation  = canvas face orientation (tool must stay perpendicular)

        The canvas normal in optical frame is the forward-facing direction
        of the canvas (the face the brush draws on).  In camera optical frame
        +Z = into scene, so canvas_depth - standoff keeps the brush
        standoff_m in front of the surface.

        Member 2 usage:
          lock EEF z to msg.pose.position.z while allowing XY motion for strokes.
        """
        standoff = self.get_parameter(
            'brush_standoff_m').get_parameter_value().double_value
        frame = 'camera_color_optical_frame'

        ps                    = PoseStamped()
        ps.header.frame_id    = frame
        ps.header.stamp       = stamp
        ps.pose.position.x    = float(t_real[0])
        ps.pose.position.y    = float(t_real[1])
        ps.pose.position.z    = float(t_real[2]) - standoff   # brush tip depth
        # Orientation: canvas face normal as quaternion (raw from detector)
        eq = rotmat_to_quat(R_real)
        ps.pose.orientation.x = float(eq[0])
        ps.pose.orientation.y = float(eq[1])
        ps.pose.orientation.z = float(eq[2])
        ps.pose.orientation.w = float(eq[3])
        self._z_constraint_pub.publish(ps)

    def _publish_error(self, stamp, t_err, R_err):
        ps                    = PoseStamped()
        ps.header.frame_id    = 'canvas_centre_assumed'
        ps.header.stamp       = stamp
        ps.pose.position.x    = float(t_err[0])
        ps.pose.position.y    = float(t_err[1])
        ps.pose.position.z    = float(t_err[2])
        eq = rotmat_to_quat(R_err)
        ps.pose.orientation.x = float(eq[0])
        ps.pose.orientation.y = float(eq[1])
        ps.pose.orientation.z = float(eq[2])
        ps.pose.orientation.w = float(eq[3])
        self._error_pub.publish(ps)

    def _publish_data(self, d: dict):
        msg      = String()
        msg.data = json.dumps(d)
        self._data_pub.publish(msg)
        status = d.get('status', '')
        if status == 'COLLECTING':
            self.get_logger().info(
                f'[CORRECTOR] Collecting reference -- '
                f'{d["remaining_s"]:.1f}s remaining  '
                f'({d["n_frames"]} frames)',
                throttle_duration_sec=1.0)
        elif status not in ('OK', ''):
            self.get_logger().warn(
                f'[CORRECTOR] {status}: {d.get("reason","")}',
                throttle_duration_sec=2.0)


# =============================================================================
# Entry point
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = CanvasStrokeCorrector()
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