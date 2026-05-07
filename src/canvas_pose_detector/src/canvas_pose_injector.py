#!/usr/bin/env python3
"""
canvas_pose_injector.py  --  Synthetic canvas pose publisher for camera-free validation.

Publishes the same topics as canvas_pose_detector so the full downstream pipeline
(canvas_pose_visualiser, Member 2 TF transform, RViz, URSim) can be validated
without any physical camera or AprilTag markers.

Topics published (identical to the real detector):
  /canvas/pose            PoseStamped     -- 6-DOF canvas pose in parent_frame
  /canvas/viewing_angles  Vector3Stamped  -- x=Roll, y=H, z=V (degrees)
  /canvas/markers         MarkerArray     -- RViz: plane, corners, axes, label

TF frames broadcast:
  canvas_centre           child of parent_frame
  canvas_marker_TL/TR/BR/BL  children of parent_frame

All pose values are ROS2 parameters -- nothing is hardcoded.
Live-tune any value without rebuilding:
  ros2 param set /canvas_pose_injector canvas_z 0.7
  ros2 param set /canvas_pose_injector animate true

Parameters
----------
parent_frame        str    TF parent frame (default: camera_color_optical_frame)
canvas_x/y/z        float  Canvas centre position in parent_frame (metres)
canvas_roll_deg     float  Canvas roll  in degrees (ROS ZYX extrinsic)
canvas_pitch_deg    float  Canvas pitch in degrees
canvas_yaw_deg      float  Canvas yaw   in degrees
canvas_w_m          float  Physical canvas width  (metres, A4 landscape = 0.297)
canvas_h_m          float  Physical canvas height (metres, A4 landscape = 0.210)
publish_rate_hz     float  Publish rate (default: 10.0 Hz)
animate             bool   If true, oscillate one axis sinusoidally
animate_axis        str    Axis to oscillate: x | y | z | roll | pitch | yaw
animate_range       float  Half-amplitude of oscillation (degrees or metres)
animate_period_s    float  Period of one full sine cycle (seconds)
"""

import math

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, PoseStamped, TransformStamped, Vector3Stamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


# =============================================================================
# Geometry helpers
# =============================================================================

def euler_to_quat(roll_rad: float, pitch_rad: float, yaw_rad: float):
    """ZYX extrinsic Euler -> quaternion (ROS convention).
    Returns (qx, qy, qz, qw).
    """
    cr, sr = math.cos(roll_rad  / 2.0), math.sin(roll_rad  / 2.0)
    cp, sp = math.cos(pitch_rad / 2.0), math.sin(pitch_rad / 2.0)
    cy, sy = math.cos(yaw_rad   / 2.0), math.sin(yaw_rad   / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,  # qx
        cr * sp * cy + sr * cp * sy,  # qy
        cr * cp * sy - sr * sp * cy,  # qz
        cr * cp * cy + sr * sp * sy,  # qw
    )


def quat_to_rotmat(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Quaternion -> 3x3 rotation matrix (numpy)."""
    return np.array([
        [1 - 2*(qy*qy + qz*qz),     2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [    2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz),     2*(qy*qz - qx*qw)],
        [    2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)],
    ])


def viewing_angles_from_rotation(R: np.ndarray):
    """Compute Roll, H (horizontal), V (vertical) viewing angles in degrees.

    Replicates the geometric approach from canvas_pose_detector.cpp:
      - Roll  = atan2(h_edge.y, h_edge.x)  -- canvas in-plane rotation
      - H     = atan2(n_u.x,  -n_u.z)      -- horizontal viewing angle
      - V     = atan2(-n_u.y, -n_u.z)      -- vertical viewing angle
    where n_u is the canvas normal unrolled by -Roll around the Z axis.
    Naturally zeroed when canvas faces camera flat-on.
    """
    # Canvas normal in camera frame (-Z canvas axis, forced toward camera)
    n = R @ np.array([0.0, 0.0, -1.0])
    if n[2] > 0.0:
        n = -n

    # Roll from horizontal canvas edge direction
    h_edge = R @ np.array([1.0, 0.0, 0.0])
    roll_rad = math.atan2(h_edge[1], h_edge[0])

    # Unroll the normal by -Roll around optical (Z) axis
    cos_r = math.cos(-roll_rad)
    sin_r = math.sin(-roll_rad)
    n_u = np.array([
        cos_r * n[0] - sin_r * n[1],
        sin_r * n[0] + cos_r * n[1],
        n[2],
    ])

    roll_deg = math.degrees(roll_rad)
    H_deg    = math.degrees(math.atan2( n_u[0], -n_u[2]))
    V_deg    = math.degrees(math.atan2(-n_u[1], -n_u[2]))
    return roll_deg, H_deg, V_deg


def corner_positions_world(centre: np.ndarray, R: np.ndarray,
                            canvas_w: float, canvas_h: float):
    """Compute 4 corner 3D positions in the camera frame.

    Canvas local frame:
      TL = (-w/2, +h/2, 0)
      TR = (+w/2, +h/2, 0)
      BR = (+w/2, -h/2, 0)
      BL = (-w/2, -h/2, 0)
    """
    hw, hh = canvas_w / 2.0, canvas_h / 2.0
    local_corners = [
        np.array([-hw,  hh, 0.0]),  # TL
        np.array([ hw,  hh, 0.0]),  # TR
        np.array([ hw, -hh, 0.0]),  # BR
        np.array([-hw, -hh, 0.0]),  # BL
    ]
    return [centre + R @ v for v in local_corners]


# =============================================================================
# Node
# =============================================================================

class CanvasPoseInjector(Node):

    CORNER_FRAME_NAMES = [
        'canvas_marker_TL',
        'canvas_marker_TR',
        'canvas_marker_BR',
        'canvas_marker_BL',
    ]

    def __init__(self):
        super().__init__('canvas_pose_injector')

        # ── Declare all parameters ────────────────────────────────────────────
        # Position of canvas centre in parent_frame (metres)
        self.declare_parameter('parent_frame',      'camera_color_optical_frame')
        self.declare_parameter('canvas_x',          0.0)
        self.declare_parameter('canvas_y',          0.0)
        self.declare_parameter('canvas_z',          0.5)

        # Orientation of canvas in degrees (ZYX extrinsic, ROS convention)
        # 0/0/0 = canvas face pointing directly at the camera (flat-on)
        self.declare_parameter('canvas_roll_deg',   0.0)
        self.declare_parameter('canvas_pitch_deg',  0.0)
        self.declare_parameter('canvas_yaw_deg',    0.0)

        # Physical canvas size (metres)
        # A4 landscape: w=0.297, h=0.210
        self.declare_parameter('canvas_w_m',        0.297)
        self.declare_parameter('canvas_h_m',        0.210)

        # Publish rate
        self.declare_parameter('publish_rate_hz',   10.0)

        # Animation: oscillate one parameter sinusoidally for dynamic testing
        # Set animate:=true animate_axis:=yaw animate_range:=30.0 to sweep yaw
        self.declare_parameter('animate',           False)
        self.declare_parameter('animate_axis',      'yaw')   # x|y|z|roll|pitch|yaw
        self.declare_parameter('animate_range',     30.0)    # degrees or metres
        self.declare_parameter('animate_period_s',  4.0)     # seconds per cycle

        # ── Publishers ────────────────────────────────────────────────────────
        self.pose_pub_   = self.create_publisher(PoseStamped,    '/canvas/pose',           10)
        self.angles_pub_ = self.create_publisher(Vector3Stamped, '/canvas/viewing_angles', 10)
        self.marker_pub_ = self.create_publisher(MarkerArray,    '/canvas/markers',        5)

        # ── TF broadcaster ────────────────────────────────────────────────────
        self.tf_br_ = TransformBroadcaster(self)

        # ── Timer ─────────────────────────────────────────────────────────────
        rate_hz = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self.create_timer(1.0 / rate_hz, self._publish_cb)

        self.start_time_ = self.get_clock().now()

        self.get_logger().info(
            '\ncanvas_pose_injector ready -- no camera required.\n'
            '  Publishes: /canvas/pose, /canvas/viewing_angles, /canvas/markers\n'
            '  TF frames: canvas_centre, canvas_marker_TL/TR/BR/BL\n'
            '\n  Live-tune pose:\n'
            '    ros2 param set /canvas_pose_injector canvas_z 0.7\n'
            '    ros2 param set /canvas_pose_injector canvas_yaw_deg 30.0\n'
            '\n  Animate sweep:\n'
            '    ros2 param set /canvas_pose_injector animate true\n'
            '    ros2 param set /canvas_pose_injector animate_axis yaw\n'
            '    ros2 param set /canvas_pose_injector animate_range 30.0\n'
        )

    # ─────────────────────────────────────────────────────────────────────────
    # Main publish callback
    # ─────────────────────────────────────────────────────────────────────────

    def _publish_cb(self):
        now   = self.get_clock().now()
        stamp = now.to_msg()

        # Read all params each cycle (enables live ros2 param set without rebuild)
        parent_frame = self.get_parameter('parent_frame').get_parameter_value().string_value
        cx  = self.get_parameter('canvas_x').get_parameter_value().double_value
        cy  = self.get_parameter('canvas_y').get_parameter_value().double_value
        cz  = self.get_parameter('canvas_z').get_parameter_value().double_value
        r_d = self.get_parameter('canvas_roll_deg').get_parameter_value().double_value
        p_d = self.get_parameter('canvas_pitch_deg').get_parameter_value().double_value
        y_d = self.get_parameter('canvas_yaw_deg').get_parameter_value().double_value
        cw  = self.get_parameter('canvas_w_m').get_parameter_value().double_value
        ch  = self.get_parameter('canvas_h_m').get_parameter_value().double_value

        animate      = self.get_parameter('animate').get_parameter_value().bool_value
        anim_axis    = self.get_parameter('animate_axis').get_parameter_value().string_value
        anim_range   = self.get_parameter('animate_range').get_parameter_value().double_value
        anim_period  = self.get_parameter('animate_period_s').get_parameter_value().double_value

        # Apply sinusoidal offset to the chosen axis
        if animate and anim_period > 0.0:
            elapsed_s = (now - self.start_time_).nanoseconds * 1e-9
            offset = anim_range * math.sin(2.0 * math.pi * elapsed_s / anim_period)
            if   anim_axis == 'x':     cx  += offset
            elif anim_axis == 'y':     cy  += offset
            elif anim_axis == 'z':     cz  += offset
            elif anim_axis == 'roll':  r_d += offset
            elif anim_axis == 'pitch': p_d += offset
            elif anim_axis == 'yaw':   y_d += offset

        # Convert to quaternion
        qx, qy, qz, qw = euler_to_quat(
            math.radians(r_d),
            math.radians(p_d),
            math.radians(y_d),
        )
        R = quat_to_rotmat(qx, qy, qz, qw)
        centre = np.array([cx, cy, cz])
        corners = corner_positions_world(centre, R, cw, ch)
        roll_deg, H_deg, V_deg = viewing_angles_from_rotation(R)

        self._publish_pose(parent_frame, stamp, cx, cy, cz, qx, qy, qz, qw)
        self._publish_viewing_angles(parent_frame, stamp, roll_deg, H_deg, V_deg)
        self._broadcast_tf_frames(parent_frame, stamp, cx, cy, cz, qx, qy, qz, qw, corners)
        self._publish_markers(parent_frame, stamp, centre, R, corners,
                              qx, qy, qz, qw, cw, ch, roll_deg, H_deg, V_deg)

    # ─────────────────────────────────────────────────────────────────────────
    # Publishers
    # ─────────────────────────────────────────────────────────────────────────

    def _publish_pose(self, frame, stamp, cx, cy, cz, qx, qy, qz, qw):
        ps = PoseStamped()
        ps.header.stamp    = stamp
        ps.header.frame_id = frame
        ps.pose.position.x    = cx
        ps.pose.position.y    = cy
        ps.pose.position.z    = cz
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        self.pose_pub_.publish(ps)

    def _publish_viewing_angles(self, frame, stamp, roll_deg, H_deg, V_deg):
        vs = Vector3Stamped()
        vs.header.stamp    = stamp
        vs.header.frame_id = frame
        vs.vector.x = roll_deg
        vs.vector.y = H_deg
        vs.vector.z = V_deg
        self.angles_pub_.publish(vs)

    def _broadcast_tf_frames(self, parent, stamp, cx, cy, cz, qx, qy, qz, qw, corners):
        self._send_tf(parent, 'canvas_centre', cx, cy, cz, qx, qy, qz, qw, stamp)
        for name, corner in zip(self.CORNER_FRAME_NAMES, corners):
            self._send_tf(parent, name,
                          corner[0], corner[1], corner[2],
                          qx, qy, qz, qw, stamp)

    def _send_tf(self, parent, child, x, y, z, qx, qy, qz, qw, stamp):
        ts = TransformStamped()
        ts.header.stamp           = stamp
        ts.header.frame_id        = parent
        ts.child_frame_id         = child
        ts.transform.translation.x = x
        ts.transform.translation.y = y
        ts.transform.translation.z = z
        ts.transform.rotation.x   = qx
        ts.transform.rotation.y   = qy
        ts.transform.rotation.z   = qz
        ts.transform.rotation.w   = qw
        self.tf_br_.sendTransform(ts)

    # ─────────────────────────────────────────────────────────────────────────
    # RViz markers
    # ─────────────────────────────────────────────────────────────────────────

    def _publish_markers(self, frame, stamp, centre, R, corners,
                         qx, qy, qz, qw, cw, ch, roll_deg, H_deg, V_deg):
        arr  = MarkerArray()
        life = Duration(sec=1, nanosec=0)

        def base(ns, mid, mtype):
            m = Marker()
            m.header.frame_id = frame
            m.header.stamp    = stamp
            m.ns              = ns
            m.id              = mid
            m.type            = mtype
            m.action          = Marker.ADD
            m.pose.orientation.w = 1.0
            m.lifetime        = life
            return m

        def pt(v):
            p = Point()
            p.x, p.y, p.z = float(v[0]), float(v[1]), float(v[2])
            return p

        # Canvas outline (cyan loop)
        outline = base('canvas', 0, Marker.LINE_STRIP)
        outline.scale.x = 0.003
        outline.color.r = 0.0
        outline.color.g = 0.8
        outline.color.b = 0.8
        outline.color.a = 1.0
        for corner in list(corners) + [corners[0]]:
            outline.points.append(pt(corner))
        arr.markers.append(outline)

        # Corner spheres + labels
        corner_labels = ['TL', 'TR', 'BR', 'BL']
        for i, (corner, lbl) in enumerate(zip(corners, corner_labels)):
            sph = base('canvas', 1 + i, Marker.SPHERE)
            sph.pose.position.x = corner[0]
            sph.pose.position.y = corner[1]
            sph.pose.position.z = corner[2]
            sph.scale.x = sph.scale.y = sph.scale.z = 0.012
            sph.color.r = 0.0
            sph.color.g = 1.0
            sph.color.b = 1.0
            sph.color.a = 1.0
            arr.markers.append(sph)

            txt = base('canvas', 10 + i, Marker.TEXT_VIEW_FACING)
            txt.pose.position.x = corner[0]
            txt.pose.position.y = corner[1] - 0.015
            txt.pose.position.z = corner[2]
            txt.scale.z = 0.018
            txt.text    = lbl
            txt.color.r = 1.0
            txt.color.g = 1.0
            txt.color.b = 0.0
            txt.color.a = 1.0
            arr.markers.append(txt)

        # Canvas normal arrow (magenta, shows which way the face points)
        normal_world = R @ np.array([0.0, 0.0, -1.0])
        normal_tip   = centre + 0.06 * normal_world
        norm_arrow = base('canvas', 20, Marker.ARROW)
        norm_arrow.points = [pt(centre), pt(normal_tip)]
        norm_arrow.scale.x = 0.004
        norm_arrow.scale.y = 0.010
        norm_arrow.scale.z = 0.0
        norm_arrow.color.r = 1.0
        norm_arrow.color.g = 0.0
        norm_arrow.color.b = 1.0
        norm_arrow.color.a = 1.0
        arr.markers.append(norm_arrow)

        # XYZ axes at canvas centre (red=X, green=Y, blue=Z)
        axis_len = max(cw, ch) * 0.4
        axes = [
            (np.array([1.0, 0.0, 0.0]), (1.0, 0.0, 0.0), 30),
            (np.array([0.0, 1.0, 0.0]), (0.0, 1.0, 0.0), 31),
            (np.array([0.0, 0.0, 1.0]), (0.0, 0.0, 1.0), 32),
        ]
        for local_ax, (ar, ag, ab), aid in axes:
            tip   = centre + axis_len * (R @ local_ax)
            arrow = base('canvas_axes', aid, Marker.ARROW)
            arrow.points = [pt(centre), pt(tip)]
            arrow.scale.x = 0.004
            arrow.scale.y = 0.010
            arrow.scale.z = 0.0
            arrow.color.r = ar
            arrow.color.g = ag
            arrow.color.b = ab
            arrow.color.a = 1.0
            arr.markers.append(arrow)

        # Pose summary text label at canvas centre
        lbl_m = base('canvas', 40, Marker.TEXT_VIEW_FACING)
        lbl_m.pose.position.x = centre[0]
        lbl_m.pose.position.y = centre[1] - 0.03
        lbl_m.pose.position.z = centre[2]
        lbl_m.scale.z = 0.022
        lbl_m.text = (
            f'[INJECTED]\n'
            f'XYZ: ({centre[0]:+.3f}, {centre[1]:+.3f}, {centre[2]:+.3f}) m\n'
            f'Roll={roll_deg:+.1f}  H={H_deg:+.1f}  V={V_deg:+.1f} deg'
        )
        lbl_m.color.r = 1.0
        lbl_m.color.g = 1.0
        lbl_m.color.b = 1.0
        lbl_m.color.a = 1.0
        arr.markers.append(lbl_m)

        self.marker_pub_.publish(arr)


# =============================================================================
# Entry point
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = CanvasPoseInjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()