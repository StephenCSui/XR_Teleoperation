#!/usr/bin/env python3
"""
canvas_depth_visualiser.py  --  Z depth constraint side-view visualiser.

Shows a top-down XZ view (camera frame: X=right, Z=depth/forward).
The canvas plane is drawn as an ANGLED LINE based on the live canvas
orientation -- it updates every frame as the canvas moves or tilts.

The plane angle comes from projecting the canvas face normal onto XZ:
  normal_xz = [n.x, n.z]  (from canvas quaternion)
  plane_direction = perpendicular to normal_xz = [-n.z, n.x]
  plane line drawn through canvas centre in that direction

Elements:
  Amber box + cone    -- Camera at origin, scanning beam
  Cyan angled line    -- REAL canvas plane (live, angles with tilt)
  Green dashed line   -- ASSUMED canvas plane (reference at latch)
  Blue filled band    -- Standoff zone between brush and canvas
  Yellow dash-dot     -- Brush Z constraint line (parallel to canvas plane)
  Orange bracket      -- Depth error at canvas centre
  Grey filled rect    -- Canvas cross-section (physical thickness)
  Bottom panel        -- All depth values + tilt angle in degrees

Subscribes:
  /canvas/pose            PoseStamped  Live canvas pose (position + orientation)
  /canvas/pose_assumed    PoseStamped  Reference canvas pose
  /canvas/z_constraint    PoseStamped  Brush Z constraint
  /canvas/correction_data String       JSON with depth error + latency

Publishes:
  /canvas/depth_view  Image  700x480 side-view plot at 15fps
"""

import json
import math
import threading

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


# =============================================================================
# Drawing constants
# =============================================================================

FONT       = cv2.FONT_HERSHEY_SIMPLEX
W, H       = 700, 480
MARGIN_L   = 80
MARGIN_R   = 20
MARGIN_T   = 50
MARGIN_B   = 75

COL_BG       = (18,  18,  18)
COL_GRID     = (40,  40,  40)
COL_CAMERA   = (200, 160,  60)
COL_REAL     = (60,  180, 220)
COL_ASSUMED  = (60,  200,  60)
COL_STANDOFF = (30,  100, 200)
COL_BRUSH    = (50,  220, 220)
COL_ERROR    = (30,  140, 255)
COL_CANVAS   = (80,  80,  80)
COL_TEXT     = (210, 210, 210)
COL_OK       = (60,  220,  80)
COL_WARN     = (0,   165, 255)


# =============================================================================
# Geometry helpers
# =============================================================================

def quat_to_rotmat(qx, qy, qz, qw):
    return np.array([
        [1-2*(qy*qy+qz*qz),   2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
        [  2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz),   2*(qy*qz-qx*qw)],
        [  2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)],
    ])


def canvas_normal_xz(qx, qy, qz, qw):
    """
    Returns the canvas face normal projected onto the XZ camera plane,
    normalised.  In camera optical frame, canvas -Z axis faces toward
    the camera, so the face-forward normal is R @ [0,0,-1].

    Returns (nx, nz) -- the direction the canvas faces in XZ.
    Also returns tilt_deg: angle of canvas from frontal (0 = facing camera).
    """
    R  = quat_to_rotmat(qx, qy, qz, qw)
    n  = R @ np.array([0.0, 0.0, -1.0])   # canvas face normal

    # Force toward camera (nz should be negative in optical frame, i.e. toward origin)
    if n[2] > 0:
        n = -n

    # Project to XZ plane
    nx, nz = float(n[0]), float(n[2])
    mag = math.sqrt(nx*nx + nz*nz)
    if mag < 1e-6:
        nx, nz = 0.0, -1.0   # fallback: frontal
    else:
        nx /= mag; nz /= mag

    # Tilt angle from frontal (frontal = nz=-1, nx=0)
    tilt_deg = math.degrees(math.atan2(abs(nx), abs(nz)))
    return nx, nz, tilt_deg


def plane_line_endpoints(cx_m, cz_m, nx, nz, half_span_m, depth_to_x, scene_y,
                         scene_half_h_px, scale_x):
    """
    Compute pixel endpoints for a plane line passing through (cx_m, cz_m)
    with normal (nx, nz) in XZ space.

    plane direction = perpendicular to normal = (-nz, nx) in (X, Z)
    We draw the line extending ±half_span_m in scene X on either side.

    Returns (p0_px, p1_px) as integer (x,y) pixel tuples.
    """
    # Plane tangent direction in XZ: perpendicular to normal
    # (-nz, nx) rotates normal 90° CCW in XZ plane
    tx, tz = -nz, nx

    # Two endpoints at ±half_span_m along the tangent from canvas centre
    p0_x_m = cx_m - half_span_m * tx
    p0_z_m = cz_m - half_span_m * tz
    p1_x_m = cx_m + half_span_m * tx
    p1_z_m = cz_m + half_span_m * tz

    # Convert to pixel coords
    # Z → horizontal pixel via depth_to_x
    # X → vertical pixel: scene_y is where X=0, scale_x px/m
    def to_px(xm, zm):
        px_x = depth_to_x(zm)
        px_y = int(scene_y - xm * scale_x)
        return (px_x, px_y)

    return to_px(p0_x_m, p0_z_m), to_px(p1_x_m, p1_z_m)


def draw_dashed_line(img, p0, p1, col, dash=12, thick=1):
    dx = p1[0]-p0[0]; dy = p1[1]-p0[1]
    length = math.sqrt(dx*dx+dy*dy)
    if length < 1: return
    n_steps = max(1, int(length/dash))
    draw = True
    for k in range(n_steps):
        t0 = k/n_steps; t1 = (k+1)/n_steps
        a  = (int(p0[0]+t0*dx), int(p0[1]+t0*dy))
        b  = (int(p0[0]+t1*dx), int(p0[1]+t1*dy))
        if draw:
            cv2.line(img, a, b, col, thick)
        draw = not draw


def draw_dashdot_line(img, p0, p1, col, thick=2):
    dx = p1[0]-p0[0]; dy = p1[1]-p0[1]
    length = math.sqrt(dx*dx+dy*dy)
    if length < 1: return
    segs  = [12, 4, 3, 4]   # dash, gap, dot, gap
    total = sum(segs)
    t     = 0.0
    idx   = 0
    draw  = True
    while t < 1.0:
        seg_len = segs[idx % 4] / (length / 1.0) if length > 0 else 1.0
        seg_frac = min(segs[idx % 4] / length, 1.0 - t)
        t1 = t + seg_frac
        if draw:
            a = (int(p0[0]+t*dx),  int(p0[1]+t*dy))
            b = (int(p0[0]+t1*dx), int(p0[1]+t1*dy))
            cv2.line(img, a, b, col, thick)
        t  = t1 + (segs[(idx+1) % 4] / length if length > 0 else 0)
        idx += 2
        if t >= 1.0: break


# =============================================================================
# Main plot builder
# =============================================================================

def build_depth_view(
    real_pose,        # (cx, cy, cz, qx, qy, qz, qw) or None
    assumed_pose,     # (cx, cy, cz, qx, qy, qz, qw) or None
    brush_z_m,
    standoff_m,
    depth_err_mm,
    latency_ms,
    pose_age_ms,
    status,
    canvas_h_m=0.210,
):
    img = np.full((H, W, 3), COL_BG, dtype=np.uint8)
    pw  = W - MARGIN_L - MARGIN_R
    ph  = H - MARGIN_T - MARGIN_B

    # ── Depth axis ────────────────────────────────────────────────────────────
    # Depth (Z) mapped to horizontal pixel axis.
    # Scene X mapped to vertical axis (X=0 = optical axis = scene centre).

    ref_depth  = (assumed_pose[2] if assumed_pose else
                  real_pose[2] if real_pose else 0.4)
    view_half  = 0.200   # ±200mm
    depth_min  = ref_depth - view_half
    depth_max  = ref_depth + view_half

    def depth_to_x(z_m):
        frac = (z_m - depth_min) / (depth_max - depth_min)
        return int(MARGIN_L + np.clip(frac, 0.0, 1.0) * pw)

    # Scene centre Y (optical axis, X=0)
    scene_y   = MARGIN_T + ph // 2
    # Scale: how many pixels per metre in the scene X direction
    scale_x   = ph / (2 * view_half * 1.2)   # shows ±1.2× view range in X

    # ── Title ─────────────────────────────────────────────────────────────────
    age_col = COL_OK if pose_age_ms < 200 else COL_WARN
    cv2.putText(img,
                f'Z Depth Constraint  '
                f'[age: {pose_age_ms:.0f}ms  latency: {latency_ms:.1f}ms]',
                (MARGIN_L, 22), FONT, 0.40, age_col, 1)
    if status not in ('OK',):
        cv2.putText(img, f'STATUS: {status}',
                    (MARGIN_L, 42), FONT, 0.40, COL_WARN, 1)

    # ── Grid ──────────────────────────────────────────────────────────────────
    d = depth_min
    while d <= depth_max:
        x = depth_to_x(d)
        cv2.line(img, (x, MARGIN_T), (x, MARGIN_T+ph), COL_GRID, 1)
        lbl = f'{d*1000:.0f}'
        (tw,_),_ = cv2.getTextSize(lbl, FONT, 0.27, 1)
        cv2.putText(img, lbl, (x-tw//2, MARGIN_T+ph+14), FONT, 0.27, (65,65,65), 1)
        d = round(d+0.05, 6)

    # Optical axis (horizontal centre line)
    cv2.line(img, (MARGIN_L, scene_y), (MARGIN_L+pw, scene_y), (35,35,35), 1)

    # Axis labels
    cv2.putText(img, 'Depth Z (mm, camera frame)',
                (MARGIN_L+pw//2-90, H-10), FONT, 0.34, (75,75,75), 1)
    cv2.putText(img, 'X', (MARGIN_L-16, MARGIN_T+10), FONT, 0.30, (70,70,70), 1)
    cv2.putText(img, 'Camera', (2, scene_y+4), FONT, 0.32, COL_CAMERA, 1)

    half_span = canvas_h_m * 0.8   # half the line drawn on each side

    # ── ASSUMED canvas plane (green dashed angled line) ───────────────────────
    if assumed_pose is not None:
        acx, acy, acz, aqx, aqy, aqz, aqw = assumed_pose
        anx, anz, atilt = canvas_normal_xz(aqx, aqy, aqz, aqw)
        p0a, p1a = plane_line_endpoints(
            acx, acz, anx, anz, half_span,
            depth_to_x, scene_y, ph//2, scale_x)
        draw_dashed_line(img, p0a, p1a, COL_ASSUMED, dash=14, thick=2)
        # Label at top end
        lx = depth_to_x(acz)
        cv2.putText(img, f'ASSUMED  {acz*1000:.1f}mm  tilt:{atilt:.1f}°',
                    (lx+4, MARGIN_T+14), FONT, 0.34, COL_ASSUMED, 1)

    # ── REAL canvas plane (solid cyan angled line) ────────────────────────────
    if real_pose is not None:
        rcx, rcy, rcz, rqx, rqy, rqz, rqw = real_pose
        rnx, rnz, rtilt = canvas_normal_xz(rqx, rqy, rqz, rqw)

        # Canvas cross-section fill (grey filled polygon showing canvas body)
        thickness_m = 0.006   # 6mm physical thickness
        # Offset points along normal direction for canvas body
        nx_px = int(rnx * thickness_m * scale_x)
        nz_px = int(rnz * thickness_m * (pw / (depth_max-depth_min)))
        p0r, p1r = plane_line_endpoints(
            rcx, rcz, rnx, rnz, half_span,
            depth_to_x, scene_y, ph//2, scale_x)
        # Back face offset
        p0rb = (p0r[0]+nz_px, p0r[1]-nx_px)
        p1rb = (p1r[0]+nz_px, p1r[1]-nx_px)
        cv2.fillPoly(img,
                     [np.array([p0r, p1r, p1rb, p0rb], dtype=np.int32)],
                     COL_CANVAS)

        # Standoff zone fill (semi-transparent band in front of canvas)
        if standoff_m > 0:
            so_px_z = int(standoff_m * pw / (depth_max-depth_min))
            so_px_x = int(standoff_m * scale_x)
            # Offset toward camera (along -normal)
            p0s = (p0r[0]-nz_px*int(standoff_m/0.006),
                   p0r[1]+nx_px*int(standoff_m/0.006))
            p1s = (p1r[0]-nz_px*int(standoff_m/0.006),
                   p1r[1]+nx_px*int(standoff_m/0.006))
            overlay = img.copy()
            cv2.fillPoly(overlay,
                         [np.array([p0r, p1r, p1s, p0s], dtype=np.int32)],
                         COL_STANDOFF)
            cv2.addWeighted(overlay, 0.20, img, 0.80, 0, img)

        # Real canvas plane line (front face)
        cv2.line(img, p0r, p1r, COL_REAL, 3)

        # Brush Z constraint line: parallel to canvas plane, standoff in front
        # Offset the plane line along the normal direction by standoff_m
        so_z_px = int(standoff_m * pw / (depth_max-depth_min))
        so_x_px = int(standoff_m * scale_x)
        p0b_brush = (p0r[0] - int(rnz*standoff_m*pw/(depth_max-depth_min)),
                     p0r[1] + int(rnx*standoff_m*scale_x))
        p1b_brush = (p1r[0] - int(rnz*standoff_m*pw/(depth_max-depth_min)),
                     p1r[1] + int(rnx*standoff_m*scale_x))
        draw_dashdot_line(img, p0b_brush, p1b_brush, COL_BRUSH, thick=2)

        # Canvas centre dot
        cx_px = depth_to_x(rcz)
        cy_px = int(scene_y - rcx*scale_x)
        cv2.circle(img, (cx_px, cy_px), 7, COL_REAL, -1)
        cv2.circle(img, (cx_px, cy_px), 10, COL_REAL, 1)

        # Real label
        cv2.putText(img, f'REAL  {rcz*1000:.1f}mm  tilt:{rtilt:.1f}°',
                    (cx_px+4, MARGIN_T+ph-18), FONT, 0.34, COL_REAL, 1)

        # Brush centre dot on brush line
        bx_px = depth_to_x(rcz - standoff_m*abs(rnz))
        by_px = int(scene_y - rcx*scale_x + standoff_m*rnx*scale_x)
        cv2.circle(img, (bx_px, by_px), 5, COL_BRUSH, -1)
        cv2.putText(img, f'BRUSH  {(rcz-standoff_m)*1000:.1f}mm',
                    (bx_px+6, by_px-6), FONT, 0.32, COL_BRUSH, 1)

        # Depth error bracket (at optical axis height)
        if assumed_pose is not None and abs(depth_err_mm) > 0.5:
            xa = depth_to_x(assumed_pose[2])
            xr = depth_to_x(rcz)
            brk_y = scene_y - ph//3
            cv2.line(img, (min(xa,xr), brk_y), (max(xa,xr), brk_y), COL_ERROR, 2)
            for bx in (xa, xr):
                cv2.line(img, (bx, brk_y-6), (bx, brk_y+6), COL_ERROR, 2)
            err_col = COL_OK if abs(depth_err_mm) < 3.0 else COL_WARN
            cv2.putText(img, f'Z err: {depth_err_mm:+.2f}mm',
                        (min(xa,xr), brk_y-8), FONT, 0.34, err_col, 1)

    # ── Camera body ───────────────────────────────────────────────────────────
    cam_x = depth_to_x(0.0)
    cam_x = max(MARGIN_L+2, cam_x)
    cam_w, cam_hp = 18, 26
    cv2.rectangle(img, (cam_x-cam_w, scene_y-cam_hp),
                  (cam_x, scene_y+cam_hp), COL_CAMERA, 2)
    cv2.rectangle(img, (cam_x-cam_w//2-2, scene_y-10),
                  (cam_x-cam_w//2+2, scene_y+10), COL_CAMERA, -1)
    # Scan cone toward real canvas
    if real_pose is not None:
        cone_z = depth_to_x(real_pose[2])
        cone_h = ph//4
        pts = np.array([[cam_x, scene_y],
                        [cone_z, scene_y-cone_h//2],
                        [cone_z, scene_y+cone_h//2]], dtype=np.int32)
        overlay = img.copy()
        cv2.fillPoly(overlay, [pts], COL_CAMERA)
        cv2.addWeighted(overlay, 0.07, img, 0.93, 0, img)
        cv2.polylines(img, [pts], True, COL_CAMERA, 1)
        # Distance label
        mid_x = (cam_x + cone_z) // 2
        cv2.putText(img, f'{real_pose[2]*1000:.0f}mm',
                    (mid_x-18, scene_y+cam_hp+18), FONT, 0.32, (100,100,100), 1)
    cv2.putText(img, 'CAM',
                (cam_x-cam_w-2, scene_y+cam_hp+14), FONT, 0.28, COL_CAMERA, 1)

    # ── Data panel ────────────────────────────────────────────────────────────
    panel_y = MARGIN_T + ph + 26
    rd_z = real_pose[2] if real_pose else 0.0
    ad_z = assumed_pose[2] if assumed_pose else 0.0
    rt   = rtilt if real_pose else 0.0
    at   = atilt if assumed_pose else 0.0
    entries = [
        (f'Canvas depth (real)    : {rd_z*1000:.2f} mm  tilt: {rt:.2f}°', COL_REAL),
        (f'Canvas depth (assumed) : {ad_z*1000:.2f} mm  tilt: {at:.2f}°', COL_ASSUMED),
        (f'Standoff               : {standoff_m*1000:.2f} mm',            COL_STANDOFF),
        (f'Brush Z constraint     : {(rd_z-standoff_m)*1000:.2f} mm',     COL_BRUSH),
        (f'Depth error            : {depth_err_mm:+.3f} mm',
         COL_OK if abs(depth_err_mm) < 3.0 else COL_WARN),
    ]
    for i, (text, col) in enumerate(entries):
        cv2.putText(img, text, (MARGIN_L, panel_y+i*14), FONT, 0.31, col, 1)

    # Legend
    lx = W - 178
    legend = [
        (COL_REAL,     '─── Real canvas'),
        (COL_ASSUMED,  '- - Assumed canvas'),
        (COL_BRUSH,    '-.- Brush constraint'),
        (COL_STANDOFF, '▌   Standoff zone'),
        (COL_ERROR,    '[   Depth error'),
    ]
    for i, (col, lbl) in enumerate(legend):
        cv2.putText(img, lbl, (lx, panel_y+i*14), FONT, 0.27, col, 1)

    return img


# =============================================================================
# Node
# =============================================================================

class CanvasDepthVisualiser(Node):

    def __init__(self):
        super().__init__('canvas_depth_visualiser')

        self.declare_parameter('canvas_h_m',   0.210)
        self.declare_parameter('plot_fps',     15.0)

        self._bridge = CvBridge()
        self._lock   = threading.Lock()

        # Full pose tuples: (cx, cy, cz, qx, qy, qz, qw)
        self._real_pose     = None
        self._assumed_pose  = None
        self._brush_z_m     = None
        self._standoff_m    = 0.005
        self._depth_err_mm  = 0.0
        self._latency_ms    = 0.0
        self._pose_age_ms   = 0.0
        self._status        = 'WAITING'

        self._plot_pub = self.create_publisher(Image, '/canvas/depth_view', 5)

        self.create_subscription(PoseStamped, '/canvas/pose',
                                 self._real_cb,       10)
        self.create_subscription(PoseStamped, '/canvas/pose_assumed',
                                 self._assumed_cb,    10)
        self.create_subscription(PoseStamped, '/canvas/z_constraint',
                                 self._constraint_cb, 10)
        self.create_subscription(String, '/canvas/correction_data',
                                 self._data_cb,       10)

        fps = self.get_parameter('plot_fps').get_parameter_value().double_value
        self.create_timer(1.0/fps, self._publish_plot)

        self.get_logger().info(
            '\ncanvas_depth_visualiser ready.\n'
            '  Publishes: /canvas/depth_view\n'
            '  View: ros2 run rqt_image_view rqt_image_view  ->  /canvas/depth_view\n')

    def _pose_to_tuple(self, msg):
        p = msg.pose.position
        o = msg.pose.orientation
        return (p.x, p.y, p.z, o.x, o.y, o.z, o.w)

    def _real_cb(self, msg):
        with self._lock:
            self._real_pose = self._pose_to_tuple(msg)

    def _assumed_cb(self, msg):
        with self._lock:
            self._assumed_pose = self._pose_to_tuple(msg)

    def _constraint_cb(self, msg):
        with self._lock:
            self._brush_z_m = msg.pose.position.z

    def _data_cb(self, msg):
        try:
            d = json.loads(msg.data)
            with self._lock:
                self._status      = d.get('status', 'UNKNOWN')
                self._latency_ms  = d.get('latency_ms', 0.0)
                self._pose_age_ms = d.get('pose_age_ms', 0.0)
                if d.get('status') == 'OK':
                    self._depth_err_mm = d['error']['depth_mm']
                    self._standoff_m   = d.get('z_constraint', {}).get(
                        'standoff_m', 0.005)
        except Exception:
            pass

    def _publish_plot(self):
        with self._lock:
            rp   = self._real_pose
            ap   = self._assumed_pose
            bz   = self._brush_z_m
            so   = self._standoff_m
            derr = self._depth_err_mm
            lat  = self._latency_ms
            age  = self._pose_age_ms
            stat = self._status
            ch   = self.get_parameter('canvas_h_m').get_parameter_value().double_value

        bz = bz if bz is not None else ((rp[2]-so) if rp else 0.395)

        img = build_depth_view(rp, ap, bz, so, derr, lat, age, stat, ch)
        try:
            self._plot_pub.publish(self._bridge.cv2_to_imgmsg(img, 'bgr8'))
        except Exception:
            pass


# =============================================================================
# Entry point
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = CanvasDepthVisualiser()
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