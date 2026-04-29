#!/usr/bin/env python3
"""
canvas_correction_visualiser.py  --  SE3 error correction visualiser.

Shows the gap between assumed (fixed Unity reference) and real (live detector)
canvas origins, with error numbers and processing latency.

Publishes:
  /canvas/debug_corrected   Image   Camera view with assumed/real origins
  /canvas/correction_2d     Image   Canvas-frame 2D plot with error arrow + data panel

No stroke target -- purely shows pose estimation accuracy for validation.
"""

import json
import math
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


# ── Colours ───────────────────────────────────────────────────────────────────
FONT        = cv2.FONT_HERSHEY_SIMPLEX
COL_BG      = (20,  20,  20)
COL_GRID    = (45,  45,  45)
COL_CANVAS  = (180, 160,  50)   # gold canvas outline
COL_ASSUMED = (50,  200,  50)   # green  -- assumed (Unity reference)
COL_REAL    = (60,  100, 230)   # blue-orange -- real (detector)
COL_ERROR   = (30,  140, 255)   # orange arrow
COL_OK      = (60,  220,  80)
COL_WARN    = (0,   165, 255)
COL_TEXT    = (210, 210, 210)
PLOT_SIZE   = 520
MARGIN      = 50


# =============================================================================
# Geometry helpers
# =============================================================================

def quat_to_rotmat(qx, qy, qz, qw):
    return np.array([
        [1-2*(qy*qy+qz*qz),   2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
        [  2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz),   2*(qy*qz-qx*qw)],
        [  2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)],
    ])


def project_to_pixel(pt3d, fx, fy, cx, cy):
    if pt3d[2] <= 0.01:
        return None
    return (int(round(fx*pt3d[0]/pt3d[2]+cx)),
            int(round(fy*pt3d[1]/pt3d[2]+cy)))


def draw_arrow(img, p0, p1, col, thick=2):
    if p0 != p1:
        cv2.arrowedLine(img, p0, p1, col, thick, tipLength=0.25)


def draw_crosshair(img, pt, size, col, thick=2):
    cv2.line(img, (pt[0]-size, pt[1]), (pt[0]+size, pt[1]), col, thick)
    cv2.line(img, (pt[0], pt[1]-size), (pt[0], pt[1]+size), col, thick)


def to_px(cx_m, cy_m, scale, ox, oy):
    return (int(round(ox + cx_m*scale)), int(round(oy - cy_m*scale)))


def draw_dashed(img, p0, p1, col, n=14, thick=1):
    for k in range(0, n, 2):
        a = (int(p0[0]+k/n*(p1[0]-p0[0])),     int(p0[1]+k/n*(p1[1]-p0[1])))
        b = (int(p0[0]+(k+1)/n*(p1[0]-p0[0])), int(p0[1]+(k+1)/n*(p1[1]-p0[1])))
        cv2.line(img, a, b, col, thick)


# =============================================================================
# Camera debug overlay
# =============================================================================

def overlay_on_debug(img, real_pose, assumed_pose, t_err, rot_deg,
                     latency_ms, pose_age_ms, fx, fy, cx_cam, cy_cam,
                     status='OK', collecting_remaining_s=0.0, n_frames=0):
    """
    Overlays the ASSUMED canvas origin on the camera image as a fixed reference dot.
    The real canvas is already visible in the image itself -- no need to mark it.
    Arrow shows where the assumed origin is relative to what the camera sees.
    """
    out = img.copy()
    h, w = out.shape[:2]

    # ── COLLECTING countdown banner ────────────────────────────────────────────
    if status == 'COLLECTING':
        banner = f'COLLECTING REFERENCE  {collecting_remaining_s:.1f}s remaining  ({n_frames} frames)  KEEP CANVAS STILL'
        cv2.rectangle(out, (0, 0), (w, 32), (10, 10, 60), -1)
        cv2.putText(out, banner, (6, 22), FONT, 0.45, (100, 200, 255), 1)
        return out

    can_project = (assumed_pose is not None and
                   assumed_pose.header.frame_id == 'camera_color_optical_frame'
                   and fx > 0)

    if can_project:
        t_assumed = np.array([assumed_pose.pose.position.x,
                               assumed_pose.pose.position.y,
                               assumed_pose.pose.position.z])
        p_assumed = project_to_pixel(t_assumed, fx, fy, cx_cam, cy_cam)

        # ASSUMED origin: fixed green crosshair at canvas centre reference position.
        # This is where the canvas centre was at latch time.
        if p_assumed and 0<=p_assumed[0]<w and 0<=p_assumed[1]<h:
            cv2.circle(out, p_assumed, 18, COL_ASSUMED, 2)
            cv2.circle(out, p_assumed,  6, COL_ASSUMED, -1)
            draw_crosshair(out, p_assumed, 16, COL_ASSUMED, 1)
            cv2.putText(out, 'CANVAS CENTRE (ref)',
                        (p_assumed[0]+20, p_assumed[1]-8),
                        FONT, 0.42, COL_ASSUMED, 1)

            # Dashed error line: assumed → real (error direction only, no real dot)
            if t_err is not None and real_pose is not None:
                t_real = np.array([real_pose.pose.position.x,
                                    real_pose.pose.position.y,
                                    real_pose.pose.position.z])
                p_real = project_to_pixel(t_real, fx, fy, cx_cam, cy_cam)
                if (p_real and 0<=p_real[0]<w and 0<=p_real[1]<h
                        and p_assumed != p_real):
                    draw_dashed(out, p_assumed, p_real, COL_ERROR, 14, 2)
    else:
        if status == 'WAITING':
            cv2.putText(out, 'WAITING for canvas detection...',
                        (6, 22), FONT, 0.45, COL_WARN, 1)
        else:
            cv2.putText(out,
                        'Assumed pose not in camera frame -- see /canvas/correction_2d',
                        (6, 22), FONT, 0.38, COL_WARN, 1)

    # ── Error panel ────────────────────────────────────────────────────────────
    if t_err is not None and status == 'OK':
        lat_mm  = t_err[0]*1000.0
        vert_mm = t_err[1]*1000.0
        dep_mm  = t_err[2]*1000.0
        tot_mm  = np.linalg.norm(t_err)*1000.0
        ok_col  = COL_OK if tot_mm < 3.0 else COL_WARN
        lines = [
            ('POSE ERROR (real vs assumed)', COL_TEXT),
            (f'  lateral : {lat_mm:+.2f} mm',    COL_ERROR),
            (f'  vertical: {vert_mm:+.2f} mm',   COL_ERROR),
            (f'  depth   : {dep_mm:+.2f} mm',    COL_REAL),
            (f'  rotation: {rot_deg:.2f} deg',   COL_WARN),
            (f'  3D total: {tot_mm:.2f} mm',     ok_col),
            (f'  latency : {latency_ms:.1f} ms', COL_TEXT),
            (f'  pose age: {pose_age_ms:.0f} ms',
             COL_OK if pose_age_ms < 200 else COL_WARN),
        ]
    else:
        lines = [('POSE ERROR -- waiting for data', COL_WARN)]

    panel_y = h - len(lines)*17 - 8
    ovl = out.copy()
    cv2.rectangle(ovl, (0, panel_y-6), (285, h), (10,10,10), -1)
    cv2.addWeighted(ovl, 0.65, out, 0.35, 0, out)
    for i, (text, col) in enumerate(lines):
        cv2.putText(out, text, (6, panel_y+i*17), FONT, 0.38, col, 1)

    return out


# =============================================================================
# 2D correction plot
# =============================================================================

def build_2d_plot(t_err, rot_deg, latency_ms, pose_age_ms,
                  canvas_w, canvas_h, status,
                  collecting_remaining_s=0.0, collecting_elapsed_s=0.0,
                  latch_window_s=10.0, n_frames=0):
    """
    Canvas-frame top-down plot.
    ASSUMED origin is always at (0,0) -- it is the fixed reference.
    REAL origin is plotted at (t_err.x, t_err.y) -- shows drift in canvas frame.
    The arrow from ASSUMED to REAL is the active correction vector.
    """
    size = PLOT_SIZE
    img  = np.full((size, size, 3), COL_BG, dtype=np.uint8)

    pw = size - 2*MARGIN - 70   # right: 70px for depth bar
    ph = size - 2*MARGIN - 90   # bottom: 90px for data panel

    # Fixed view range: ±150mm around canvas centre.
    view_range_m = 0.150   # ±150mm
    scale = min(pw, ph) / (2.0 * view_range_m)
    ox    = MARGIN + pw//2
    oy    = MARGIN + ph//2

    def px(cx, cy):
        return to_px(cx, cy, scale, ox, oy)

    # ── COLLECTING: full-plot countdown screen ────────────────────────────────
    if status == 'COLLECTING':
        total    = max(latch_window_s, 0.001)
        elapsed  = min(collecting_elapsed_s, total)
        fraction = elapsed / total           # 0.0 → 1.0

        # Dark blue background tint
        overlay = img.copy()
        cv2.rectangle(overlay, (0,0), (size,size), (10,10,50), -1)
        cv2.addWeighted(overlay, 0.7, img, 0.3, 0, img)

        # Circular progress ring
        cx_ring, cy_ring = size//2, size//2 - 30
        r_outer, r_inner = 110, 80
        # Background ring (dark)
        cv2.circle(img, (cx_ring, cy_ring), r_outer, (40,40,80), r_outer-r_inner)
        # Progress arc (bright blue), drawn as filled sectors using ellipse
        angle_start = -90   # top of circle
        angle_end   = int(-90 + 360 * fraction)
        if angle_end != angle_start:
            cv2.ellipse(img,
                        (cx_ring, cy_ring),
                        (r_outer, r_outer),
                        0,
                        angle_start, angle_end,
                        (80, 200, 255),
                        r_outer - r_inner)
        # Inner circle fill (dark)
        cv2.circle(img, (cx_ring, cy_ring), r_inner-1, (15,15,40), -1)

        # Countdown number in centre
        secs_left = max(0.0, collecting_remaining_s)
        count_str = f'{secs_left:.1f}'
        font_scale = 1.8 if secs_left >= 10 else 2.2
        (tw, th), _ = cv2.getTextSize(count_str, FONT, font_scale, 3)
        cv2.putText(img, count_str,
                    (cx_ring - tw//2, cy_ring + th//2),
                    FONT, font_scale, (255,255,255), 3)

        # "seconds" label below number
        cv2.putText(img, 'seconds',
                    (cx_ring - 28, cy_ring + th//2 + 22),
                    FONT, 0.45, (150,180,220), 1)

        # Title above ring
        cv2.putText(img, 'INITIALISING REFERENCE POSE',
                    (size//2 - 155, cy_ring - r_outer - 18),
                    FONT, 0.52, (100,200,255), 1)

        # Instruction below ring
        cv2.putText(img, 'Keep canvas STILL',
                    (cx_ring - 88, cy_ring + r_outer + 22),
                    FONT, 0.52, (80,220,80), 1)

        # Frame counter
        cv2.putText(img, f'{n_frames} frames collected',
                    (cx_ring - 70, cy_ring + r_outer + 46),
                    FONT, 0.40, (100,140,180), 1)

        # Progress bar at bottom
        bar_x0  = MARGIN
        bar_y0  = size - 30
        bar_w   = size - 2*MARGIN
        bar_h_px = 12
        cv2.rectangle(img, (bar_x0, bar_y0),
                      (bar_x0+bar_w, bar_y0+bar_h_px), (40,40,80), -1)
        filled = int(bar_w * fraction)
        if filled > 0:
            cv2.rectangle(img, (bar_x0, bar_y0),
                          (bar_x0+filled, bar_y0+bar_h_px), (80,200,255), -1)
        cv2.rectangle(img, (bar_x0, bar_y0),
                      (bar_x0+bar_w, bar_y0+bar_h_px), (60,60,100), 1)
        cv2.putText(img, f'{fraction*100:.0f}%',
                    (bar_x0+bar_w+4, bar_y0+bar_h_px),
                    FONT, 0.35, (100,150,200), 1)
        return img

    # ── Status / title ────────────────────────────────────────────────────────
    age_col = COL_OK if pose_age_ms < 200 else COL_WARN
    cv2.putText(img,
                f'Canvas Correction  [age: {pose_age_ms:.0f}ms  '
                f'latency: {latency_ms:.1f}ms]',
                (MARGIN, 20), FONT, 0.38, age_col, 1)

    if status not in ('OK', 'COLLECTING'):
        cv2.putText(img, f'STATUS: {status}',
                    (MARGIN, 44), FONT, 0.45, COL_WARN, 1)

    # ── Grid (5cm) ────────────────────────────────────────────────────────────
    hw, hh = view_range_m, view_range_m   # grid covers full ±150mm view
    step   = 0.05
    gx = round(-hw, 6)
    while gx <= hw:
        cv2.line(img, px(gx,-hh), px(gx,hh), COL_GRID, 1)
        if abs(gx) > 0.001:
            lp = px(gx, -hh)
            cv2.putText(img, f'{gx*100:.0f}',
                        (lp[0]-8, lp[1]+12), FONT, 0.26, (65,65,65), 1)
        gx = round(gx+step, 6)
    gy = round(-hh, 6)
    while gy <= hh:
        cv2.line(img, px(-hw,gy), px(hw,gy), COL_GRID, 1)
        if abs(gy) > 0.001:
            lp = px(-hw, gy)
            cv2.putText(img, f'{gy*100:.0f}',
                        (lp[0]-28, lp[1]+4), FONT, 0.26, (65,65,65), 1)
        gy = round(gy+step, 6)

    cv2.putText(img, 'canvas X (cm)',
                (ox-40, MARGIN+ph+14), FONT, 0.30, (80,80,80), 1)
    cv2.putText(img, 'Y', (MARGIN-18, oy+4), FONT, 0.30, (80,80,80), 1)

    # ── Canvas outline ────────────────────────────────────────────────────────
    hw2, hh2 = canvas_w/2, canvas_h/2
    corners  = [(-hw2,hh2),(hw2,hh2),(hw2,-hh2),(-hw2,-hh2)]
    cpx      = [px(*c) for c in corners]
    for i in range(4):
        cv2.line(img, cpx[i], cpx[(i+1)%4], COL_CANVAS, 2)
    for (cx,cy), lbl in zip(corners, ['TL','TR','BR','BL']):
        p = px(cx,cy)
        cv2.putText(img, lbl,
                    (p[0]+(4 if cx>0 else -22),
                     p[1]+(14 if cy<0 else -6)),
                    FONT, 0.30, COL_CANVAS, 1)

    # ── Assumed origin -- always at (0,0) ─────────────────────────────────────
    p_assumed = px(0.0, 0.0)
    cv2.circle(img, p_assumed, 12, COL_ASSUMED, 2)
    cv2.circle(img, p_assumed,  5, COL_ASSUMED, -1)
    draw_crosshair(img, p_assumed, 10, COL_ASSUMED, 1)
    cv2.putText(img, 'ASSUMED (0,0)',
                (p_assumed[0]+14, p_assumed[1]-8),
                FONT, 0.34, COL_ASSUMED, 1)

    # ── Real origin -- at (t_err.x, t_err.y) in canvas frame ─────────────────
    # This dot moves when the canvas drifts.
    # Arrow from ASSUMED to REAL = correction vector.
    if t_err is not None and status == 'OK':
        rx = float(t_err[0])   # lateral error (metres)
        ry = float(t_err[1])   # vertical error (metres)
        p_real = px(rx, ry)

        # Error arrow
        if p_assumed != p_real:
            draw_arrow(img, p_assumed, p_real, COL_ERROR, 2)
            # Label midpoint with XY magnitude
            mid = ((p_assumed[0]+p_real[0])//2,
                   (p_assumed[1]+p_real[1])//2)
            xy_mm = math.hypot(rx, ry)*1000.0
            cv2.putText(img, f'{xy_mm:.2f}mm',
                        (mid[0]+5, mid[1]-5), FONT, 0.35, COL_ERROR, 1)
        else:
            # Zero error -- show pulsing ring
            cv2.circle(img, p_assumed, 18, COL_OK, 1)
            cv2.putText(img, '0.00mm', (p_assumed[0]+16, p_assumed[1]+18),
                        FONT, 0.34, COL_OK, 1)

        # Real origin dot
        cv2.circle(img, p_real, 12, COL_REAL, 2)
        cv2.circle(img, p_real,  5, COL_REAL, -1)
        cv2.putText(img,
                    f'REAL ({rx*100:.2f},{ry*100:.2f}) cm',
                    (p_real[0]+14, p_real[1]+10),
                    FONT, 0.34, COL_REAL, 1)

        # ── Depth error bar (right side) ──────────────────────────────────────
        bar_x   = MARGIN + pw + 14
        bar_top = MARGIN
        bar_bot = MARGIN + ph
        bar_mid = (bar_top+bar_bot)//2
        bar_h   = bar_bot - bar_top
        max_d   = 30.0   # ±30mm full scale
        cv2.rectangle(img, (bar_x,bar_top), (bar_x+16,bar_bot), (40,40,40), -1)
        cv2.line(img, (bar_x,bar_mid), (bar_x+16,bar_mid), (80,80,80), 1)
        dep_mm  = float(t_err[2])*1000.0
        frac    = max(-1.0, min(1.0, dep_mm/max_d))
        fill    = int(abs(frac)*bar_h/2)
        barcol  = COL_REAL if frac >= 0 else (100,80,220)
        if frac >= 0:
            cv2.rectangle(img, (bar_x+2,bar_mid-fill),
                          (bar_x+14,bar_mid), barcol, -1)
        else:
            cv2.rectangle(img, (bar_x+2,bar_mid),
                          (bar_x+14,bar_mid+fill), barcol, -1)
        cv2.putText(img, 'Z err', (bar_x-4,bar_top-6),
                    FONT, 0.28, COL_REAL, 1)
        cv2.putText(img, f'+{max_d:.0f}', (bar_x,bar_top+10),
                    FONT, 0.25, (70,70,70), 1)
        cv2.putText(img, '0', (bar_x+4,bar_mid-2),
                    FONT, 0.25, (90,90,90), 1)
        cv2.putText(img, f'-{max_d:.0f}', (bar_x,bar_bot),
                    FONT, 0.25, (70,70,70), 1)
        cv2.putText(img, f'{dep_mm:+.2f}mm',
                    (bar_x-6, bar_bot+14), FONT, 0.30, COL_REAL, 1)

    # ── Data panel (bottom) ───────────────────────────────────────────────────
    panel_y = MARGIN + ph + 24
    if t_err is not None and status == 'OK':
        lat_mm  = t_err[0]*1000.0
        vert_mm = t_err[1]*1000.0
        dep_mm  = t_err[2]*1000.0
        tot_mm  = np.linalg.norm(t_err)*1000.0
        ok_col  = COL_OK if tot_mm < 3.0 else COL_WARN
        entries = [
            (f'Lateral  (X): {lat_mm:+.3f} mm',   COL_ERROR),
            (f'Vertical (Y): {vert_mm:+.3f} mm',  COL_ERROR),
            (f'Depth    (Z): {dep_mm:+.3f} mm',   COL_REAL),
            (f'Rotation    : {rot_deg:.3f} deg',  COL_WARN),
            (f'3D total    : {tot_mm:.3f} mm',    ok_col),
            (f'Latency     : {latency_ms:.1f} ms  '
             f'Pose age: {pose_age_ms:.0f} ms',  COL_TEXT),
        ]
        for i, (text, col) in enumerate(entries):
            cv2.putText(img, text,
                        (MARGIN, panel_y+i*13), FONT, 0.32, col, 1)
    else:
        cv2.putText(img, f'Waiting for pose error data -- status: {status}',
                    (MARGIN, panel_y), FONT, 0.38, COL_WARN, 1)

    # ── Legend ────────────────────────────────────────────────────────────────
    lx = MARGIN + pw - 140
    legend = [
        (COL_ASSUMED, 'Assumed (Unity ref)'),
        (COL_REAL,    'Real (detector)'),
        (COL_ERROR,   'Error vector'),
    ]
    for i, (col, lbl) in enumerate(legend):
        ly = panel_y + i*13
        cv2.circle(img, (lx, ly-3), 4, col, -1)
        cv2.putText(img, lbl, (lx+8, ly), FONT, 0.28, col, 1)

    return img


# =============================================================================
# Node
# =============================================================================

class CanvasCorrectionVisualiser(Node):

    def __init__(self):
        super().__init__('canvas_correction_visualiser')

        self.declare_parameter('canvas_w_m',  0.297)
        self.declare_parameter('canvas_h_m',  0.210)
        self.declare_parameter('debug_fps',   10.0)
        self.declare_parameter('plot_fps',    10.0)

        self._bridge = CvBridge()
        self._lock   = threading.Lock()

        # State
        self._real_pose              = None
        self._real_pose_time         = None
        self._assumed_pose           = None
        self._t_err                  = None
        self._rot_deg                = 0.0
        self._latency_ms             = 0.0
        self._pose_age_ms            = 0.0
        self._status                 = 'WAITING'
        self._collecting_remaining_s = 0.0
        self._n_frames               = 0
        self._debug_img              = None
        self._fx = self._fy = self._cx = self._cy = 0.0

        # Publishers
        self._debug_pub = self.create_publisher(Image, '/canvas/debug_corrected', 5)
        self._plot_pub  = self.create_publisher(Image, '/canvas/correction_2d',   5)

        # Subscribers
        self.create_subscription(PoseStamped, '/canvas/pose',
                                 self._real_cb,    10)
        self.create_subscription(PoseStamped, '/canvas/pose_assumed',
                                 self._assumed_cb, 10)
        self.create_subscription(PoseStamped, '/canvas/pose_error',
                                 self._error_cb,   10)
        self.create_subscription(String, '/canvas/correction_data',
                                 self._data_cb,    10)
        self.create_subscription(Image, '/canvas/debug',
                                 self._debug_img_cb, 5)
        self.create_subscription(CameraInfo,
                                 '/camera/camera/color/camera_info',
                                 self._cam_info_cb, 1)

        d = self.get_parameter('debug_fps').get_parameter_value().double_value
        p = self.get_parameter('plot_fps').get_parameter_value().double_value
        self.create_timer(1.0/d, self._publish_debug)
        self.create_timer(1.0/p, self._publish_plot)

        self.get_logger().info(
            '\ncanvas_correction_visualiser ready.\n'
            '  /canvas/debug_corrected -- camera feed with assumed/real origins\n'
            '  /canvas/correction_2d   -- canvas-frame 2D drift plot + data\n')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _real_cb(self, msg):
        with self._lock:
            self._real_pose      = msg
            self._real_pose_time = time.monotonic()

    def _assumed_cb(self, msg):
        with self._lock:
            self._assumed_pose = msg

    def _error_cb(self, msg):
        with self._lock:
            p = msg.pose.position
            self._t_err = np.array([p.x, p.y, p.z])
            o   = msg.pose.orientation
            dot = abs(float(o.w))
            self._rot_deg = math.degrees(2.0*math.acos(min(1.0, dot)))

    def _data_cb(self, msg):
        try:
            d = json.loads(msg.data)
            with self._lock:
                self._status      = d.get('status', 'UNKNOWN')
                self._latency_ms  = d.get('latency_ms', 0.0)
                self._pose_age_ms = d.get('pose_age_ms', 0.0)
                self._collecting_remaining_s = d.get('remaining_s', 0.0)
                self._n_frames    = d.get('n_frames', 0)
        except Exception:
            pass

    def _debug_img_cb(self, msg):
        try:
            img = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
            with self._lock:
                self._debug_img = img
        except Exception:
            pass

    def _cam_info_cb(self, msg):
        with self._lock:
            self._fx = msg.k[0]; self._fy = msg.k[4]
            self._cx = msg.k[2]; self._cy = msg.k[5]

    # ── Publish callbacks ─────────────────────────────────────────────────────

    def _publish_debug(self):
        with self._lock:
            img           = self._debug_img.copy() if self._debug_img is not None else None
            real_pose     = self._real_pose
            assumed_pose  = self._assumed_pose
            t_err         = self._t_err.copy() if self._t_err is not None else None
            rot_deg       = self._rot_deg
            latency_ms    = self._latency_ms
            pose_age_ms   = self._pose_age_ms
            status        = self._status
            remaining_s   = self._collecting_remaining_s
            n_frames      = self._n_frames
            fx,fy,cx,cy   = self._fx,self._fy,self._cx,self._cy

        # Allow overlay even without debug img if collecting (show countdown on blank frame)
        if img is None and status != 'COLLECTING':
            return
        if img is None:
            img = np.full((480, 640, 3), COL_BG, dtype=np.uint8)

        result = overlay_on_debug(
            img, real_pose, assumed_pose,
            t_err, rot_deg, latency_ms, pose_age_ms,
            fx, fy, cx, cy,
            status=status,
            collecting_remaining_s=remaining_s,
            n_frames=n_frames,
        )
        try:
            self._debug_pub.publish(self._bridge.cv2_to_imgmsg(result, 'bgr8'))
        except Exception:
            pass

    def _publish_plot(self):
        with self._lock:
            t_err        = self._t_err.copy() if self._t_err is not None else None
            rot_deg      = self._rot_deg
            latency_ms   = self._latency_ms
            pose_age_ms  = self._pose_age_ms
            status       = self._status
            remaining_s  = self._collecting_remaining_s
            n_frames     = self._n_frames
            canvas_w     = self.get_parameter('canvas_w_m').get_parameter_value().double_value
            canvas_h     = self.get_parameter('canvas_h_m').get_parameter_value().double_value

        latch_window_s = 10.0   # must match corrector param
        elapsed_s = max(0.0, latch_window_s - remaining_s)

        plot = build_2d_plot(
            t_err, rot_deg, latency_ms, pose_age_ms,
            canvas_w, canvas_h, status,
            collecting_remaining_s=remaining_s,
            collecting_elapsed_s=elapsed_s,
            latch_window_s=latch_window_s,
            n_frames=n_frames,
        )
        try:
            self._plot_pub.publish(self._bridge.cv2_to_imgmsg(plot, 'bgr8'))
        except Exception:
            pass


# =============================================================================
# Entry point
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = CanvasCorrectionVisualiser()
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