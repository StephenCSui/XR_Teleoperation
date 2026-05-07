#!/usr/bin/env python3
"""
canvas_pose_visualiser.py  --  2D/3D pose visualiser  (v5)

Publishes THREE image topics:

  /canvas/pose_2d_xy   --  X vs Y  (lateral vs vertical, camera frame)
  /canvas/pose_2d_xz   --  X vs Z  (lateral vs depth, top-down bird's eye)
  /canvas/pose_3d_tf   --  3D projected view of camera + canvas TF frames
                           Shows XYZ axes of both frames, line between them,
                           and RPY arc indicators for each rotation axis.

Camera optical frame: X=right, Y=down, Z=forward.
RPY: ZYX extrinsic (ROS convention), degrees.
Roll baseline ~130-135 deg = canvas flat-on facing camera (expected normal).

View with:
  ros2 run rqt_image_view rqt_image_view  -> select topic from dropdown
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from collections import deque


# =============================================================================
# RPY conversion
# =============================================================================

def quaternion_to_rpy(x, y, z, w):
    """ZYX extrinsic RPY (ROS convention), returns radians."""
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    if abs(sinp) < 0.9999:
        roll  = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x*x + y*y))
        yaw   = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y*y + z*z))
    else:
        roll = 0.0
        yaw  = math.atan2(-2.0 * (x*z - w*y), 1.0 - 2.0 * (x*x + z*z))
    return roll, pitch, yaw


# Quaternion helpers removed -- H/V/Roll computed in C++ node and received via /canvas/viewing_angles


def quat_to_matrix(qx, qy, qz, qw):
    """Quaternion -> 3x3 rotation matrix (numpy)."""
    return np.array([
        [1-2*(qy*qy+qz*qz),   2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
        [  2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz),   2*(qy*qz-qx*qw)],
        [  2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)],
    ])


# =============================================================================
# Shared drawing constants / helpers
# =============================================================================

MARGIN_TOP    = 28
MARGIN_BOTTOM = 44
MARGIN_LEFT   = 36
MARGIN_RIGHT  = 10
FONT          = cv2.FONT_HERSHEY_SIMPLEX

# Axis colours: X=red, Y=green, Z=blue  (matches RViz convention)
COL_X = (60,  60, 220)
COL_Y = (60, 200,  60)
COL_Z = (220, 80,  60)
COL_W = (200, 200, 200)   # white/grey for labels


def make_canvas(size: int) -> np.ndarray:
    img = np.zeros((size, size, 3), dtype=np.uint8)
    img[:] = (20, 20, 20)
    return img


def plot_area(size: int):
    x0 = MARGIN_LEFT
    y0 = MARGIN_TOP
    w  = size - MARGIN_LEFT - MARGIN_RIGHT
    h  = size - MARGIN_TOP  - MARGIN_BOTTOM
    return x0, y0, w, h


def draw_rpy_panel(img, roll, pitch, yaw):
    size    = img.shape[0]
    panel_w = 200
    panel_h = 80
    panel_x = size - panel_w - 6
    panel_y = size - panel_h - 6
    overlay = img.copy()
    cv2.rectangle(overlay,
                  (panel_x-4, panel_y-4),
                  (panel_x+panel_w+4, panel_y+panel_h+4),
                  (30,30,30), -1)
    cv2.addWeighted(overlay, 0.75, img, 0.25, 0, img)
    cv2.putText(img, 'Roll | H (left+) | V (up+)  deg',
                (panel_x, panel_y-6), FONT, 0.30, (180,180,180), 1)
    bars     = [('Roll', roll, COL_X), ('H', pitch, COL_Y), ('V', yaw, COL_Z)]
    bar_h    = 18
    bar_gap  = 26
    bar_maxw = panel_w - 44
    for i, (lbl, val, col) in enumerate(bars):
        by = panel_y + i * bar_gap
        cv2.rectangle(img, (panel_x+14, by),
                      (panel_x+14+bar_maxw, by+bar_h), (60,60,60), -1)
        mid_x   = panel_x + 14 + bar_maxw // 2
        fill_px = int(val / 180.0 * (bar_maxw / 2))
        fill_px = max(-bar_maxw//2, min(bar_maxw//2, fill_px))
        x1, x2  = (mid_x, mid_x+fill_px) if fill_px>=0 else (mid_x+fill_px, mid_x)
        cv2.rectangle(img, (x1, by+2), (x2, by+bar_h-2), col, -1)
        cv2.line(img, (mid_x, by), (mid_x, by+bar_h), (150,150,150), 1)
        cv2.putText(img, lbl,
                    (panel_x, by+bar_h-4), FONT, 0.38, col, 1)
        cv2.putText(img, f'{val:+.1f}',
                    (panel_x+14+bar_maxw+3, by+bar_h-4), FONT, 0.32, col, 1)


def draw_legend_2d(img):
    x0, y0, w, h = plot_area(img.shape[0])
    leg_y = y0 + h + 30
    cv2.circle(img, (x0, leg_y),      6, (220, 80,  0), -1)
    cv2.putText(img, 'Camera', (x0+10, leg_y+4), FONT, 0.30, (180,180,180), 1)
    cv2.circle(img, (x0+75, leg_y),   6, (0, 200, 60), -1)
    cv2.putText(img, 'Canvas', (x0+85, leg_y+4), FONT, 0.30, (180,180,180), 1)
    cv2.line(img, (x0+150, leg_y), (x0+162, leg_y), (0,0,200), 1)
    cv2.putText(img, 'Offset', (x0+166, leg_y+4), FONT, 0.30, (180,180,180), 1)


def draw_canvas_labels(img, cvs_cl, cx, cy, cz, roll, pitch, yaw):
    size    = img.shape[0]
    label_x = min(cvs_cl[0]+13, size-115)
    label_y = cvs_cl[1]
    cv2.putText(img, 'Canvas',         (label_x, label_y-8),  FONT, 0.40, (80,230,100), 1)
    cv2.putText(img, f'x={cx:+.3f}m', (label_x, label_y+8),  FONT, 0.35, (80,230,100), 1)
    cv2.putText(img, f'y={cy:+.3f}m', (label_x, label_y+22), FONT, 0.35, (80,230,100), 1)
    cv2.putText(img, f'z= {cz:.3f}m', (label_x, label_y+36), FONT, 0.35, (80,230,100), 1)
    cv2.putText(img, f'Rol={roll:+.1f}d',  (label_x, label_y+52), FONT, 0.33, (120,220,255), 1)
    cv2.putText(img, f'H  ={pitch:+.1f}d', (label_x, label_y+65), FONT, 0.33, (120,220,255), 1)
    cv2.putText(img, f'V  ={yaw:+.1f}d',   (label_x, label_y+78), FONT, 0.33, (120,220,255), 1)


# =============================================================================
# XY plot
# =============================================================================

def build_xy_plot(size, xy_range, cx, cy, cz, roll, pitch, yaw, trail_xy):
    img = make_canvas(size)
    x0, y0, w, h = plot_area(size)
    cam_cx = x0 + w // 2
    cam_cy = y0 + h // 2
    sc     = (w / 2) / xy_range

    def w2p(wx, wy):
        return (int(cam_cx + wx*sc), int(cam_cy - wy*sc))

    n = int(xy_range / 0.1)
    for i in range(-n, n+1):
        px  = int(cam_cx + i*0.1*sc)
        py  = int(cam_cy - i*0.1*sc)
        col = (70,70,70) if i%2==0 else (50,50,50)
        cv2.line(img, (px,y0), (px,y0+h), col, 1)
        cv2.line(img, (x0,py), (x0+w,py), col, 1)
        if i%2==0 and i!=0:
            cv2.putText(img, f'{i*0.1:.1f}', (px-10, y0+h+14), FONT, 0.28, (110,110,110), 1)
            cv2.putText(img, f'{i*0.1:.1f}', (2, py+4),         FONT, 0.28, (110,110,110), 1)
    cv2.line(img, (cam_cx-8,cam_cy), (cam_cx+8,cam_cy), (90,90,90), 1)
    cv2.line(img, (cam_cx,cam_cy-8), (cam_cx,cam_cy+8), (90,90,90), 1)
    cv2.putText(img, 'X lateral (m)',  (x0+w-85, y0+h+26), FONT, 0.35, (180,180,180), 1)
    cv2.putText(img, 'Y vertical (m)', (x0+2, y0+12),       FONT, 0.35, (180,180,180), 1)

    for i, (tx, ty) in enumerate(trail_xy):
        alpha = int(60 + 160*(i/max(len(trail_xy)-1, 1)))
        pp = w2p(tx, ty)
        if 0<=pp[0]<size and 0<=pp[1]<size:
            cv2.circle(img, pp, 2, (alpha,alpha,0), -1)

    cam_px = (cam_cx, cam_cy)
    cv2.circle(img, cam_px, 10, (220,80,0), -1)
    cv2.circle(img, cam_px, 10, (255,150,50), 1)
    cv2.putText(img, 'Camera', (cam_px[0]+13, cam_px[1]+5), FONT, 0.38, (220,130,80), 1)

    cvs_px = w2p(cx, -cy)
    cvs_cl = (max(4,min(size-4,cvs_px[0])), max(4,min(size-4,cvs_px[1])))
    cv2.line(img, cam_px, cvs_cl, (0,0,200), 1)
    mid = ((cam_px[0]+cvs_cl[0])//2, (cam_px[1]+cvs_cl[1])//2)
    cv2.putText(img, f'd_xy={(cx**2+cy**2)**0.5:.3f}m', (mid[0]+5,mid[1]-5), FONT, 0.35, (80,80,220), 1)
    if 0<=cvs_px[0]<size and 0<=cvs_px[1]<size:
        cv2.circle(img, cvs_px, 10, (0,200,60), -1)
        cv2.circle(img, cvs_px, 10, (80,255,120), 1)
    draw_canvas_labels(img, cvs_cl, cx, cy, cz, roll, pitch, yaw)
    draw_rpy_panel(img, roll, pitch, yaw)
    draw_legend_2d(img)
    cv2.putText(img, 'Canvas X-Y View (lateral vs vertical, camera frame)',
                (8,18), FONT, 0.38, (200,200,200), 1)
    return img


# =============================================================================
# XZ plot
# =============================================================================

def build_xz_plot(size, x_range, z_range, cx, cy, cz, roll, pitch, yaw, trail_xz):
    img = make_canvas(size)
    x0, y0, w, h = plot_area(size)
    cam_cx  = x0 + w // 2
    cam_py  = y0 + h
    x_scale = w / (2.0 * x_range)
    z_scale = h / z_range

    def w2p(wx, wz):
        return (int(cam_cx + wx*x_scale), int(cam_py - wz*z_scale))

    n_x = int(x_range / 0.1)
    for i in range(-n_x, n_x+1):
        px  = int(cam_cx + i*0.1*x_scale)
        col = (70,70,70) if i%2==0 else (50,50,50)
        cv2.line(img, (px,y0), (px,y0+h), col, 1)
        if i%2==0 and i!=0:
            cv2.putText(img, f'{i*0.1:.1f}', (px-10,y0+h+14), FONT, 0.28, (110,110,110), 1)
    n_z = int(z_range / 0.1)
    for i in range(0, n_z+1):
        py  = int(cam_py - i*0.1*z_scale)
        col = (70,70,70) if i%5==0 else (50,50,50)
        cv2.line(img, (x0,py), (x0+w,py), col, 1)
        if i%5==0:
            cv2.putText(img, f'{i*0.1:.1f}', (2,py+4), FONT, 0.28, (110,110,110), 1)
    cv2.line(img, (cam_cx,y0), (cam_cx,y0+h), (80,80,80), 1)
    cv2.putText(img, 'X lateral (m)', (x0+w-85,y0+h+26), FONT, 0.35, (180,180,180), 1)
    cv2.putText(img, 'Z depth (m)',   (x0+2, y0+12),      FONT, 0.35, (180,180,180), 1)

    for i, (tx, tz) in enumerate(trail_xz):
        alpha = int(60 + 160*(i/max(len(trail_xz)-1, 1)))
        pp = w2p(tx, tz)
        if 0<=pp[0]<size and 0<=pp[1]<size:
            cv2.circle(img, pp, 2, (alpha,alpha,0), -1)

    cam_px = (cam_cx, cam_py)
    cv2.circle(img, cam_px, 10, (220,80,0), -1)
    cv2.circle(img, cam_px, 10, (255,150,50), 1)
    cv2.putText(img, 'Camera (origin)', (cam_px[0]+13,cam_px[1]-4), FONT, 0.38, (220,130,80), 1)

    cvs_px = w2p(cx, cz)
    cvs_cl = (max(4,min(size-4,cvs_px[0])), max(4,min(size-4,cvs_px[1])))
    cv2.line(img, cam_px, cvs_cl, (0,0,200), 1)
    dist_3d = (cx**2+cy**2+cz**2)**0.5
    mid = ((cam_px[0]+cvs_cl[0])//2, (cam_px[1]+cvs_cl[1])//2)
    cv2.putText(img, f'd={dist_3d:.3f}m', (mid[0]+5,mid[1]-5), FONT, 0.35, (80,80,220), 1)
    if 0<=cvs_px[0]<size and 0<=cvs_px[1]<size:
        cv2.circle(img, cvs_px, 10, (0,200,60), -1)
        cv2.circle(img, cvs_px, 10, (80,255,120), 1)
    draw_canvas_labels(img, cvs_cl, cx, cy, cz, roll, pitch, yaw)
    draw_rpy_panel(img, roll, pitch, yaw)
    draw_legend_2d(img)
    cv2.putText(img, 'Canvas X-Z Top-Down View (lateral vs depth, camera frame)',
                (8,18), FONT, 0.36, (200,200,200), 1)
    return img


# =============================================================================
# 3D TF frame view
# =============================================================================

def project(pts_3d, R_view, scale, ox, oy):
    """
    Orthographic projection of 3D points using view rotation matrix.
    Returns list of (px, py, depth) tuples.
    Camera optical frame (X=right, Y=down, Z=forward) is remapped to a
    screen-friendly display frame before projection:
      display_X =  world_X   (right)
      display_Y = -world_Y   (flip: world Y=down -> display Y=up)
      display_Z = -world_Z   (flip: world Z=forward -> display Z=into screen)
    """
    result = []
    for p in pts_3d:
        # Remap to display convention
        pd = np.array([p[0], -p[1], -p[2]])
        pr = R_view @ pd
        px = int(ox + pr[0] * scale)
        py = int(oy - pr[1] * scale)   # screen Y is flipped
        result.append((px, py, pr[2]))
    return result


def make_view_matrix(az_deg, el_deg):
    """
    Build view rotation from azimuth (around world-up Y) and
    elevation (tilt down toward scene).
    """
    az = math.radians(az_deg)
    el = math.radians(el_deg)
    Ry = np.array([[ math.cos(az), 0, math.sin(az)],
                   [0,             1, 0            ],
                   [-math.sin(az), 0, math.cos(az)]])
    Rx = np.array([[1, 0,            0           ],
                   [0, math.cos(el), -math.sin(el)],
                   [0, math.sin(el),  math.cos(el)]])
    return Rx @ Ry


def draw_axis_arrow(img, R_view, origin_3d, axis_3d, colour,
                    scale, ox, oy, label, label_col=None):
    """Draw a single 3D axis arrow from origin_3d in direction axis_3d."""
    tip_3d = [origin_3d[i] + axis_3d[i] for i in range(3)]
    pts    = project([origin_3d, tip_3d], R_view, scale, ox, oy)
    p0     = pts[0][:2]
    p1     = pts[1][:2]

    size = img.shape[0]
    in0  = 0<=p0[0]<size and 0<=p0[1]<size
    in1  = 0<=p1[0]<size and 0<=p1[1]<size
    if not (in0 or in1):
        return

    cv2.arrowedLine(img, p0, p1, colour, 2, tipLength=0.2)
    if label and in1:
        lc = label_col if label_col else colour
        cv2.putText(img, label, (p1[0]+4, p1[1]+4), FONT, 0.38, lc, 1)


def draw_frame(img, R_view, origin_3d, R_frame,
               axis_len, scale, ox, oy, label,
               origin_colour, dim=False):
    """
    Draw XYZ axes of a coordinate frame at origin_3d with orientation R_frame.
    R_frame is a 3x3 numpy rotation matrix (columns are X, Y, Z axes).
    """
    alpha = 0.5 if dim else 1.0

    def col(c):
        if dim:
            return tuple(int(v * 0.55) for v in c)
        return c

    for i, (ax_col, ax_lbl) in enumerate(
            [(COL_X, 'X'), (COL_Y, 'Y'), (COL_Z, 'Z')]):
        axis_world = R_frame[:, i] * axis_len
        draw_axis_arrow(img, R_view,
                        origin_3d, axis_world.tolist(),
                        col(ax_col), scale, ox, oy, ax_lbl)

    # Origin dot
    pts = project([origin_3d], R_view, scale, ox, oy)
    size = img.shape[0]
    if 0<=pts[0][0]<size and 0<=pts[0][1]<size:
        cv2.circle(img, pts[0][:2], 6, origin_colour, -1)
        cv2.putText(img, label, (pts[0][0]+8, pts[0][1]-6),
                    FONT, 0.40, COL_W, 1)


def draw_rpy_arcs(img, R_view, origin_3d, R_canvas,
                  roll, pitch, yaw, scale, ox, oy):
    """
    Draw three small arcs at the canvas frame origin showing the RPY angles.
    Each arc is drawn in the plane of its rotation axis.
    Arc radius scales with the axis_len used for the frame.
    """
    arc_r  = 0.06   # arc radius in metres
    n_seg  = 24     # segments per arc

    # For each RPY axis, draw an arc in its rotation plane
    # Roll  = rotation around canvas X axis -> arc in canvas Y-Z plane
    # Pitch = rotation around canvas Y axis -> arc in canvas X-Z plane
    # Yaw   = rotation around canvas Z axis -> arc in canvas X-Y plane
    arcs = [
        (roll,  R_canvas[:, 0], R_canvas[:, 1], R_canvas[:, 2], COL_X, 'R'),
        (pitch, R_canvas[:, 1], R_canvas[:, 0], R_canvas[:, 2], COL_Y, 'P'),
        (yaw,   R_canvas[:, 2], R_canvas[:, 0], R_canvas[:, 1], COL_Z, 'Y'),
    ]

    for angle_deg, _rot_ax, u_ax, v_ax, col, lbl in arcs:
        angle_rad = math.radians(angle_deg)
        # Draw arc from 0 to angle_rad in the u-v plane
        prev_pt = None
        for k in range(n_seg + 1):
            t   = angle_rad * k / n_seg
            pt3 = [
                origin_3d[i] + arc_r * (math.cos(t) * u_ax[i] + math.sin(t) * v_ax[i])
                for i in range(3)
            ]
            proj = project([pt3], R_view, scale, ox, oy)[0]
            curr = proj[:2]
            size = img.shape[0]
            if prev_pt is not None:
                if (0<=prev_pt[0]<size and 0<=prev_pt[1]<size and
                        0<=curr[0]<size and 0<=curr[1]<size):
                    cv2.line(img, prev_pt, curr, col, 1)
            prev_pt = curr

        # Label at arc tip
        if prev_pt is not None and 0<=prev_pt[0]<img.shape[0] and 0<=prev_pt[1]<img.shape[0]:
            cv2.putText(img, f'{lbl}={angle_deg:+.1f}d',
                        (prev_pt[0]+4, prev_pt[1]), FONT, 0.30, col, 1)


def draw_depth_line(img, R_view, cam_origin, cvs_origin, scale, ox, oy):
    """Dashed line between camera and canvas origins."""
    n_dash = 12
    pts    = []
    for k in range(n_dash + 1):
        t   = k / n_dash
        pt3 = [cam_origin[i] + t*(cvs_origin[i]-cam_origin[i]) for i in range(3)]
        pts.append(project([pt3], R_view, scale, ox, oy)[0][:2])

    size = img.shape[0]
    for k in range(0, n_dash, 2):
        p0, p1 = pts[k], pts[k+1]
        if (0<=p0[0]<size and 0<=p0[1]<size and
                0<=p1[0]<size and 0<=p1[1]<size):
            cv2.line(img, p0, p1, (120,120,120), 1)


def build_3d_tf_plot(size, cx, cy, cz,
                     qx, qy, qz, qw,
                     roll, pitch, yaw):
    """
    3D orthographic view of camera frame and canvas frame.

    Camera frame: identity rotation at origin (0,0,0).
    Canvas frame: quaternion rotation at position (cx, cy, cz).

    View angle: azimuth=-30deg (looking from slight right),
                elevation=20deg (looking slightly from above).
    This gives a clear view of both Z (depth) and the canvas orientation.
    """
    img  = make_canvas(size)

    # View matrix
    R_view = make_view_matrix(az_deg=-30, el_deg=20)

    # Scale and centre: the canvas is at depth cz, so we need enough
    # scale to see both origin and canvas comfortably.
    # Use 60% of half-image-size per metre, capped so canvas fits.
    max_dim  = max(abs(cx), abs(cy), abs(cz), 0.3)
    scale    = min(int((size * 0.38) / max_dim), 300)
    ox       = size // 2
    oy       = size // 2 + size // 8   # shift centre down slightly so Z has room

    axis_len = 0.10   # 10cm axis arrows

    # Canvas rotation matrix from quaternion
    R_canvas = quat_to_matrix(qx, qy, qz, qw)

    # Camera frame: identity rotation at origin
    R_cam = np.eye(3)
    cam_origin = [0.0, 0.0, 0.0]
    cvs_origin = [cx, cy, cz]

    # Depth line (dashed) between frames
    draw_depth_line(img, R_view, cam_origin, cvs_origin, scale, ox, oy)

    # Distance label at midpoint
    mid3  = [cam_origin[i]*0.5 + cvs_origin[i]*0.5 for i in range(3)]
    mid2  = project([mid3], R_view, scale, ox, oy)[0][:2]
    dist  = (cx**2 + cy**2 + cz**2)**0.5
    if 0<=mid2[0]<size and 0<=mid2[1]<size:
        cv2.putText(img, f'{dist:.3f}m', (mid2[0]+5, mid2[1]-5),
                    FONT, 0.35, (140,140,140), 1)

    # RPY arcs at canvas origin (draw before frames so frames are on top)
    draw_rpy_arcs(img, R_view, cvs_origin, R_canvas,
                  roll, pitch, yaw, scale, ox, oy)

    # Camera frame (dimmed -- reference, always identity)
    draw_frame(img, R_view, cam_origin, R_cam,
               axis_len, scale, ox, oy,
               label='Camera', origin_colour=(220,80,0), dim=True)

    # Canvas frame (bright -- this is what changes)
    draw_frame(img, R_view, cvs_origin, R_canvas,
               axis_len, scale, ox, oy,
               label='Canvas', origin_colour=(0,200,60), dim=False)

    # ---- Info panel (top-left) ----
    pad = 10
    cv2.putText(img, f'XYZ:  x={cx:+.3f}  y={cy:+.3f}  z={cz:.3f} m',
                (pad, 44), FONT, 0.36, (80,230,100), 1)
    cv2.putText(img, f'Roll (in-plane): {roll:+.1f} deg',
                (pad, 62), FONT, 0.36, COL_X, 1)
    cv2.putText(img, f'H (horizontal, neg=cam right): {pitch:+.1f} deg',
                (pad, 80), FONT, 0.36, COL_Y, 1)
    cv2.putText(img, f'V (vertical,   pos=cam above): {yaw:+.1f} deg',
                (pad, 98), FONT, 0.36, COL_Z, 1)

    # ---- Axis legend (bottom-left) ----
    x0, y0, w, h = plot_area(size)
    leg_y = y0 + h + 10
    for i, (lbl, col) in enumerate([('X axis', COL_X),
                                     ('Y axis', COL_Y),
                                     ('Z axis', COL_Z)]):
        cv2.putText(img, lbl, (x0 + i*90, leg_y+18), FONT, 0.32, col, 1)

    cv2.putText(img, 'dim = camera frame (identity)',
                (x0, leg_y+32), FONT, 0.28, (130,130,130), 1)
    cv2.putText(img, 'bright = canvas frame  |  arcs = RPY angles',
                (x0, leg_y+44), FONT, 0.28, (130,130,130), 1)

    # ---- Title ----
    cv2.putText(img, 'Canvas TF Frame vs Camera Frame  (3D projected)',
                (8, 18), FONT, 0.40, (200,200,200), 1)

    return img


# =============================================================================
# Node
# =============================================================================

class PoseVisualiser(Node):
    def __init__(self):
        super().__init__('canvas_pose_visualiser')

        self.declare_parameter('image_size',    700)
        self.declare_parameter('xy_range_m',    0.5)
        self.declare_parameter('x_range_m',     0.6)
        self.declare_parameter('z_range_m',     2.0)
        self.declare_parameter('trail_length',   80)
        self.declare_parameter('publish_fps',   15.0)


        self.size_      = self.get_parameter('image_size').value
        self.xy_range_  = self.get_parameter('xy_range_m').value
        self.x_range_   = self.get_parameter('x_range_m').value
        self.z_range_   = self.get_parameter('z_range_m').value
        self.trail_len_ = self.get_parameter('trail_length').value
        self.fps_       = self.get_parameter('publish_fps').value


        self.bridge_ = CvBridge()
        self.trail_xy_ = deque(maxlen=self.trail_len_)
        self.trail_xz_ = deque(maxlen=self.trail_len_)

        # Pose state
        self.cx_ = self.cy_ = self.cz_   = None
        self.qx_ = self.qy_ = self.qz_   = 0.0
        self.qw_ = 1.0
        self.roll_ = self.pitch_ = self.yaw_ = None
        self.total_angle_ = 0.0

        self.sub_ = self.create_subscription(
            PoseStamped, '/canvas/pose', self.pose_cb, 10)
        self.ang_sub_ = self.create_subscription(
            Vector3Stamped, '/canvas/viewing_angles', self.angles_cb, 10)

        self.pub_xy_ = self.create_publisher(Image, '/canvas/pose_2d_xy', 5)
        self.pub_xz_ = self.create_publisher(Image, '/canvas/pose_2d_xz', 5)
        self.pub_3d_ = self.create_publisher(Image, '/canvas/pose_3d_tf',  5)

        self.timer_ = self.create_timer(1.0 / self.fps_, self.publish_plots)

        self.get_logger().info(
            'Canvas pose visualiser v5 ready.\n'
            '  /canvas/pose_2d_xy  -- X vs Y (lateral vs vertical)\n'
            '  /canvas/pose_2d_xz  -- X vs Z (lateral vs depth, top-down)\n'
            '  /canvas/pose_3d_tf  -- 3D projected TF frames + RPY arcs\n'
            '  View: ros2 run rqt_image_view rqt_image_view')

    def pose_cb(self, msg: PoseStamped):
        self.cx_ = msg.pose.position.x
        self.cy_ = msg.pose.position.y
        self.cz_ = msg.pose.position.z
        self.qx_ = msg.pose.orientation.x
        self.qy_ = msg.pose.orientation.y
        self.qz_ = msg.pose.orientation.z
        self.qw_ = msg.pose.orientation.w
        self.trail_xy_.append((self.cx_, self.cy_))
        self.trail_xz_.append((self.cx_, self.cz_))

    def angles_cb(self, msg: Vector3Stamped):
        """
        Receives precomputed viewing angles from the C++ detector node.
        x = Roll (in-plane canvas rotation)
        y = H    (horizontal viewing angle, neg = camera to right)
        z = V    (vertical viewing angle, pos = camera above)
        """
        self.roll_        = msg.vector.x
        self.pitch_       = msg.vector.y   # H
        self.yaw_         = msg.vector.z   # V

    def publish_plots(self):
        stamp = self.get_clock().now().to_msg()
        frame = 'camera_color_optical_frame'

        def pub_img(publisher, img):
            m = self.bridge_.cv2_to_imgmsg(img, encoding='bgr8')
            m.header.stamp    = stamp
            m.header.frame_id = frame
            publisher.publish(m)

        if self.cx_ is None:
            waiting = make_canvas(self.size_)
            cv2.putText(waiting, 'Waiting for /canvas/pose ...',
                        (self.size_//2-120, self.size_//2+30),
                        FONT, 0.5, (0,165,255), 1)
            for pub in (self.pub_xy_, self.pub_xz_, self.pub_3d_):
                pub_img(pub, waiting)
            return

        pub_img(self.pub_xy_, build_xy_plot(
            self.size_, self.xy_range_,
            self.cx_, self.cy_, self.cz_,
            self.roll_, self.pitch_, self.yaw_, self.trail_xy_))

        pub_img(self.pub_xz_, build_xz_plot(
            self.size_, self.x_range_, self.z_range_,
            self.cx_, self.cy_, self.cz_,
            self.roll_, self.pitch_, self.yaw_, self.trail_xz_))

        pub_img(self.pub_3d_, build_3d_tf_plot(
            self.size_,
            self.cx_, self.cy_, self.cz_,
            self.qx_, self.qy_, self.qz_, self.qw_,
            self.roll_, self.pitch_, self.yaw_))


# =============================================================================

def main():
    rclpy.init()
    node = PoseVisualiser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()