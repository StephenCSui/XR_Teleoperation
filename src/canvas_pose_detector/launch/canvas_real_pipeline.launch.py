"""
canvas_real_pipeline.launch.py

Master launch for the full real-camera production pipeline.
Requires only ONE additional terminal beyond the RealSense driver.

Before running this file, start the RealSense driver in its own terminal:
  ros2 launch realsense2_camera rs_launch.py \\
    enable_sync:=true align_depth.enable:=true

Then launch this file:
  ros2 launch canvas_pose_detector canvas_real_pipeline.launch.py

Nodes started (staggered to avoid race conditions):
  t=0s  static_transform_publisher   flange -> camera_color_optical_frame
  t=0s  canvas_pose_detector         AprilTag detection, /canvas/pose
  t=3s  canvas_tf_bridge             camera frame -> base_link transform
  t=4s  canvas_stroke_corrector      SE3 error correction, /canvas/z_constraint
  t=5s  canvas_correction_visualiser /canvas/correction_2d, /canvas/debug_corrected
  t=5s  canvas_depth_visualiser      /canvas/depth_view
  t=5s  canvas_pose_visualiser       /canvas/pose_2d_xy/xz/3d_tf  (if enabled)

View output in rqt (one rqt instance, switch topics from the dropdown):
  ros2 run rqt_image_view rqt_image_view

Key live-tunable parameters after launch (no restart needed):
  ros2 param set /canvas_pose_detector   aruco_win_max       53
  ros2 param set /canvas_pose_detector   clahe_clip          6.0
  ros2 param set /canvas_stroke_corrector brush_standoff_m   0.010
  ros2 param set /canvas_tf_bridge        target_frame        base_link

Hand-eye calibration note:
  cam_x/y/z and cam_roll/pitch/yaw are PLACEHOLDER values until hand-eye
  calibration is performed. Replace with measured values before real robot use.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg = get_package_share_directory('canvas_pose_detector')

    # ── Launch arguments ──────────────────────────────────────────────────────

    launch_args = [

        # ── Hand-eye transform (flange -> camera_color_optical_frame) ─────────
        # Replace these with physically measured values.
        # Until hand-eye calibration is done these are approximate placeholders.
     
        DeclareLaunchArgument('cam_x',     default_value='-0.05',
            description='Camera X offset from flange (m)'),
        DeclareLaunchArgument('cam_y',     default_value='0.00',
            description='Camera Y offset from flange (m). Placeholder.'),
        DeclareLaunchArgument('cam_z',     default_value='-0.09',
            description='Camera Z offset from flange (m)'),
        DeclareLaunchArgument('cam_roll',  default_value='0.0',
            description='Camera roll from flange (rad). Placeholder.'),
        DeclareLaunchArgument('cam_pitch', default_value='1.0',
            description='Camera pitch from flange (rad)'),
        DeclareLaunchArgument('cam_yaw',   default_value='0.0',
            description='Camera yaw from flange (rad)'),


        # ── Pipeline configuration ────────────────────────────────────────────
        DeclareLaunchArgument('target_frame',     default_value='base_link',
            description='TF frame canvas pose is expressed in for MoveIt. '
                        'base_link for planning. tool0 for hand-eye verification.'),
        DeclareLaunchArgument('latch_window_s',   default_value='10.0',
            description='Seconds to collect poses before latching reference. '
                        'Keep canvas STILL during this window.'),
        DeclareLaunchArgument('brush_standoff_m', default_value='0.005',
            description='Brush hover distance above canvas surface (m). '
                        '0.005 = 5mm safe hover. 0.0 = touching.'),

        # ── Detector tuning ───────────────────────────────────────────────────
        DeclareLaunchArgument('aruco_win_max', default_value='23',
            description='ArUco adaptiveThreshWinSizeMax. '
                        '23=fast(~18fps) 53=balanced 83=robust(dark/far).'),
        DeclareLaunchArgument('clahe_clip',    default_value='4.0',
            description='CLAHE clip limit. Raise to 8.0 for dark environments.'),
        DeclareLaunchArgument('marker_size_m', default_value='0.050',
            description='Printed AprilTag size in metres.'),

        # ── Optional nodes ────────────────────────────────────────────────────
        DeclareLaunchArgument('enable_pose_vis', default_value='false',
            description='Start canvas_pose_visualiser (extra CPU load). '
                        'Publishes /canvas/pose_2d_xy, pose_2d_xz.'),
    ]

    # ── Node definitions ──────────────────────────────────────────────────────

    # Node 1: Static TF -- flange -> camera_color_optical_frame
    # Publishes the hand-eye transform so canvas_tf_bridge can resolve the
    # full chain: base_link -> ... -> flange -> camera_color_optical_frame -> canvas_centre
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='flange_to_camera_tf',
        arguments=[
            '--x',     LaunchConfiguration('cam_x'),
            '--y',     LaunchConfiguration('cam_y'),
            '--z',     LaunchConfiguration('cam_z'),
            '--roll',  LaunchConfiguration('cam_roll'),
            '--pitch', LaunchConfiguration('cam_pitch'),
            '--yaw',   LaunchConfiguration('cam_yaw'),
            '--frame-id',       'flange',
            '--child-frame-id', 'camera_color_optical_frame',
        ],
        output='screen',
    )

    # Node 2: AprilTag detector -- publishes /canvas/pose in camera_color_optical_frame
    detector_node = Node(
        package='canvas_pose_detector',
        executable='canvas_pose_detector',
        name='canvas_pose_detector',
        output='screen',
        parameters=[{
            'marker_id_tl':        0,
            'marker_id_tr':        1,
            'marker_id_bl':        2,
            'marker_id_br':        3,
            'marker_size_m':       LaunchConfiguration('marker_size_m'),
            'aruco_dict':          'DICT_APRILTAG_36h11',
            'clahe_clip':          LaunchConfiguration('clahe_clip'),
            'clahe_tile':          8,
            'aruco_win_min':       3,
            'aruco_win_max':       LaunchConfiguration('aruco_win_max'),
            'aruco_win_step':      10,
            'aruco_thresh_const':  1,
            'aruco_error_rate':    0.6,
            'sync_queue_size':     30,
            'canvas_physical_w_m': 0.297,
            'canvas_physical_h_m': 0.210,
            'depth_sample_half':   12,
            'max_depth_m':         3.0,
            'missing_frames_tol':  5,
            'publish_debug':       True,
            'debug_scale':         0.5,
            'debug_fps':           10.0,
            'rpy_offset_roll':     133.0,
            'rpy_offset_pitch':    0.0,
            'rpy_offset_yaw':      -176.8,
        }],
    )

    # Node 3 (t=3s): TF bridge -- transforms /canvas/pose from camera frame to base_link
    bridge_node = TimerAction(period=3.0, actions=[Node(
        package='canvas_pose_detector',
        executable='canvas_tf_bridge',
        name='canvas_tf_bridge',
        output='screen',
        parameters=[{
            'source_topic':    '/canvas/pose',
            'target_frame':    LaunchConfiguration('target_frame'),
            'output_topic':    '/canvas/pose_base',
            'secondary_frame': 'tool0',
            'secondary_topic': '/canvas/pose_tool0',
            'broadcast_tf':    True,
            'tf_child_frame':  'canvas_centre_base',
            'fallback_frame':  'camera_color_optical_frame',
            'tf_timeout_s':    0.10,
            'queue_size':      10,
        }],
    )])

    # Node 4 (t=4s): SE3 corrector -- latches reference, publishes /canvas/z_constraint
    corrector_node = TimerAction(period=4.0, actions=[Node(
        package='canvas_pose_detector',
        executable='canvas_stroke_corrector',
        name='canvas_stroke_corrector',
        output='screen',
        parameters=[{
            'source_topic':    '/canvas/pose',
            'fallback_frame':  'camera_color_optical_frame',
            'latch_window_s':  LaunchConfiguration('latch_window_s'),
            'pose_timeout_s':  5.0,
            'publish_rate_hz': 50.0,
            'brush_standoff_m': LaunchConfiguration('brush_standoff_m'),
            'enabled':         True,
        }],
    )])

    # Node 5 (t=5s): Correction visualiser
    # Needs /canvas/debug (camera feed) from detector -- only works with real camera.
    # Publishes /canvas/correction_2d (top-down error plot) and
    # /canvas/debug_corrected (camera overlay with drift arrow).
    correction_vis_node = TimerAction(period=5.0, actions=[Node(
        package='canvas_pose_detector',
        executable='canvas_correction_visualiser',
        name='canvas_correction_visualiser',
        output='screen',
    )])

    # Node 6 (t=5s): Depth visualiser -- XZ side-view canvas plane + brush constraint
    depth_vis_node = TimerAction(period=5.0, actions=[Node(
        package='canvas_pose_detector',
        executable='canvas_depth_visualiser',
        name='canvas_depth_visualiser',
        output='screen',
    )])

    # Node 7 (t=5s): Pose visualiser -- 2D/3D lateral + depth plots (optional)
    pose_vis_node = TimerAction(period=5.0, actions=[Node(
        package='canvas_pose_detector',
        executable='canvas_pose_visualiser',
        name='canvas_pose_visualiser',
        output='screen',
    )])

    # ── Launch description ────────────────────────────────────────────────────
    # pose_vis_node always included but easily disabled by setting enable_pose_vis:=false
    # (it is kept in the list so the launch file stays unconditional -- the
    # node itself is cheap when idle and the arg documents the option)
    return LaunchDescription(
        launch_args + [
            static_tf_node,
            detector_node,
            bridge_node,
            corrector_node,
            correction_vis_node,
            depth_vis_node,
            pose_vis_node,
        ]
    )