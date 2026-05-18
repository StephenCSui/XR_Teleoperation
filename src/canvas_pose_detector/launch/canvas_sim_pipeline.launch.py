"""
canvas_sim_pipeline.launch.py

Camera-free simulation pipeline. No physical camera or AprilTag markers needed.
Runs the full correction and visualisation stack using a synthetic canvas pose.

Useful for:
  - Testing the corrector and visualisers without hardware
  - Demonstrating the pipeline to the team
  - Verifying parameter changes before lab sessions

Single terminal:
  ros2 launch canvas_pose_detector canvas_sim_pipeline.launch.py

With custom canvas position:
  ros2 launch canvas_pose_detector canvas_sim_pipeline.launch.py \\
    canvas_z:=0.6 canvas_yaw_deg:=20.0

With live animation (canvas sweeps left/right):
  ros2 launch canvas_pose_detector canvas_sim_pipeline.launch.py \\
    animate:=true animate_axis:=yaw animate_range:=25.0

Nodes started:
  t=0s  canvas_pose_injector       Synthetic /canvas/pose + TF + RViz markers
  t=2s  canvas_stroke_corrector    SE3 error + /canvas/z_constraint
  t=3s  canvas_depth_visualiser    /canvas/depth_view  (XZ side-view)
  t=3s  canvas_pose_visualiser     /canvas/pose_2d_xy  /canvas/pose_2d_xz

Note: canvas_correction_visualiser is NOT included here -- it requires a live
camera feed (/canvas/debug + /camera/camera/color/camera_info). Use
canvas_real_pipeline.launch.py for full visualisation with a real camera.

View output:
  ros2 run rqt_image_view rqt_image_view
  # Select /canvas/depth_view, /canvas/pose_2d_xy, /canvas/correction_2d

Live controls after launch (no restart needed):
  ros2 param set /canvas_pose_injector canvas_z           0.7
  ros2 param set /canvas_pose_injector canvas_yaw_deg     30.0
  ros2 param set /canvas_pose_injector animate            true
  ros2 param set /canvas_pose_injector animate_axis       yaw
  ros2 param set /canvas_stroke_corrector brush_standoff_m 0.010
  ros2 param set /canvas_stroke_corrector enabled         false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Launch arguments ──────────────────────────────────────────────────────

    launch_args = [

        # ── Canvas pose ───────────────────────────────────────────────────────
        # camera_color_optical_frame convention: X=right, Y=down, Z=forward
        DeclareLaunchArgument('parent_frame',     default_value='camera_color_optical_frame',
            description='TF parent frame for the synthetic canvas pose.'),
        DeclareLaunchArgument('canvas_x',         default_value='0.0',
            description='Canvas centre X in parent_frame (m). Right = +.'),
        DeclareLaunchArgument('canvas_y',         default_value='0.0',
            description='Canvas centre Y in parent_frame (m). Down = + in camera frame.'),
        DeclareLaunchArgument('canvas_z',         default_value='0.5',
            description='Canvas centre Z in parent_frame (m). Forward = +.'),
        DeclareLaunchArgument('canvas_roll_deg',  default_value='0.0',
            description='Canvas roll in degrees. 0 = flat-on to camera.'),
        DeclareLaunchArgument('canvas_pitch_deg', default_value='0.0',
            description='Canvas pitch in degrees.'),
        DeclareLaunchArgument('canvas_yaw_deg',   default_value='0.0',
            description='Canvas yaw in degrees. 30 = camera 30 deg to the right.'),

        # ── Animation ─────────────────────────────────────────────────────────
        DeclareLaunchArgument('animate',          default_value='false',
            description='Oscillate one axis sinusoidally. '
                        'Useful for watching the visualisers respond in real time.'),
        DeclareLaunchArgument('animate_axis',     default_value='yaw',
            description='Axis to animate: x | y | z | roll | pitch | yaw'),
        DeclareLaunchArgument('animate_range',    default_value='30.0',
            description='Animation half-amplitude (deg for angles, m for xyz).'),
        DeclareLaunchArgument('animate_period_s', default_value='4.0',
            description='Seconds for one full oscillation cycle.'),

        # ── Corrector settings ────────────────────────────────────────────────
        DeclareLaunchArgument('latch_window_s',   default_value='10.0',
            description='Seconds to collect poses before latching T_assumed. '
                        'Keep canvas STILL during this window.'),
        DeclareLaunchArgument('brush_standoff_m', default_value='0.005',
            description='Brush hover distance above canvas surface (m).'),
    ]

    # ── Nodes ─────────────────────────────────────────────────────────────────

    # Node 1 (t=0s): Injector -- synthetic canvas pose publisher
    injector_node = Node(
        package='canvas_pose_detector',
        executable='canvas_pose_injector',
        name='canvas_pose_injector',
        output='screen',
        parameters=[{
            'parent_frame':      LaunchConfiguration('parent_frame'),
            'canvas_x':          LaunchConfiguration('canvas_x'),
            'canvas_y':          LaunchConfiguration('canvas_y'),
            'canvas_z':          LaunchConfiguration('canvas_z'),
            'canvas_roll_deg':   LaunchConfiguration('canvas_roll_deg'),
            'canvas_pitch_deg':  LaunchConfiguration('canvas_pitch_deg'),
            'canvas_yaw_deg':    LaunchConfiguration('canvas_yaw_deg'),
            'canvas_w_m':        0.297,
            'canvas_h_m':        0.210,
            'publish_rate_hz':   10.0,
            'animate':           LaunchConfiguration('animate'),
            'animate_axis':      LaunchConfiguration('animate_axis'),
            'animate_range':     LaunchConfiguration('animate_range'),
            'animate_period_s':  LaunchConfiguration('animate_period_s'),
        }],
    )

    # Node 2 (t=2s): SE3 corrector
    # source_topic matches injector output (/canvas/pose)
    corrector_node = TimerAction(period=2.0, actions=[Node(
        package='canvas_pose_detector',
        executable='canvas_stroke_corrector',
        name='canvas_stroke_corrector',
        output='screen',
        parameters=[{
            'source_topic':    '/canvas/pose',
            'fallback_frame':  LaunchConfiguration('parent_frame'),
            'latch_window_s':  LaunchConfiguration('latch_window_s'),
            'pose_timeout_s':  5.0,
            'publish_rate_hz': 50.0,
            'brush_standoff_m': LaunchConfiguration('brush_standoff_m'),
            'enabled':         True,
        }],
    )])

    # Node 3 (t=3s): Depth visualiser -- XZ side-view canvas plane
    # Works without camera: uses only /canvas/pose, /canvas/pose_assumed,
    # /canvas/z_constraint, /canvas/correction_data
    depth_vis_node = TimerAction(period=3.0, actions=[Node(
        package='canvas_pose_detector',
        executable='canvas_depth_visualiser',
        name='canvas_depth_visualiser',
        output='screen',
    )])

    # Node 4 (t=3s): Pose visualiser -- 2D lateral + depth plots
    # Works without camera: uses only /canvas/pose and /canvas/viewing_angles
    pose_vis_node = TimerAction(period=3.0, actions=[Node(
        package='canvas_pose_detector',
        executable='canvas_pose_visualiser',
        name='canvas_pose_visualiser',
        output='screen',
    )])

    return LaunchDescription(
        launch_args + [
            injector_node,
            corrector_node,
            depth_vis_node,
            pose_vis_node,
        ]
    )