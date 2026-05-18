"""
canvas_tf_bridge.launch.py

Launches the full hand-eye transform pipeline:

  1. static_transform_publisher   flange -> camera_color_optical_frame
  2. canvas_tf_bridge             /canvas/pose (camera frame)
                                    -> /canvas/pose_base  (base_link)
                                    -> /canvas/pose_tool0 (tool0, debug)
  3. canvas_stroke_corrector      subscribes /canvas/pose_base
                                    -> /canvas/z_constraint (base_link)

Prerequisites (must be running before this launch):
  - RealSense camera driver (publishes /canvas/pose via detector)
  - UR robot driver or URSim (publishes robot TF tree: base_link, flange, tool0)

Command-line overrides:
  ros2 launch canvas_pose_detector canvas_tf_bridge.launch.py \\
    cam_x:=0.0 cam_y:=0.09 cam_z:=0.0 cam_roll:=-0.785 \\
    latch_window_s:=15.0 brush_standoff_m:=0.005

Verify the TF chain after launch:
  ros2 run tf2_tools view_frames
  ros2 run tf2_ros tf2_echo base_link canvas_centre_base
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Launch arguments ──────────────────────────────────────────────────────
    # Camera extrinsics: position of camera_color_optical_frame origin
    # expressed relative to the flange frame, in metres / radians.
    # Measure from the physical flange face centre (the bolt circle).
    launch_args = [
        DeclareLaunchArgument('cam_x',    default_value='0.0',
            description='Camera X offset from flange (m)'),
        DeclareLaunchArgument('cam_y',    default_value='0.0',
            description='Camera Y offset from flange (m). '
                        'Placeholder -- measure physically before use.'),
        DeclareLaunchArgument('cam_z',    default_value='-0.9',
            description='Camera Z offset from flange (m)'),
        DeclareLaunchArgument('cam_roll', default_value='0.0',
            description='Camera roll from flange (rad)'),
        DeclareLaunchArgument('cam_pitch', default_value='0.0',
            description='Camera pitch from flange (rad)'),
        DeclareLaunchArgument('cam_yaw',  default_value='-0.785',
            description='Camera yaw from flange (rad)'),

        DeclareLaunchArgument('target_frame',    default_value='base_link',
            description='Primary output frame for canvas pose. '
                        'base_link for MoveIt planning. '
                        'tool0 for hand-eye verification.'),
        DeclareLaunchArgument('latch_window_s',  default_value='10.0',
            description='Seconds to collect canvas poses before latching reference.'),
        DeclareLaunchArgument('brush_standoff_m', default_value='0.005',
            description='Brush hover distance above canvas surface (m).'),
        DeclareLaunchArgument('tf_timeout_s',    default_value='0.10',
            description='TF lookup timeout in canvas_tf_bridge (s).'),
    ]

    # ── Node 1: Static TF flange -> camera_color_optical_frame ───────────────
    # This is the hand-eye transform: where the camera sits relative to the EEF.
    # Replace cam_x/y/z/roll/pitch/yaw with physically measured values.
    # Until hand-eye calibration is done, this is an approximate placeholder.
    #
    # Convention (flange frame on UR3e):
    #   +X = toward robot shoulder
    #   +Y = toward robot base (roughly)
    #   +Z = out of flange face (tool direction)
    #
    # Verify after launch:
    #   ros2 run tf2_ros tf2_echo flange camera_color_optical_frame
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

    # ── Node 2: canvas_tf_bridge ──────────────────────────────────────────────
    # Starts after 1s to ensure static TF is in the buffer before first lookup.
    bridge_node = TimerAction(
        period=1.0,
        actions=[Node(
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
                'tf_timeout_s':    LaunchConfiguration('tf_timeout_s'),
                'queue_size':      10,
            }],
        )],
    )

    # ── Node 3: canvas_stroke_corrector ───────────────────────────────────────
    # Starts after 2s to ensure bridge is publishing before corrector latches.
    corrector_node = TimerAction(
        period=2.0,
        actions=[Node(
            package='canvas_pose_detector',
            executable='canvas_stroke_corrector',
            name='canvas_stroke_corrector',
            output='screen',
            parameters=[{
                # Subscribes to bridge output, not raw camera-frame pose
                'source_topic':    '/canvas/pose_base',
                'fallback_frame':  LaunchConfiguration('target_frame'),
                'latch_window_s':  LaunchConfiguration('latch_window_s'),
                'pose_timeout_s':  5.0,
                'publish_rate_hz': 50.0,
                'brush_standoff_m': LaunchConfiguration('brush_standoff_m'),
                'enabled':         True,
            }],
        )],
    )

    return LaunchDescription(
        launch_args + [
            static_tf_node,
            bridge_node,
            corrector_node,
        ]
    )