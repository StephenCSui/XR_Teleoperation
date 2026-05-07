"""
canvas_tf_bridge_validation.launch.py

Validates the canvas_tf_bridge TF chain without any camera or AprilTag markers.
Lets you visually confirm in RViz2 that every frame is in the right place
relative to the physical robot and camera mount BEFORE lab testing.

What this launches
------------------
  t=0s  robot_state_publisher      UR3e URDF -> full TF tree (base_link to tool0)
  t=0s  joint_state_publisher_gui  Slider GUI to pose the arm in any configuration
  t=0s  static_transform_publisher flange -> camera_color_optical_frame (hand-eye TF)
  t=1s  canvas_pose_injector       Synthetic canvas in camera frame
                                   Publishes /canvas/pose + TF canvas_centre
  t=2s  canvas_tf_bridge           Transforms canvas_centre to base_link
                                   Publishes /canvas/pose_base + TF canvas_centre_base
  t=0s  RViz2                      Preconfigured: robot model + all key frame axes

What you can validate
---------------------
  1. Completeness
       TF display shows unbroken chain:
       base_link -> ... -> wrist_3_link -> flange -> camera_color_optical_frame
                                        -> tool0
       canvas_centre (child of camera_color_optical_frame)
       canvas_centre_base (child of base_link, published by bridge)

  2. Camera mount direction
       Drag arm to your known test position in the slider GUI.
       Check that camera_color_optical_frame Z axis (blue) points toward
       where the canvas would be in the real setup.
       If not, adjust cam_roll/pitch/yaw args.

  3. Canvas position in base_link
       canvas_centre_base should appear at a plausible location on/above the
       table relative to the robot base. Adjust canvas_z (injector) to match
       the real canvas distance.

  4. Frame doesn't drift with arm movement
       canvas_centre_base is expressed in base_link. When you move the arm
       sliders, canvas_centre_base WILL move (because the injector puts canvas
       in camera frame which moves with the arm). This is expected in this
       validation -- the injector simulates "camera sees canvas at constant
       offset." In real use the detector sees the fixed physical canvas.

  5. tf2_echo spot checks
       See the terminal commands printed after launch.

Terminal (single):
  ros2 launch canvas_pose_detector canvas_tf_bridge_validation.launch.py

With custom hand-eye values:
  ros2 launch canvas_pose_detector canvas_tf_bridge_validation.launch.py \\
    cam_x:=0.05 cam_y:=0.08 cam_z:=-0.02 cam_roll:=-1.57 cam_yaw:=0.0

With a closer/farther synthetic canvas:
  ros2 launch canvas_pose_detector canvas_tf_bridge_validation.launch.py \\
    canvas_z:=0.4

After launch -- useful terminal commands
-----------------------------------------
  # Full TF tree (text)
  ros2 run tf2_tools view_frames

  # Live transform: where is canvas relative to base_link?
  ros2 run tf2_ros tf2_echo base_link canvas_centre_base

  # Live transform: where is canvas relative to flange?
  ros2 run tf2_ros tf2_echo flange canvas_centre

  # What is the exact hand-eye transform (flange -> camera)?
  ros2 run tf2_ros tf2_echo flange camera_color_optical_frame

  # What is the exact tool0 -> camera transform?
  ros2 run tf2_ros tf2_echo tool0 camera_color_optical_frame

  # Distance from base_link to canvas_centre_base
  ros2 run tf2_ros tf2_echo base_link canvas_centre_base --timeout 5
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


def generate_launch_description():

    # ── Package paths ─────────────────────────────────────────────────────────
    canvas_pkg  = get_package_share_directory('canvas_pose_detector')
    ur_desc_pkg = get_package_share_directory('ur_description')

    rviz_config = os.path.join(
        canvas_pkg, 'config', 'rviz_tf_validation.rviz'
    )

    # ── Robot description (xacro -> URDF string at launch time) ──────────────
    # Uses the ur_description package xacro -- same URDF the real driver uses.
    # ur_type=ur3e gives the exact link/joint/frame names the driver publishes.
    xacro_file = os.path.join(ur_desc_pkg, 'urdf', 'ur.urdf.xacro')
    robot_description_content = Command([
        'xacro ', xacro_file,
        ' ur_type:=ur3e',
        ' name:=ur',
        ' prefix:=',
        ' use_fake_hardware:=true',
        ' fake_sensor_commands:=false',
        ' simulation_controllers:=',
        os.path.join(ur_desc_pkg, 'config', 'ur3e', 'default_controllers.yaml'),
    ])
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    # ── Launch arguments ──────────────────────────────────────────────────────

    launch_args = [

        # ── Hand-eye transform: flange -> camera_color_optical_frame ──────────
        # These are the values you are VALIDATING. Set them to your best
        # current measurement, confirm they look right in RViz, adjust if not.
          DeclareLaunchArgument('cam_x',     default_value='-0.05',
            description='Camera X offset from flange (m)'),
        DeclareLaunchArgument('cam_y',     default_value='0.00',
            description='Camera Y offset from flange (m). Placeholder.'),
        DeclareLaunchArgument('cam_z',     default_value='-0.09',
            description='Camera Z offset from flange (m)'),
        DeclareLaunchArgument('cam_roll',  default_value='0.0',
            description='Camera roll from flange (rad). Placeholder.'),
        DeclareLaunchArgument('cam_pitch', default_value='0.0',
            description='Camera pitch from flange (rad)'),
        DeclareLaunchArgument('cam_yaw',   default_value='-0.785',
            description='Camera yaw from flange (rad)'),

        # ── Synthetic canvas position (camera_color_optical_frame) ────────────
        # Set canvas_z to approximately the real canvas-to-camera distance.
        # camera_color_optical_frame: X=right, Y=down, Z=forward
        DeclareLaunchArgument('canvas_x',   default_value='0.0',
            description='Synthetic canvas X in camera frame (m)'),
        DeclareLaunchArgument('canvas_y',   default_value='0.0',
            description='Synthetic canvas Y in camera frame (m)'),
        DeclareLaunchArgument('canvas_z',   default_value='0.5',
            description='Synthetic canvas Z in camera frame (m). '
                        'Set to your real canvas-to-camera distance.'),
        DeclareLaunchArgument('canvas_roll_deg',  default_value='0.0'),
        DeclareLaunchArgument('canvas_pitch_deg', default_value='0.0'),
        DeclareLaunchArgument('canvas_yaw_deg',   default_value='0.0',
            description='Rotate canvas to match physical tilt.'),

        # ── Target frame for tf_bridge output ─────────────────────────────────
        DeclareLaunchArgument('target_frame', default_value='base_link',
            description='Frame canvas_centre_base will be expressed in.'),
    ]

    # ── Nodes ─────────────────────────────────────────────────────────────────

    # Node 1: robot_state_publisher
    # Reads the UR3e URDF and publishes the full TF tree from base_link to
    # flange and tool0. No driver or robot connection needed.
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # Node 2: joint_state_publisher_gui
    # Opens a slider GUI for all 6 UR joints. Drag sliders to pose the arm
    # in different configurations and watch all TF frames update in RViz.
    # This is the key tool for checking that camera_color_optical_frame
    # moves with flange correctly.
    joint_state_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )

    # Node 3: Static TF -- flange -> camera_color_optical_frame
    # THIS is what you are validating. The bridge only works if this transform
    # correctly describes where the camera sits on the flange.
    # Adjust cam_* args until camera_color_optical_frame Z axis points toward
    # the canvas in RViz when the arm is in a realistic drawing pose.
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

    # Node 4 (t=1s): canvas_pose_injector
    # Publishes a synthetic canvas as a child of camera_color_optical_frame.
    # This simulates what the detector would publish if the camera saw the
    # canvas at canvas_z metres directly in front of it.
    # Also publishes TF canvas_centre and RViz MarkerArray of the canvas plane.
    injector_node = TimerAction(period=1.0, actions=[Node(
        package='canvas_pose_detector',
        executable='canvas_pose_injector',
        name='canvas_pose_injector',
        output='screen',
        parameters=[{
            'parent_frame':      'camera_color_optical_frame',
            'canvas_x':          LaunchConfiguration('canvas_x'),
            'canvas_y':          LaunchConfiguration('canvas_y'),
            'canvas_z':          LaunchConfiguration('canvas_z'),
            'canvas_roll_deg':   LaunchConfiguration('canvas_roll_deg'),
            'canvas_pitch_deg':  LaunchConfiguration('canvas_pitch_deg'),
            'canvas_yaw_deg':    LaunchConfiguration('canvas_yaw_deg'),
            'canvas_w_m':        0.297,
            'canvas_h_m':        0.210,
            'publish_rate_hz':   10.0,
            'animate':           False,
        }],
    )])

    # Node 5 (t=2s): canvas_tf_bridge
    # Transforms /canvas/pose from camera_color_optical_frame to base_link.
    # Publishes /canvas/pose_base and broadcasts TF canvas_centre_base.
    # The bridge will only work once the full TF chain is available:
    #   base_link -> ... -> flange -> camera_color_optical_frame
    bridge_node = TimerAction(period=2.0, actions=[Node(
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

    # Node 6: RViz2 with validation config
    # Shows: robot model, all TF frames, per-frame axes at different scales,
    # and the injector's canvas marker outline.
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    # ── Startup instructions printed to terminal ──────────────────────────────
    instructions = LogInfo(msg="""
================================================================================
  canvas_tf_bridge_validation  --  RViz2 validation guide
================================================================================

  WHAT TO CHECK IN RVIZ
  ---------------------
  1. TF display shows ALL frames without red warning text.
     Required chain: base_link -> shoulder_pan_joint -> ... -> flange
                     flange -> camera_color_optical_frame -> canvas_centre
                     flange -> tool0
                     base_link (canvas_centre_base appears here once bridge starts)

  2. camera_color_optical_frame Z axis (blue) points roughly toward where
     the physical canvas would be when the arm is in drawing position.
     If it points the wrong way: adjust cam_roll/pitch/yaw args.

  3. canvas_centre_base (large cyan axes in RViz) appears at a plausible
     position in the robot's workspace (0.3-0.8m in front of base_link).
     Adjust canvas_z arg to match your real canvas distance.

  4. Drag the joint_state_publisher_gui sliders to a realistic arm pose
     (wrist near the canvas). Confirm camera_color_optical_frame moves
     with flange correctly -- the two frames should stay rigidly attached.

  SPOT-CHECK COMMANDS (new terminals)
  ------------------------------------
  # Full TF tree diagram saved to frames.gv
  ros2 run tf2_tools view_frames

  # canvas position relative to robot base (should be ~0.3-0.8m forward)
  ros2 run tf2_ros tf2_echo base_link canvas_centre_base

  # exact hand-eye transform (what you set vs what TF reports)
  ros2 run tf2_ros tf2_echo flange camera_color_optical_frame

  # camera offset relative to tool0 (for MoveIt planning reference)
  ros2 run tf2_ros tf2_echo tool0 camera_color_optical_frame

  LIVE TUNING (no restart needed)
  ---------------------------------
  # Move synthetic canvas closer/farther
  ros2 param set /canvas_pose_injector canvas_z 0.4

  # Tilt canvas to match real setup angle
  ros2 param set /canvas_pose_injector canvas_yaw_deg 15.0

  # Sweep canvas back and forth to watch bridge output update
  ros2 param set /canvas_pose_injector animate            true
  ros2 param set /canvas_pose_injector animate_axis       z
  ros2 param set /canvas_pose_injector animate_range      0.1
================================================================================
""")

    return LaunchDescription(
        launch_args + [
            instructions,
            robot_state_pub,
            joint_state_gui,
            static_tf_node,
            rviz_node,
            injector_node,
            bridge_node,
        ]
    )