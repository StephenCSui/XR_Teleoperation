"""
canvas_pose_validation.launch.py

One-command launch for the full camera-free UR3e pose validation stack:

  1. canvas_pose_injector  -- publishes synthetic /canvas/pose (no camera needed)
  2. canvas_pose_follower  -- commands UR3e EEF to follow the canvas pose via MoveIt
  3. static_transform_publisher (OPTIONAL, commented out by default)
     -- provides camera_color_optical_frame -> base_link if injector uses camera frame

What you need running BEFORE this launch:
  - URSim + ur_robot_driver (sim mode)   for the robot URDF and joint states
  - move_group                           for MoveIt planning

Quick start:

  # Terminal 1: URSim + driver + move_group (your existing UR MoveIt launch)
  ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e use_mock_hardware:=true

  # Terminal 2: this validation stack
  ros2 launch canvas_pose_detector canvas_pose_validation.launch.py

  # RViz: add the following displays
  #   TF                                   (see canvas_centre TF frame)
  #   MarkerArray  /canvas/markers          (cyan canvas + corners + axes)
  #   MarkerArray  /follower/goal_marker    (yellow sphere = EEF goal)
  #   RobotModel                            (see UR3e moving)

Live controls (no relaunch needed):
  # Pause the follower
  ros2 param set /canvas_pose_follower enabled false

  # Move the canvas to a new position
  ros2 param set /canvas_pose_injector canvas_z 0.4
  ros2 param set /canvas_pose_injector canvas_x 0.05

  # Change standoff distance
  ros2 param set /canvas_pose_follower standoff_m 0.10

  # Tilt canvas and watch robot reorient
  ros2 param set /canvas_pose_injector canvas_pitch_deg 20.0

  # Animate canvas sweeping left/right (watch robot track it)
  ros2 param set /canvas_pose_injector animate true
  ros2 param set /canvas_pose_injector animate_axis yaw
  ros2 param set /canvas_pose_injector animate_range 20.0
  ros2 param set /canvas_pose_injector animate_period_s 6.0
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # ── Launch arguments (override on command line) ───────────────────────────
    # Default: horizontal A4 canvas, 30cm in front of robot, 5cm above base.
    # EEF hovers 15cm above canvas pointing straight down (drawing position).
    #
    # UR3e base_link convention: +X = forward, +Y = left, +Z = up
    # Canvas orientation 0/0/0 = face pointing UP = horizontal/flat canvas.
    # approach_from_normal=True flips tool-Z downward to face the canvas surface.
    launch_args = [
        DeclareLaunchArgument('canvas_x',         default_value='0.4',
            description='Canvas X in base_link (metres). +X = forward from robot.'),
        DeclareLaunchArgument('canvas_y',         default_value='-0.1',
            description='Canvas Y in base_link (metres). +Y = left.'),
        DeclareLaunchArgument('canvas_z',         default_value='0.0',
            description='Canvas Z in base_link (metres). '
                        '0.0 = at base_link origin (floor level of robot base).'),
        DeclareLaunchArgument('canvas_roll_deg',  default_value='0.0',
            description='0 = horizontal canvas face pointing up.'),
        DeclareLaunchArgument('canvas_pitch_deg', default_value='0.0'),
        DeclareLaunchArgument('canvas_yaw_deg',   default_value='0.0'),
        DeclareLaunchArgument('animate',          default_value='false'),
        DeclareLaunchArgument('animate_axis',     default_value='yaw'),
        DeclareLaunchArgument('animate_range',    default_value='15.0'),
        DeclareLaunchArgument('animate_period_s', default_value='8.0'),
        DeclareLaunchArgument('standoff_m',       default_value='0.40',
            description='EEF hover height above canvas (metres). '
                        '0.40 = 40cm above canvas = safe high start position.'),
        DeclareLaunchArgument('velocity_scaling', default_value='0.15'),
    ]

    # ── Node 1: Canvas pose injector ──────────────────────────────────────────
    # Horizontal A4 canvas, 30cm in front of robot, 5cm above base_link origin.
    #
    # Orientation 0/0/0 in base_link:
    #   canvas +X = forward, +Y = left, +Z = UP
    #   canvas face normal points UP -- this is a flat horizontal canvas.
    #
    # approach_from_normal=True in the follower then:
    #   1. Offsets EEF standoff_m ABOVE the canvas (along +Z)
    #   2. Flips tool orientation so tool-Z points DOWN into canvas surface
    #      -> EEF is in drawing position, pointing perpendicular to canvas
    injector_node = Node(
        package='canvas_pose_detector',
        executable='canvas_pose_injector',
        name='canvas_pose_injector',
        output='screen',
        parameters=[{
            'parent_frame':      'base_link',
            'canvas_x':          LaunchConfiguration('canvas_x'),
            'canvas_y':          LaunchConfiguration('canvas_y'),
            'canvas_z':          LaunchConfiguration('canvas_z'),
            'canvas_roll_deg':   LaunchConfiguration('canvas_roll_deg'),
            'canvas_pitch_deg':  LaunchConfiguration('canvas_pitch_deg'),
            'canvas_yaw_deg':    LaunchConfiguration('canvas_yaw_deg'),
            'canvas_w_m':        0.297,   # A4 landscape width
            'canvas_h_m':        0.210,   # A4 landscape height
            'animate':           LaunchConfiguration('animate'),
            'animate_axis':      LaunchConfiguration('animate_axis'),
            'animate_range':     LaunchConfiguration('animate_range'),
            'animate_period_s':  LaunchConfiguration('animate_period_s'),
            'publish_rate_hz':   10.0,
        }]
    )

    # ── Node 2: Canvas pose follower ──────────────────────────────────────────
    # Commands UR3e EEF to hover above the canvas in drawing position.
    # approach_from_normal=True ensures tool-Z points straight down into canvas.
    follower_node = Node(
        package='canvas_pose_detector',
        executable='canvas_pose_follower',
        name='canvas_pose_follower',
        output='screen',
        parameters=[{
            'planning_frame':        'base_link',
            'pose_topic':            '/canvas/pose',
            'planning_group':        'ur_manipulator',
            'eef_link':              'tool0',
            # EEF stops standoff_m above the canvas surface.
            # On launch this puts the arm high (0.40m above canvas = safe start).
            # Lower it progressively: standoff_m 0.40 -> 0.20 -> 0.05 to draw.
            'standoff_m':            LaunchConfiguration('standoff_m'),
            # Flip tool-Z to point INTO canvas (straight down for horizontal canvas)
            'approach_from_normal':  True,
            'pos_deadband_m':        0.010,
            'rot_deadband_deg':      3.0,
            'velocity_scaling':      LaunchConfiguration('velocity_scaling'),
            'accel_scaling':         0.15,
            'planning_time_s':       5.0,
            # Loosen tolerances: strict orientation + position combined kills IK.
            # 0.01m position, 0.3rad (~17deg) orientation gives IK room to solve
            # while still keeping the tool pointed meaningfully downward.
            'pos_tolerance_m':       0.01,
            'ori_tolerance_rad':     0.3,
            'enabled':               True,
        }]
    )

    # ── (OPTIONAL) Static TF: camera_color_optical_frame -> base_link ─────────
    # Uncomment this block if you switch the injector's parent_frame to
    # 'camera_color_optical_frame' to simulate the real camera pipeline.
    #
    # Values below place the camera 60cm in front of the robot, 50cm elevated,
    # oriented so camera +Z (depth axis) points toward the robot workspace.
    # Adjust to match your actual camera mount.
    #
    # To find the right values: move robot to a known position, measure camera
    # location relative to robot base, convert to x/y/z/roll/pitch/yaw.
    #
    # camera_static_tf = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='camera_to_base_static_tf',
    #     arguments=[
    #         # x     y     z      qx    qy    qz    qw
    #         '0.60', '0.0', '0.50', '0.0', '0.707', '0.0', '0.707',
    #         # parent frame      child frame
    #         'base_link',        'camera_color_optical_frame',
    #     ],
    #     output='screen',
    # )

    return LaunchDescription(
        launch_args + [
            injector_node,
            follower_node,
            # RViz is launched by ur_moveit.launch.py -- pass our config to it:
            #   ros2 launch ur_moveit_config ur_moveit.launch.py \
            #     ur_type:=ur3e use_mock_hardware:=true \
            #     rviz_config:=$(ros2 pkg prefix canvas_pose_detector)/share/canvas_pose_detector/config/rviz_config.rviz
        ]
    )