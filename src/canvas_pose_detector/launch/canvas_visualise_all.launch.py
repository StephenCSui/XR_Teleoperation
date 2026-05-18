"""
canvas_visualise_all.launch.py

Opens ALL visualisation tools for the canvas_perception pipeline in one command.
Run this ALONGSIDE the main pipeline (canvas_real_pipeline.launch.py).

What opens
----------
  robot_state_publisher   UR3e URDF -> TF tree (base_link to flange/tool0)
  joint_state_publisher   Forwards /joint_states from the real driver (or GUI if no driver)
  RViz2                   Robot model + TF tree + all pose arrows + canvas markers
  rqt window 1            /canvas/debug_corrected  (camera overlay + drift arrow)
  rqt window 2            /canvas/correction_2d    (top-down 2D error plot)
  rqt window 3            /canvas/depth_view       (XZ side-view canvas plane)
  rqt window 4            /canvas/pose_2d_xy       (lateral X vs Y plot)

Usage
-----
  # Real camera, no robot driver (use joint sliders):
  T1: ros2 launch realsense2_camera rs_launch.py enable_sync:=true align_depth.enable:=true
  T2: ros2 launch canvas_pose_detector canvas_real_pipeline.launch.py
  T3: ros2 launch canvas_pose_detector canvas_visualise_all.launch.py

  # Real robot driver running (driver publishes /joint_states -- skip the GUI):
  T3: ros2 launch canvas_pose_detector canvas_visualise_all.launch.py use_joint_gui:=false

  # Sim, no camera:
  T1: ros2 launch canvas_pose_detector canvas_sim_pipeline.launch.py
  T2: ros2 launch canvas_pose_detector canvas_visualise_all.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    canvas_pkg  = get_package_share_directory('canvas_pose_detector')
    ur_desc_pkg = get_package_share_directory('ur_description')

    rviz_cfg   = os.path.join(canvas_pkg, 'config', 'rviz_full.rviz')
    xacro_file = os.path.join(ur_desc_pkg, 'urdf', 'ur.urdf.xacro')

    # ── Robot description (xacro -> URDF at launch time) ─────────────────────
    robot_description_content = Command([
        'xacro ', xacro_file,
        ' ur_type:=ur3e',
        ' name:=ur',
        ' prefix:=',
        ' use_fake_hardware:=true',
        ' fake_sensor_commands:=false',
    ])
    robot_description = {
        'robot_description': ParameterValue(
            robot_description_content, value_type=str
        )
    }

    # ── Launch arguments ──────────────────────────────────────────────────────
    launch_args = [
        DeclareLaunchArgument(
            'use_joint_gui',
            default_value='true',
            description=(
                'true  = open joint_state_publisher_gui sliders '
                '(use when no real robot driver is running). '
                'false = real driver already publishing /joint_states.'
            ),
        ),
    ]

    # ── robot_state_publisher ────────────────────────────────────────────────
    # Publishes the UR3e TF tree from the URDF.
    # Resolves all the red-circle errors in RViz for flange/tool0/etc.
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # ── joint_state_publisher_gui (no driver) ────────────────────────────────
    # 6-slider window to pose the arm manually.
    # Initial positions set to the real lab drawing pose (read from GUI screenshot).
    # Sliders open at these values -- drag to adjust.
    # Only started when use_joint_gui:=true (default).
    joint_state_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_joint_gui')),
        parameters=[{
            'zeros': {
                'shoulder_pan_joint':  0.000,
                'shoulder_lift_joint': -0.034,
                'elbow_joint':         -1.545,
                'wrist_1_joint':       -1.596,
                'wrist_2_joint':       -0.034,
                'wrist_3_joint':        0.051,
            }
        }],
    )

    # ── joint_state_publisher (real driver running) ───────────────────────────
    # Passive -- lets robot_state_publisher receive /joint_states from the driver.
    # Only started when use_joint_gui:=false.
    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('use_joint_gui')),
    )

    # ── RViz2 ─────────────────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        output='screen',
    )

    # ── rqt image windows ────────────────────────────────────────────────────

    rqt_correction_overlay = TimerAction(period=1.0, actions=[Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_debug_corrected',
        arguments=['/canvas/debug_corrected'],
        output='screen',
    )])

    rqt_correction_2d = TimerAction(period=1.5, actions=[Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_correction_2d',
        arguments=['/canvas/correction_2d'],
        output='screen',
    )])

    rqt_depth_view = TimerAction(period=2.0, actions=[Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_depth_view',
        arguments=['/canvas/depth_view'],
        output='screen',
    )])

    rqt_pose_2d = TimerAction(period=2.5, actions=[Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_pose_2d_xy',
        arguments=['/canvas/pose_2d_xy'],
        output='screen',
    )])

    return LaunchDescription(
        launch_args + [
            robot_state_pub,
            joint_state_gui,
            joint_state_pub,
            rviz_node,
            rqt_correction_overlay,
            rqt_correction_2d,
            rqt_depth_view,
            rqt_pose_2d,
        ]
    )