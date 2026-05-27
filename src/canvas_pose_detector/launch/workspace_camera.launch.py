"""
workspace_camera.launch.py

Launches Camera B (tripod-mounted RealSense D435i) for external workspace
viewing. Streams RGB only -- no depth alignment, no AprilTag detection.

The RGB feed is published to /workspace_cam/color/image_raw for Unity to
consume. This launch file includes an rqt_image_view for local testing
without Unity.

Find your Camera B serial number:
  rs-enumerate-devices | grep "Serial Number"

Usage:
  ros2 launch canvas_pose_detector workspace_camera.launch.py

  # If two RealSense cameras are connected, specify the serial number:
  ros2 launch canvas_pose_detector workspace_camera.launch.py \\
    serial_no:=<CAMERA_B_SERIAL>

  # Without the built-in viewer (e.g. when Unity consumes the feed):
  ros2 launch canvas_pose_detector workspace_camera.launch.py \\
    open_viewer:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    launch_args = [
        DeclareLaunchArgument('serial_no', default_value="''",
            description='RealSense serial number for Camera B. '
                        'Leave empty if only one camera is connected.'),
        DeclareLaunchArgument('fps', default_value='30',
            description='RGB stream frame rate (6, 15, 30).'),
        DeclareLaunchArgument('width', default_value='640',
            description='RGB stream width (424, 640, 848, 1280).'),
        DeclareLaunchArgument('height', default_value='480',
            description='RGB stream height (240, 480, 720, 800).'),
        DeclareLaunchArgument('open_viewer', default_value='true',
            description='Open rqt_image_view for local testing.'),
    ]

    # RealSense camera node under /workspace_cam namespace
    # RGB only -- depth disabled to save bandwidth
    camera_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='workspace_cam',
        namespace='workspace_cam',
        output='screen',
        arguments=['--ros-args', '--log-level', 'error'],
        parameters=[{
            'serial_no':              LaunchConfiguration('serial_no'),
            'camera_name':            'workspace_cam',
            'enable_color':           True,
            'enable_depth':           False,
            'enable_infra1':          False,
            'enable_infra2':          False,
            'enable_accel':           False,
            'enable_gyro':            False,
            'enable_color_to_depth':  False,
            'rgb_camera.color_profile': LaunchConfiguration('width') ,
            'color_width':            LaunchConfiguration('width'),
            'color_height':           LaunchConfiguration('height'),
            'color_fps':              LaunchConfiguration('fps'),
        }],
    )

    # rqt_image_view for local testing (no Unity needed)
    viewer = ExecuteProcess(
        cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view',
             '/workspace_cam/color/image_raw'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('open_viewer')),
    )

    return LaunchDescription(launch_args + [camera_node, viewer])
