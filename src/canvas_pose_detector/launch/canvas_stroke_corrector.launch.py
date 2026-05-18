from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='canvas_pose_detector',
            executable='canvas_stroke_corrector',
            name='canvas_stroke_corrector',
            output='screen',
            parameters=[{
                # Subscribe to detector output directly.
                # /canvas/pose_base (bridge output) requires robot TF tree --
                # only switch to that when the UR driver is running.
                'source_topic':    '/canvas/pose',
                'fallback_frame':  'camera_color_optical_frame',

                # Seconds to collect canvas poses and average them before
                # freezing as the assumed (reference) pose T_assumed.
                # Keep canvas STILL during this window.
                # Raise to 20.0 for noisier environments.
                'latch_window_s':   10.0,

                # Max pose age before output is paused.
                'pose_timeout_s':    5.0,

                # 50Hz matches MoveIt Servo streaming rate.
                'publish_rate_hz':  50.0,

                'brush_standoff_m': 0.005,

                'enabled':          True,
            }]
        ),
    ])