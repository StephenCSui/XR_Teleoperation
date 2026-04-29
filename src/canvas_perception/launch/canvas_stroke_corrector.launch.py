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
                # Seconds to collect canvas poses and average them before
                # freezing as the assumed (reference) pose T_assumed.
                # Keep canvas STILL during this window.
                # Raise to 20.0 for noisier environments.
                'latch_window_s':   10.0,

                # Max pose age before output is paused.
                'pose_timeout_s':    5.0,

                # 50Hz matches MoveIt Servo streaming rate.
                'publish_rate_hz':  50.0,

                'enabled':          True,
            }]
        ),
    ])