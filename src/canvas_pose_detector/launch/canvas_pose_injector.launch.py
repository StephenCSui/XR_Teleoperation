from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='canvas_pose_detector',
            executable='canvas_pose_injector',
            name='canvas_pose_injector',
            output='screen',
            parameters=[{

                # ── Frame ──────────────────────────────────────────────────
                # The TF parent frame that canvas_centre will be expressed in.
                # Must match what Member 2's transform listens to.
                # Change to 'base_link' if you want to inject in robot frame directly.
                'parent_frame':      'camera_color_optical_frame',

                # ── Canvas centre position (metres) ────────────────────────
                # camera_color_optical_frame convention:
                #   X = right,   Y = down,   Z = forward (into scene)
                #
                # Default: canvas 0.5m directly in front of camera, centred
                'canvas_x':          0.0,
                'canvas_y':          0.0,
                'canvas_z':          0.5,

                # ── Canvas orientation (degrees, ZYX extrinsic / ROS) ──────
                # 0 / 0 / 0 = canvas face pointing directly at the camera (flat-on)
                #
                # Test poses to try:
                #   Flat-on centre:       0 / 0 / 0
                #   Camera 30 deg right:  0 / 0 / 30   (positive yaw tilts left edge away)
                #   Camera 20 deg above:  20 / 0 / 0   (positive roll tilts bottom away)
                #   Angled top corner:    15 / 10 / 20
                'canvas_roll_deg':   0.0,
                'canvas_pitch_deg':  0.0,
                'canvas_yaw_deg':    0.0,

                # ── Physical canvas dimensions (metres) ────────────────────
                # A4 landscape: 0.297 x 0.210
                # A4 portrait:  0.210 x 0.297
                'canvas_w_m':        0.297,
                'canvas_h_m':        0.210,

                # ── Publish rate ───────────────────────────────────────────
                'publish_rate_hz':   10.0,

                # ── Animation (optional dynamic sweep for pipeline testing) ─
                # Set animate to true to oscillate one axis sinusoidally.
                # Useful for checking Member 2's transform updates in real time
                # and confirming RViz/rqt visualiser respond correctly.
                #
                # animate_axis options:  x | y | z | roll | pitch | yaw
                # animate_range:  half-amplitude  (degrees for angles, metres for xyz)
                # animate_period_s: time for one full oscillation
                #
                # Example -- sweep yaw ±30 deg every 4 seconds:
                #   animate: true, animate_axis: yaw, animate_range: 30.0
                #
                # Override live without relaunch:
                #   ros2 param set /canvas_pose_injector animate true
                'animate':           False,
                'animate_axis':      'yaw',
                'animate_range':     30.0,
                'animate_period_s':  4.0,
            }]
        ),
    ])