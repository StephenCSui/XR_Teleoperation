from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='canvas_pose_detector',
            executable='canvas_pose_detector',
            name='canvas_pose_detector',
            output='screen',
            parameters=[{
                # ── Marker IDs at each corner ──────────────────────────────
                # Print AprilTag 36h11 markers at: https://chev.me/arucogen/
                #   Dictionary: AprilTag 36h11
                #   Size: 50mm (minimum for reliable detection at 1.5m)
                #   IDs: 0 (TL), 1 (TR), 2 (BR), 3 (BL)
                #
                #   [ID:0] ──────── [ID:1]
                #     |    Canvas    |
                #   [ID:3] ──────── [ID:2]
                'marker_id_tl':       0,
                'marker_id_tr':       1,
                'marker_id_bl':       2,
                'marker_id_br':       3,

                # Physical printed size of each marker in metres
                'marker_size_m':      0.050,

                # Dictionary: AprilTag 36h11 is best for distance + low-light
                # Options: DICT_APRILTAG_36h11 (best)
                #          DICT_APRILTAG_36h10
                #          DICT_APRILTAG_25h9
                #          DICT_4X4_50 / DICT_5X5_100 (ArUco, if needed)
                'aruco_dict':         'DICT_APRILTAG_36h11',

                # CLAHE: raise clahe_clip up to 8.0 for dark environments
                'clahe_clip':         4.0,
                'clahe_tile':         8,

                # ── ArUco detector speed tuning ────────────────────────────
                # adaptiveThreshWinSizeMax is the dominant performance cost.
                # Steps = (max - min) / step  ->  fewer = faster.
                #
                #   Fast    (recommended): max=23  step=10 -> 3 passes  ~5ms/frame
                #   Balanced:              max=53  step=10 -> 6 passes  ~30ms/frame
                #   Robust (dark/far):     max=83  step=10 -> 9 passes  ~60ms/frame
                #   Original (too slow):   max=100 step=4  -> 25 passes ~3500ms/frame
                #
                # Start with fast. Raise max if markers fail to detect.
                'aruco_win_min':      3,
                'aruco_win_max':      23,    # raise to 53 if detection unreliable
                'aruco_win_step':     10,
                'aruco_thresh_const': 1,
                'aruco_error_rate':   0.6,
                'sync_queue_size':    30,

                # Physical A4 canvas size for partial marker reconstruction.
                # These MUST be set correctly for 2-marker reconstruction to work.
                # A4 landscape: 297mm wide x 210mm tall
                'canvas_physical_w_m': 0.297,
                'canvas_physical_h_m': 0.210,

                # Depth sampling: increase if depth invalid at 1.5m distance
                'depth_sample_half':  12,
                'max_depth_m':        3.0,

                # Persistence: frames a marker can be missing before lost
                'missing_frames_tol': 5,

                # Debug image: lower values = less CPU and network load
                'publish_debug':      True,
                'debug_scale':        0.5,
                'debug_fps':          10.0,

                # ── RPY display offsets ────────────────────────────────────
                # Subtracted from computed RPY before terminal logs and RViz
                # markers. Does NOT affect /canvas/pose quaternion output.
                # Set to your observed flat-on baseline so display reads 0.
                # To recalibrate: hold canvas flat-on, read raw RPY values,
                # update these numbers to match.
                'rpy_offset_roll':    133.0,   # observed flat-on roll
                'rpy_offset_pitch':   0.0,
                'rpy_offset_yaw':     -176.8,  # observed flat-on yaw
            }]
        ),
    ])