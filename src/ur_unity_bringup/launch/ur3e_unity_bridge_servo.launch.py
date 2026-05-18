from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type              = LaunchConfiguration("ur_type")
    robot_ip             = LaunchConfiguration("robot_ip")
    use_fake_hardware    = LaunchConfiguration("use_fake_hardware")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    ros_ip               = LaunchConfiguration("ros_ip")
    ros_tcp_port         = LaunchConfiguration("ros_tcp_port")
    moveit_launch_rviz   = LaunchConfiguration("moveit_launch_rviz")
    debug_ee_forward     = LaunchConfiguration("debug_ee_forward")

    ur_driver_share = FindPackageShare("ur_robot_driver")
    ur_moveit_share = FindPackageShare("ur_moveit_config")

    ur_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ur_driver_share, "launch", "ur_control.launch.py"])
        ),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": robot_ip,
            "use_fake_hardware": use_fake_hardware,
            "launch_rviz": "false",
            "initial_joint_controller": initial_joint_controller,
        }.items(),
    )

    ros_tcp_server = Node(
        package="ros_tcp_endpoint",
        executable="default_server_endpoint",
        name="ros_tcp_endpoint_unity",
        output="screen",
        arguments=["--ros-args", "--log-level", "warn"],
        parameters=[
            {"ROS_IP": ros_ip},
            {"ROS_TCP_PORT": ros_tcp_port},
        ],
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ur_moveit_share, "launch", "ur_moveit.launch.py"])
        ),
        launch_arguments={
            "ur_type": ur_type,
            "use_fake_hardware": use_fake_hardware,
            "launch_rviz": moveit_launch_rviz,
        }.items(),
    )

    ee_tf_to_pose = Node(
        package="ur_unity_bringup",
        executable="ee_tf_to_pose",
        name="ee_tf_to_pose",
        output="screen",
        arguments=["--ros-args", "--log-level", "warn"],
        parameters=[{
            "base_frame": "base_link",
            "ee_frame": "tool0",
            "topic_name": "/robot/ee_pose",
            "rate_hz": 60.0,
        }],
    )

    # ------------------------------------------------------------------ #
    # Mode / state manager                                                 #
    # Publishes active mode and canvas proximity factor to the filter.     #
    #                                                                      #
    # CANVAS TUNING — set canvas_enabled=True and configure the remaining  #
    # parameters when a canvas is physically present.                      #
    # ------------------------------------------------------------------ #
    teleop_mode_manager = Node(
        package="ur_unity_bringup",
        executable="teleop_mode_manager",
        name="teleop_mode_manager",
        output="screen",
        parameters=[{
            "publish_rate_hz": 10.0,
            "mode": "NORMAL",

            # ---- Canvas (disabled until physical setup is ready) ----
            "canvas_enabled": False,
            "canvas_frame": "base_link",

            # canvas_center_xyz: position of canvas centre in base_link (m)
            # TUNE: measure from robot base to canvas surface centre
            "canvas_center_xyz": [0.5, 0.0, 0.2],

            # canvas_normal_xyz: direction pointing away from canvas toward robot
            # TUNE: set to match actual canvas orientation
            "canvas_normal_xyz": [1.0, 0.0, 0.0],

            # canvas_up_xyz: canvas "up" direction in base_link
            # TUNE: must not be parallel to canvas_normal_xyz
            "canvas_up_xyz": [0.0, 0.0, 1.0],

            # canvas_width / canvas_height: physical canvas size (m)
            # TUNE: measure actual canvas
            "canvas_width": 0.3,
            "canvas_height": 0.3,

            # canvas_edge_margin_m: expand active zone beyond canvas edge (m)
            # TUNE: small positive value to avoid clipping at edges
            "canvas_edge_margin_m": 0.05,

            # near_canvas_enter_distance_m: distance (m) at which near-canvas
            # behaviour begins ramping in.  exit > enter for hysteresis.
            # TUNE: start conservative (e.g. 0.15) and reduce if transition is too early
            "near_canvas_enter_distance_m": 0.15,

            # near_canvas_exit_distance_m: distance (m) at which near-canvas
            # behaviour stops (must be >= enter for hysteresis to work).
            # TUNE: set ~0.05 m above enter to prevent flicker
            "near_canvas_exit_distance_m": 0.20,

            # near_canvas_full_precision_distance_m: distance (m) at which
            # maximum tightening is applied.
            # TUNE: ~tool-to-canvas contact distance (e.g. 0.01–0.03 m)
            "near_canvas_full_precision_distance_m": 0.02,

            # ---- Tool tip ----
            # tool_tip_offset_xyz: offset from EE frame origin to actual tool tip,
            # in base_link frame (m).  Zero = use EE origin directly.
            # TUNE: measure your tool length
            "tool_tip_offset_xyz": [0.0, 0.0, 0.0],

            # ---- Proximity smoothing ----
            # proximity_smoothing_alpha: [0, 1). Higher = slower response.
            # At 10 Hz: 0.5 → ~0.14 s time constant; 0.8 → ~0.45 s
            # TUNE: increase if transitions feel abrupt; decrease for faster response
            "proximity_smoothing_alpha": 0.5,

            # ---- Debug ----
            "debug_canvas": False,
        }],
    )

    # ------------------------------------------------------------------ #
    # Authority filter                                                     #
    # Existing "far" params are unchanged baselines.                       #
    # Near-canvas params apply only when proximity > 0.                   #
    #                                                                      #
    # NEAR-CANVAS TUNING — all _near params below need your review.       #
    # ------------------------------------------------------------------ #
    authority_filter_servo = Node(
        package="unity_authority_filter_servo_cpp",
        executable="unity_authority_filter_servo",
        name="unity_authority_filter_servo",
        output="screen",
        arguments=["--ros-args", "--log-level", "info",
                   "--log-level", "moveit_robot_state.robot_state:=fatal"],
        parameters=[{
            "hand_topic": "/unity/hand_pose",
            "robot_ee_topic": "/robot/ee_pose",
            "command_topic": "/unity/command_pose",
            "teleop_enable_topic": "/unity/teleop_enabled",
            "base_frame": "base_link",
            "rate_hz": 60.0,

            # ---- Existing "far" (open-space) tuning — current baselines ----
            "position_scale": 1.0,
            "hand_deadband_m": 0.003,
            "max_total_delta_m": 0.30,
            "max_cmd_step_m": 0.005,  # was 0.02 — smaller steps reduce P-controller chase distance
            "x_min": 0.10,
            "x_max": 0.80,
            "y_min": -0.50,
            "y_max":  0.50,
            "z_min": 0.05,
            "z_max": 0.70,
            "orientation_mode": "full_relative",
            "angular_deadband_deg": 0.5,
            "max_angle_from_anchor_deg": 90.0,
            "max_cmd_angle_step_deg": 5.0,   # was 12.0

            # ---- Near-canvas tightening — TUNE all values below ----

            # hand_deadband_m_near: deadband (m) applied at full proximity.
            # Increase to suppress more hand jitter when drawing.
            # Reasonable start: 2–4× the far value.
            "hand_deadband_m_near": 0.008,

            # max_cmd_step_m_near: maximum position step (m) per filter tick
            # at full proximity.  Smaller = slower but steadier movement.
            # Reasonable start: 0.003–0.006 m.
            "max_cmd_step_m_near": 0.004,

            # max_cmd_angle_step_deg_near: maximum angular step (deg) per tick
            # at full proximity.  Reduces orientation jitter near canvas.
            # Reasonable start: 1.5–4.0 deg.
            "max_cmd_angle_step_deg_near": 3.0,

            # position_scale_near: uniform position scale at full proximity,
            # used when no canvas normal is available (canvas_enabled=False).
            # TUNE: 0.3–0.6 for drawing-friendly sensitivity.
            "position_scale_near": 0.4,

            # tangential_scale_near: scale applied to motion along the canvas
            # surface at full proximity (when canvas_normal is available).
            # TUNE: 0.4–0.8 — some sensitivity needed for drawing strokes.
            "tangential_scale_near": 0.6,

            # normal_scale_near: scale applied to motion into/away from canvas
            # at full proximity (when canvas_normal is available).
            # TUNE: keep low (0.05–0.2) to prevent accidental canvas contact.
            "normal_scale_near": 0.15,

            # ---- PRECISION far (open-space, tighter than NORMAL) ----
            # Reasonable baseline: ~half step/scale vs NORMAL far.
            # TUNE: reduce position_scale_precision further if motion still too fast.
            "hand_deadband_m_precision": 0.005,
            "max_cmd_step_m_precision": 0.008,
            "max_cmd_angle_step_deg_precision": 4.0,
            "position_scale_precision": 0.5,

            # ---- PRECISION near (contact / detailed work) ----
            # TUNE: increase hand_deadband_m_precision_near to suppress more jitter.
            "hand_deadband_m_precision_near": 0.010,
            "max_cmd_step_m_precision_near": 0.003,
            "max_cmd_angle_step_deg_precision_near": 2.0,
            "position_scale_precision_near": 0.3,
            "tangential_scale_precision_near": 0.4,
            "normal_scale_precision_near": 0.10,

            # ---- Detachment / nudge mode (PRECISION only) ----
            # nudge_step_m: metres moved per button press (position nudge)
            # nudge_step_deg: degrees rotated per button press (orientation nudge)
            # TUNE: reduce if nudges feel too coarse; increase if too fine.
            "nudge_step_m": 0.005,
            "nudge_step_deg": 2.0,

            # ---- Precision bounding box ----
            # Box is drawn by the user in Unity; its size drives position_scale.
            # Geometric mean of box width/height is lerped between min and max bounds.
            # precision_scale_at_min: scale at minimum box size (2×2 cm) — tightest
            # precision_scale_at_max: scale at maximum box size (5×5 cm) — loosest
            # TUNE: reduce precision_scale_at_min for finer high-precision control.
            "precision_box_min_m": 0.02,
            "precision_box_max_m": 0.05,
            "precision_scale_at_min": 0.20,
            "precision_scale_at_max": 0.50,

            # ---- Canvas plane constraint ----
            # canvas_stop_plane_x: hard forward limit (base_link X) in normal mode
            # canvas_touch_plane_x: forward limit when pen-down trigger is held
            "canvas_stop_plane_x": 0.45,
            "canvas_touch_plane_x": 0.47,
            "pen_down_topic": "/unity/pen_down",

            # ---- Mode manager topic names (change only if remapped) ----
            "mode_topic": "/teleop_mode/active_mode",
            "proximity_topic": "/teleop_mode/canvas_proximity",
            "canvas_normal_topic": "/teleop_mode/canvas_normal",

            # ---- Connection robustness ----
            # hand_timeout_ms: declare disconnect after this many ms with no hand message.
            # Should be well above 1000/rate_hz (~17 ms at 60 Hz). Default 500 ms.
            "hand_timeout_ms": 500.0,
            # anchor_settle_ms: hold position after teleop enables before latching anchor.
            # Gives Unity time to complete its hand snap. Increase if teleport persists.
            "anchor_settle_ms": 150.0,

            # ---- Debug ----
            "debug_verbose": True,
            "debug_rpy": True,
            "debug_raw_hand_quat": False,
            # debug_ee_forward: prints EE forward vector in base_link at 2s throttle
            # compare against Unity HUD hand[ROS] readout — numbers should match
            "debug_ee_forward": debug_ee_forward,
            # debug_canvas: shows blended tuning values at 250 ms throttle
            "debug_canvas": False,
        }],
    )

    pose_error_to_twist = Node(
        package="ur_unity_bringup",
        executable="pose_error_to_twist",
        name="pose_error_to_twist",
        output="screen",
        arguments=["--ros-args", "--log-level", "warn"],
        parameters=[{
            "desired_pose_topic": "/unity/command_pose",
            "current_pose_topic": "/robot/ee_pose",
            "twist_topic": "/servo_node/delta_twist_cmds",
            "base_frame": "base_link",
            # Conservative gains for physical UR3e — pure P-controller oscillates at
            # high gain due to real actuator latency. Tune upward from here once stable.
            "linear_gain": 2.0,       # was 4.0 — reduce if still jittering
            "angular_gain": 3.0,      # was 3.0
            "max_linear_speed": 0.08, # was 0.15 m/s — keeps velocity in safe range
            "max_angular_speed": 0.6, # was 0.8 rad/s
            "position_deadband_m": 0.005,  # was 0.001 — widened to reduce jackhammer at higher robot speed
            "angle_deadband_rad": 0.03,    # was 0.01
            "publish_rate_hz": 60.0,
            "debug_verbose": False,
        }],
    )

    servo_auto_start = Node(
        package="ur_unity_bringup",
        executable="servo_auto_start",
        name="servo_auto_start",
        output="screen",
        parameters=[{
            "robot_program_running_topic": "/io_and_status_controller/robot_program_running",
            "switch_service": "/controller_manager/switch_controller",
            "forward_controller": "forward_velocity_controller",
            "deactivate_controllers": [
                "scaled_joint_trajectory_controller",
                "joint_trajectory_controller",
            ],
            "reset_service": "/servo_node/reset_servo_status",
            "start_service": "/servo_node/start_servo",
            "unpause_service": "/servo_node/unpause_servo",
            "wait_timeout_sec": 5.0,
            "max_startup_wait_sec": 30.0,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("ur_type",   default_value="ur3e"),
        DeclareLaunchArgument("robot_ip",  default_value="127.0.0.1"),
        DeclareLaunchArgument("use_fake_hardware", default_value="true"),
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="forward_velocity_controller",
        ),
        DeclareLaunchArgument("ros_ip",       default_value="127.0.0.1"),
        DeclareLaunchArgument("ros_tcp_port", default_value="10000"),
        DeclareLaunchArgument("moveit_launch_rviz", default_value="false"),
        DeclareLaunchArgument("debug_ee_forward",   default_value="false"),

        ur_control,
        ros_tcp_server,
        moveit,
        ee_tf_to_pose,
        teleop_mode_manager,
        authority_filter_servo,
        pose_error_to_twist,
        servo_auto_start,
    ])
