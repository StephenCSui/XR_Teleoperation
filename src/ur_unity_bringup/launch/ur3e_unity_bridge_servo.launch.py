from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    initial_joint_controller = LaunchConfiguration("initial_joint_controller")

    ros_ip = LaunchConfiguration("ros_ip")
    ros_tcp_port = LaunchConfiguration("ros_tcp_port")

    moveit_launch_rviz = LaunchConfiguration("moveit_launch_rviz")

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
        parameters=[
            {
                "base_frame": "base_link",
                "ee_frame": "tool0",
                "topic_name": "/robot/ee_pose",
                "rate_hz": 60.0,
            }
        ],
    )

    authority_filter_servo = Node(
        package="unity_authority_filter_servo_cpp",
        executable="unity_authority_filter_servo",
        name="unity_authority_filter_servo",
        output="screen",
        arguments=["--ros-args", "--log-level", "warn"],
        parameters=[
            {
                "hand_topic": "/unity/hand_pose",
                "robot_ee_topic": "/robot/ee_pose",
                "command_topic": "/unity/command_pose",
                "teleop_enable_topic": "/unity/teleop_enabled",
                "base_frame": "base_link",
                "rate_hz": 60.0,
                "position_scale": 1.0,
                "hand_deadband_m": 0.003,
                "max_total_delta_m": 0.30,
                "max_cmd_step_m": 0.02,
                "x_min": 0.10,
                "x_max": 0.80,
                "y_min": -0.50,
                "y_max": 0.50,
                "z_min": 0.05,
                "z_max": 0.70,
                "orientation_mode": "full_relative",
                "angular_deadband_deg": 0.5,
                "max_angle_from_anchor_deg": 90.0,
                "max_cmd_angle_step_deg": 12.0,
                "debug_verbose": False,
                "debug_rpy": False,
                "debug_raw_hand_quat": False,
            },
        ],
    )

    pose_error_to_twist = Node(
        package="ur_unity_bringup",
        executable="pose_error_to_twist",
        name="pose_error_to_twist",
        output="screen",
        arguments=["--ros-args", "--log-level", "warn"],
        parameters=[
            {
                "desired_pose_topic": "/unity/command_pose",
                "current_pose_topic": "/robot/ee_pose",
                "twist_topic": "/servo_node/delta_twist_cmds",
                "base_frame": "base_link",
                "linear_gain": 4.0,
                "angular_gain": 3.0,
                "max_linear_speed": 0.15,
                "max_angular_speed": 0.8,
                "position_deadband_m": 0.001,
                "angle_deadband_rad": 0.01,
                "publish_rate_hz": 60.0,
                "debug_verbose": False,
            },
        ],
    )

    servo_auto_start = Node(
        package="ur_unity_bringup",
        executable="servo_auto_start",
        name="servo_auto_start",
        output="screen",
        parameters=[
            {
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
            }
        ],
    )

    actual_pose_to_unity_visual = Node(
        package="ur_unity_bringup",
        executable="pose_to_unity_visual",
        name="actual_pose_to_unity_visual",
        output="screen",
        arguments=["--ros-args", "--log-level", "warn"],
        parameters=[
            {
                "input_topic": "/robot/ee_pose",
                "output_topic": "/unity_vis/actual_pose",
                "rotation_mode": "full",
            }
        ],
    )

    command_pose_to_unity_visual = Node(
        package="ur_unity_bringup",
        executable="pose_to_unity_visual",
        name="command_pose_to_unity_visual",
        output="screen",
        arguments=["--ros-args", "--log-level", "warn"],
        parameters=[
            {
                "input_topic": "/unity/command_pose",
                "output_topic": "/unity_vis/command_pose",
                "rotation_mode": "full",
            }
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("ur_type", default_value="ur3e"),
        DeclareLaunchArgument("robot_ip", default_value="127.0.0.1"),
        DeclareLaunchArgument("use_fake_hardware", default_value="true"),

        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="forward_velocity_controller",
        ),

        DeclareLaunchArgument("ros_ip", default_value="127.0.0.1"),
        DeclareLaunchArgument("ros_tcp_port", default_value="10000"),

        DeclareLaunchArgument("moveit_launch_rviz", default_value="false"),

        ur_control,
        ros_tcp_server,
        moveit,
        ee_tf_to_pose,
        authority_filter_servo,
        pose_error_to_twist,
        servo_auto_start,
        actual_pose_to_unity_visual,
        command_pose_to_unity_visual,
    ])