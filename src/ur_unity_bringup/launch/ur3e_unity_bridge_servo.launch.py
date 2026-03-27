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

    startup_controller = LaunchConfiguration("startup_controller")
    forward_controller = LaunchConfiguration("forward_controller")

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
            "launch_rviz": "true",
            "initial_joint_controller": startup_controller,
        }.items(),
    )

    ros_tcp_server = Node(
        package="ros_tcp_endpoint",
        executable="default_server_endpoint",
        name="ros_tcp_endpoint_unity",
        output="screen",
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
        parameters=[
            {
                "base_frame": "base_link",
                "ee_frame": "tool0",
                "topic_name": "/robot/ee_pose",
                "rate_hz": 60.0,
            }
        ],
    )

    kinematics_yaml = PathJoinSubstitution(
        [ur_moveit_share, "config", "kinematics.yaml"]
    )

    authority_filter_servo = Node(
        package="unity_authority_filter_servo_cpp",
        executable="unity_authority_filter_servo",
        name="unity_authority_filter_servo",
        output="screen",
        parameters=[
            kinematics_yaml,
            {
                "hand_topic": "/unity/hand_pose",
                "robot_ee_topic": "/robot/ee_pose",
                "joint_states_topic": "/joint_states",
                "command_topic": "/unity/command_pose",
                "teleop_enable_topic": "/unity/teleop_enabled",
                "group_name": "ur_manipulator",
                "ee_link": "tool0",
                "base_frame": "base_link",
                "rate_hz": 60.0,
                "hand_deadband_m": 0.003,
                "hand_yaw_deadband_deg": 1.5,
                "max_cmd_step_m": 0.01,
                "max_cmd_yaw_step_deg": 5.0,
                "x_min": 0.10,
                "x_max": 0.80,
                "y_min": -0.50,
                "y_max": 0.50,
                "z_min": 0.05,
                "z_max": 0.70,
                "debug_verbose": True,
            },
        ],
    )

    pose_error_to_twist = Node(
        package="ur_unity_bringup",
        executable="pose_error_to_twist",
        name="pose_error_to_twist",
        output="screen",
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

    startup_pose_and_servo_bootstrap = Node(
        package="ur_unity_bringup",
        executable="startup_pose_and_servo_bootstrap",
        name="startup_pose_and_servo_bootstrap",
        output="screen",
        parameters=[
            {
                "startup_controller": startup_controller,
                "forward_controller": forward_controller,
                "switch_service": "/controller_manager/switch_controller",
                "reset_service": "/servo_node/reset_servo_status",
                "start_service": "/servo_node/start_servo",
                "unpause_service": "/servo_node/unpause_servo",
                "startup_joint_names": [
                    "shoulder_pan_joint",
                    "shoulder_lift_joint",
                    "elbow_joint",
                    "wrist_1_joint",
                    "wrist_2_joint",
                    "wrist_3_joint",
                ],
                "startup_positions": [0.0, -1.1, 1.6, -2.0, -1.57, 0.0],
                "startup_time_sec": 3.0,
                "settle_time_sec": 0.75,
                "wait_timeout_sec": 30.0,
            }
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("ur_type", default_value="ur3e"),
        DeclareLaunchArgument("robot_ip", default_value="127.0.0.1"),
        DeclareLaunchArgument("use_fake_hardware", default_value="true"),

        DeclareLaunchArgument(
            "startup_controller",
            default_value="joint_trajectory_controller",
        ),
        DeclareLaunchArgument(
            "forward_controller",
            default_value="forward_position_controller",
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
        startup_pose_and_servo_bootstrap,
    ])