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
            "launch_rviz": "true",
            "initial_joint_controller": initial_joint_controller,
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
    )

    kinematics_yaml = PathJoinSubstitution([ur_moveit_share, "config", "kinematics.yaml"])

    authority_filter = Node(
        package="unity_authority_filter_cpp",
        executable="unity_authority_filter",
        name="unity_authority_filter",
        output="screen",
        parameters=[
            kinematics_yaml,
            {"hand_topic": "/unity/hand_pose"},
            {"robot_ee_topic": "/robot/ee_pose"},
            {"joint_states_topic": "/joint_states"},
            {"command_topic": "/unity/command_pose"},
            {"group_name": "ur_manipulator"},
            {"ee_link": "tool0"},
            {"base_frame": "base_link"},
            {"rate_hz": 20.0},
            {"ik_timeout_s": 0.02},
            {"sigma_min_threshold": 0.02},
            {"pos_step_m": 0.01},
            {"yaw_step_deg": 5.0},
            {"max_layers": 5},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("ur_type", default_value="ur3e"),
        DeclareLaunchArgument("robot_ip", default_value="127.0.0.1"),
        DeclareLaunchArgument("use_fake_hardware", default_value="true"),
        DeclareLaunchArgument("initial_joint_controller", default_value="forward_position_controller"),

        DeclareLaunchArgument("ros_ip", default_value="127.0.0.1"),
        DeclareLaunchArgument("ros_tcp_port", default_value="10000"),

        DeclareLaunchArgument("moveit_launch_rviz", default_value="false"),

        ur_control,
        ros_tcp_server,
        moveit,
        ee_tf_to_pose,
        authority_filter,
    ])