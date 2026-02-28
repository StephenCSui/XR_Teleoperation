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

    bridge = Node(
        package="ur_unity_bringup",
        executable="unity_pose_to_servo_twist",
        name="unity_pose_to_servo_twist",
        output="screen",
        parameters=[
            {"unity_pose_topic": "/unity/target_pose"},
            {"servo_twist_topic": "/servo_node/delta_twist_cmds"},
            {"output_frame": "base_link"},
            {"rate_hz": 60.0},
            {"pos_gain": 1.2},
            {"yaw_gain": 1.2},
            {"max_lin_vel": 0.12},
            {"max_ang_vel": 0.12},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("ur_type", default_value="ur3e"),
        DeclareLaunchArgument("robot_ip", default_value="127.0.0.1"),
        DeclareLaunchArgument("use_fake_hardware", default_value="true"),
        DeclareLaunchArgument("initial_joint_controller", default_value="scaled_joint_trajectory_controller"),

        DeclareLaunchArgument("ros_ip", default_value="127.0.0.1"),
        DeclareLaunchArgument("ros_tcp_port", default_value="10000"),

        DeclareLaunchArgument("moveit_launch_rviz", default_value="true"),

        ur_control,
        ros_tcp_server,
        moveit,
        bridge,
    ])
