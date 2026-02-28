from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    launch_rviz = LaunchConfiguration("launch_rviz")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")

    test_delay_s = LaunchConfiguration("test_delay_s")
    check_starting_point = LaunchConfiguration("check_starting_point")

    ur_pkg_share = FindPackageShare("ur_robot_driver")

    ur_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ur_pkg_share, "launch", "ur_control.launch.py"])
        ),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": robot_ip,
            "use_fake_hardware": use_fake_hardware,
            "launch_rviz": launch_rviz,
            "initial_joint_controller": initial_joint_controller,
        }.items(),
    )

    position_goals_yaml = PathJoinSubstitution(
        [ur_pkg_share, "config", "test_goal_publishers_config.yaml"]
    )

    test_publisher = Node(
        package="ros2_controllers_test_nodes",
        executable="publisher_joint_trajectory_controller",
        name="publisher_joint_trajectory_controller",
        parameters=[
            position_goals_yaml,
            {"check_starting_point": check_starting_point},
        ],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("ur_type", default_value="ur3e"),
        DeclareLaunchArgument("robot_ip", default_value="127.0.0.1"),
        DeclareLaunchArgument("use_fake_hardware", default_value="true"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument("initial_joint_controller", default_value="scaled_joint_trajectory_controller"),

        DeclareLaunchArgument("test_delay_s", default_value="5.0"),
        DeclareLaunchArgument("check_starting_point", default_value="true"),

        ur_control,

        TimerAction(
            period=test_delay_s,
            actions=[test_publisher],
        ),
    ])
