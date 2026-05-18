# XR Teleoperation — Wiki

Real-time teleoperation of a UR3e robot arm using a Meta Quest 2 VR headset. The operator's hand pose is streamed from Unity into ROS 2, filtered, and converted into velocity commands that drive the robot via MoveIt Servo.

## Pages

- [Setup](Setup.md) — hardware, software, network configuration, installation
- [Running the System](Running.md) — step-by-step startup for real robot and URSim
- [Architecture](Architecture.md) — control pipeline, ROS nodes, topics, Unity scripts
- [Tuning](Tuning.md) — configurable parameters reference
- [Troubleshooting](Troubleshooting.md) — known issues and fixes

## Quick Reference

**Real robot launch:**
```bash
cd ~/XR_Teleoperation && source install/setup.bash && ros2 launch ur_unity_bringup ur3e_unity_bridge_servo.launch.py use_fake_hardware:=false robot_ip:=192.168.0.191 ros_ip:=0.0.0.0
```

**Clean reset (run before every launch):**
```bash
pkill -f pose_error_to_twist; pkill -f unity_authority_filter_servo; pkill -f ee_tf_to_pose; pkill -f teleop_mode_manager; pkill -f servo_auto_start; pkill -f default_server_endpoint; pkill -f ur_ros2_control_node; pkill -f servo_node; pkill -f robot_state_publisher; pkill -f controller_manager; pkill -f move_group
ros2 daemon stop && sleep 1 && ros2 daemon start
```

**APK start/stop:**
```bash
adb shell monkey -p com.StephenCSui.XRTeleop 1
adb shell am force-stop com.StephenCSui.XRTeleop
```

**Starting joint pose (UR3e):**
Base=0°, Shoulder=-60°, Elbow=80°, Wrist1=-20°, Wrist2=-270°, Wrist3=90°
