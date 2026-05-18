# XR Teleoperation Wiki

A ROS 2 Humble system for **teleoperated drawing on a physical canvas** using a UR3e robot arm, Meta Quest 2 VR headset, and Intel RealSense D435if depth camera. The operator draws strokes in VR; the robot reproduces them on a real A4 canvas while camera perception continuously tracks the canvas pose and corrects for drift.

## Key Features

- **VR Teleoperation** — draw naturally in Meta Quest 2; strokes are mapped to robot end-effector trajectories via MoveIt Servo
- **Real-Time Canvas Perception** — AprilTag-based 6-DOF canvas detection at up to 18 fps with partial-marker recovery and 3D position caching
- **SE3 Drift Correction** — continuous pose error estimation keeps the brush aligned to the canvas surface even when the canvas moves
- **Unity-ROS 2 Bridge** — bidirectional TCP link between Unity (VR) and ROS 2 via ROS-TCP-Endpoint

## Subsystems

| Package | Responsibility | Wiki Page |
|---|---|---|
| `canvas_pose_detector` | Camera perception, canvas pose estimation, SE3 correction | [Canvas Pose Detector](Subsystem:-Canvas-Pose-Detector.md) |
| `ur_unity_bringup` | UR3e driver, MoveIt Servo, VR teleoperation bridge | [UR Unity Bringup](Subsystem:-UR-Unity-Bringup.md) |
| `ROS-TCP-Endpoint` | TCP bridge between Unity and ROS 2 (3rd party) | [ROS-TCP-Endpoint](Subsystem:-ROS-TCP-Endpoint.md) |

## Wiki Pages

1. **[Home](Home.md)** — this page
2. **[Dependencies](Dependencies.md)** — hardware and software requirements
3. **[Installation](Installation.md)** — step-by-step setup guide
4. **[Running the System](Running-the-System.md)** — launch commands and expected outcomes
5. **[Subsystem: Canvas Pose Detector](Subsystem:-Canvas-Pose-Detector.md)** — camera perception pipeline
6. **[Subsystem: UR Unity Bringup](Subsystem:-UR-Unity-Bringup.md)** — robot control and VR bridge
7. **[Subsystem: ROS-TCP-Endpoint](Subsystem:-ROS-TCP-Endpoint.md)** — Unity-ROS communication
8. **[Unity Integration Guide](Unity-Integration-Guide.md)** — building the VR interface
9. **[Troubleshooting & FAQ](Troubleshooting-&-FAQ.md)** — common issues and fixes
10. **[Teleoperation Setup](Setup.md)** — VR hardware, network config, APK deployment
11. **[Teleoperation Tuning](Tuning.md)** — authority filter and P-controller parameters

## Quick Reference (Teleoperation)

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

**Starting joint pose:** Base=0°, Shoulder=-60°, Elbow=80°, Wrist1=-20°, Wrist2=-270°, Wrist3=90°

## Technology Stack

| Component | Version |
|---|---|
| Ubuntu | 22.04 LTS |
| ROS 2 | Humble |
| MoveIt 2 | Humble release |
| Unity | 2022.3 LTS |
| Robot | UR3e |
| Camera | Intel RealSense D435if |
| VR Headset | Meta Quest 2 |
