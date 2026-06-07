# XR Teleoperation Subsystem — Technical Documentation

## Overview

This subsystem enables real-time teleoperation of a UR3e robot arm using a Meta Quest 2 VR headset. The operator's hand/controller pose is streamed from Unity into ROS 2, filtered, and converted into velocity commands that drive the robot via MoveIt Servo.

**Key features:**
- 6-DoF position and orientation control via VR controller
- Startup snap: hand position and orientation align to the robot's end-effector on enable
- Authority filter: deadband, workspace clamping, step limiting, and precision modes
- Real-time EE pose feedback visualised in the headset
- PC keyboard fallback for testing without a headset

---

## Dependencies

### Hardware
- UR3e robot arm
- Meta Quest 2 headset (with developer mode enabled)
- Host PC with both ethernet (to robot) and WiFi (to headset) interfaces

### Software
- Ubuntu 22.04
- ROS 2 Humble
- `moveit2` — MoveIt 2 with MoveIt Servo (`moveit/moveit2`)
- `Universal_Robots_ROS2_Driver` — includes `ur_robot_driver` and `ur_moveit_config` (`UniversalRobots/Universal_Robots_ROS2_Driver`)
- `ROS-TCP-Endpoint` v0.7.0 — ROS side of Unity bridge (`Unity-Technologies/ROS-TCP-Endpoint`)
- Unity 2022.3.62f3 with Android Build Support module installed
- `ROS-TCP-Connector` v0.7.0-preview — Unity package, must match endpoint version (`Unity-Technologies/ROS-TCP-Connector`)
- ADB (Android Debug Bridge) — for APK deployment to Quest 2

---

## Network Configuration

| Role | Address |
|---|---|
| UR3e robot | `192.168.0.191` |
| Host PC (ethernet, to robot) | `192.168.0.101` |
| Host PC (WiFi, to headset) | `10.93.37.16` |
| ROS-TCP port | `10000` |

**URCaps External Control** (set on teach pendant):
- Host IP: `192.168.0.101`
- Port: `50002` (default)

**Unity APK ROS-TCP-Connector**:
- IP: `10.93.37.16`
- Port: `10000`

---

## Installation

### ROS workspace

```bash
# Clone and build
cd ~/XR_Teleoperation
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### Unity APK deployment

```bash
# Install APK to connected Quest 2
adb install -r /home/steph/UnityRos2/<build>.apk

# Verify device is connected
adb devices
```

---

## Running the System

### Step 1 — Full clean reset (always run first)

```bash
pkill -f pose_error_to_twist; pkill -f unity_authority_filter_servo; pkill -f ee_tf_to_pose; pkill -f teleop_mode_manager; pkill -f servo_auto_start; pkill -f default_server_endpoint; pkill -f ur_ros2_control_node; pkill -f servo_node; pkill -f robot_state_publisher; pkill -f controller_manager; pkill -f move_group
ros2 daemon stop && sleep 1 && ros2 daemon start
```

This eliminates stale node instances that accumulate across launches. Multiple instances of `pose_error_to_twist` will all publish twist commands simultaneously, causing the robot to move on its own.

### Step 2 — Launch ROS stack

**Real robot:**
```bash
cd ~/XR_Teleoperation && source install/setup.bash
ros2 launch ur_unity_bringup ur3e_unity_bridge_servo.launch.py \
  use_fake_hardware:=false \
  robot_ip:=192.168.0.191 \
  ros_ip:=0.0.0.0
```

**URSim (simulation):**
```bash
# Start simulator
source /opt/ros/humble/setup.bash
docker rm -f ursim 2>/dev/null || true
ros2 run ur_client_library start_ursim.sh -m ur3e

# Then launch ROS stack
cd ~/XR_Teleoperation && source install/setup.bash
ros2 launch ur_unity_bringup ur3e_unity_bridge_servo.launch.py \
  use_fake_hardware:=false \
  robot_ip:=192.168.56.101 \
  ros_ip:=0.0.0.0
```

For URSim, set joints to: **Base=0°, Shoulder=-90°, Elbow=90°, Wrist1=180°, Wrist2=-90°, Wrist3=270°** in the browser UI before hitting Play.

### Step 3 — Start robot program (real robot only)

On the teach pendant, run the External Control program. Wait for `servo_auto_start` to confirm the velocity controller is active in the terminal output.

### Step 4 — Start APK and record

```bash
# Launch app on headset
adb shell monkey -p com.StephenCSui.XRTeleop 1

# Optional: screen record
adb shell screenrecord /sdcard/capture.mp4
# Stop with Ctrl+C, then pull:
adb pull /sdcard/capture.mp4 ~/Desktop/capture.mp4
```

### Step 5 — Enable teleoperation

In the headset, press the teleop enable button (trigger or T key in keyboard mode). The hand will snap to the robot EE position. Move the controller to drive the robot.

---

## Subsystem Architecture

### Control Pipeline

```
Quest 2 controller pose
        ↓
TeleopHandPosePublisher.cs   →  /unity/hand_pose  (PoseStamped, 60 Hz)
                              →  /unity/teleop_enabled  (Bool)
        ↓
unity_authority_filter_servo (C++ ROS node)
  - Startup anchor latch (150 ms settle after teleop enable)
  - Deadband filtering  (3 mm position, 0.5° orientation)
  - Workspace clamping  (x: 0.10–0.80, y: ±0.50, z: 0.05–0.70 m)
  - Step limiting       (5 mm/tick position, 5°/tick orientation)
  - Relative orientation control (hand rotation relative to anchor)
        ↓
        →  /unity/command_pose  (PoseStamped)
        ↓
pose_error_to_twist (Python ROS node)
  - P-controller: error between command_pose and ee_pose → twist
  - linear_gain: 2.0,  max_linear_speed: 0.08 m/s
  - angular_gain: 2.0, max_angular_speed: 0.4 rad/s
  - position_deadband: 2 mm, angle_deadband: 0.02 rad
        ↓
        →  /servo_node/delta_twist_cmds  (TwistStamped)
        ↓
MoveIt Servo → forward_velocity_controller → UR3e joints
```

### Feedback Loop

```
UR3e joints
        ↓
ee_tf_to_pose (reads tool0 TF at 60 Hz)
        →  /robot/ee_pose  (PoseStamped)
        ↓
Unity: ActualEePoseSubscriber.cs  →  visualises actual EE box
Unity: TeleopHandPosePublisher.cs →  snaps hand to EE on startup
unity_authority_filter_servo      →  anchors command pose to EE on enable
```

---

## ROS Nodes

| Node | Package | Purpose |
|---|---|---|
| `ee_tf_to_pose` | `ur_unity_bringup` | Reads `tool0` TF, publishes `/robot/ee_pose` at 60 Hz |
| `unity_authority_filter_servo` | `unity_authority_filter_servo_cpp` | Main filter: gates, clamps, scales hand commands |
| `pose_error_to_twist` | `ur_unity_bringup` | P-controller: command pose error → twist velocity |
| `servo_auto_start` | `ur_unity_bringup` | Waits for robot program, activates velocity controller |
| `teleop_mode_manager` | `ur_unity_bringup` | Publishes active mode (NORMAL/PRECISION) and proximity |
| `ros_tcp_endpoint` | `ros_tcp_endpoint` | TCP bridge between Unity and ROS |

---

## ROS Topics

| Topic | Type | Publisher | Subscriber |
|---|---|---|---|
| `/unity/hand_pose` | `PoseStamped` | Unity | `unity_authority_filter_servo` |
| `/unity/teleop_enabled` | `Bool` | Unity | `unity_authority_filter_servo` |
| `/unity/command_pose` | `PoseStamped` | `unity_authority_filter_servo` | `pose_error_to_twist`, Unity visualiser |
| `/robot/ee_pose` | `PoseStamped` | `ee_tf_to_pose` | Unity, `unity_authority_filter_servo` |
| `/servo_node/delta_twist_cmds` | `TwistStamped` | `pose_error_to_twist` | MoveIt Servo |
| `/teleop_mode/active_mode` | `String` | `teleop_mode_manager` | `unity_authority_filter_servo` |

---

## Unity Scripts

| Script | Purpose |
|---|---|
| `TeleopHandPosePublisher.cs` | Reads controller pose, publishes hand pose, snaps to EE on start |
| `CommandSubscriber.cs` | Subscribes to `/unity/command_pose`, drives command box visual |
| `ActualEePoseSubscriber.cs` | Subscribes to `/robot/ee_pose`, drives actual EE visual |
| `HandTargetDemoPublisher.cs` | PC keyboard fallback — WASD/arrow keys to drive hand pose |
| `CanvasPainter.cs` | Canvas drawing feature (pen-down interaction) |

---

## Coordinate Conventions

Unity and ROS use different coordinate systems. All conversions are applied at the ROS-TCP boundary:

| Axis | Unity → ROS |
|---|---|
| Unity X | → ROS +Y |
| Unity Y | → ROS +Z |
| Unity Z | → ROS −X |

Position: `RosToUnityPosition(ros) = new Vector3(ros.y, ros.z, -ros.x)`

Rotation (Unity → ROS quaternion): `(-rosQ.y, -rosQ.z, rosQ.x, rosQ.w)`

---

## Key Configurable Parameters

All tunable parameters are in:
`src/ur_unity_bringup/launch/ur3e_unity_bridge_servo.launch.py`

No rebuild is required — parameters are read at launch time.

### Authority filter (`unity_authority_filter_servo`)

| Parameter | Default | Effect |
|---|---|---|
| `hand_deadband_m` | 0.003 | Ignore hand motion below this (m) |
| `max_cmd_step_m` | 0.005 | Max command position step per tick (m) |
| `max_cmd_angle_step_deg` | 5.0 | Max command orientation step per tick (°) |
| `position_scale` | 1.0 | Uniform position scaling |
| `x_min / x_max` | -0.80 / -0.10 | Workspace X bounds in base_link (m) |
| `anchor_settle_ms` | 150 | Settle time after teleop enable before anchor latches |

### P-controller (`pose_error_to_twist`)

| Parameter | Default | Effect |
|---|---|---|
| `linear_gain` | 2.0 | Position error → linear speed gain |
| `max_linear_speed` | 0.08 | Speed cap (m/s) |
| `angular_gain` | 3.0 | Orientation error → angular speed gain |
| `max_angular_speed` | 0.6 | Angular speed cap (rad/s) |
| `position_deadband_m` | 0.002 | Don't move if error below this (m) |

---

## Testing Without Headset

Use `HandTargetDemoPublisher.cs` in the Unity editor:

1. Disable `TeleopHandPosePublisher` GameObject in scene
2. Enable `HandTargetDemoPublisher` GameObject
3. Hit Play in Unity editor
4. WASD = forward/back/left/right, Q/E = up/down
5. Arrow keys = yaw/pitch, Z/X = roll

The script snaps to the EE on start and publishes to the same topics as the VR path.

---

## Troubleshooting

### Robot moves by itself on launch

Multiple stale `pose_error_to_twist` instances are publishing simultaneously.

**Fix:** Run the full clean reset (Step 1 above). Verify with:
```bash
ros2 topic info /servo_node/delta_twist_cmds --verbose | grep "Publisher count"
# Should be 1
```

### Command box jumps away from hand on teleop enable

The EE starting position is outside the workspace bounds (`x_min`/`x_max`).

**Fix:** Check `ee_r` in the `[ANCHOR]` log. If `ee_r.x < x_min`, lower `x_min` in the launch file. Also verify the robot is in the correct starting pose (see joint angles above).

### ADB device shows `unauthorized`

USB debugging prompt did not appear in headset. Developer mode must be enabled from the **owner** Meta account — a non-owner profile cannot accept USB auth.

**Fix:** Switch to the owner profile in the headset, replug USB.

### APK connects but no robot motion

Check that the ROS-TCP-Connector IP in the APK matches the host WiFi IP (`10.93.37.16`), and that the Quest 2 and host are on the same WiFi network.

### `servo_auto_start` never confirms

The robot program on the teach pendant is not running, or the External Control URCap is not installed/configured.

**Fix:** Confirm External Control host IP is `192.168.0.101`. Run the program on the teach pendant manually.

---

## Known Limitations

- Orientation control is relative to the anchor at teleop-enable. Large rotations near gimbal-lock poses (pitch ≈ ±90°) cause RPY display instability (physically correct, visually confusing).
- No collision avoidance — workspace bounds are the only safety constraint. Ensure the robot has clearance before enabling teleoperation.
- The ROS-TCP-Connector version is pinned to v0.7.0-preview and must match the `ros_tcp_endpoint` version exactly. Do not update either independently.
- APK ROS-TCP IP is hardcoded at build time — changing networks requires a rebuild.
