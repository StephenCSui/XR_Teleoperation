# XR Teleoperation — ROS 2 Workspace

XR-based teleoperation of a UR3e robot arm using Unity as the operator interface.
Hand pose data from Unity drives the robot end-effector via MoveIt Servo,
with an authority filter providing motion shaping, workspace clamping, and mode switching.

---

## Packages

| Package | Type | Description |
|---|---|---|
| `unity_authority_filter_servo_cpp` | C++ | Core filter — anchor-based delta teleop, tuning profiles, mode blending |
| `ur_unity_bringup` | Python + launch | Supporting nodes and main launch file |
| `ROS-TCP-Endpoint` | Third-party | TCP bridge between ROS and Unity |

---

## Nodes

### `unity_authority_filter_servo` (C++)
The core of the system. Receives hand pose from Unity and outputs a command pose for the robot.

- Anchor-based delta: on teleop enable, latches hand position and EE position as anchors. Robot moves by the delta between current hand and anchor, applied from the EE anchor.
- Four tuning profiles blended by mode and canvas proximity: NORMAL×far, NORMAL×near, PRECISION×far, PRECISION×near
- Workspace clamping in base_link frame
- Precision bounding box: box size from Unity drives `position_scale` (smaller = finer)
- Detach/nudge mode: suspends hand tracking, applies discrete nudge commands edge-triggered
- Publishes `/teleop_mode/precision_exit` when hand leaves the precision box

### `teleop_mode_manager` (Python)
Manages NORMAL / PRECISION mode state and canvas proximity factor.

- Subscribes to `/teleop_mode/set_mode` from Unity
- Publishes `/teleop_mode/active_mode`, `/teleop_mode/canvas_proximity`, `/teleop_mode/canvas_normal`
- Canvas proximity currently disabled (`canvas_enabled: False`) — enable when physical canvas is present

### `ee_tf_to_pose` (Python)
Reads the EE transform from TF and publishes it as a PoseStamped at 60 Hz.
- Publishes `/robot/ee_pose` (base_link frame)

### `pose_error_to_twist` (Python)
P-controller. Converts the pose error between command pose and current EE pose into a Twist for MoveIt Servo.
- Subscribes: `/unity/command_pose`, `/robot/ee_pose`
- Publishes: `/servo_node/delta_twist_cmds`

### `servo_auto_start` (Python)
Waits for the UR program to be running, switches to `forward_velocity_controller`, then starts and unpauses MoveIt Servo. Handles reconnect automatically.

### `ros_tcp_endpoint` (Third-party)
TCP bridge. Unity connects to `ROS_IP:ROS_TCP_PORT` (default `127.0.0.1:10000`).

---

## Topic Map

```
Unity
  /unity/hand_pose           →  authority_filter_servo
  /unity/teleop_enabled      →  authority_filter_servo
  /unity/detach_mode         →  authority_filter_servo
  /unity/nudge_cmd           →  authority_filter_servo
  /unity/precision_box       →  authority_filter_servo
  /unity/precision_box_size  →  authority_filter_servo
  /teleop_mode/set_mode      →  teleop_mode_manager

authority_filter_servo
  /unity/command_pose        →  pose_error_to_twist
  /teleop_mode/precision_exit →  Unity (TeleopModeController)

teleop_mode_manager
  /teleop_mode/active_mode   →  authority_filter_servo
  /teleop_mode/canvas_proximity → authority_filter_servo
  /teleop_mode/canvas_normal →  authority_filter_servo

ee_tf_to_pose
  /robot/ee_pose             →  authority_filter_servo, pose_error_to_twist, Unity

pose_error_to_twist
  /servo_node/delta_twist_cmds → MoveIt Servo → robot
```

---

## Teleop Modes

**NORMAL** — standard open-space teleoperation with full workspace range.

**PRECISION** — tighter gains, reduced step size, position scale driven by bounding box size.
- Press **P** in Unity to toggle
- Draw a precision box in Unity (2–5 cm); smaller box = finer control
- Hand outside box → robot freezes, exit prompt shown

**PRECISION + Detach** — suspends hand tracking, robot holds position.
- Press **Tab** in Unity to toggle
- Use WASDQE / arrow keys / ZX for discrete nudge steps
- Each key press = one fixed step (no continuous motion)

---

## Build

```bash
cd ~/XR_Teleoperation
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

---

## Launch

**Simulation (fake hardware):**
```bash
ros2 launch ur_unity_bringup ur3e_unity_bridge_servo.launch.py \
    use_fake_hardware:=true
```

**Physical UR3e:**
```bash
ros2 launch ur_unity_bringup ur3e_unity_bridge_servo.launch.py \
    use_fake_hardware:=false \
    robot_ip:=<ROBOT_IP>
```

**With RViz:**
```bash
ros2 launch ur_unity_bringup ur3e_unity_bridge_servo.launch.py \
    use_fake_hardware:=true \
    moveit_launch_rviz:=true
```

**Key launch arguments:**

| Argument | Default | Description |
|---|---|---|
| `ur_type` | `ur3e` | Robot model |
| `robot_ip` | `127.0.0.1` | UR controller IP |
| `use_fake_hardware` | `true` | Simulation or physical |
| `ros_ip` | `127.0.0.1` | IP for Unity TCP connection |
| `ros_tcp_port` | `10000` | Port for Unity TCP connection |
| `moveit_launch_rviz` | `false` | Launch RViz with MoveIt |

---

## WSL2

See `WSL2_SETUP_NOTES.txt` — the `ros_ip` argument must be set to the WSL2 IP
(not 127.0.0.1) so Unity on Windows can connect.

---

## Unity Side

The Unity project (`UnityRos2`) contains the operator interface scripts.
See `Downloads/XR_Unity_Scripts/NOTES.txt` for a full breakdown of each script.

Key scripts:
- `HandTargetDemoPublisher` — keyboard debug hand publisher (toggle `useOffsetAlignment` for VR)
- `TeleopModeController` — mode switching, detach toggle, exit confirmation UI
- `NudgeCommander` — discrete nudge commands in PRECISION+detach mode
- `PrecisionBoxPublisher` — publishes precision zone centre and size
- `ActualEePoseSubscriber` — visualises real EE position in scene
- `CommandSubscriber` — visualises filter command pose in scene
