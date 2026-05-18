# Architecture

## Control Pipeline

```
Quest 2 controller pose
        ↓
TeleopHandPosePublisher.cs   →  /unity/hand_pose  (PoseStamped, 60 Hz)
                              →  /unity/teleop_enabled  (Bool)
                              →  /unity/pen_down  (Bool)
        ↓
unity_authority_filter_servo (C++ ROS node)
  - Startup anchor latch (150 ms settle after teleop enable)
  - Deadband filtering  (3 mm position, 0.5° orientation)
  - Workspace clamping  (x: 0.10–0.80, y: ±0.50, z: 0.05–0.70 m)
  - Step limiting       (5 mm/tick position, 5°/tick orientation)
  - Canvas plane clamp  (stop at x=0.45, pen-down allows x=0.47)
  - Relative orientation control (hand rotation relative to anchor)
  - PRECISION / NORMAL mode switching
        ↓
        →  /unity/command_pose  (PoseStamped)
        ↓
pose_error_to_twist (Python ROS node)
  - P-controller: error between command_pose and ee_pose → twist
  - linear_gain: 2.0,  max_linear_speed: 0.08 m/s
  - angular_gain: 3.0, max_angular_speed: 0.6 rad/s
  - position_deadband: 5 mm, angle_deadband: 0.03 rad
        ↓
        →  /servo_node/delta_twist_cmds  (TwistStamped)
        ↓
MoveIt Servo → forward_velocity_controller → UR3e joints
```

## Feedback Loop

```
UR3e joints
        ↓
ee_tf_to_pose (reads tool0 TF at 60 Hz)
        →  /robot/ee_pose  (PoseStamped)
        ↓
Unity: ActualEePoseSubscriber.cs  →  visualises actual EE box
Unity: TeleopHandPosePublisher.cs →  snaps XR origin to EE on startup
unity_authority_filter_servo      →  anchors command pose to EE on enable
```

## ROS Nodes

| Node | Package | Purpose |
|---|---|---|
| `unity_authority_filter_servo` | `unity_authority_filter_servo_cpp` | Main filter: gates, clamps, scales hand commands |
| `pose_error_to_twist` | `ur_unity_bringup` | P-controller: command pose error → twist velocity |
| `ee_tf_to_pose` | `ur_unity_bringup` | Reads `tool0` TF, publishes `/robot/ee_pose` at 60 Hz |
| `servo_auto_start` | `ur_unity_bringup` | Waits for robot program, activates velocity controller |
| `teleop_mode_manager` | `ur_unity_bringup` | Publishes active mode (NORMAL/PRECISION) and canvas proximity |
| `ros_tcp_endpoint` | `ros_tcp_endpoint` | TCP bridge between Unity and ROS |

## ROS Topics

| Topic | Type | Publisher | Subscriber |
|---|---|---|---|
| `/unity/hand_pose` | `PoseStamped` | Unity | `unity_authority_filter_servo` |
| `/unity/teleop_enabled` | `Bool` | Unity | `unity_authority_filter_servo` |
| `/unity/pen_down` | `Bool` | Unity | `unity_authority_filter_servo` |
| `/unity/command_pose` | `PoseStamped` | `unity_authority_filter_servo` | `pose_error_to_twist`, Unity |
| `/robot/ee_pose` | `PoseStamped` | `ee_tf_to_pose` | Unity, `unity_authority_filter_servo` |
| `/servo_node/delta_twist_cmds` | `TwistStamped` | `pose_error_to_twist` | MoveIt Servo |
| `/teleop_mode/active_mode` | `String` | `teleop_mode_manager` | `unity_authority_filter_servo` |

## Unity Scripts

| Script | Purpose |
|---|---|
| `TeleopHandPosePublisher.cs` | Reads controller pose, publishes hand pose, snaps XR origin to EE on startup |
| `CommandSubscriber.cs` | Subscribes to `/unity/command_pose`, drives command box visual |
| `ActualEePoseSubscriber.cs` | Subscribes to `/robot/ee_pose`, drives actual EE visual |
| `CanvasPainter.cs` | Canvas drawing — raycasts from command box along tool approach axis |
| `HandTargetDemoPublisher.cs` | PC keyboard fallback — WASD/arrow keys to drive hand pose |

## Coordinate Conventions

Unity and ROS use different coordinate systems. All conversions are applied at the ROS-TCP boundary.

| ROS axis | Unity axis |
|---|---|
| +X (forward) | +Z (forward) |
| +Y (left) | -X |
| +Z (up) | +Y (up) |

Position: `UnityPos = new Vector3(-ros.y, ros.z, ros.x)`

At the current starting pose, the tool Z axis (blue, approach direction) points in ROS +X = Unity +Z. The command box's local `up` vector (`transform.up`) tracks the tool approach direction — this is what the pointer ray and canvas raycast use.
