# Subsystem: UR Unity Bringup

## Purpose

Launches the UR3e robot driver, MoveIt 2 motion planning, and a bridge node that converts VR controller pose targets from Unity into MoveIt Servo twist commands for real-time teleoperation.

**Package:** `ur_unity_bringup`
**Build type:** `ament_python`
**Maintainer:** Stephen Suiwinata

---

## Package Layout

```
ur_unity_bringup/
├── launch/
│   ├── ur3e_unity_bridge.launch.py       # Full robot + MoveIt + Unity bridge
│   └── ur3e_trajectory_test.launch.py    # Pre-defined trajectory test
├── ur_unity_bringup/
│   ├── __init__.py
│   └── unity_pose_to_servo_twist.py      # Pose-to-twist bridge node
├── test/
├── setup.py
├── setup.cfg
└── package.xml
```

---

## Nodes

### `unity_pose_to_servo_twist`

**Purpose:** Converts VR controller target poses from Unity into velocity twist commands for MoveIt Servo, enabling real-time teleoperation of the UR3e.

**Algorithm:**

1. When teleoperation is enabled (`/unity/teleop_enable` = `true`), the node latches the current Unity pose as the **zero reference**
2. Each timer tick (60 Hz), it computes the **delta** between the current Unity pose and the zero pose
3. The delta is scaled by proportional gains (`pos_gain`, `yaw_gain`) and clamped to velocity limits
4. The result is published as a `TwistStamped` to MoveIt Servo

This relative-control scheme means the operator can grab/release control without jumps -- re-enabling always resets the zero reference.

**Subscribed Topics:**

| Topic | Type | Description |
|---|---|---|
| `/unity/target_pose` | `geometry_msgs/PoseStamped` | VR controller position from Unity |
| `/unity/teleop_enable` | `std_msgs/Bool` | Enable/disable teleoperation |

**Published Topics:**

| Topic | Type | QoS | Description |
|---|---|---|---|
| `/servo_node/delta_twist_cmds` | `geometry_msgs/TwistStamped` | Best Effort | Velocity commands for MoveIt Servo |

**Parameters:**

| Parameter | Default | Description |
|---|---|---|
| `unity_pose_topic` | `/unity/target_pose` | Input pose topic |
| `enable_topic` | `/unity/teleop_enable` | Enable/disable topic |
| `servo_twist_topic` | `/servo_node/delta_twist_cmds` | Output twist topic |
| `output_frame` | `base_link` | Twist reference frame |
| `rate_hz` | 60.0 | Control loop rate (Hz) |
| `pos_gain` | 2.0 (node default) / 1.2 (launch default) | Position proportional gain |
| `yaw_gain` | 2.0 (node default) / 1.2 (launch default) | Yaw proportional gain |
| `max_lin_vel` | 0.15 (node default) / 0.12 (launch default) | Max linear velocity (m/s) |
| `max_ang_vel` | 0.8 (node default) / 0.12 (launch default) | Max angular velocity (rad/s) |

> **Note:** The launch file overrides the node defaults with more conservative values suitable for real robot operation.

---

## Launch Files

### `ur3e_unity_bridge.launch.py`

Starts the complete teleoperation stack:

1. **UR3e driver** (`ur_robot_driver/ur_control.launch.py`) -- hardware interface
2. **ROS-TCP-Endpoint** -- Unity-ROS TCP server on port 10000
3. **MoveIt 2** (`ur_moveit_config/ur_moveit.launch.py`) -- motion planning + servo
4. **unity_pose_to_servo_twist** -- VR-to-robot bridge

```bash
# With simulated robot
ros2 launch ur_unity_bringup ur3e_unity_bridge.launch.py

# With real robot
ros2 launch ur_unity_bringup ur3e_unity_bridge.launch.py \
  robot_ip:=192.168.1.102 \
  use_fake_hardware:=false \
  ros_ip:=192.168.1.100
```

**Launch Arguments:**

| Argument | Default | Description |
|---|---|---|
| `ur_type` | `ur3e` | UR robot model |
| `robot_ip` | `127.0.0.1` | UR controller IP address |
| `use_fake_hardware` | `true` | Use mock hardware (no real robot) |
| `initial_joint_controller` | `scaled_joint_trajectory_controller` | Active controller |
| `ros_ip` | `127.0.0.1` | ROS-TCP-Endpoint bind IP |
| `ros_tcp_port` | `10000` | ROS-TCP-Endpoint port |
| `moveit_launch_rviz` | `true` | Launch RViz with MoveIt |

### `ur3e_trajectory_test.launch.py`

Runs a pre-defined joint trajectory to verify the driver works. Starts the UR driver, waits 5 seconds, then publishes test joint goals.

```bash
ros2 launch ur_unity_bringup ur3e_trajectory_test.launch.py
```

---

## How to Test Independently

```bash
# Start with mock hardware (no robot needed)
ros2 launch ur_unity_bringup ur3e_unity_bridge.launch.py use_fake_hardware:=true

# In another terminal, simulate Unity input
ros2 topic pub /unity/teleop_enable std_msgs/Bool "data: true"
ros2 topic pub /unity/target_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'base_link'}, pose: {position: {x: 0.3, y: 0.1, z: 0.4}, orientation: {w: 1.0}}}"

# Check that twist commands are being published
ros2 topic echo /servo_node/delta_twist_cmds
```

---

## Configurable Settings

Tune gains and velocity limits at runtime:

```bash
ros2 param set /unity_pose_to_servo_twist pos_gain 1.5
ros2 param set /unity_pose_to_servo_twist max_lin_vel 0.10
```

---

## Known Limitations

- Only yaw rotation is controlled; roll and pitch are not extracted from Unity input
- The node requires MoveIt Servo to be running and accepting twist commands on the configured topic
- Gain values in the launch file differ from node defaults -- the launch file values are tuned for real hardware safety
