# Setup

## Hardware Requirements

- UR3e robot arm
- Meta Quest 2 headset (developer mode enabled from the owner Meta account)
- Host PC with both ethernet (to robot) and WiFi (to headset) interfaces

## Software Requirements

- Ubuntu 22.04
- ROS 2 Humble
- MoveIt 2 with MoveIt Servo
- `Universal_Robots_ROS2_Driver` — `ur_robot_driver` and `ur_moveit_config`
- `ROS-TCP-Endpoint` v0.7.0 — must match Unity package version exactly
- Unity 2022.3.62f3 with Android Build Support and OpenXR plugin
- `ROS-TCP-Connector` v0.7.0-preview — Unity package, pinned to match endpoint
- ADB (Android Debug Bridge) — for APK deployment

Do not update `ROS-TCP-Connector` or `ROS-TCP-Endpoint` independently. They must remain at matching versions.

## Network Configuration

Find your laptop IPs first:
```bash
hostname -I
```

| Role | Address |
|---|---|
| UR3e robot | `192.168.0.191` |
| Laptop ethernet (to robot) | `192.168.0.101` |
| Laptop WiFi (to Quest 2) | `10.93.37.16` |
| ROS-TCP port | `10000` |

The Quest 2 and the laptop must be on the same WiFi network. The APK has the laptop WiFi IP hardcoded at build time — changing networks requires a rebuild.

**URCaps External Control** (set on teach pendant):
- Host IP: your laptop ethernet IP (`192.168.0.101`)
- Port: `50002` (default)

**Unity APK ROS-TCP-Connector:**
- IP: your laptop WiFi IP (`10.93.37.16`)
- Port: `10000`

## Installation

```bash
cd ~/XR_Teleoperation
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## APK Deployment

```bash
# Verify Quest 2 is connected
adb devices

# Install APK
adb install -r /home/steph/UnityRos2/<build>.apk
```

If `adb devices` shows `unauthorized`: the USB debugging prompt did not appear in the headset. Developer mode must be enabled from the owner Meta account — switch to the owner profile in the headset and replug USB.
