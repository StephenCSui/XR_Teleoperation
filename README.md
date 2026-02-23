# XR_Teleoperation (ROS2 workspace)

## Contents
- src/ROS-TCP-Endpoint
- src/ur_unity_bringup

# XR Teleoperation – UR3e + Unity ROS-TCP + MoveIt (ROS 2)

This ROS 2 package launches the full local dev stack on one machine:
- UR3e (fake hardware) via `ur_control.launch.py`
- ROS-TCP server for Unity (`127.0.0.1:10000`)
- MoveIt (`ur_moveit_config`)
- Unity → MoveIt Servo bridge

## Build
```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select ur_unity_bringup
source install/setup.bash
```
## Launch
```bash
ros2 launch ur_unity_bringup ur3e_unity_bridge.launch.py
```
