# Dependencies

## Hardware

### Bill of Materials

| Component | Model | Purpose | Notes |
|---|---|---|---|
| Robot Arm | Universal Robots UR3e | Drawing end-effector | 6-DOF collaborative robot |
| Depth Camera | Intel RealSense D435if | Canvas pose detection | Wrist-mounted, USB 3.0 required |
| VR Headset | Meta Quest 2 | Operator teleoperation interface | Connected to Unity on PC via Link/Air Link |
| Workstation | Ubuntu 22.04 PC | Runs ROS 2, Unity, and drivers | See computing specs below |
| Canvas | A4 paper (297 x 210 mm) | Drawing surface | Landscape orientation |
| AprilTag Markers | 4x AprilTag 36h11, 50 mm | Canvas corner detection | IDs 0-3, printed and affixed to canvas corners |
| Drawing Tool | Brush/pen | End-effector tool | Mounted to UR3e flange |
| USB 3.0 Cable | Type-A to Type-C | RealSense connection | Must be USB 3.0 for depth stream |

### AprilTag Marker Layout

Print AprilTag 36h11 markers from https://chev.me/arucogen/ (Dictionary: AprilTag 36h11, Size: 50 mm minimum).

```
[ID:0  TL] ------------------- [ID:1  TR]
    |                               |
    |         A4 Canvas             |
    |        297 x 210 mm           |
    |                               |
[ID:3  BL] ------------------- [ID:2  BR]
```

Affix markers at the four corners on the **front face** of the canvas. The black tape border is the recommended physical boundary.

## Computing Specs

| Requirement | Minimum | Recommended |
|---|---|---|
| OS | Ubuntu 22.04 LTS (Jammy) | Ubuntu 22.04 LTS |
| CPU | Intel i5 / AMD Ryzen 5 | Intel i7 / AMD Ryzen 7 |
| RAM | 8 GB | 16 GB |
| GPU | Integrated (for ROS only) | NVIDIA GTX 1060+ (for Unity VR) |
| USB | 1x USB 3.0 port | 2x USB 3.0 ports |
| Network | Ethernet to UR3e controller | Gigabit Ethernet |

## Software Dependencies

### Core

| Software | Version | Purpose |
|---|---|---|
| Ubuntu | 22.04 LTS | Operating system |
| ROS 2 | Humble Hawksbill | Robotics middleware |
| MoveIt 2 | Humble release | Motion planning and servo |
| Unity | 2022.3 LTS | VR application |
| Python | 3.10 | Python nodes |
| CMake | 3.22+ | C++ build system |

### ROS 2 Packages (install via `apt`)

```bash
# UR Robot Driver
sudo apt install ros-humble-ur-robot-driver

# MoveIt 2
sudo apt install ros-humble-moveit

# RealSense
sudo apt install ros-humble-realsense2-camera

# TF2 and geometry
sudo apt install ros-humble-tf2-ros ros-humble-tf2-geometry-msgs ros-humble-tf2-py

# Visualisation
sudo apt install ros-humble-rqt-image-view ros-humble-rviz2

# Message filters and bridge
sudo apt install ros-humble-message-filters ros-humble-cv-bridge

# Controller test nodes (for trajectory testing)
sudo apt install ros-humble-ros2-controllers-test-nodes
```

### System Libraries (install via `apt`)

```bash
# OpenCV (C++ headers + Python bindings)
sudo apt install libopencv-dev python3-opencv

# Eigen3 (linear algebra for C++)
sudo apt install libeigen3-dev

# NumPy
sudo apt install python3-numpy
```

### Unity Packages

| Package | Source | Purpose |
|---|---|---|
| ROS-TCP-Connector | Unity Robotics Hub | Unity-side TCP client for ROS 2 communication |
| XR Interaction Toolkit | Unity Package Manager | VR controller input handling |
| Oculus Integration / Meta XR SDK | Meta Developer Hub | Quest 2 device support |

### Optional

| Software | Purpose |
|---|---|
| URSim | UR robot simulator for testing without physical robot |
| `rqt` | ROS 2 GUI for topic inspection |
