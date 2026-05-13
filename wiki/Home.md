# XR Teleoperation Wiki

A ROS 2 Humble system for **teleoperated drawing on a physical canvas** using a UR3e robot arm, Meta Quest 2 VR headset, and Intel RealSense D435if depth camera. The operator draws strokes in VR; the robot reproduces them on a real A4 canvas while camera perception continuously tracks the canvas pose and corrects for drift.

## Key Features

- **VR Teleoperation** -- draw naturally in Meta Quest 2; strokes are mapped to robot end-effector trajectories via MoveIt Servo
- **Real-Time Canvas Perception** -- AprilTag-based 6-DOF canvas detection at up to 18 fps with partial-marker recovery and 3D position caching
- **SE3 Drift Correction** -- continuous pose error estimation keeps the brush aligned to the canvas surface even when the canvas moves
- **Camera-Free Simulation** -- full pipeline runs without any hardware for development and testing
- **Unity-ROS 2 Bridge** -- bidirectional TCP link between Unity (VR) and ROS 2 via ROS-TCP-Endpoint

## System Architecture

```
 ┌─────────────────┐                    ┌──────────────────────┐
 │  Meta Quest 2   │                    │    Ubuntu 22.04 PC   │
 │  (VR Headset)   │                    │                      │
 │                 │   USB/Air Link     │  ┌────────────────┐  │
 │  Unity App      │◄──────────────────►│  │ Unity 2022.3   │  │
 │  (C# / XR)      │                    │  │ LTS            │  │
 └─────────────────┘                    │  └───────┬────────┘  │
                                        │          │ TCP:10000  │
                                        │  ┌───────▼────────┐  │
                                        │  │ ROS-TCP-       │  │
                                        │  │ Endpoint       │  │
                                        │  └───────┬────────┘  │
                                        │          │           │
                              ┌─────────┼──────────┼───────────┤
                              │         │          │           │
                     ┌────────▼──────┐  │  ┌───────▼────────┐  │
                     │ ur_unity_     │  │  │ canvas_pose_   │  │
                     │ bringup       │  │  │ detector       │  │
                     │               │  │  │                │  │
                     │ - UR3e driver │  │  │ - AprilTag det │  │
                     │ - MoveIt Servo│  │  │ - TF bridge    │  │
                     │ - Pose-to-    │  │  │ - SE3 correct  │  │
                     │   twist bridge│  │  │ - Visualisers  │  │
                     └───────┬───────┘  │  └───────┬────────┘  │
                             │         │          │           │
                             │         └──────────┼───────────┘
                             │                    │
                    ┌────────▼──────┐    ┌────────▼──────┐
                    │   UR3e Robot  │    │  RealSense    │
                    │   Arm         │    │  D435if       │
                    │               │    │  (wrist-      │
                    │  MoveIt Servo │    │   mounted)    │
                    └───────────────┘    └───────────────┘
                             │                    │
                             │    ┌───────────┐   │
                             └───►│ A4 Canvas │◄──┘
                                  │ (drawing  │
                                  │  surface) │
                                  └───────────┘
```

### Data Flow

```
Unity VR Controller                          RealSense Camera
  /unity/target_pose ──►                  ◄── /camera/.../image_raw
  /unity/teleop_enable ─►  ROS 2 Network  ◄── /camera/.../depth
                            │         │
                            ▼         ▼
                 unity_pose_to_    canvas_pose_
                 servo_twist      detector (C++)
                            │         │
                            ▼         ▼
              /servo_node/       /canvas/pose
              delta_twist_cmds        │
                            │         ▼
                            │    canvas_tf_bridge
                            │         │
                            │         ▼
                            │    canvas_stroke_corrector
                            │         │
                            │         ▼
                            │    /canvas/z_constraint
                            │         │
                            ▼         ▼
                         UR3e MoveIt Servo
                              │
                              ▼
                         Robot moves brush
                         on A4 canvas
```

## Subsystems

| Package | Owner | Responsibility | Wiki Page |
|---|---|---|---|
| `canvas_pose_detector` | Johan (Member 3) | Camera perception, canvas pose estimation, SE3 correction, visualisers | [Canvas Pose Detector](Subsystem:-Canvas-Pose-Detector.md) |
| `ur_unity_bringup` | Stephen (Member 2) | UR3e driver, MoveIt Servo, Unity pose-to-twist bridge | [UR Unity Bringup](Subsystem:-UR-Unity-Bringup.md) |
| `ROS-TCP-Endpoint` | Unity Robotics (3rd party) | TCP bridge between Unity and ROS 2 | [ROS-TCP-Endpoint](Subsystem:-ROS-TCP-Endpoint.md) |

## Wiki Pages

1. **[Home](Home.md)** -- this page (project overview, architecture)
2. **[Dependencies](Dependencies.md)** -- hardware bill of materials and software requirements
3. **[Installation](Installation.md)** -- step-by-step setup guide
4. **[Running the System](Running-the-System.md)** -- launch commands, expected outcomes, live tuning
5. **[Subsystem: Canvas Pose Detector](Subsystem:-Canvas-Pose-Detector.md)** -- camera perception pipeline (Johan)
6. **[Subsystem: UR Unity Bringup](Subsystem:-UR-Unity-Bringup.md)** -- robot control and VR teleoperation bridge (Stephen)
7. **[Subsystem: ROS-TCP-Endpoint](Subsystem:-ROS-TCP-Endpoint.md)** -- Unity-ROS communication layer
8. **[Unity Integration Guide](Unity-Integration-Guide.md)** -- building the VR interface with canvas data
9. **[Troubleshooting & FAQ](Troubleshooting-&-FAQ.md)** -- common issues and solutions

## Team

| Role | Member | Email |
|---|---|---|
| Member 1 -- Unity / VR | Nhut Han Pham | -- |
| Member 2 -- MoveIt Servo | Stephen Suiwinata | stephenchristian.suiwinata@student.uts.edu.au |
| Member 3 -- Camera Perception | Johan | -- |

## Technology Stack

| Component | Version |
|---|---|
| Ubuntu | 22.04 LTS (Jammy) |
| ROS 2 | Humble Hawksbill |
| MoveIt 2 | Humble release |
| Unity | 2022.3 LTS |
| OpenCV | 4.5.4 |
| Python | 3.10 |
| Robot | Universal Robots UR3e |
| Camera | Intel RealSense D435if |
| VR Headset | Meta Quest 2 |
