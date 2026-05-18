# Running the System

This page covers all launch configurations, from full hardware to fully simulated.

> **Always source the workspace first in every terminal:**
> ```bash
> source ~/RS2/XR_Teleoperation/install/setup.bash
> ```

---

## Full System (Real Robot + Camera + VR)

This is the production configuration with all three subsystems running.

### Terminal 1 -- RealSense Camera Driver

```bash
ros2 launch realsense2_camera rs_launch.py \
  enable_sync:=true align_depth.enable:=true
```

### Terminal 2 -- Canvas Perception Pipeline

```bash
ros2 launch canvas_pose_detector canvas_real_pipeline.launch.py
```

This starts (staggered to avoid race conditions):
- t=0s: Static TF (`flange -> camera_color_optical_frame`)
- t=0s: `canvas_pose_detector` (AprilTag detection)
- t=3s: `canvas_tf_bridge` (camera frame -> base_link)
- t=4s: `canvas_stroke_corrector` (SE3 error + Z constraint)
- t=5s: Visualiser nodes (correction, depth, pose)

### Terminal 3 -- UR3e Robot + MoveIt + Unity Bridge

```bash
ros2 launch ur_unity_bringup ur3e_unity_bridge.launch.py \
  robot_ip:=<UR3e_IP> \
  use_fake_hardware:=false \
  ros_ip:=<THIS_PC_IP>
```

### Terminal 4 (optional) -- Visualisation

```bash
ros2 run rqt_image_view rqt_image_view
```

Select from the dropdown: `/canvas/correction_2d`, `/canvas/depth_view`, `/canvas/debug_corrected`, `/canvas/debug`

Or launch the full visualisation suite (RViz + 4 image views):
```bash
ros2 launch canvas_pose_detector canvas_visualise_all.launch.py
```

### Expected Startup Timeline

| Time | Event | What You Should See |
|---|---|---|
| t=0s | Camera driver starts | RealSense LED turns on, colour + depth topics active |
| t=0s | Detector starts | Logs `[canvas_pose_detector] Waiting for synced colour+depth...` |
| t=2-4s | Markers detected | Logs `[canvas_pose_detector] Canvas detected (4/4 markers)` |
| t=3s | TF bridge starts | Logs `[canvas_tf_bridge] Publishing /canvas/pose_base in base_link` |
| t=4s | Corrector starts | Logs `[canvas_stroke_corrector] Status: COLLECTING (10.0s remaining)` |
| t=14s | Reference latched | Logs `[canvas_stroke_corrector] Status: OK` -- **system is ready** |

---

## Camera-Free Simulation (No Hardware)

Runs the full perception and correction pipeline using a synthetic canvas pose. Ideal for development, testing, and demonstrations.

### Single Terminal

```bash
ros2 launch canvas_pose_detector canvas_sim_pipeline.launch.py
```

This starts:
- t=0s: `canvas_pose_injector` (synthetic `/canvas/pose` at 10 Hz)
- t=2s: `canvas_stroke_corrector` (SE3 error + Z constraint)
- t=3s: `canvas_depth_visualiser` + `canvas_pose_visualiser`

### With Animation

The synthetic canvas sweeps continuously, useful for demonstrating the pipeline tracking a moving target:

```bash
ros2 launch canvas_pose_detector canvas_sim_pipeline.launch.py \
  animate:=true animate_axis:=yaw animate_range:=25.0 animate_period_s:=4.0
```

### With Custom Canvas Position

```bash
ros2 launch canvas_pose_detector canvas_sim_pipeline.launch.py \
  canvas_z:=0.6 canvas_yaw_deg:=20.0
```

### Expected Startup Timeline (Sim)

| Time | Event | Verification |
|---|---|---|
| t=0s | Injector publishes `/canvas/pose` at 10 Hz | `ros2 topic hz /canvas/pose` shows ~10 Hz |
| t=2s | Corrector starts collecting | `ros2 topic echo /canvas/correction_data --once` shows `COLLECTING` |
| t=12s | Status reaches `OK` | `ros2 topic echo /canvas/correction_data --once` shows `OK` |

### View Sim Visualisations

```bash
ros2 run rqt_image_view rqt_image_view
# Select: /canvas/depth_view, /canvas/pose_2d_xy, /canvas/pose_2d_xz
```

> **Note:** `/canvas/correction_2d` and `/canvas/debug_corrected` are **not available** in sim mode -- they require a live camera feed.

---

## Robot-Only (MoveIt + Unity Bridge, No Camera)

For testing the teleoperation bridge without camera perception.

### With Simulated Robot (URSim / Mock Hardware)

```bash
ros2 launch ur_unity_bringup ur3e_unity_bridge.launch.py \
  use_fake_hardware:=true
```

This starts:
- UR3e driver with mock hardware
- MoveIt 2 with RViz
- ROS-TCP-Endpoint on port 10000
- `unity_pose_to_servo_twist` bridge node

### With Real Robot

```bash
ros2 launch ur_unity_bringup ur3e_unity_bridge.launch.py \
  robot_ip:=192.168.1.102 \
  use_fake_hardware:=false \
  ros_ip:=192.168.1.100
```

---

## Trajectory Test (No Unity)

Runs a pre-defined joint trajectory to verify the UR3e driver works:

```bash
ros2 launch ur_unity_bringup ur3e_trajectory_test.launch.py \
  use_fake_hardware:=true
```

The robot (or simulated robot in RViz) should move through a sequence of joint positions after a 5-second delay.

---

## Individual Node Launch

Each perception node can be launched independently for debugging or testing.

### Detector Only (requires RealSense running)

```bash
ros2 launch canvas_pose_detector canvas_pose_detector.launch.py
```

### Injector Only (no camera)

```bash
ros2 launch canvas_pose_detector canvas_pose_injector.launch.py
```

### TF Bridge Only

```bash
ros2 launch canvas_pose_detector canvas_tf_bridge.launch.py
```

### Stroke Corrector Only

```bash
ros2 launch canvas_pose_detector canvas_stroke_corrector.launch.py
```

---

## Live Parameter Tuning

All key parameters can be changed at runtime without restarting nodes:

```bash
# === Detection Tuning ===
# Increase robustness (more ArUco threshold passes, but slower):
ros2 param set /canvas_pose_detector aruco_win_max 53

# Boost contrast for dark environments:
ros2 param set /canvas_pose_detector clahe_clip 6.0

# === Corrector Tuning ===
# Adjust brush standoff:
ros2 param set /canvas_stroke_corrector brush_standoff_m 0.010   # 10mm hover
ros2 param set /canvas_stroke_corrector enabled false             # pause output

# === Injector (sim only) ===
# Move synthetic canvas:
ros2 param set /canvas_pose_injector canvas_z 0.7
ros2 param set /canvas_pose_injector canvas_yaw_deg 30.0
ros2 param set /canvas_pose_injector animate true

# === Teleoperation Bridge ===
# Adjust control gains:
ros2 param set /unity_pose_to_servo_twist pos_gain 1.5
ros2 param set /unity_pose_to_servo_twist max_lin_vel 0.10
```

---

## Quick Diagnostic Commands

```bash
# List all running canvas nodes
ros2 node list | grep canvas

# Check canvas pose rate
ros2 topic hz /canvas/pose

# See latest correction status (JSON)
ros2 topic echo /canvas/correction_data --once

# List all canvas topics
ros2 topic list | grep canvas

# Check TF tree (generates frames.pdf)
ros2 run tf2_tools view_frames

# View specific TF transform
ros2 run tf2_ros tf2_echo base_link canvas_centre_base

# View all parameters of a node
ros2 param list /canvas_pose_detector

# Get a specific parameter value
ros2 param get /canvas_pose_detector aruco_win_max
```

---

## Launch Argument Reference

### `canvas_real_pipeline.launch.py`

| Argument | Default | Description |
|---|---|---|
| `cam_x`, `cam_y`, `cam_z` | `-0.05, 0.0, -0.09` | Hand-eye translation (m) -- **placeholder, must be calibrated** |
| `cam_roll`, `cam_pitch`, `cam_yaw` | `0.0, 1.0, 0.0` | Hand-eye rotation (rad) -- **placeholder** |
| `target_frame` | `base_link` | TF frame for MoveIt output |
| `latch_window_s` | `10.0` | Calibration collection window (s) |
| `brush_standoff_m` | `0.005` | Brush hover above canvas (m) |
| `aruco_win_max` | `23` | ArUco threshold passes (23=fast, 53=balanced, 83=robust) |
| `clahe_clip` | `4.0` | CLAHE contrast limit (raise for dark environments) |
| `marker_size_m` | `0.050` | Printed AprilTag size (m) |
| `enable_pose_vis` | `false` | Start pose visualiser (extra CPU) |

### `canvas_sim_pipeline.launch.py`

| Argument | Default | Description |
|---|---|---|
| `canvas_x`, `canvas_y`, `canvas_z` | `0.0, 0.0, 0.5` | Synthetic canvas position (m) |
| `canvas_roll_deg`, `canvas_pitch_deg`, `canvas_yaw_deg` | `0.0, 0.0, 0.0` | Synthetic canvas orientation (deg) |
| `animate` | `false` | Enable sinusoidal animation |
| `animate_axis` | `yaw` | Axis to animate (`x`, `y`, `z`, `roll`, `pitch`, `yaw`) |
| `animate_range` | `30.0` | Animation half-amplitude |
| `animate_period_s` | `4.0` | Oscillation period (s) |
| `latch_window_s` | `10.0` | Calibration window |
| `brush_standoff_m` | `0.005` | Brush standoff |

### `ur3e_unity_bridge.launch.py`

| Argument | Default | Description |
|---|---|---|
| `ur_type` | `ur3e` | Robot model |
| `robot_ip` | `127.0.0.1` | UR controller IP |
| `use_fake_hardware` | `true` | Use mock hardware (no real robot) |
| `initial_joint_controller` | `scaled_joint_trajectory_controller` | Controller type |
| `ros_ip` | `127.0.0.1` | IP for ROS-TCP-Endpoint |
| `ros_tcp_port` | `10000` | TCP port for Unity connection |
| `moveit_launch_rviz` | `true` | Launch RViz with MoveIt |
