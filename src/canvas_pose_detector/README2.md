# Camera Perception Interface -- Unity GUI Integration Guide


This document tells you everything you need to connect your Unity GUI to the
camera perception subsystem, both with and without physical hardware.
You do not need a camera, a real canvas, or a robot to begin GUI development.

---

## Quick Answer: What Topics Does Unity Send and Receive?

### Unity SENDS (you publish from Unity via ROS-TCP-Connector)

```
Topic:      /canvas/stroke_target
Type:       geometry_msgs/msg/PoseStamped
frame_id:   canvas_centre_assumed
```

Each message = one target point on the canvas the brush should move to.

| Field | Meaning | Range |
|---|---|---|
| `pose.position.x` | Horizontal position from canvas centre (m), right = + | -0.148 to +0.148 |
| `pose.position.y` | Vertical position from canvas centre (m), up = + | -0.105 to +0.105 |
| `pose.position.z` | Standoff hover distance above canvas (m) | 0.005 (fixed, do not change) |
| `pose.orientation` | Tool orientation (identity = brush pointing into canvas) | leave as identity |

The canvas frame matches A4 landscape (297 mm wide x 210 mm tall). The origin
is the canvas centre. Think of it as a 2D drawing canvas where x/y are the
brush position in metres from the middle of the page.

### Unity RECEIVES (you subscribe in Unity)

```
Topic:      /canvas/correction_data
Type:       std_msgs/msg/String   (JSON payload)
```

Parse this JSON every frame. It tells you whether the canvas is ready and how
accurately the camera is tracking it.

```json
{
  "status": "OK",
  "pose_age_ms": 35.0,
  "latency_ms": 3.7,
  "error": {
    "lateral_mm": -2.1,
    "vertical_mm": 1.4,
    "depth_mm": -0.8,
    "rotation_deg": 0.3,
    "total_3d_mm": 2.6
  }
}
```

| `status` value | What it means | What Unity should do |
|---|---|---|
| `WAITING` | No canvas detected yet | Show "Searching for canvas..." |
| `COLLECTING` | Calibrating -- 10s window | Show countdown, disable draw button |
| `PAUSED` | Canvas lost (occluded >5s) | Pause strokes, show warning |
| `OK` | Fully calibrated, live tracking | Enable draw button |

**Rule: only publish `/canvas/stroke_target` when `status == "OK"`.**

---

## Canvas Coordinate System

```
         canvas_centre_assumed frame

              -Y  (top of canvas)
               |
   -X ────── [0,0] ────── +X  (right)
    (left)     |          (right)
              +Y  (bottom of canvas)

   +Z = out of canvas face (toward camera -- away from drawing surface)
   -Z = into canvas

   A4 landscape physical size:
     width  (X axis): 297 mm total  -->  x in [-0.148, +0.148] m
     height (Y axis): 210 mm total  -->  y in [-0.105, +0.105] m
```

When the user draws a point at the top-left of the canvas in Unity, send:
```
position.x = -0.148
position.y = -0.105
position.z =  0.005
```

---

## ROS-TCP-Connector Setup

The `ros_tcp_endpoint` package is already in the repository under `src/`.

**In Unity (Robotics > ROS Settings):**
```
ROS IP Address:  <Ubuntu machine IP on your local network>
ROS Port:        10000
```

To find the Ubuntu IP:
```bash
hostname -I | awk '{print $1}'
```

The endpoint starts automatically as part of the pipeline launch -- you do not
need to start it separately.

---

## Mode A: Development Without Camera or Canvas (Recommended to Start)

Use this mode to build and test your Unity GUI without any hardware.
A synthetic canvas pose is published by the injector node.
The full correction + status pipeline runs exactly as in production.

### What to run on Ubuntu (one terminal)

```bash
# Source the workspace first (do this once per terminal)
source ~/RS2/XR_Teleoperation/install/setup.bash

# Start the full camera-free pipeline
ros2 launch canvas_pose_detector canvas_sim_pipeline.launch.py
```

This starts:
- `canvas_pose_injector` -- synthetic canvas at 0.5m in front of camera, facing forward
- `canvas_stroke_corrector` -- 10s calibration window, then publishes `OK`
- `canvas_depth_visualiser` -- XZ depth plot (optional debug view)
- `canvas_pose_visualiser` -- 2D lateral plot (optional debug view)

After ~12 seconds (10s calibration + startup), `status` will change to `OK`
and your GUI draw button should enable.

### Verify it is working

```bash
# Watch status in real time
ros2 topic echo /canvas/correction_data

# Confirm stroke_target is being received (after you publish from Unity)
ros2 topic echo /canvas/stroke_target
```

### Move the synthetic canvas (test your GUI response)

Without restarting anything, you can change the canvas position to test that
your GUI updates correctly:

```bash
# Move canvas farther away
ros2 param set /canvas_pose_injector canvas_z 0.7

# Tilt canvas 20 degrees (test orientation display)
ros2 param set /canvas_pose_injector canvas_yaw_deg 20.0

# Sweep canvas left/right continuously (test live tracking UI)
ros2 param set /canvas_pose_injector animate true
ros2 param set /canvas_pose_injector animate_axis yaw
ros2 param set /canvas_pose_injector animate_range 25.0
ros2 param set /canvas_pose_injector animate_period_s 4.0

# Stop animation
ros2 param set /canvas_pose_injector animate false

# Reset to centre
ros2 param set /canvas_pose_injector canvas_yaw_deg 0.0
```

### Simulate canvas going missing (test PAUSED state in GUI)

```bash
# Pause the injector -- status will go to PAUSED after 5 seconds
ros2 param set /canvas_pose_injector publish_rate_hz 0.0

# Restore
ros2 param set /canvas_pose_injector publish_rate_hz 10.0
```

### Trigger a re-calibration (test COLLECTING state in GUI)

```bash
# Restart just the corrector -- it will go back through the 10s window
ros2 run canvas_pose_detector canvas_stroke_corrector
```

---

## Mode B: Full Pipeline With Real Camera and Canvas

Use this mode for integration testing once hardware is available.

### Prerequisites (Ubuntu side)

- Intel RealSense D435if connected via USB 3
- AprilTag markers printed at 50mm, attached to canvas corners:
  ```
  [ID:0  TL] ─────── [ID:1  TR]
      |    A4 Canvas    |
  [ID:3  BL] ─────── [ID:2  BR]
  ```
- UR3e robot running with driver (or URSim for mock)

### What to run on Ubuntu

```bash
# Terminal 1 -- RealSense camera
ros2 launch realsense2_camera rs_launch.py \
  enable_sync:=true align_depth.enable:=true

# Terminal 2 -- Full pipeline (detector + corrector + visualisers)
ros2 launch canvas_pose_detector canvas_real_pipeline.launch.py
```

### What happens

| Time | Event |
|---|---|
| t=0s | Detector starts. Begins detecting AprilTag markers |
| t=4s | Corrector starts. `status = COLLECTING` for 10 seconds |
| t=14s | Corrector latches. `status = OK`. GUI draw button enables |

Keep the canvas completely still during the 10-second COLLECTING window.
After that, the canvas can be moved and the system tracks the drift.

### Optional -- open all visualisation tools

```bash
# Terminal 3 -- RViz + all rqt plots
ros2 launch canvas_pose_detector canvas_visualise_all.launch.py
```

This shows the robot model, all TF frames, camera overlay, 2D error plot,
and depth view. Useful for integration debugging.

---

## Unity GUI Recommendations (Based on Contract)

From the contract (Subsystem 3 -- Simulation Environment):

**Pass (P) -- minimum to achieve:**
- Display virtual canvas in correct A4 proportions in the Unity scene
- Show system status from `/canvas/correction_data` (`status` field)
- Disable stroke input when `status != "OK"`
- Receive and display live camera feed (available on `/canvas/debug` as ROS Image)

**Credit (C):**
- Mode-based interaction: separate "calibrating" state vs "ready to draw" state
- Show countdown during `COLLECTING` state (use `remaining_s` from JSON)

**Distinction (D) and above:**
- Show real-time canvas drift indicator (use `error.total_3d_mm` from JSON)
- Show canvas position relative to robot in the virtual scene (use TF frame
  `canvas_centre_assumed` via ROS-TCP-Connector TF subscription)
- Show warning overlay when `status == "PAUSED"` (canvas lost)

---

## Summary of All Relevant Topics

| Topic | Direction | Type | Purpose |
|---|---|---|---|
| `/canvas/stroke_target` | Unity -> ROS | PoseStamped | Brush target point on canvas |
| `/canvas/correction_data` | ROS -> Unity | String (JSON) | Status + all error values |
| `/canvas/debug` | ROS -> Unity | Image | Raw camera feed with marker overlay |
| `/canvas/pose` | ROS internal | PoseStamped | Raw canvas pose (camera frame) |
| `/canvas/z_constraint` | ROS -> Stephen | PoseStamped | Z depth for robot control (not Unity) |

You only need `/canvas/stroke_target` and `/canvas/correction_data` for the
GUI. The others are internal to the perception and motion planning subsystems.

---

## Contact

If the ROS-TCP-Connector drops connection or topics are missing, first check:

```bash
# Is the endpoint running?
ros2 node list | grep tcp

# Is the pipeline running?
ros2 node list | grep canvas

# What is the current status?
ros2 topic echo /canvas/correction_data --once
```

Raise any interface changes (topic names, message fields, coordinate frame
changes) with Johan before implementing, as they affect the full pipeline.