# canvas_pose_detector -- Member 3: Camera Perception

**RS2 XR Teleoperation | UR3e Drawing Robot**

Detects an A4 canvas using AprilTag 36h11 corner markers, publishes a 6-DOF canvas pose, and applies real-time SE3 pose error correction so that drawing strokes land accurately on the physical canvas surface.

---

## Team

| Member | Role |
|---|---|
| Member 1 -- Nhut Han Pham | Unity / VR interface, stroke path planning |
| Member 2 -- Stephen Suiwinata | MoveIt Servo, robot motion control |
| Member 3 -- Johan (this package) | Camera perception, pose estimation, error correction |

**Stack:** Ubuntu 22.04, ROS2 Humble, UR3e, Intel RealSense D435i, OpenCV 4.5.4, MoveIt 2, Meta Quest 2

---

## Package Layout

```
canvas_pose_detector/
├── CMakeLists.txt
├── package.xml
├── README.md
│
├── src/
│   ├── canvas_pose_detector.cpp        C++ node -- AprilTag detection, pose estimation
│   ├── canvas_pose_visualiser.py       rqt 2D/3D pose plots
│   ├── canvas_pose_injector.py         Synthetic pose publisher (no camera needed)
│   ├── canvas_pose_follower.py         MoveIt EEF follower (validation only)
│   ├── canvas_stroke_corrector.py      SE3 error correction algorithm
│   └── canvas_correction_visualiser.py rqt correction overlay plots
│
├── launch/
│   ├── canvas_pose_detector.launch.py      Real camera pipeline
│   ├── canvas_pose_injector.launch.py      Injector only
│   ├── canvas_stroke_corrector.launch.py   Corrector only
│   ├── canvas_pose_validation.launch.py    Injector + MoveIt follower (UR3e validation)
│   └── canvas_full_pipeline.launch.py      Full stack (camera + correction)
│
└── config/
    └── rviz_config.rviz                    Pre-configured RViz layout
```

---

## Physical Setup

Print four AprilTag 36h11 markers at https://chev.me/arucogen/
- Dictionary: AprilTag 36h11
- Size: 50mm
- IDs: 0 (TL), 1 (TR), 2 (BR), 3 (BL)

Attach to corners of A4 canvas (landscape):
```
[ID:0 TL] ─────────── [ID:1 TR]
    |      A4 canvas       |
[ID:3 BL] ─────────── [ID:2 BR]
```

---

## Dependencies

### One-time apt install

```bash
sudo apt install \
  ros-humble-cv-bridge \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-message-filters \
  ros-humble-realsense2-camera \
  libopencv-dev \
  libeigen3-dev
```

### Workspace source (add to ~/.bashrc)

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Build

```bash
# Create config directory if it does not exist
mkdir -p ~/ros2_ws/src/canvas_pose_detector/config

cd ~/ros2_ws
colcon build --packages-select canvas_pose_detector
source install/setup.bash
```

---

## Pipeline A: Full Real-Camera Pipeline

This is the main pipeline for lab use with the physical RealSense camera and canvas.

### Terminal 1 -- RealSense camera driver

```bash
ros2 launch realsense2_camera rs_launch.py \
  enable_sync:=true align_depth.enable:=true
```

Wait until camera topics appear:
```bash
ros2 topic list | grep /camera
```

### Terminal 2 -- Canvas pose detector

```bash
ros2 launch canvas_pose_detector canvas_pose_detector.launch.py
```

Hold the canvas in front of the camera. You should see:
```
[POSE] XYZ: (x=+0.009  y=+0.022  z=+0.325) m | Roll=+0.0 H=+1.4 V=-8.3 deg | markers=4/4
```

### Terminal 3 -- Stroke corrector

```bash
ros2 launch canvas_pose_detector canvas_stroke_corrector.launch.py
```

On first detection the assumed pose is auto-latched:
```
[CORRECTOR] Assumed pose latched from first detection:
  t=(+0.009, +0.022, +0.325) m
```

### Terminal 4 -- Correction visualiser

```bash
ros2 run canvas_pose_detector canvas_correction_visualiser
```

### Terminal 5 -- rqt image viewers (open two instances)

```bash
ros2 run rqt_image_view rqt_image_view   # select /canvas/debug_corrected
ros2 run rqt_image_view rqt_image_view   # select /canvas/correction_2d
```

### Terminal 6 -- Send a stroke target (canvas-frame metres from centre)

```bash
# Canvas centre
ros2 topic pub /canvas/stroke_target geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'canvas_centre_assumed'},
    pose: {position: {x: 0.0, y: 0.0, z: 0.005},
           orientation: {w: 1.0}}}" --rate 10

# Top-left corner of A4 canvas
ros2 topic pub /canvas/stroke_target geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'canvas_centre_assumed'},
    pose: {position: {x: -0.148, y: 0.105, z: 0.005},
           orientation: {w: 1.0}}}" --rate 10
```

### Optional -- rqt pose visualiser (2D/3D plots)

```bash
python3 ~/ros2_ws/src/canvas_pose_detector/src/canvas_pose_visualiser.py
ros2 run rqt_image_view rqt_image_view   # select /canvas/pose_2d_xz or _xy
```

---

## Pipeline B: Camera-Free Validation (no RealSense needed)

Uses the synthetic pose injector to simulate canvas detection. Useful for testing the correction algorithm and MoveIt integration on any machine.

### Terminal 1 -- UR3e mock hardware + MoveIt + RViz

```bash
ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur3e \
  use_mock_hardware:=true \
  rviz_config:=$(ros2 pkg prefix canvas_pose_detector)/share/canvas_pose_detector/config/rviz_config.rviz
```

Wait for: `You can start planning now!`

### Terminal 2 -- Injector + follower

```bash
ros2 launch canvas_pose_detector canvas_pose_validation.launch.py
```

Default: horizontal A4 canvas at x=0.3, z=0.0, EEF hovers 40cm above.

### Live controls (no relaunch needed)

```bash
# Move canvas
ros2 param set /canvas_pose_injector canvas_x 0.3
ros2 param set /canvas_pose_injector canvas_y 0.0
ros2 param set /canvas_pose_injector canvas_z 0.0

# Change EEF standoff height
ros2 param set /canvas_pose_follower standoff_m 0.20

# Tilt canvas
ros2 param set /canvas_pose_injector canvas_pitch_deg 25.0

# Animate canvas sweep (watch robot track it)
ros2 param set /canvas_pose_injector animate true
ros2 param set /canvas_pose_injector animate_axis yaw
ros2 param set /canvas_pose_injector animate_range 20.0
ros2 param set /canvas_pose_injector animate_period_s 8.0

# Pause robot
ros2 param set /canvas_pose_follower enabled false
```

---

## Pipeline C: Full Stack Single Launch

Launches camera driver, detector, corrector, and correction visualiser in one command.

```bash
ros2 launch canvas_pose_detector canvas_full_pipeline.launch.py
```

Then in separate terminals:

```bash
# Stroke target
ros2 topic pub /canvas/stroke_target geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'canvas_centre_assumed'},
    pose: {position: {x: 0.0, y: 0.0, z: 0.005},
           orientation: {w: 1.0}}}" --rate 10

# rqt viewers
ros2 run rqt_image_view rqt_image_view   # /canvas/debug_corrected
ros2 run rqt_image_view rqt_image_view   # /canvas/correction_2d
```

---

## Error Correction Algorithm

The core algorithm corrects for physical canvas placement tolerances.

```
T_real    = camera → real canvas centre     (live, from detector)
T_assumed = camera → assumed canvas centre  (fixed, latched on first detection)

SE3 error:
  R_err = R_assumed^T * R_real
  t_err = R_assumed^T * (t_real - t_assumed)

  t_err.x = lateral error   (canvas X, right = +)   in metres
  t_err.y = vertical error  (canvas Y, up    = +)   in metres
  t_err.z = depth error     (canvas Z, toward camera = +)  in metres

Stroke correction:
  t_corrected = R_real * R_assumed^T * t_stroke + t_real
  R_corrected = R_real * R_assumed^T * R_stroke
```

The corrected pose is published as `PoseStamped` for direct consumption by MoveIt Servo (Member 2).

To validate: physically move the canvas slightly after latching. The green assumed origin stays fixed in the 2D plot. The red real origin drifts. The orange arrow between them shows the active correction.

---

## Topics Reference

### Published by canvas_pose_detector (C++ node)

| Topic | Type | Description |
|---|---|---|
| `/canvas/pose` | `PoseStamped` | Canvas 6-DOF pose in `camera_color_optical_frame` |
| `/canvas/viewing_angles` | `Vector3Stamped` | x=Roll, y=H, z=V in degrees |
| `/canvas/markers` | `MarkerArray` | RViz: canvas plane, corners, axes, RPY arcs |
| `/canvas/debug` | `Image` | Camera feed with marker overlays (throttled) |
| `/tf` | TF | `canvas_centre`, `canvas_marker_TL/TR/BR/BL` |

### Published by canvas_stroke_corrector

| Topic | Type | Description |
|---|---|---|
| `/canvas/corrected_stroke_pose` | `PoseStamped` | Corrected EEF pose for MoveIt Servo |
| `/canvas/pose_error` | `PoseStamped` | SE3 error (position=translation, orientation=rotation) |
| `/canvas/correction_debug` | `String` | Human-readable correction summary |

### Published by canvas_correction_visualiser

| Topic | Type | Description |
|---|---|---|
| `/canvas/debug_corrected` | `Image` | Camera feed with real/assumed origins + error |
| `/canvas/correction_2d` | `Image` | Canvas-frame 2D plot with drift arrow |

### Subscribed by canvas_stroke_corrector

| Topic | Type | Source | Description |
|---|---|---|---|
| `/canvas/pose` | `PoseStamped` | Detector | Live canvas pose |
| `/canvas/pose_assumed` | `PoseStamped` | External / auto-latched | Reference canvas pose |
| `/canvas/stroke_target` | `PoseStamped` | Unity (Member 1) | Desired stroke in assumed canvas frame |

---

## Stroke Target Format (for Member 1 -- Unity)

Publish `PoseStamped` on `/canvas/stroke_target`:

```
frame_id:       "canvas_centre_assumed"
position.x:     horizontal position on canvas (metres from centre, right = +)
position.y:     vertical position on canvas   (metres from centre, up   = +)
position.z:     brush standoff above surface  (metres, 0.005 = 5mm hover)
orientation:    desired tool orientation in canvas frame
                identity (w=1) = tool-Z points into canvas face
```

A4 canvas bounds: x in [-0.148, +0.148], y in [-0.105, +0.105]

---

## Interface with Member 2 (MoveIt Servo)

Subscribe to `/canvas/corrected_stroke_pose` (PoseStamped).

This pose is:
- Expressed in `camera_color_optical_frame` (change `planning_frame` param to `base_link` after hand-eye calibration)
- Orientation already flipped so tool-Z points into the canvas face
- Low-pass filtered and step-clamped (no sudden jumps)
- Updated at 50Hz

To switch to `base_link` output once hand-eye TF is available:

```bash
ros2 param set /canvas_stroke_corrector planning_frame base_link
```

---

## Key Parameters (live-tunable, no rebuild)

### canvas_pose_detector

```bash
ros2 param set /canvas_pose_detector clahe_clip 6.0        # raise for dark rooms
ros2 param set /canvas_pose_detector depth_sample_half 16   # raise at >1.5m range
ros2 param set /canvas_pose_detector missing_frames_tol 10  # frames before marker lost
```

### canvas_stroke_corrector

```bash
ros2 param set /canvas_stroke_corrector correction_alpha 0.5   # faster response
ros2 param set /canvas_stroke_corrector brush_standoff_m 0.002 # closer to canvas
ros2 param set /canvas_stroke_corrector enabled false          # freeze output
ros2 param set /canvas_stroke_corrector planning_frame base_link  # for Member 2
```

### Override assumed pose (re-latch)

```bash
# Publish current real pose as the new reference
ros2 topic pub /canvas/pose_assumed geometry_msgs/msg/PoseStamped \
  "$(ros2 topic echo /canvas/pose --once)" --once
```

---

## Known Issues and Pending Work

| Item | Status |
|---|---|
| Hand-eye calibration (camera to base_link) | Not started -- static TF workaround in place |
| Close-range marker occlusion at <20cm | Deferred -- wrist cam misses corners at drawing distance |
| Continuous drift correction over 10+ min sessions | Implemented (50Hz correction loop) |
| Camera B PointCloud2 into MoveIt collision scene | Not started |
| Ground truth accuracy re-test with current code | Pending |
| rqt visualiser at 30fps | Low priority (currently 10fps) |

---

## Troubleshooting

**Markers not detected**
- Raise `clahe_clip` to 6-8 for dark environments
- Ensure markers are printed at exactly 50mm
- Check `aruco_dict` matches printed markers (must be `DICT_APRILTAG_36h11`)

**Depth invalid**
- Raise `depth_sample_half` to 16-20 at distances > 1.5m
- Move canvas closer to camera (optimal range: 0.3m - 1.5m)

**Correction always 0mm**
- The assumed pose was just latched -- move the canvas slightly to see error
- Check `/canvas/correction_debug`: `pose_age` should be < 500ms

**corrected_stroke_pose not publishing**
- Check pose age in debug topic: if > `pose_timeout_s` the corrector pauses
- Ensure `/canvas/stroke_target` is being published

**TF lookup failed (base_link)**
- Switch planning_frame back to camera frame: `ros2 param set /canvas_stroke_corrector planning_frame camera_color_optical_frame`
- Hand-eye calibration TF must be running for base_link output

**Disk full during colcon build**
```bash
rm -rf ~/ros2_ws/log/* ~/.ros/log/*
sudo apt clean
df -h
```