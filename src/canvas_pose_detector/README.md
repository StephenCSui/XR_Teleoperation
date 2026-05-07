# canvas_pose_detector

ROS2 Humble package for **Member 3 (Camera Perception)** of the RS2 XR Teleoperation project.

Detects an A4 canvas in 3D space using a wrist-mounted Intel RealSense D435if and AprilTag markers.
Publishes the canvas pose, SE3 correction error, and brush Z depth constraint for MoveIt Servo.

---

## Project Context

| Role | Member | Responsibility |
|---|---|---|
| Member 1 | Nhut Han Pham | Unity / Meta Quest 2 VR interface, stroke path planning |
| Member 2 | Stephen Suiwinata | MoveIt Servo, UR3e motion control |
| Member 3 (this package) | Johan | Camera perception, canvas pose estimation, SE3 correction |

**Stack:** Ubuntu 22.04 · ROS2 Humble · UR3e · Intel RealSense D435if · OpenCV 4.5.4 · MoveIt 2 · Meta Quest 2 · Unity 2022.3 LTS

---

## Package Layout

```
canvas_perception/
├── launch/
│   ├── canvas_real_pipeline.launch.py      Master: real camera (2 terminals total)
│   ├── canvas_sim_pipeline.launch.py       Master: no camera needed (1 terminal)
│   ├── canvas_moveit_validation.launch.py  UR3e MoveIt follower validation
│   ├── canvas_tf_bridge.launch.py          TF bridge + corrector only
│   ├── canvas_pose_detector.launch.py      Detector node only
│   ├── canvas_pose_injector.launch.py      Injector node only
│   ├── canvas_stroke_corrector.launch.py   Corrector node only
│   └── canvas_pose_validation.launch.py    Injector + follower (legacy)
├── src/
│   ├── canvas_pose_detector.cpp            C++ detection node
│   ├── canvas_tf_bridge.py                 PoseStamped frame transformer
│   ├── canvas_stroke_corrector.py          SE3 error correction
│   ├── canvas_correction_visualiser.py     Camera overlay + 2D error plot
│   ├── canvas_depth_visualiser.py          XZ side-view depth visualiser
│   ├── canvas_pose_visualiser.py           2D/3D pose plots
│   ├── canvas_pose_injector.py             Synthetic pose publisher (no camera)
│   └── canvas_pose_follower.py             MoveIt EEF follower (validation)
├── config/
│   └── rviz_config.rviz
├── CMakeLists.txt
└── package.xml
```

---

## Quick Start

### Real Camera Pipeline (production)

**Terminal 1** -- RealSense driver (always in its own terminal):
```bash
ros2 launch realsense2_camera rs_launch.py \
  enable_sync:=true align_depth.enable:=true
```

**Terminal 2** -- Everything else:
```bash
ros2 launch canvas_pose_detector canvas_real_pipeline.launch.py
```

Optional overrides:
```bash
ros2 launch canvas_pose_detector canvas_real_pipeline.launch.py \
  latch_window_s:=15.0 \
  brush_standoff_m:=0.010 \
  aruco_win_max:=53 \
  clahe_clip:=6.0
```

View output in rqt (select topic from the dropdown):
```bash
ros2 run rqt_image_view rqt_image_view
```
Topics to view: `/canvas/correction_2d`, `/canvas/depth_view`, `/canvas/debug_corrected`

---

### Camera-Free Simulation (no hardware needed)

**Terminal 1** -- Everything:
```bash
ros2 launch canvas_pose_detector canvas_sim_pipeline.launch.py
```

With animation (canvas sweeps left/right, watch visualisers respond):
```bash
ros2 launch canvas_pose_detector canvas_sim_pipeline.launch.py \
  animate:=true animate_axis:=yaw animate_range:=25.0 animate_period_s:=4.0
```

---

### UR3e MoveIt Validation (requires URSim or real robot + MoveIt)

**Terminal 1** -- UR MoveIt:
```bash
ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur3e use_mock_hardware:=true \
  rviz_config:=$(ros2 pkg prefix canvas_pose_detector)/share/canvas_pose_detector/config/rviz_config.rviz
```

**Terminal 2** -- Injector + EEF follower:
```bash
ros2 launch canvas_pose_detector canvas_moveit_validation.launch.py
```

---

## Build

```bash
cd ~/RS2/XR_Teleoperation
colcon build --packages-select canvas_perception --symlink-install
source install/setup.bash
```

---

## Pipeline Architecture

```
RealSense D435if
     |
     | /camera/camera/color/image_raw
     | /camera/camera/depth/image_rect_raw
     | /camera/camera/color/camera_info
     v
canvas_pose_detector (C++, ~18 fps)
     |
     | /canvas/pose          PoseStamped  (camera_color_optical_frame)
     | /canvas/viewing_angles Vector3Stamped  (Roll/H/V degrees)
     | /canvas/markers       MarkerArray  (RViz)
     | /canvas/debug         Image        (annotated camera feed)
     | TF: canvas_centre, canvas_marker_TL/TR/BR/BL
     v
canvas_tf_bridge
     |
     | /canvas/pose_base     PoseStamped  (base_link)      <- primary
     | /canvas/pose_tool0    PoseStamped  (tool0)          <- verification
     | TF: canvas_centre_base
     v
canvas_stroke_corrector (50 Hz output)
     |
     | /canvas/z_constraint  PoseStamped  (base_link)      -> Member 2
     | /canvas/pose_error    PoseStamped  (canvas_centre_assumed)
     | /canvas/pose_assumed  PoseStamped  (latched reference)
     | /canvas/correction_data String     (JSON status)
     | TF: canvas_centre_assumed
```

For camera-free simulation, `canvas_pose_injector` replaces the detector and
publishes identical topics directly to `/canvas/pose`.

---

## All Topics

### Published by canvas_pose_detector

| Topic | Type | Description |
|---|---|---|
| `/canvas/pose` | `PoseStamped` | 6-DOF canvas centre in `camera_color_optical_frame` |
| `/canvas/viewing_angles` | `Vector3Stamped` | x=Roll y=H z=V in degrees |
| `/canvas/markers` | `MarkerArray` | RViz: plane, corners, RPY arcs, axes |
| `/canvas/debug` | `Image` | Camera feed with marker overlays (10 fps) |
| TF `canvas_centre` | -- | Canvas centre frame, child of `camera_color_optical_frame` |
| TF `canvas_marker_TL/TR/BR/BL` | -- | Per-marker frames |

### Published by canvas_tf_bridge

| Topic | Type | Description |
|---|---|---|
| `/canvas/pose_base` | `PoseStamped` | Canvas pose in `base_link` (primary) |
| `/canvas/pose_tool0` | `PoseStamped` | Canvas pose in `tool0` (verification) |
| TF `canvas_centre_base` | -- | Canvas centre in `base_link` |

### Published by canvas_stroke_corrector

| Topic | Type | Description |
|---|---|---|
| `/canvas/z_constraint` | `PoseStamped` | Brush Z constraint for MoveIt Servo |
| `/canvas/pose_error` | `PoseStamped` | SE3 error in `canvas_centre_assumed` |
| `/canvas/pose_assumed` | `PoseStamped` | Latched reference pose |
| `/canvas/correction_data` | `String` | JSON with all error values + latency |
| TF `canvas_centre_assumed` | -- | Latched reference frame |

### Published by visualisers

| Topic | Type | Node | Description |
|---|---|---|---|
| `/canvas/correction_2d` | `Image` | correction_visualiser | Top-down +-150mm 2D error plot |
| `/canvas/debug_corrected` | `Image` | correction_visualiser | Camera overlay with drift arrow |
| `/canvas/depth_view` | `Image` | depth_visualiser | XZ side-view canvas plane + brush |
| `/canvas/pose_2d_xy` | `Image` | pose_visualiser | X vs Y lateral plot |
| `/canvas/pose_2d_xz` | `Image` | pose_visualiser | X vs Z bird's eye plot |
| `/canvas/pose_3d_tf` | `Image` | pose_visualiser | 3D TF frame illustration |

---

## Interface with Member 2 (MoveIt Servo)

Primary topic: `/canvas/z_constraint` (PoseStamped, frame: `base_link`)

```
position.x/y  = live canvas centre lateral position in base_link
position.z    = canvas surface depth MINUS standoff (brush tip Z)
orientation   = canvas face normal quaternion (tool must stay perpendicular)
```

Member 2 locks EEF Z to `msg.pose.position.z` while allowing XY freedom for strokes.

After hand-eye calibration is complete, the frame is already `base_link` -- no additional configuration needed from Member 2.

---

## Interface with Member 1 (Unity / VR)

Member 1 publishes stroke targets (currently not consumed by the corrector -- corrector is purely validation):

```
Topic:    /canvas/stroke_target   PoseStamped
frame_id: canvas_centre_assumed
position.x/y:  stroke point on canvas (metres from centre)
position.z:    standoff (0.005 = 5mm hover)
orientation:   tool orientation in canvas frame (identity = into canvas)
```

A4 bounds: `x ∈ [-0.148, +0.148]`, `y ∈ [-0.105, +0.105]`

---

## Key Parameters

### canvas_pose_detector

| Parameter | Default | Description |
|---|---|---|
| `aruco_win_max` | 23 | ArUco threshold passes. 23=fast(18fps) 53=balanced 83=robust |
| `aruco_win_step` | 10 | Window size step |
| `clahe_clip` | 4.0 | CLAHE contrast limit. Raise to 8.0 for dark environments |
| `marker_size_m` | 0.050 | Printed AprilTag size in metres |
| `canvas_physical_w_m` | 0.297 | A4 landscape width |
| `canvas_physical_h_m` | 0.210 | A4 landscape height |
| `missing_frames_tol` | 5 | Frames a marker can be absent before lost |
| `depth_sample_half` | 12 | Depth sampling radius in pixels |
| `rpy_offset_roll` | 133.0 | Roll offset for display only (not pose output) |
| `rpy_offset_yaw` | -176.8 | Yaw offset for display only |

### canvas_tf_bridge

| Parameter | Default | Description |
|---|---|---|
| `source_topic` | `/canvas/pose` | Input PoseStamped topic |
| `target_frame` | `base_link` | Primary output TF frame |
| `output_topic` | `/canvas/pose_base` | Primary output topic |
| `secondary_frame` | `tool0` | Secondary output frame (verification) |
| `tf_timeout_s` | 0.10 | TF lookup timeout |

### canvas_stroke_corrector

| Parameter | Default | Description |
|---|---|---|
| `source_topic` | `/canvas/pose_base` | Input topic (must match bridge output) |
| `latch_window_s` | 10.0 | Collection window before latching T_assumed |
| `brush_standoff_m` | 0.005 | Brush hover above canvas surface |
| `publish_rate_hz` | 50.0 | Output rate (matches MoveIt Servo) |
| `pose_timeout_s` | 5.0 | Max pose age before PAUSED status |
| `enabled` | true | False = freeze output without killing node |

---

## Live Tuning (no restart needed)

```bash
# Detection -- raise aruco_win_max if markers fail at distance
ros2 param set /canvas_pose_detector aruco_win_max 53
ros2 param set /canvas_pose_detector clahe_clip    6.0

# Corrector -- adjust standoff for drawing vs hover
ros2 param set /canvas_stroke_corrector brush_standoff_m 0.010
ros2 param set /canvas_stroke_corrector enabled false   # pause output

# Injector (sim only) -- move synthetic canvas
ros2 param set /canvas_pose_injector canvas_z      0.7
ros2 param set /canvas_pose_injector canvas_yaw_deg 30.0
ros2 param set /canvas_pose_injector animate       true
```

---

## AprilTag Marker Layout

Print AprilTag 36h11 markers at https://chev.me/arucogen/
(Dictionary: AprilTag 36h11, Size: 50mm minimum for reliable detection at 1.5m)

```
[ID:0  TL] ─────────── [ID:1  TR]
    |                       |
    |       A4 Canvas        |
    |      297 x 210 mm      |
    |                       |
[ID:3  BL] ─────────── [ID:2  BR]
```

Stick markers at the four corners on the front face of the canvas.
The canvas black tape border is the recommended physical boundary.

---

## Partial Detection

The detector recovers from missing markers before failing:

| Markers visible | Method | Notes |
|---|---|---|
| 4 | Normal | Full detection |
| 3 | Parallelogram rule | Exact reconstruction (TL+BR = TR+BL) |
| 2 adjacent | A4 aspect ratio | Rotate edge vector 90 deg, scale by H/W |
| 2 diagonal | Rejected | Cannot reconstruct without orientation |
| 1 or 0 | Rejected | Insufficient |

Additionally, markers missing for fewer than `missing_frames_tol` frames (default 5) are held at their last known position before reconstruction is attempted.

---

## SE3 Error Algorithm

```
T_real    = camera -> real canvas centre    (live from /canvas/pose)
T_assumed = camera -> assumed canvas centre (averaged + latched)

R_err = R_assumed^T * R_real
t_err = R_assumed^T * (t_real - t_assumed)

t_err.x = lateral error   (canvas X, right = +)  metres
t_err.y = vertical error  (canvas Y, up    = +)  metres
t_err.z = depth error     (canvas Z, toward camera = +)  metres
```

The 10-second averaging latch collects N frames, averages translations (mean) and quaternions (eigenvector method, Markley 2007 -- handles sign ambiguity). Keep canvas still during this window.

---

## Hand-Eye Calibration Status

**Not yet complete.** The static TF `flange -> camera_color_optical_frame` uses placeholder values:

```
x=0.0  y=0.09  z=0.0  roll=-0.785  pitch=0.0  yaw=0.0
```

Replace these with measured values in `canvas_real_pipeline.launch.py` before real robot use. After calibration, update the `cam_x/y/z/cam_roll/pitch/yaw` arguments.

---

## Known Issues

| Issue | Status |
|---|---|
| Hand-eye calibration (camera to base_link) | Not started -- placeholder TF in use |
| Close-range corner marker occlusion (<20cm wrist cam) | Deferred |
| Diagonal 2-marker pair cannot reconstruct | By design -- logs warning |
| canvas_correction_visualiser needs live camera | Does not run in sim pipeline |

---

## Key Architecture Decisions

- **ArUco speed:** `aruco_win_max=100 step=4` gives 25 passes and 0.2fps. Use `max=23 step=10` (3 passes, 18fps). Exposed as live parameters.
- **Viewing angles:** Computed from depth-measured canvas normal via cross product of edge vectors, NOT from solvePnP quaternion RPY decomposition (axis coupling causes 35deg pitch to read as 4deg).
- **MoveIt action server:** In this UR Humble config the action is `/move_action`, not `/move_group`. `wait_for_server()` from background threads deadlocks -- use executor-side `server_is_ready()` timer instead.
- **SE3 error:** `T_error = inv(T_assumed) * T_real`, not naive subtraction.
- **Quaternion averaging:** Eigenvector method (Markley 2007), not element-wise average.
- **Frame agnosticism:** The corrector reads `frame_id` from incoming messages. No frame names are hardcoded anywhere in Python nodes.