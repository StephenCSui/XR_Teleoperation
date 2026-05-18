# Camera Perception -- Unity VR GUI Integration Guide

---

## Overview

The camera perception subsystem detects the real physical A4 canvas and
publishes its exact 3D position and orientation every frame. Your job is to
use that data to render a matching virtual canvas in the Unity VR scene so the
user sees the drawing surface represented correctly in the headset.

This document focuses on two things:
1. How to build the virtual canvas plane in Unity from the ROS data
2. How to develop and test everything without any physical hardware

---

## 1. Building the Virtual Canvas Plane

### What the pose data gives you

The topic `/canvas/pose` publishes a `PoseStamped` message. Think of it as
giving you the answer to: "where is the centre of the canvas, and which way
is it facing?"

```
Topic:   /canvas/pose
Type:    geometry_msgs/msg/PoseStamped
```

| Field | What it means in Unity terms |
|---|---|
| `pose.position.x/y/z` | World position of canvas centre (metres) |
| `pose.orientation.x/y/z/w` | Canvas face direction as a quaternion |
| `header.frame_id` | `camera_color_optical_frame` (the ROS coordinate space) |

The canvas is always A4 landscape: **297 mm wide, 210 mm tall**.

### Step-by-step: placing the canvas plane in Unity

**Step 1 -- Subscribe to `/canvas/pose`**

In your ROS subscriber callback, extract position and orientation:

```csharp
void OnCanvasPoseReceived(PoseStampedMsg msg)
{
    // Store as target -- apply in Update() with smoothing
    targetPosition = new Vector3(
        (float) msg.pose.position.x,
       -(float) msg.pose.position.y,   // ROS Y-down -> Unity Y-up: negate Y
        (float) msg.pose.position.z
    );

    targetRotation = new Quaternion(
        (float) msg.pose.orientation.x,
       -(float) msg.pose.orientation.y, // negate for handedness conversion
        (float) msg.pose.orientation.z,
       -(float) msg.pose.orientation.w
    );
}
```

**Step 2 -- Create the canvas GameObject**

In Unity:
- Create a 3D Quad (GameObject > 3D Object > Quad)
- Name it `VirtualCanvas`
- Set scale to match A4 landscape:
  ```
  Scale X = 0.297   (297 mm -- A4 width)
  Scale Y = 0.210   (210 mm -- A4 height)
  Scale Z = 0.001   (near-zero thickness)
  ```
- Apply a white/cream unlit material to represent the paper

The quad's local X axis = canvas width direction (left/right).
The quad's local Y axis = canvas height direction (up/down).
The quad's local Z axis = canvas face normal (pointing toward camera).

**Step 3 -- Parent to the camera frame origin**

The pose arrives in `camera_color_optical_frame`. In your Unity scene, create
an empty parent GameObject called `CameraFrame` that represents this coordinate
origin. Place `VirtualCanvas` as a child. The ROS-TCP-Connector TF listener
can drive `CameraFrame`'s transform automatically if you subscribe to the
`camera_color_optical_frame` TF frame.

Alternatively, drive the canvas directly in world space by composing the
camera-to-world transform with the pose offset.

**Step 4 -- Smooth the canvas in Update()**

`/canvas/pose` publishes at 6-18 Hz. Snapping to every new message causes
jitter. Use Lerp and Slerp in `Update()` for a stable VR experience:

```csharp
Vector3    targetPosition;
Quaternion targetRotation;

void Update()
{
    canvasPlane.transform.localPosition = Vector3.Lerp(
        canvasPlane.transform.localPosition,
        targetPosition,
        Time.deltaTime * 12f      // tune 12f for speed vs smoothness
    );
    canvasPlane.transform.localRotation = Quaternion.Slerp(
        canvasPlane.transform.localRotation,
        targetRotation,
        Time.deltaTime * 12f
    );
}
```

Only `targetPosition` and `targetRotation` are written from the ROS callback.
`Update()` smoothly chases them every frame.

### Using the four corner TF frames

Four TF frames are broadcast for the physical marker corners:

```
canvas_marker_TL    top-left  corner
canvas_marker_TR    top-right corner
canvas_marker_BR    bottom-right corner
canvas_marker_BL    bottom-left corner
```

All are children of `camera_color_optical_frame`. With these you can:
- Place small visual indicator spheres at each corner in the VR scene
- Show which corners are freshly detected (cyan) vs reconstructed (orange)
  -- matching the colour coding in the debug camera feed
- Fit the canvas plane exactly to the detected corner positions

Subscribe to each TF frame via the ROS-TCP-Connector TF listener and attach
small sphere GameObjects to them. This makes the virtual canvas visually
anchored to the real marker positions rather than floating at the estimated
centre.

### Showing canvas state through appearance

Monitor `/canvas/correction_data` (see Section 3) and change the canvas
material based on `status`:

```csharp
void UpdateCanvasVisual(string status, float remaining_s, float drift_mm)
{
    switch (status)
    {
        case "WAITING":
            // No detection -- hide canvas, show search prompt
            canvasRenderer.enabled = false;
            statusText.text = "Point camera at canvas markers";
            drawButton.interactable = false;
            break;

        case "COLLECTING":
            // Calibrating -- show amber ghost with countdown
            canvasRenderer.enabled = true;
            canvasMaterial.color = new Color(1f, 0.6f, 0f, 0.4f);
            statusText.text = $"Calibrating -- hold still  {remaining_s:F0}s";
            ShowCountdownRing(remaining_s, latch_window_s);
            drawButton.interactable = false;
            break;

        case "PAUSED":
            // Lost tracking -- ghost the canvas at last known position
            canvasRenderer.enabled = true;
            canvasMaterial.color = new Color(1f, 1f, 1f, 0.15f);
            statusText.text = "Canvas lost -- reposition camera";
            drawButton.interactable = false;
            break;

        case "OK":
            // Fully tracked -- opaque canvas, enable drawing
            canvasRenderer.enabled = true;
            // Tint edge red if drift is high (>5mm), white if stable
            float t = Mathf.Clamp01(drift_mm / 5f);
            canvasMaterial.color = Color.Lerp(Color.white, Color.red, t);
            statusText.text = drift_mm < 1f ? "" : $"Drift: {drift_mm:F1} mm";
            drawButton.interactable = true;
            break;
    }
}
```

---

## 2. Drawing on the Canvas in VR

### What you publish when the user draws

```
Topic:    /canvas/stroke_target
Type:     geometry_msgs/msg/PoseStamped
frame_id: canvas_centre_assumed
```

`canvas_centre_assumed` is the stable latched reference frame. It does not
move even if the real canvas drifts slightly. Robot correction handles the
difference automatically.

### Canvas drawing coordinate system

```
      canvas_centre_assumed frame

           -Y  (top edge)
            |
  -X ──── [0,0] ──── +X   right = +X
            |
           +Y  (bottom edge)

  A4 bounds:
    X: -0.148 m (left)   to  +0.148 m (right)
    Y: -0.105 m (top)    to  +0.105 m (bottom)
    Z:  0.005 m fixed    (5mm hover above canvas face -- do not change)
```

### Converting VR controller position to a stroke message

If you raycast from the VR controller onto the virtual canvas plane, the hit
point gives you a local UV coordinate. Convert that to canvas metres:

```csharp
PoseStampedMsg BuildStrokeMsg(Vector2 canvasUV)
{
    // canvasUV: (0,0) = top-left, (1,1) = bottom-right of the quad
    // Map to canvas metres
    float x = Mathf.Lerp(-0.148f,  0.148f, canvasUV.x);
    float y = Mathf.Lerp(-0.105f,  0.105f, canvasUV.y);

    // Clamp to A4 bounds
    x = Mathf.Clamp(x, -0.148f, 0.148f);
    y = Mathf.Clamp(y, -0.105f, 0.105f);

    var msg = new PoseStampedMsg();
    msg.header.frame_id        = "canvas_centre_assumed";
    msg.pose.position.x        = x;
    msg.pose.position.y        = y;
    msg.pose.position.z        = 0.005f;   // fixed standoff
    msg.pose.orientation.w     = 1.0f;     // identity -- brush perpendicular
    return msg;
}
```

**Only publish when `status == "OK"`.**

---

## 3. Canvas Status Topic (JSON)

```
Topic:   /canvas/correction_data
Type:    std_msgs/msg/String
```

Full JSON structure:

```json
{
  "status": "OK",
  "pose_age_ms": 35.0,
  "latency_ms": 3.7,
  "remaining_s": 0.0,
  "error": {
    "lateral_mm": -2.1,
    "vertical_mm": 1.4,
    "depth_mm": -0.8,
    "rotation_deg": 0.3,
    "total_3d_mm": 2.6
  }
}
```

Parse in C# with `JsonUtility`:

```csharp
[System.Serializable]
public class CanvasError {
    public float lateral_mm, vertical_mm, depth_mm, rotation_deg, total_3d_mm;
}

[System.Serializable]
public class CanvasStatus {
    public string status;
    public float  pose_age_ms, latency_ms, remaining_s;
    public CanvasError error;
}

void OnCorrectionData(StringMsg msg) {
    var data = JsonUtility.FromJson<CanvasStatus>(msg.data);
    UpdateCanvasVisual(data.status, data.remaining_s, data.error.total_3d_mm);
}
```

| `status` | Meaning | VR response |
|---|---|---|
| `WAITING` | No canvas seen yet | Hide canvas, show search prompt |
| `COLLECTING` | 10s calibration window | Amber canvas, countdown, no drawing |
| `PAUSED` | Canvas lost (>5s) | Ghost canvas at last known position |
| `OK` | Tracking live | Full canvas visible, draw enabled |

---

## 4. All Topics Summary

| Topic | Direction | Type | Use in Unity |
|---|---|---|---|
| `/canvas/stroke_target` | Unity -> ROS | PoseStamped | Publish brush position |
| `/canvas/correction_data` | ROS -> Unity | String (JSON) | Status + drift for UI |
| `/canvas/pose` | ROS -> Unity | PoseStamped | Canvas centre + orientation |
| `/canvas/debug` | ROS -> Unity | Image | Live camera feed for VR display |
| TF `canvas_centre` | ROS -> Unity | TF | Canvas centre (alternative to pose) |
| TF `canvas_marker_TL/TR/BR/BL` | ROS -> Unity | TF | Four corner positions |

You do NOT need: `/canvas/z_constraint`, `/canvas/pose_error`,
`/canvas/pose_base` -- those are internal to the robot control subsystem.

---

## 5. ROS-TCP-Connector Setup

```
ROS IP Address:   <Ubuntu machine LAN IP on your network>
ROS Port:         10000
```

Find Ubuntu IP:
```bash
hostname -I | awk '{print $1}'
```

`ros_tcp_endpoint` starts automatically with every pipeline launch.

---

## 6. Developing Without Hardware

You do not need a camera, canvas, or robot. The sim pipeline publishes
identical topics and TF frames to the real detector.

### Start

```bash
source ~/RS2/XR_Teleoperation/install/setup.bash
ros2 launch canvas_pose_detector canvas_sim_pipeline.launch.py
```

After ~12s the status reaches `OK`. Connect Unity and all topics are live.

### Control synthetic canvas from terminal

```bash
# Move canvas (test depth/distance representation)
ros2 param set /canvas_pose_injector canvas_z 0.4

# Tilt canvas (test orientation handling in Unity)
ros2 param set /canvas_pose_injector canvas_yaw_deg 20.0
ros2 param set /canvas_pose_injector canvas_pitch_deg 10.0

# Animate -- canvas sweeps continuously (test live tracking in VR)
ros2 param set /canvas_pose_injector animate true
ros2 param set /canvas_pose_injector animate_axis yaw
ros2 param set /canvas_pose_injector animate_range 25.0
ros2 param set /canvas_pose_injector animate_period_s 4.0
ros2 param set /canvas_pose_injector animate false   # stop

# Simulate canvas loss (test PAUSED state)
ros2 param set /canvas_pose_injector publish_rate_hz 0.0
ros2 param set /canvas_pose_injector publish_rate_hz 10.0   # restore
```

---

## 7. Integration With Real Hardware

No Unity changes needed when switching from sim to real hardware:

```bash
# Terminal 1
ros2 launch realsense2_camera rs_launch.py \
  enable_sync:=true align_depth.enable:=true

# Terminal 2
ros2 launch canvas_pose_detector canvas_real_pipeline.launch.py
```

Real hardware startup timeline:

| Time | Status | Unity should show |
|---|---|---|
| t=0s | `WAITING` | "Searching for canvas markers..." |
| t=4s | `COLLECTING` | Amber canvas + countdown |
| t=14s | `OK` | Full canvas + draw enabled |

---

## 8. Contract Checklist (Subsystem 3)

**Pass:**
- Virtual canvas plane at correct position/orientation from `/canvas/pose`
- Status shown from `correction_data`
- Draw gated on `status == "OK"`
- Camera feed displayed from `/canvas/debug`

**Credit:**
- Status state machine: WAITING / COLLECTING / OK / PAUSED all visually distinct
- Countdown during COLLECTING using `remaining_s`

**Distinction:**
- Corner markers using `canvas_marker_TL/TR/BR/BL` TF frames
- Canvas plane smoothed with Lerp/Slerp -- no jitter in VR
- Drift indicator using `error.total_3d_mm`
- Auto-recovery when status returns to `OK` after PAUSED

**HD:**
- Canvas tinting reflects live tracking quality
- Stroke positions clamped to A4 bounds before publishing
- Clear haptic or visual warning when tracking degrades mid-stroke
- Draw input immediately disabled when status leaves `OK`

---

## Contact

For any changes to topic names, message fields, or coordinate frames,
contact Johan first -- changes affect Stephen's subsystem too.

```bash
# Quick diagnosis
ros2 node list | grep canvas
ros2 topic echo /canvas/correction_data --once
ros2 topic hz /canvas/pose
```