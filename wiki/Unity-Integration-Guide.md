# Unity Integration Guide

This page explains how to build the VR interface that displays the detected canvas and sends stroke commands to the robot.

---

## Overview

The Unity app needs to:

1. **Display the virtual canvas** at the correct position and orientation using `/canvas/pose`
2. **Show canvas status** (calibrating, tracking, lost) from `/canvas/correction_data`
3. **Publish brush strokes** to `/canvas/stroke_target` when the operator draws

---

## 1. ROS-TCP-Connector Setup

```
ROS IP Address:   <Ubuntu machine LAN IP>
ROS Port:         10000
```

Find the Ubuntu machine's IP:

```bash
hostname -I | awk '{print $1}'
```

The `ros_tcp_endpoint` starts automatically with every pipeline launch.

---

## 2. Subscribing to Canvas Pose

**Topic:** `/canvas/pose` (`geometry_msgs/PoseStamped`)

The pose gives you the canvas centre position and facing direction in `camera_color_optical_frame`.

### Coordinate Conversion (ROS -> Unity)

```csharp
void OnCanvasPoseReceived(PoseStampedMsg msg)
{
    // ROS: X-right, Y-down, Z-forward (camera optical frame)
    // Unity: X-right, Y-up, Z-forward
    targetPosition = new Vector3(
        (float) msg.pose.position.x,
       -(float) msg.pose.position.y,   // negate Y for handedness
        (float) msg.pose.position.z
    );

    targetRotation = new Quaternion(
        (float) msg.pose.orientation.x,
       -(float) msg.pose.orientation.y, // negate for handedness
        (float) msg.pose.orientation.z,
       -(float) msg.pose.orientation.w
    );
}
```

### Canvas GameObject Setup

1. Create a **3D Quad** (`GameObject > 3D Object > Quad`)
2. Name it `VirtualCanvas`
3. Set scale to A4 landscape:
   ```
   Scale X = 0.297   (297 mm)
   Scale Y = 0.210   (210 mm)
   Scale Z = 0.001   (near-zero thickness)
   ```
4. Apply a white/cream unlit material

### Smooth Tracking

`/canvas/pose` publishes at 6-18 Hz. Use Lerp/Slerp to avoid jitter:

```csharp
void Update()
{
    canvasPlane.transform.localPosition = Vector3.Lerp(
        canvasPlane.transform.localPosition,
        targetPosition,
        Time.deltaTime * 12f
    );
    canvasPlane.transform.localRotation = Quaternion.Slerp(
        canvasPlane.transform.localRotation,
        targetRotation,
        Time.deltaTime * 12f
    );
}
```

---

## 3. Canvas Corner TF Frames

Four TF frames are broadcast for physical marker positions:

| Frame | Position |
|---|---|
| `canvas_marker_TL` | Top-left corner |
| `canvas_marker_TR` | Top-right corner |
| `canvas_marker_BR` | Bottom-right corner |
| `canvas_marker_BL` | Bottom-left corner |

Subscribe via TF listener and attach small sphere GameObjects for visual anchoring.

---

## 4. Canvas Status (JSON)

**Topic:** `/canvas/correction_data` (`std_msgs/String`)

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

### C# Parsing

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

### Status State Machine

| Status | Meaning | VR Response |
|---|---|---|
| `WAITING` | No canvas detected | Hide canvas, show "Point camera at canvas" |
| `COLLECTING` | Calibrating (10s) | Amber translucent canvas + countdown |
| `OK` | Tracking live | Full canvas visible, drawing enabled |
| `PAUSED` | Canvas lost (>5s) | Ghost canvas at last position, disable drawing |

---

## 5. Publishing Stroke Targets

**Topic:** `/canvas/stroke_target` (`geometry_msgs/PoseStamped`)
**Frame:** `canvas_centre_assumed`

### Canvas Drawing Coordinates

```
           -Y  (top edge)
            |
  -X ---- [0,0] ---- +X   right = +X
            |
           +Y  (bottom edge)

  A4 bounds:
    X: -0.148 m (left)   to  +0.148 m (right)
    Y: -0.105 m (top)    to  +0.105 m (bottom)
    Z:  0.005 m fixed    (5 mm hover)
```

### Converting VR Hit Point to Stroke Message

```csharp
PoseStampedMsg BuildStrokeMsg(Vector2 canvasUV)
{
    // canvasUV: (0,0) = top-left, (1,1) = bottom-right
    float x = Mathf.Lerp(-0.148f,  0.148f, canvasUV.x);
    float y = Mathf.Lerp(-0.105f,  0.105f, canvasUV.y);

    x = Mathf.Clamp(x, -0.148f, 0.148f);
    y = Mathf.Clamp(y, -0.105f, 0.105f);

    var msg = new PoseStampedMsg();
    msg.header.frame_id        = "canvas_centre_assumed";
    msg.pose.position.x        = x;
    msg.pose.position.y        = y;
    msg.pose.position.z        = 0.005f;   // fixed standoff
    msg.pose.orientation.w     = 1.0f;     // identity = brush perpendicular
    return msg;
}
```

**Only publish when `status == "OK"`.**

---

## 6. Topics Summary

| Topic | Direction | Type | Use |
|---|---|---|---|
| `/canvas/stroke_target` | Unity -> ROS | PoseStamped | Brush position |
| `/canvas/correction_data` | ROS -> Unity | String (JSON) | Status + drift |
| `/canvas/pose` | ROS -> Unity | PoseStamped | Canvas centre + orientation |
| `/canvas/debug` | ROS -> Unity | Image | Live camera feed |
| TF `canvas_centre` | ROS -> Unity | TF | Canvas centre frame |
| TF `canvas_marker_*` | ROS -> Unity | TF | Corner positions |

You do **not** need: `/canvas/z_constraint`, `/canvas/pose_error`, `/canvas/pose_base` -- those are internal to robot control.

---

## 7. Developing Without Hardware

The sim pipeline publishes identical topics:

```bash
source ~/RS2/XR_Teleoperation/install/setup.bash
ros2 launch canvas_pose_detector canvas_sim_pipeline.launch.py
```

After ~12s the status reaches `OK`. Connect Unity and all topics are live.

Control the synthetic canvas from the terminal:

```bash
ros2 param set /canvas_pose_injector canvas_z 0.4
ros2 param set /canvas_pose_injector canvas_yaw_deg 20.0
ros2 param set /canvas_pose_injector animate true
```

---

## 8. Switching to Real Hardware

No Unity code changes needed:

```bash
# Terminal 1
ros2 launch realsense2_camera rs_launch.py \
  enable_sync:=true align_depth.enable:=true

# Terminal 2
ros2 launch canvas_pose_detector canvas_real_pipeline.launch.py
```

| Time | Status | Unity Shows |
|---|---|---|
| t=0s | `WAITING` | "Searching for canvas markers..." |
| t=4s | `COLLECTING` | Amber canvas + countdown |
| t=14s | `OK` | Full canvas + draw enabled |
