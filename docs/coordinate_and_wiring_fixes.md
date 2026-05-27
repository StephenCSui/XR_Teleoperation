# Coordinate Frame & Scene Wiring Fixes
*XR Teleoperation — UR3e with 180° base rotation*

---

## Background

The UR3e base was physically rotated 180° around its Z axis compared to the original software setup. This rotation was propagated through the ROS launch config (bounding box limits, canvas plane X values) but **not** through the Unity↔ROS coordinate conversion math. Simultaneously, several Unity scene object references were unset (null FileID entries in the YAML) meaning those scripts silently did nothing at runtime.

The two root issues compounded each other: even if wiring had been correct the robot would have moved in the wrong direction, and even if the math had been correct some scripts would not have run at all.

---

## Issue 1 — Unity Scene FileID Wiring (null references)

Unity serializes inspector field assignments as `{fileID: XXXXXXXX}` entries in the scene YAML. Fields that are dragged in the editor but not saved, or lost during reimport, fall back to `{fileID: 0}` (null). At runtime the script receives a null reference and silently skips that code path with no error.

### Fields that were null and have been wired

| Script (component) | Field | Wired to fileID |
|--------------------|-------|----------------|
| `TeleopHandPosePublisher` | `xrOrigin` | `461248083` (XROrigin Transform) |
| `TeleopHandPosePublisher` | `modeController` | `35868104` (TeleopModeController) |
| `TeleopHandPosePublisher` | `canvasPainter` | `1091956555` (CanvasPainter) |
| `TeleopModeController` | `handPosePublisher` | `283756414` (TeleopHandPosePublisher) |
| `CommandSubscriber` | `commandBox` | `2038908788` (CommandBox Transform) |
| `ActualEePoseSubscriber` | `actualBox` | `210573051` (ActualBox Transform) |
| `NudgeCommander` | `modeController` | `35868104` (TeleopModeController) |

**Symptom when null:**
- `xrOrigin = null` → snap never fires → XR rig never moves to EE position at startup
- `commandBox = null` → CommandBox sits at world origin (0,0,0) regardless of filter output
- `actualBox = null` → ActualBox (blue) sits at world origin
- `modeController = null` (on publisher) → PRECISION mode publishing never suppressed
- `handPosePublisher = null` (on mode controller) → PRECISION entry uses Quaternion.identity for rotation

### Snap gate restored

`publishOnlyAfterSnap` was set to `0` (disabled) in the scene YAML. This caused the filter to latch its anchor position **before** the XR snap fired, so the filter was chasing a target 1.14 m away from the actual EE position the entire session.

```yaml
# SampleScene.unity — restored to 1
publishOnlyAfterSnap: 1
```

---

## Issue 2 — 180° Base Rotation Coordinate Math

### The mapping before and after

With the base rotated 180° around Z, the ROS `base_link` frame now points:
- **+X**: away from robot (toward canvas / operator) — previously away from operator
- **+Y**: to the robot's left (swapped sign vs. original)
- **+Z**: up (unchanged)

Unity world frame is unchanged: +X right, +Y up, +Z forward (left-handed).

| Axis | Old mapping | New mapping |
|------|-------------|-------------|
| ROS X | = +Unity Z | = **−Unity Z** |
| ROS Y | = −Unity X | = **+Unity X** |
| ROS Z | = +Unity Y | = +Unity Y (unchanged) |

Inverse (ROS → Unity):

| Axis | Old | New |
|------|-----|-----|
| Unity X | = −ROS Y | = **+ROS Y** |
| Unity Y | = +ROS Z | = +ROS Z (unchanged) |
| Unity Z | = +ROS X | = **−ROS X** |

---

### C++ filter: `unity_to_ros_delta()` (`unity_authority_filter_servo.cpp`)

Converts a position vector from Unity frame to ROS `base_link` frame.

```cpp
// Before
return make_point(u.z, -u.x, u.y);

// After — 180° base rotation
return make_point(-u.z, u.x, u.y);
```

---

### C++ filter: `unity_to_ros_basis()` (`unity_authority_filter_servo.cpp`)

Rotation matrix used to convert orientation from Unity frame to ROS frame. Applied as `R_ros = M * R_unity * M^{-1}`.

```cpp
// Before
c <<  0.0,  0.0,  1.0,
     -1.0,  0.0,  0.0,
      0.0,  1.0,  0.0;

// After — 180° base rotation
c <<  0.0,  0.0, -1.0,
      1.0,  0.0,  0.0,
      0.0,  1.0,  0.0;
```

---

### C# Unity scripts: `RosToUnityPosition()`

Inverse of `unity_to_ros_delta` — converts a ROS position back to Unity world frame for visual box placement.

```csharp
// Before
return new Vector3(-ros.y, ros.z, ros.x);

// After — 180° base rotation
return new Vector3(ros.y, ros.z, -ros.x);
```

**Applies to:** `TeleopHandPosePublisher.cs`, `CommandSubscriber.cs`, `ActualEePoseSubscriber.cs`

---

### C# Unity scripts: `RosToUnityRotation()`

Converts a ROS quaternion to a Unity quaternion. Derivation:
1. Transform axis: `n_unity = M^{-1} * n_ros = (n_y, n_z, −n_x)` (using new M)
2. Negate rotation angle for left-handed→right-handed handedness change
3. Result: `q_unity = (−q_y, −q_z, q_x, q_w)`

```csharp
// Before
var r = new Quaternion(-q.y, q.z, q.x, -q.w);

// After — 180° base rotation
var r = new Quaternion(-q.y, -q.z, q.x, q.w);
```

For `CommandSubscriber` and `ActualEePoseSubscriber` the same change:

```csharp
// Before
new Quaternion(-rosQ.y,  rosQ.z, rosQ.x, -rosQ.w)

// After
new Quaternion(-rosQ.y, -rosQ.z, rosQ.x,  rosQ.w)
```

**Applies to:** `TeleopHandPosePublisher.cs`, `CommandSubscriber.cs`, `ActualEePoseSubscriber.cs`

---

### `TeleopModeController.cs`: PRECISION entry Z coordinate

When entering PRECISION mode the script sets a virtual hand position so the filter drives the EE to the canvas touch plane. The Z component (Unity Z = −ROS X) was not negated.

```csharp
// Before — Unity Z was being set to ROS value directly (wrong sign)
_virtualHandPos.z = stopPlaneRosX - 0.05f;
// → Unity Z = −0.50 → filter sees ROS X = +0.50 (away from canvas, completely wrong)

// After — correct sign
_virtualHandPos.z = -(stopPlaneRosX - 0.05f);
// → Unity Z = +0.50 → filter sees ROS X = −0.50 (past touch plane at −0.47, correct)
```

---

## PRECISION stick X direction — confirmed inverted, fixed

Confirmed on hardware: left/right was inverted. Fixed by flipping sign:

```csharp
// TeleopModeController.cs
// Before
_virtualHandPos.x -= stick.x * ...;
// After
_virtualHandPos.x += stick.x * ...;
```

---

## Canvas touch plane reach (trigger)

Holding the right trigger shifts the EE forward clamp from `canvas_stop_plane_x` to `canvas_touch_plane_x`, pressing the tool into the canvas. The P-controller drives the EE to the new limit.

```python
# ur3e_unity_bridge_servo.launch.py
"canvas_stop_plane_x":  -0.45,   # normal forward limit
"canvas_touch_plane_x": -0.49,   # trigger-held limit — 4 cm press
```

Reach when trigger held = **4 cm** (previously 2 cm).

---

## Files modified

| Repo | File |
|------|------|
| `XR_Teleoperation` | `src/unity_authority_filter_servo_cpp/src/unity_authority_filter_servo.cpp` |
| `XR_Teleoperation` | `src/ur_unity_bringup/launch/ur3e_unity_bridge_servo.launch.py` |
| `UnityRos2` | `Assets/Scenes/SampleScene.unity` |
| `UnityRos2` | `Assets/TeleopHandPosePublisher.cs` |
| `UnityRos2` | `Assets/TeleopModeController.cs` |
| `UnityRos2` | `Assets/CommandSubscriber.cs` |
| `UnityRos2` | `Assets/ActualEePoseSubscriber.cs` |

---

## Rebuild required after these changes

```bash
# ROS side — C++ filter
cd ~/XR_Teleoperation && colcon build --packages-select unity_authority_filter_servo_cpp && source install/setup.bash

# Unity side — full Android APK rebuild required (C# and scene changes)
```
