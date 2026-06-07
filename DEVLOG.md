# XR Teleoperation — Development Log

Focuses on what was tried, what worked, what didn't, and why decisions were made.
Unity-side entries are included where documented; gaps exist because Unity lives in a separate repo with no shared history.

---

## Phase 1 — MoveIt Goal Posting (Feb–Mar 2026)

**Approach:** Unity publishes hand pose → ROS filter → MoveIt `ExecuteTrajectory` goal → plan and execute.

**Unity side:**
- ROS-TCP-Connector set up in Unity project
- `TeleopHandPosePublisher.cs` created — publishes hand pose to `/unity/hand_pose` and teleop enable/disable flag to `/unity/teleop_enabled`
- Keyboard toggle (T key) for enabling/disabling teleop

**What didn't work:**
- High latency from the MoveIt planning cycle — arm response felt disconnected from hand movement
- Goal rejection when the hand moved too fast (planner couldn't keep up)
- Not suitable for real-time teleoperation

**Decision:** Abandon goal posting, switch to direct servo control.

---

## Phase 2 — Servo Control + Singularity (Mar 2026)

**Approach:** Replace MoveIt planning with MoveIt Servo. Built `pose_error_to_twist` P-controller to convert pose error into twist commands fed directly to `/servo_node/delta_twist_cmds`. Added `unity_authority_filter_servo_cpp` for safety gating, workspace limits, and deadbands.

**What didn't work:**
- Starting pose was the default robot pose (arm pointing straight down) — immediately hit singularities on any movement attempt
- Attempted to fix via servo YAML parameter tuning (singularity thresholds, condition number limits) — no effect
- `forward_velocity_controller` kept being inactive on startup — had to set it as the default active controller in the launch arguments so it was always on from launch
- Servo node kept dying mid-session — wrote `servo_auto_start` to watch for the `robot_program_running` signal and auto-reset/restart servo on each connection

**What worked:**
- Changed starting joint configuration to a non-singular pose — singularities stopped, servo responded correctly
- First confirmed control: keyboard input used to drive the arm in simulation via servo

**Unity side:**
- Keyboard teleop confirmed working end-to-end from Unity keyboard input through to servo

---

## Phase 3 — URSim Integration + Real Hardware (Apr 2026)

**Approach:** Validate the full pipeline in URSim (Docker-based simulator) and then on a physical UR3e.

**What didn't work:**
- Wrong launch order left the velocity controller inactive — `servo_auto_start` must see the rising edge of `robot_program_running` to switch controllers correctly; launching ROS before hitting Play in URSim caused it to miss the edge
- Gains were too aggressive for real hardware — caused oscillation and overshoot (`linear_gain` 4.0→1.5, `max_linear_speed` 0.15→0.06 m/s, step sizes halved)
- Without `anchor_settle_ms`, the ROS side latched the hand anchor before Unity finished its position snap, causing the arm to teleport on teleop enable — fixed with a 150 ms settle delay

**What worked:**
- URSim Docker pipeline confirmed working
- First confirmed control on real hardware: keyboard teleop working with physical UR3e
- ROS-TCP-Connector version pinned to v0.7.0-preview — updating breaks Unity/ROS compatibility

**Unity side:**
- EE pose sent back to Unity via `/robot/ee_pose` — Unity visualised both the actual EE pose and the command pose for debugging
- ROS IP configured for network connection to ROS machine

---

## Phase 4 — Precision Mode + Features (Apr 2026)

**Approach:** Add finer control modes for detailed work and near-surface tasks.

**What was built:**
- **Precision mode:** tighter deadband, smaller max step per tick, scaled position sensitivity
- **Precision bounding box:** user draws a box in Unity; geometric mean of box dimensions drives position scale — smaller box = finer control
- **Detachment + nudge mode:** in PRECISION, hand detaches from direct control; button presses give incremental position/orientation nudges
- **Canvas proximity mode:** near-canvas tightening of all gains, separate normal/tangential scale split for drawing tasks — parameters built but deferred, no physical canvas yet
- **Mode manager node:** publishes active mode and canvas proximity factor to the filter

**Unity side:**
- Precision bounding box drawn interactively by user in scene
- Mode switching UI wired up
- Detachment mode controller input wired up

---

## Phase 5 — Orientation Snap + VR Polish (May 2026)

**Approach:** Get the system working on a Quest 2 headset end-to-end.

**What didn't work:**
- Position-only snap was working but orientation snap was missing — EE frame axes didn't align with the controller axes, making controls feel sideways (forward/backward could map to left/right depending on starting controller angle)
- Wrist3=0° gave a suboptimal EE orientation at snap time (X=right, Y=down), making the snapped frame feel rolled

**What was fixed:**
- Orientation snap implemented: on startup, `xrOrigin` is rotated so the hand's world rotation matches the EE orientation — rotation applied before position delta so the position snap lands correctly after the rotation
- Wrist3 adjusted to 270° — gives natural EE orientation (red/X=up, green/Y=right, blue/Z=forward) that matches VR controller ergonomics at snap time
- Pointer ray visual added on controller; pitch offset applied to ray only, not to the hand mesh
- `TeleopDebugHUD` added to scene for in-headset debug readout (TextMeshPro)
- APK built for Quest 2: OpenXR / Oculus profile configured for Android, ROS IP set for network

**Current state:**
- APK runs on Quest 2, end-to-end servo pipeline confirmed working in VR
- Orientation control direction not yet verified against robot (debug flag added for this)
- Canvas configuration deferred — user has a physical approach planned

---

## Phase 6 — Canvas Plane Tuning + PRECISION Pen-Down Fix (May 2026)

### Plane positions shifted 3 cm closer to robot

| Parameter | Old | New |
|---|---|---|
| `canvas_stop_plane_x` | −0.45 | −0.42 |
| `canvas_touch_plane_x` | −0.49 | −0.46 |
| Stop→touch gap | 4 cm | 4 cm (unchanged) |

Files changed: `ur3e_unity_bridge_servo.launch.py`, `TeleopModeController.cs` (`stopPlaneRosX`).

### Joint angles updated — required for plane shift to be safe

Old joints: Base=180°, Shoulder=−60°, Elbow=80°, Wrist1=−20°, Wrist2=−270°, Wrist3=90°  
→ EE starting X = **−0.414 m** — already past the new stop plane (−0.42). Would cause robot to jerk backward on teleop enable.

New joints: Base=180°, Shoulder=−65°, Elbow=105°, Wrist1=−40°, Wrist2=−270°, Wrist3=90°  
→ EE starting X = **−0.358 m** — 6 cm behind new stop plane. Safe.

Y and Z are unchanged (0.131, 0.150). The joint change pulls the EE back ~5.6 cm in X while keeping lateral position the same.

**Rule established:** EE starting X must be less negative than `canvas_stop_plane_x`. Check this whenever planes are moved or starting joints change. FK formula used: UR3e standard DH (d=[0.15185, 0, 0, 0.13105, 0.08535, 0.0921], a=[0, −0.24355, −0.21325, 0, 0, 0]).

### PRECISION mode pen-down fix

**Root cause of old failure:** Virtual hand Z was set only 5 cm past the stop plane (`stopPlaneRosX − 0.05`). With position_scale=0.5, the scaled command delta only reached ~2.5 cm past stop — never enough to touch the touch plane (4 cm gap). Trigger held → pen_down=true → filter unlocked to touch plane — but the target was short of the touch plane anyway, so robot barely moved.

**Fix:** Virtual Z push increased to 0.30 m (`stopPlaneRosX − 0.30`). Scaled delta now overshoots touch plane; the plane clamp brings robot exactly to −0.46. pen_down stays trigger-gated in both NORMAL and PRECISION (no behaviour change between modes).

File changed: `TeleopModeController.cs` (`EnterPrecision()`).

---

## Phase 7 — PRECISION Polish + Base=90° Analysis (May 2026)

### PRECISION entry gate widened

Entry was gated on EE being at or past the stop plane (ROS X ≤ −0.41). Too tight in practice — had to be very close to the canvas to enter. Widened by increasing `stopPlaneTolerance` from 0.01 → 0.03, making the gate at ROS X ≤ −0.39.

File changed: `TeleopModeController.cs` (`stopPlaneTolerance`).

### PRECISION A button made global

Previously gated on EE position (had to be near canvas to enter PRECISION). Removed the positional check — A button now enters PRECISION from anywhere as long as a valid EE pose has been received. Rationale: user may want fine control at any position, not just near the canvas.

File changed: `TeleopModeController.cs` (removed `atPlane` check, line 72–74).

### CanvasPainter raycast fix

Drawing was broken after CommandBox/ActualBox were rescaled to 0.05. Root cause: `Physics.Raycast` skips colliders the ray starts inside of. At the old larger scale, the ray origin (3 cm offset from CommandBox) sat inside the CommandBox collider → collider was skipped → canvas hit correctly. At scale 0.05, the collider shrank to ±2.5 cm — ray origin now starts outside → Physics.Raycast detected CommandBox first → drawing broke.

Fix: switched to `Collider.Raycast()` on the canvas collider directly. Only the canvas collider is tested regardless of any other colliders in the scene.

File changed: `CanvasPainter.cs` (Unity repo).

### Workspace camera (ROSCameraDisplay)

Added `ROSCameraDisplay.cs` to Unity — subscribes to `/workspace_cam/color/image_raw/compressed`, decodes in `LateUpdate()` (after all control `Update()` calls), rate-limited to 15 fps. Uses URP-compatible shader (`Universal Render Pipeline/Unlit`). Auto-creates a quad above the canvas in world space. Zero impact on control pipeline.

ROS side: `workspace_camera.launch.py` added as optional include in `ur3e_unity_bridge_servo.launch.py` (`workspace_cam:=false` by default).

### Base=90° analysis — documented, not implemented

Space constraints require changing the robot base joint from 180° to 90°. Full coordinate remapping derived using the same methodology as the 0°→180° change (`docs/coordinate_and_wiring_fixes.md`).

**Key finding from git history:** 0°→180° kept X as the forward axis, only signs changed. 180°→90° changes the forward axis from X to Y — magnitudes stay the same, axis swaps.

**Changes required (not yet applied — current machine stays at 180°):**

*C++ filter (`unity_authority_filter_servo.cpp`):*
- `unity_to_ros_delta`: `make_point(-u.z, u.x, u.y)` → `make_point(-u.x, -u.z, u.y)`
- `unity_to_ros_basis`: row signs change to `[[-1,0,0],[0,0,-1],[0,1,0]]`
- Canvas plane clamp: `cmd_pos.x` → `cmd_pos.y`, params renamed `canvas_stop_plane_y` / `canvas_touch_plane_y`
- Workspace clamp: x uses y_min_/y_max_ (side), y uses x_min_/x_max_ (forward reach)

*Unity C# (TeleopHandPosePublisher, CommandSubscriber, ActualEePoseSubscriber):*
- `RosToUnityPosition`: `(ros.y, ros.z, -ros.x)` → `(-ros.x, ros.z, -ros.y)`
- `RosToUnityRotation`: `(-q.y, -q.z, q.x, q.w)` → `(q.x, -q.z, q.y, q.w)`

*Launch file:*
- `canvas_stop_plane_x: -0.42` → `canvas_stop_plane_y: -0.42`
- `x_min/x_max` ↔ `y_min/y_max` values swap
- `canvas_normal_xyz: [-1,0,0]` → `[0,-1,0]`

*No change needed:* PRECISION virtual Z push formula, XR snap direction, joystick axes, canvas plane values (physical measurement — re-tune after new setup arranged).

**Decision:** 180° configuration remains the confirmed working baseline. 90° changes held until new physical setup is ready.
