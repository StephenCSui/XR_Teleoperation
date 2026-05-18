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
