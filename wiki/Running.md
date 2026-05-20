# Running the System

## Step 1 — Full clean reset (always run first)

```bash
pkill -f pose_error_to_twist; pkill -f unity_authority_filter_servo; pkill -f ee_tf_to_pose; pkill -f teleop_mode_manager; pkill -f servo_auto_start; pkill -f default_server_endpoint; pkill -f ur_ros2_control_node; pkill -f servo_node; pkill -f robot_state_publisher; pkill -f controller_manager; pkill -f move_group
ros2 daemon stop && sleep 1 && ros2 daemon start
```

This eliminates stale node instances that accumulate across launches. Multiple instances of `pose_error_to_twist` will all publish twist commands simultaneously, causing the robot to move on its own.

## Step 2 — Set robot starting pose

Set joints to: **Base=0°, Shoulder=-60°, Elbow=80°, Wrist1=-20°, Wrist2=-270°, Wrist3=90°**

This puts the EE at approximately x=0.43m, z=0.21m in base_link with the tool Z axis (blue) pointing forward toward the canvas.

## Step 3 — Launch ROS stack

**Real robot:**
```bash
cd ~/XR_Teleoperation && source install/setup.bash
ros2 launch ur_unity_bringup ur3e_unity_bridge_servo.launch.py \
  use_fake_hardware:=false \
  robot_ip:=192.168.0.191 \
  ros_ip:=0.0.0.0
```

**URSim (simulation):**
```bash
# Start simulator
source /opt/ros/humble/setup.bash
docker rm -f ursim 2>/dev/null || true
ros2 run ur_client_library start_ursim.sh -m ur3e

# Then launch ROS stack
cd ~/XR_Teleoperation && source install/setup.bash
ros2 launch ur_unity_bringup ur3e_unity_bridge_servo.launch.py \
  use_fake_hardware:=false \
  robot_ip:=192.168.56.101 \
  ros_ip:=0.0.0.0
```

For URSim, set joints to the same starting pose in the browser UI before hitting Play.

## Step 4 — Start robot program (real robot only)

On the teach pendant, run the External Control program. Wait for `servo_auto_start` to confirm the velocity controller is active in the terminal output.

Keep teach pendant speed at 40% or below to avoid oscillation.

## Step 5 — Put on headset and start APK

Put on the Quest 2 and launch the APK. The app automatically rotates the scene to place the canvas in front of you regardless of which direction you are facing.

```bash
adb shell monkey -p com.StephenCSui.XRTeleop 1
```

## Step 6 — Enable teleoperation

In the headset, press the teleop enable button (trigger or T key in keyboard mode). The hand will snap to the robot EE position. Move the controller to drive the robot.

Hold the right trigger to activate pen-down mode — the robot can press up to 2cm past the canvas stop plane.

### Precision mode

Precision mode locks hand tracking and gives thumbstick control of the EE directly against the canvas.

| Button | Action |
|--------|--------|
| **A** (at stop plane) | Enter PRECISION mode |
| Thumbstick | Move EE along canvas (X/Y only; pen-down is always active) |
| **Trigger** | Half speed |
| **B** | Toggle position / yaw-pitch orientation control |
| **A** (in PRECISION) | Exit — returns to normal hand tracking |

Entry is gated: A only triggers PRECISION when the EE is at or past the stop plane (ROS x ≥ 0.44 m). The HUD in the headset shows the current mode and active control.

## Screen recording

```bash
# Start recording
adb shell screenrecord /sdcard/capture.mp4

# Stop with Ctrl+C, then pull to desktop
adb pull /sdcard/capture.mp4 ~/Desktop/capture.mp4
```

## Stop APK

```bash
adb shell am force-stop com.StephenCSui.XRTeleop
```
