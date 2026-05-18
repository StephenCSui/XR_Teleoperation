# Troubleshooting

## Robot moves by itself on launch

Multiple stale `pose_error_to_twist` instances are publishing simultaneously.

Verify:
```bash
ros2 topic info /servo_node/delta_twist_cmds --verbose | grep "Publisher count"
# Should be 1
```

Fix: run the full clean reset before every launch.

## Canvas appears behind you in the headset

You were not facing the laptop/robot when putting on the Quest 2. The headset sets its VR forward direction based on which way you face at startup.

Fix: remove the headset, face the laptop/robot, put it back on, then relaunch the APK.

## Command box jumps away from hand on teleop enable

The EE starting position is outside the workspace bounds or the robot is in the wrong starting pose.

Fix: check `ee_r` in the `[ANCHOR]` log. Set joints to: Base=0°, Shoulder=-60°, Elbow=80°, Wrist1=-20°, Wrist2=-270°, Wrist3=90° before launching.

## Robot oscillates / jackhammer at higher speeds

The P-controller is overcorrecting due to actuator latency.

Fix: keep teach pendant speed at 40% or below. If still oscillating, increase `position_deadband_m` in the launch file and relaunch.

## `servo_auto_start` never confirms

The robot program on the teach pendant is not running, or External Control URCap is not configured.

Fix: confirm External Control host IP matches your laptop ethernet IP. Run the program on the teach pendant manually.

## ADB device shows `unauthorized`

USB debugging prompt did not appear in the headset.

Fix: developer mode must be enabled from the owner Meta account. Switch to the owner profile in the headset, replug USB, accept the prompt.

## APK connects but no robot motion

The ROS-TCP-Connector IP in the APK does not match the host WiFi IP, or the Quest 2 and laptop are not on the same WiFi network.

Fix: confirm the APK was built with the correct WiFi IP. Check both devices are on the same network.

## Canvas plane not stopping the robot

The `unity_authority_filter_servo_cpp` package was not rebuilt after the canvas plane code was added.

Fix:
```bash
cd ~/XR_Teleoperation && colcon build --packages-select unity_authority_filter_servo_cpp && source install/setup.bash
```

## Known Limitations

- Orientation control is relative to the anchor at teleop-enable. Large rotations near gimbal-lock poses (pitch ≈ ±90°) cause RPY display instability (physically correct, visually confusing).
- No collision avoidance — workspace bounds and the canvas stop plane are the only safety constraints. Ensure the robot has clearance before enabling teleoperation.
- The ROS-TCP-Connector version is pinned to v0.7.0-preview and must match the `ros_tcp_endpoint` version. Do not update either independently.
- APK ROS-TCP IP is hardcoded at build time — changing networks requires a rebuild.
