# Troubleshooting & FAQ

## Build Issues

### `canvas_pose_detector` fails to build -- missing OpenCV

**Error:** `Could not find a package configuration file provided by "OpenCV"`

**Fix:**
```bash
sudo apt install libopencv-dev
```

### `canvas_pose_detector` fails to build -- missing Eigen3

**Error:** `Could not find a package configuration file provided by "Eigen3"`

**Fix:**
```bash
sudo apt install libeigen3-dev
```

### `rosdep install` fails with missing keys

**Fix:** Make sure ROS 2 Humble sources are configured:
```bash
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### Build succeeds but package not found

**Symptom:** `ros2 pkg list` does not show the package after building.

**Fix:** Source the workspace:
```bash
source ~/RS2/XR_Teleoperation/install/setup.bash
```

If you added this to `~/.bashrc`, open a **new** terminal for it to take effect.

---

## Camera Issues

### RealSense not detected

**Symptom:** `rs-enumerate-devices` returns nothing.

**Checks:**
1. Use a USB **3.0** cable and port (USB 2.0 will not work properly)
2. Check `dmesg | tail -20` for USB errors
3. Try unplugging and re-plugging the camera
4. Install the Intel RealSense SDK if not present:
   ```bash
   sudo apt install librealsense2-utils
   ```

### RealSense starts but no depth data

**Symptom:** `/camera/camera/color/image_raw` publishes but `/camera/camera/aligned_depth_to_color/image_raw` does not.

**Fix:** Ensure alignment and sync are enabled:
```bash
ros2 launch realsense2_camera rs_launch.py \
  enable_sync:=true align_depth.enable:=true
```

### Detector running but no canvas detected

**Symptom:** `/canvas/pose` not publishing, detector logs "No markers found".

**Checks (in order):**

1. **View the debug feed first** -- this shows exactly what the detector sees:
   ```bash
   ros2 run rqt_image_view rqt_image_view
   # Select /canvas/debug from dropdown
   ```

2. **Marker visibility:** Point the camera at the canvas. All 4 AprilTag markers should be clearly visible in the debug feed. Detected markers appear with green outlines.

3. **Lighting:** In dark environments, increase CLAHE contrast:
   ```bash
   ros2 param set /canvas_pose_detector clahe_clip 8.0
   ```

4. **Distance:** At >1.5 m, markers become too small. Increase detection passes:
   ```bash
   ros2 param set /canvas_pose_detector aruco_win_max 53
   ```

5. **Marker quality:** Verify:
   - Correct dictionary: **AprilTag 36h11** (not ArUco or other)
   - Correct IDs: **0** (TL), **1** (TR), **2** (BR), **3** (BL)
   - Correct size: **50 mm** (measure with a ruler)
   - Markers are **flat** (not curled)
   - Markers are on the **front face** (facing the camera)

### Detection rate is low (<5 fps)

**Fix:** Lower the detection passes for faster processing:
```bash
ros2 param set /canvas_pose_detector aruco_win_max 23
```

The trade-off: fewer passes = faster but less robust at distance or in poor lighting.

**Reference:**
| `aruco_win_max` | Passes | Approx. FPS | Best For |
|---|---|---|---|
| 23 | 3 | ~18 fps | Close range, good lighting |
| 53 | 6 | ~10 fps | Medium distance, average lighting |
| 83 | 9 | ~5 fps | Far range, dark/shadowed environments |

---

## TF Issues

### `canvas_tf_bridge` fails -- "Could not transform"

**Symptom:** Bridge logs TF lookup errors.

**Checks:**
1. In the real pipeline, the static TF publisher must be running:
   ```bash
   ros2 run tf2_ros tf2_echo flange camera_color_optical_frame
   ```
   If this fails, the `flange_to_camera_tf` node is not running.
2. Check the full TF tree:
   ```bash
   ros2 run tf2_tools view_frames
   # Opens frames.pdf showing the TF tree
   ```
3. Ensure the UR driver is publishing the robot TF chain (`base_link -> ... -> flange`).

### TF tree is disconnected

**Symptom:** `view_frames` shows two separate trees (robot and camera).

**Cause:** The static TF from `flange` to `camera_color_optical_frame` is not being broadcast.

**Fix:** Ensure `canvas_real_pipeline.launch.py` is running (it starts the static TF publisher), or run it manually:
```bash
ros2 run tf2_ros static_transform_publisher \
  --x -0.05 --y 0.0 --z -0.09 \
  --roll 0.0 --pitch 1.0 --yaw 0.0 \
  --frame-id flange --child-frame-id camera_color_optical_frame
```

---

## Corrector Issues

### Status stuck on "WAITING"

**Cause:** The corrector is not receiving poses on its `source_topic`.

**Check:**
```bash
ros2 topic echo /canvas/pose --once
```

If no message appears, the detector/injector is not running or not detecting the canvas.

In the **real pipeline**, the corrector subscribes to `/canvas/pose` (not `/canvas/pose_base`), so it does not need the TF bridge to function.

### Status stuck on "COLLECTING"

**Cause:** The canvas is moving during the calibration window, or not enough poses are arriving.

**Fix:**
- Keep the canvas **perfectly still** for the full latch window (default 10 seconds)
- Check pose rate: `ros2 topic hz /canvas/pose` -- should be >5 Hz
- If the window is too long, you can shorten it (but the reference will be less stable):
  ```bash
  # Must restart pipeline with shorter window
  ros2 launch canvas_pose_detector canvas_sim_pipeline.launch.py latch_window_s:=5.0
  ```

### Status goes to "PAUSED"

**Cause:** No new pose received for `pose_timeout_s` (default 5 seconds).

**Checks:**
- Ensure markers are still visible to the camera
- Check if the detector is still publishing: `ros2 topic hz /canvas/pose`
- The status will return to **OK automatically** once poses resume -- no restart needed

### How to re-calibrate (re-latch)

Restart the corrector node to re-enter the COLLECTING phase:
```bash
# Simplest: restart the entire pipeline
# Ctrl+C the launch, then re-launch

# Or restart just the corrector:
# Kill it and re-launch
ros2 launch canvas_pose_detector canvas_stroke_corrector.launch.py
```

---

## Unity Connection Issues

### Unity cannot connect to ROS-TCP-Endpoint

**Checks:**
1. Verify the endpoint is running:
   ```bash
   ros2 node list | grep tcp
   # Should show: /UnityEndpoint
   ```
2. Check the IP and port:
   ```bash
   hostname -I | awk '{print $1}'
   # Use this IP in Unity's ROS Settings, port 10000
   ```
3. Ensure no firewall blocks port 10000:
   ```bash
   sudo ufw allow 10000/tcp
   ```
4. Both machines must be on the same network

### Unity receives stale/old data

**Fix:** Restart the `ros_tcp_endpoint` node. In some configurations, disconnecting and reconnecting Unity without restarting the endpoint causes stale state.

---

## Robot Issues

### MoveIt Servo not accepting commands

**Symptom:** Twist commands publish but robot doesn't move.

**Checks:**
1. Verify the controller is active:
   ```bash
   ros2 control list_controllers
   ```
2. Ensure MoveIt Servo is running and its input topic matches (`/servo_node/delta_twist_cmds`)
3. Check QoS: the `unity_pose_to_servo_twist` node publishes with `BEST_EFFORT` reliability -- MoveIt Servo expects this

### Robot jumps when teleoperation enables

**Cause:** Unlikely with this design (the bridge latches zero on enable), but check:
- The Unity pose topic is publishing reasonable values
- `pos_gain` and `max_lin_vel` are not too high

---

## FAQ

### Can I run the system without a VR headset?

Yes. You can:
- Run the camera perception pipeline independently (`canvas_sim_pipeline.launch.py` or `canvas_real_pipeline.launch.py`)
- Publish fake Unity commands via the terminal for testing:
  ```bash
  ros2 topic pub /unity/teleop_enable std_msgs/Bool "data: true"
  ros2 topic pub /unity/target_pose geometry_msgs/PoseStamped \
    "{header: {frame_id: 'base_link'}, pose: {position: {x: 0.3, y: 0.0, z: 0.4}, orientation: {w: 1.0}}}"
  ```

### Can I run without a camera?

Yes. Use the sim pipeline:
```bash
ros2 launch canvas_pose_detector canvas_sim_pipeline.launch.py
```
This publishes identical topics and TF frames to the real detector. The rest of the pipeline (corrector, visualisers, Unity) cannot tell the difference.

### Can I run without a robot?

Yes. Use `use_fake_hardware:=true`:
```bash
ros2 launch ur_unity_bringup ur3e_unity_bridge.launch.py use_fake_hardware:=true
```

### What if only 2 or 3 markers are visible?

The detector handles partial detection:
- **3 markers:** Exact reconstruction via parallelogram rule (TL+BR = TR+BL)
- **2 adjacent markers:** Reconstruction via A4 aspect ratio
- **2 diagonal markers:** Rejected (insufficient data)
- **1 or 0 markers:** Rejected

Additionally, markers missing for fewer than `missing_frames_tol` (default 5) frames are held at their last position. The 3D cache holds positions for up to `last_known_tol` (default 300) frames (~15s) for robustness during wrist occlusion.

### How do I change the canvas size?

The detector defaults to A4 landscape (297 x 210 mm). Override via parameters:
```bash
ros2 param set /canvas_pose_detector canvas_physical_w_m 0.420
ros2 param set /canvas_pose_detector canvas_physical_h_m 0.297
```
Update the Unity canvas quad scale to match.

### Where are the debug visualisation images?

View in `rqt_image_view`:
```bash
ros2 run rqt_image_view rqt_image_view
```

| Topic | Content | Requires Camera |
|---|---|---|
| `/canvas/debug` | Annotated camera feed with marker overlays | Yes |
| `/canvas/correction_2d` | Top-down +/-150 mm 2D error plot | Yes |
| `/canvas/debug_corrected` | Camera overlay with drift arrow | Yes |
| `/canvas/depth_view` | XZ side-view canvas plane + brush | No |
| `/canvas/pose_2d_xy` | X vs Y lateral plot | No |
| `/canvas/pose_2d_xz` | X vs Z depth plot | No |
| `/canvas/pose_3d_tf` | 3D TF frame illustration | No |
