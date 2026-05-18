# Tuning

All parameters are in:
`src/ur_unity_bringup/launch/ur3e_unity_bridge_servo.launch.py`

No rebuild is required — parameters are read at launch time.

## Authority Filter (`unity_authority_filter_servo`)

### Open-space (NORMAL mode)

| Parameter | Value | Effect |
|---|---|---|
| `hand_deadband_m` | 0.003 | Ignore hand motion below this (m) |
| `max_cmd_step_m` | 0.005 | Max position step per tick (m) |
| `max_cmd_angle_step_deg` | 5.0 | Max orientation step per tick (°) |
| `position_scale` | 1.0 | Uniform position scaling |
| `anchor_settle_ms` | 150 | Settle time after teleop enable before anchor latches (ms) |
| `hand_timeout_ms` | 500 | Declare disconnect after this many ms with no hand message |

### Workspace bounds

| Parameter | Value |
|---|---|
| `x_min / x_max` | 0.10 / 0.80 m |
| `y_min / y_max` | -0.50 / 0.50 m |
| `z_min / z_max` | 0.05 / 0.70 m |

### Canvas plane constraint

| Parameter | Value | Effect |
|---|---|---|
| `canvas_stop_plane_x` | 0.45 | Robot holds here in base_link X (m) |
| `canvas_touch_plane_x` | 0.47 | Allowed limit when pen_down trigger is held (m) |

### PRECISION mode (open-space)

| Parameter | Value |
|---|---|
| `hand_deadband_m_precision` | 0.005 |
| `max_cmd_step_m_precision` | 0.008 |
| `max_cmd_angle_step_deg_precision` | 4.0 |
| `position_scale_precision` | 0.5 |

### Near-canvas tightening (NORMAL)

| Parameter | Value |
|---|---|
| `hand_deadband_m_near` | 0.008 |
| `max_cmd_step_m_near` | 0.004 |
| `max_cmd_angle_step_deg_near` | 3.0 |
| `position_scale_near` | 0.4 |
| `tangential_scale_near` | 0.6 |
| `normal_scale_near` | 0.15 |

### Nudge / detachment (PRECISION mode)

| Parameter | Value | Effect |
|---|---|---|
| `nudge_step_m` | 0.005 | Metres moved per button press |
| `nudge_step_deg` | 2.0 | Degrees rotated per button press |

## P-Controller (`pose_error_to_twist`)

| Parameter | Value | Effect |
|---|---|---|
| `linear_gain` | 2.0 | Position error → linear speed gain |
| `max_linear_speed` | 0.08 | Speed cap (m/s) |
| `angular_gain` | 3.0 | Orientation error → angular speed gain |
| `max_angular_speed` | 0.6 | Angular speed cap (rad/s) |
| `position_deadband_m` | 0.005 | Stop sending twist if error below this (m) |
| `angle_deadband_rad` | 0.03 | Stop sending twist if angle error below this (rad) |

## Oscillation / jackhammer

If the robot oscillates at higher teach pendant speeds:
- Increase `position_deadband_m` (reduces micro-corrections)
- Reduce `linear_gain`
- Reduce `max_linear_speed`
- Keep teach pendant at 40% or below
