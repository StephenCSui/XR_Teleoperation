# Subsystem: ROS-TCP-Endpoint

## Purpose

Third-party package from [Unity Robotics](https://github.com/Unity-Technologies/ROS-TCP-Endpoint) that provides a TCP socket server enabling bidirectional communication between a Unity application and ROS 2. Unity publishes and subscribes to ROS topics through this endpoint.

**Package:** `ros_tcp_endpoint`
**Build type:** `ament_python`
**Source:** https://github.com/Unity-Technologies/ROS-TCP-Endpoint (ROS2v0.7 branch)

---

## How It Works

```
Unity App (C#)                           ROS 2 Network
     |                                        |
     |  ROS-TCP-Connector (Unity package)     |
     |                                        |
     +---- TCP socket (port 10000) ---------->+
     |                                        |
     |  Serialised ROS messages               |  ros_tcp_endpoint
     |  over raw TCP                          |  (Python server)
     |                                        |
     +<---------------------------------------+
```

- **Unity side:** Install the `ROS-TCP-Connector` package in Unity. It serializes/deserializes ROS message types and manages the TCP connection.
- **ROS side:** The `ros_tcp_endpoint` node listens on a TCP port and creates ROS publishers/subscribers dynamically as Unity registers topics.

---

## Launching

The endpoint is started automatically by `ur3e_unity_bridge.launch.py` and `canvas_real_pipeline.launch.py`. To launch it standalone:

```bash
ros2 launch ros_tcp_endpoint endpoint.py
```

Or as a node:

```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args \
  -p ROS_IP:=0.0.0.0 \
  -p ROS_TCP_PORT:=10000
```

---

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `ROS_IP` | `0.0.0.0` | IP address to bind the TCP server |
| `ROS_TCP_PORT` | `10000` | TCP port for Unity connections |

---

## Configuring Unity

In the Unity project:

1. Install **ROS-TCP-Connector** from the Unity Robotics Hub
2. Go to **Robotics > ROS Settings**
3. Set:
   - **ROS IP Address:** The Ubuntu machine's LAN IP
   - **ROS Port:** `10000`

Find the Ubuntu machine's IP:

```bash
hostname -I | awk '{print $1}'
```

---

## Topics Used in This Project

The following topics flow through the TCP endpoint:

### Unity -> ROS (Published by Unity)

| Topic | Type | Description |
|---|---|---|
| `/unity/target_pose` | `geometry_msgs/PoseStamped` | VR controller target pose |
| `/unity/teleop_enable` | `std_msgs/Bool` | Teleoperation enable/disable |
| `/canvas/stroke_target` | `geometry_msgs/PoseStamped` | Brush stroke position on canvas |

### ROS -> Unity (Subscribed by Unity)

| Topic | Type | Description |
|---|---|---|
| `/canvas/pose` | `geometry_msgs/PoseStamped` | Canvas centre pose |
| `/canvas/correction_data` | `std_msgs/String` | JSON status + drift data |
| `/canvas/debug` | `sensor_msgs/Image` | Live camera feed |
| TF frames | `tf2_msgs/TFMessage` | Canvas and marker transforms |

---

## Known Limitations

- Single Unity client per endpoint instance
- Large image topics (e.g., `/canvas/debug`) may cause latency on slow networks -- consider reducing `debug_fps` or `debug_scale` on the detector
- The endpoint must be restarted if the Unity application disconnects and reconnects (in some configurations)
