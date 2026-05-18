#!/usr/bin/env python3
"""
Teleop mode/state manager.

Owns which control profile is active and publishes it for the filter to consume.
Does NOT generate robot motion — the filter owns all motion shaping.

Supported modes (this pass):
  NORMAL    — responsive hand-follow teleop (default)
  PRECISION — tighter, more conservative profile for detailed / contact-sensitive work

Runtime mode switching (no restart required):
  ros2 topic pub --once /teleop_mode/set_mode std_msgs/msg/String "data: 'PRECISION'"
  ros2 topic pub --once /teleop_mode/set_mode std_msgs/msg/String "data: 'NORMAL'"

TODO (later pass): wire the set_mode topic to a Unity controller input, gesture,
or UI element so the user can switch modes from inside the XR session.

Published topics (all transient_local/reliable — filter gets latest on late join):
  /teleop_mode/active_mode      std_msgs/String    — current mode name
  /teleop_mode/canvas_proximity std_msgs/Float64   — 0.0 far .. 1.0 full near-canvas
  /teleop_mode/canvas_normal    geometry_msgs/Vector3 — unit normal in base_link; zero if disabled

tool_tip_offset_xyz interpretation:
  Expressed in the EE/tool frame; rotated into base_link by the current EE orientation
  before canvas proximity is evaluated. This is the correct interpretation for a
  physical tool tip (pen, brush) whose position changes as the tool rotates.
  Example: [0, 0, 0.12] for a 120 mm tool along tool Z axis.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Vector3
from std_msgs.msg import String, Float64

# Modes with real behaviour implemented in this pass.
# Extend here as new modes are added.
KNOWN_MODES = {"NORMAL", "PRECISION"}


# ---------------------------------------------------------------------------
# Pure geometry helpers (no numpy)
# ---------------------------------------------------------------------------

def _norm3(v):
    return math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)


def _normalize3(v):
    n = _norm3(v)
    return (v[0] / n, v[1] / n, v[2] / n) if n > 1e-12 else (0.0, 0.0, 0.0)


def _dot3(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross3(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _sub3(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def _rotate_by_quat(q_xyzw, v):
    """
    Rotate vector v by unit quaternion q (x, y, z, w).
    Double cross-product form of Rodrigues' rotation formula.
    Converts an EE/tool-frame vector into base_link/world frame.
    """
    x, y, z, w = q_xyzw
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + y * tz - z * ty,
        vy + w * ty + z * tx - x * tz,
        vz + w * tz + x * ty - y * tx,
    )


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class TeleopModeManager(Node):

    def __init__(self) -> None:
        super().__init__("teleop_mode_manager")

        # ------------------------------------------------------------------ #
        # Parameters                                                           #
        # ------------------------------------------------------------------ #

        self._publish_rate_hz = float(
            self.declare_parameter("publish_rate_hz", 10.0).value
        )

        # Active mode at startup. Change at runtime via /teleop_mode/set_mode.
        initial_mode = self.declare_parameter("mode", "NORMAL").value
        self._mode = self._validate_mode(initial_mode)

        # -- Canvas (manual, base_link frame only for this pass) ------------ #
        self._canvas_enabled = bool(
            self.declare_parameter("canvas_enabled", False).value
        )
        self._canvas_frame = self.declare_parameter("canvas_frame", "base_link").value
        self._canvas_center = tuple(
            self.declare_parameter("canvas_center_xyz", [0.5, 0.0, 0.2]).value
        )
        self._canvas_normal_param = tuple(
            self.declare_parameter("canvas_normal_xyz", [1.0, 0.0, 0.0]).value
        )
        self._canvas_up_param = tuple(
            self.declare_parameter("canvas_up_xyz", [0.0, 0.0, 1.0]).value
        )
        self._canvas_width = float(self.declare_parameter("canvas_width", 0.3).value)
        self._canvas_height = float(self.declare_parameter("canvas_height", 0.3).value)
        self._canvas_edge_margin_m = float(
            self.declare_parameter("canvas_edge_margin_m", 0.05).value
        )
        self._enter_distance_m = float(
            self.declare_parameter("near_canvas_enter_distance_m", 0.15).value
        )
        self._exit_distance_m = float(
            self.declare_parameter("near_canvas_exit_distance_m", 0.20).value
        )
        self._full_precision_distance_m = float(
            self.declare_parameter("near_canvas_full_precision_distance_m", 0.02).value
        )

        # -- Tool tip offset (EE/tool-frame-relative) ----------------------- #
        # Rotated into base_link by EE orientation before proximity is checked.
        # TUNE: measure your physical tool geometry (metres, tool Z axis typical).
        self._tool_tip_offset = tuple(
            self.declare_parameter("tool_tip_offset_xyz", [0.0, 0.0, 0.0]).value
        )

        # -- Proximity smoothing -------------------------------------------- #
        # TUNE: higher alpha = slower transitions; lower = faster response.
        self._smoothing_alpha = float(
            self.declare_parameter("proximity_smoothing_alpha", 0.5).value
        )

        # -- Debug ---------------------------------------------------------- #
        self._debug_canvas = bool(
            self.declare_parameter("debug_canvas", True).value
        )

        # ------------------------------------------------------------------ #
        # Derived canvas geometry                                              #
        # ------------------------------------------------------------------ #
        self._canvas_valid = False
        self._canvas_normal = (0.0, 0.0, 0.0)
        self._canvas_right = (0.0, 0.0, 0.0)
        self._canvas_up_basis = (0.0, 0.0, 0.0)
        self._setup_canvas_basis()

        # ------------------------------------------------------------------ #
        # Runtime state                                                        #
        # ------------------------------------------------------------------ #
        self._ee_pose = None
        self._is_near = False
        self._proximity_smoothed = 0.0

        # ------------------------------------------------------------------ #
        # ROS setup                                                            #
        # ------------------------------------------------------------------ #
        latch_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )

        self._sub_ee = self.create_subscription(
            PoseStamped, "/robot/ee_pose", self._on_ee_pose, 10
        )

        # Runtime mode switching hook — future Unity input wires here.
        self._sub_set_mode = self.create_subscription(
            String, "/teleop_mode/set_mode", self._on_set_mode, 10
        )

        self._pub_mode = self.create_publisher(String, "/teleop_mode/active_mode", latch_qos)
        self._pub_proximity = self.create_publisher(Float64, "/teleop_mode/canvas_proximity", latch_qos)
        self._pub_canvas_normal = self.create_publisher(Vector3, "/teleop_mode/canvas_normal", latch_qos)

        period = 1.0 / max(self._publish_rate_hz, 1.0)
        self._timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f"[mode_manager] Started — mode={self._mode} "
            f"canvas_enabled={self._canvas_enabled} canvas_valid={self._canvas_valid}"
        )
        self.get_logger().info(
            "[mode_manager] tool_tip_offset is EE/tool-frame-relative "
            f"(rotated by EE orientation into base_link): {self._tool_tip_offset}"
        )
        if self._canvas_enabled and not self._canvas_valid:
            self.get_logger().warn(
                "[mode_manager] canvas_enabled=True but canvas geometry is invalid — "
                "check canvas_normal_xyz and canvas_up_xyz are not parallel."
            )
        if self._canvas_frame != "base_link":
            self.get_logger().warn(
                f"[mode_manager] canvas_frame='{self._canvas_frame}' — only 'base_link' is "
                "supported in this pass; canvas_center_xyz is treated as base_link."
            )

    # ------------------------------------------------------------------ #
    # Mode management                                                       #
    # ------------------------------------------------------------------ #

    def _validate_mode(self, mode: str) -> str:
        if mode in KNOWN_MODES:
            return mode
        self.get_logger().warn(
            f"[mode_manager] Unknown mode '{mode}' — keeping NORMAL. "
            f"Known modes: {sorted(KNOWN_MODES)}"
        )
        return "NORMAL"

    def _on_set_mode(self, msg: String) -> None:
        new_mode = self._validate_mode(msg.data.strip())
        if new_mode != self._mode:
            self.get_logger().info(f"[mode_manager] Mode: {self._mode} → {new_mode}")
            self._mode = new_mode

    # ------------------------------------------------------------------ #
    # Canvas basis                                                          #
    # ------------------------------------------------------------------ #

    def _setup_canvas_basis(self) -> None:
        n = _normalize3(self._canvas_normal_param)
        if _norm3(n) < 0.5:
            return
        up_raw = _normalize3(self._canvas_up_param)
        if _norm3(up_raw) < 0.5:
            return
        right = _normalize3(_cross3(n, up_raw))
        if _norm3(right) < 0.5:
            self.get_logger().warn(
                "[mode_manager] canvas_normal and canvas_up are parallel — canvas basis invalid."
            )
            return
        up = _normalize3(_cross3(right, n))
        self._canvas_normal = n
        self._canvas_right = right
        self._canvas_up_basis = up
        self._canvas_valid = True

    # ------------------------------------------------------------------ #
    # Subscriptions                                                         #
    # ------------------------------------------------------------------ #

    def _on_ee_pose(self, msg: PoseStamped) -> None:
        self._ee_pose = msg

    # ------------------------------------------------------------------ #
    # Timer                                                                #
    # ------------------------------------------------------------------ #

    def _on_timer(self) -> None:
        raw = self._compute_proximity()
        alpha = _clamp(self._smoothing_alpha, 0.0, 0.9999)
        self._proximity_smoothed = alpha * self._proximity_smoothed + (1.0 - alpha) * raw
        self._publish()

    # ------------------------------------------------------------------ #
    # Tool tip (EE-frame offset → base_link)                               #
    # ------------------------------------------------------------------ #

    def _tool_tip_in_base(self, p, q) -> tuple:
        """
        Return tool tip position in base_link.
        p: (x, y, z) EE position in base_link.
        q: geometry_msgs.Quaternion — EE orientation in base_link.
        The offset vector is in EE/tool frame and is rotated into base_link
        by the current EE orientation before being added to EE position.
        """
        ox, oy, oz = self._tool_tip_offset
        if abs(ox) < 1e-12 and abs(oy) < 1e-12 and abs(oz) < 1e-12:
            return p  # fast path: no offset configured
        world_offset = _rotate_by_quat((q.x, q.y, q.z, q.w), (ox, oy, oz))
        return (p[0] + world_offset[0], p[1] + world_offset[1], p[2] + world_offset[2])

    # ------------------------------------------------------------------ #
    # Proximity                                                             #
    # ------------------------------------------------------------------ #

    def _compute_proximity(self) -> float:
        """
        Raw proximity factor [0.0, 1.0].
        0.0 = far from canvas or canvas disabled.
        1.0 = at full-precision distance from canvas surface.
        Hysteresis maintained via _is_near state.
        """
        if not self._canvas_enabled or not self._canvas_valid or self._ee_pose is None:
            return 0.0

        p = self._ee_pose.pose.position
        q = self._ee_pose.pose.orientation
        tip = self._tool_tip_in_base((p.x, p.y, p.z), q)

        rel = _sub3(tip, self._canvas_center)
        signed_dist = _dot3(rel, self._canvas_normal)
        dist = abs(signed_dist)

        proj = (
            rel[0] - self._canvas_normal[0] * signed_dist,
            rel[1] - self._canvas_normal[1] * signed_dist,
            rel[2] - self._canvas_normal[2] * signed_dist,
        )
        u = _dot3(proj, self._canvas_right)
        v = _dot3(proj, self._canvas_up_basis)

        half_w = self._canvas_width * 0.5 + self._canvas_edge_margin_m
        half_h = self._canvas_height * 0.5 + self._canvas_edge_margin_m
        inside_bounds = (abs(u) <= half_w) and (abs(v) <= half_h)

        if self._is_near:
            if not inside_bounds or dist > self._exit_distance_m:
                self._is_near = False
        else:
            if inside_bounds and dist < self._enter_distance_m:
                self._is_near = True

        if not self._is_near:
            raw = 0.0
        else:
            enter = self._enter_distance_m
            full = self._full_precision_distance_m
            raw = _clamp((enter - dist) / (enter - full), 0.0, 1.0) if enter > full else 1.0

        if self._debug_canvas:
            self.get_logger().info(
                f"[mode_manager|canvas] mode={self._mode} dist={dist:.4f}m "
                f"u={u:.3f}m v={v:.3f}m inside={inside_bounds} "
                f"is_near={self._is_near} raw={raw:.3f} smoothed={self._proximity_smoothed:.3f}",
                throttle_duration_sec=0.5,
            )

        return raw

    # ------------------------------------------------------------------ #
    # Publishing                                                            #
    # ------------------------------------------------------------------ #

    def _publish(self) -> None:
        mode_msg = String()
        mode_msg.data = self._mode
        self._pub_mode.publish(mode_msg)

        prox_msg = Float64()
        prox_msg.data = self._proximity_smoothed
        self._pub_proximity.publish(prox_msg)

        cn_msg = Vector3()
        if self._canvas_enabled and self._canvas_valid:
            cn_msg.x = self._canvas_normal[0]
            cn_msg.y = self._canvas_normal[1]
            cn_msg.z = self._canvas_normal[2]
        # else zero vector — tells filter: no tangential/normal decomposition
        self._pub_canvas_normal.publish(cn_msg)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = TeleopModeManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
