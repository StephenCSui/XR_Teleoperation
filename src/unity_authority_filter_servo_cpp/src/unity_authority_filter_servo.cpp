#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <string>

using geometry_msgs::msg::PoseStamped;

static constexpr double kPi = 3.14159265358979323846;

// ---------------------------------------------------------------------------
// Scalar / vector helpers
// ---------------------------------------------------------------------------

static double clamp_scalar(double x, double lo, double hi)
{
  return std::max(lo, std::min(hi, x));
}

static double lerp(double a, double b, double t)
{
  return a + (b - a) * clamp_scalar(t, 0.0, 1.0);
}

// ---------------------------------------------------------------------------
// Tuning profile — one set of motion parameters for a mode × proximity pair
// ---------------------------------------------------------------------------

struct TuningProfile {
  double hand_deadband_m;
  double max_cmd_step_m;
  double max_cmd_angle_step_rad;
  double position_scale;      // uniform scale when canvas normal unavailable
  double tangential_scale;    // along-canvas scale when canvas normal available
  double normal_scale;        // into-canvas scale when canvas normal available
};

static TuningProfile blend_profiles(
  const TuningProfile & a, const TuningProfile & b, double t)
{
  return {
    lerp(a.hand_deadband_m,        b.hand_deadband_m,        t),
    lerp(a.max_cmd_step_m,         b.max_cmd_step_m,         t),
    lerp(a.max_cmd_angle_step_rad, b.max_cmd_angle_step_rad, t),
    lerp(a.position_scale,         b.position_scale,         t),
    lerp(a.tangential_scale,       b.tangential_scale,       t),
    lerp(a.normal_scale,           b.normal_scale,           t),
  };
}

static geometry_msgs::msg::Point make_point(double x, double y, double z)
{
  geometry_msgs::msg::Point p;
  p.x = x; p.y = y; p.z = z;
  return p;
}

static geometry_msgs::msg::Point point_add(
  const geometry_msgs::msg::Point & a,
  const geometry_msgs::msg::Point & b)
{
  return make_point(a.x + b.x, a.y + b.y, a.z + b.z);
}

static geometry_msgs::msg::Point point_sub(
  const geometry_msgs::msg::Point & a,
  const geometry_msgs::msg::Point & b)
{
  return make_point(a.x - b.x, a.y - b.y, a.z - b.z);
}

static geometry_msgs::msg::Point point_scale(
  const geometry_msgs::msg::Point & a, double s)
{
  return make_point(a.x * s, a.y * s, a.z * s);
}

static double point_norm(const geometry_msgs::msg::Point & p)
{
  return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

static geometry_msgs::msg::Point clamp_point_norm(
  const geometry_msgs::msg::Point & p, double max_norm)
{
  const double n = point_norm(p);
  if (n <= max_norm || n < 1e-12) { return p; }
  return point_scale(p, max_norm / n);
}

// ---------------------------------------------------------------------------
// Coordinate frame conversions
// Unity: +X right, +Y up, +Z forward
// ROS:   +X forward, +Y left, +Z up
// ---------------------------------------------------------------------------

static geometry_msgs::msg::Point unity_to_ros_delta(const geometry_msgs::msg::Point & u)
{
  // Base rotated 180° around Z: ROS X = -Unity Z, ROS Y = +Unity X, ROS Z = +Unity Y
  return make_point(-u.z, u.x, u.y);
}

static geometry_msgs::msg::Point ros_to_unity_position(const geometry_msgs::msg::Point & r)
{
  return make_point(-r.y, r.z, r.x);
}

static Eigen::Matrix3d unity_to_ros_basis()
{
  Eigen::Matrix3d c;
  // Base rotated 180° around Z: ROS X = -Unity Z, ROS Y = +Unity X, ROS Z = +Unity Y
  c <<  0.0,  0.0, -1.0,
        1.0,  0.0,  0.0,
        0.0,  1.0,  0.0;
  return c;
}

// ---------------------------------------------------------------------------
// Quaternion helpers
// ---------------------------------------------------------------------------

static Eigen::Quaterniond msg_to_eigen(const geometry_msgs::msg::Quaternion & q)
{
  return Eigen::Quaterniond(q.w, q.x, q.y, q.z);
}

static geometry_msgs::msg::Quaternion eigen_to_msg(const Eigen::Quaterniond & q_in)
{
  Eigen::Quaterniond q = q_in.normalized();
  geometry_msgs::msg::Quaternion out;
  out.x = q.x(); out.y = q.y(); out.z = q.z(); out.w = q.w();
  return out;
}

static geometry_msgs::msg::Quaternion unity_quat_to_ros_base(
  const geometry_msgs::msg::Quaternion & q_unity_msg)
{
  const Eigen::Quaterniond q_unity = msg_to_eigen(q_unity_msg).normalized();
  const Eigen::Matrix3d r_unity = q_unity.toRotationMatrix();
  const Eigen::Matrix3d c = unity_to_ros_basis();
  const Eigen::Matrix3d r_ros = c * r_unity * c.transpose();
  return eigen_to_msg(Eigen::Quaterniond(r_ros));
}

static double quat_shortest_angle(const Eigen::Quaterniond & q_in)
{
  Eigen::Quaterniond q = q_in.normalized();
  if (q.w() < 0.0) { q.coeffs() *= -1.0; }
  const double w = clamp_scalar(q.w(), -1.0, 1.0);
  return 2.0 * std::acos(w);
}

static Eigen::Quaterniond clamp_relative_quat_angle(
  const Eigen::Quaterniond & q_rel_in, double max_angle_rad)
{
  Eigen::Quaterniond q_rel = q_rel_in.normalized();
  if (q_rel.w() < 0.0) { q_rel.coeffs() *= -1.0; }
  const double angle = quat_shortest_angle(q_rel);
  if (angle <= max_angle_rad || angle < 1e-12) { return q_rel; }
  const double t = max_angle_rad / angle;
  return Eigen::Quaterniond::Identity().slerp(t, q_rel).normalized();
}

static Eigen::Quaterniond zero_small_relative_quat(
  const Eigen::Quaterniond & q_rel_in, double deadband_rad)
{
  Eigen::Quaterniond q_rel = q_rel_in.normalized();
  const double angle = quat_shortest_angle(q_rel);
  if (angle < deadband_rad) { return Eigen::Quaterniond::Identity(); }
  return q_rel;
}

static Eigen::Quaterniond step_clamp_world_quat(
  const Eigen::Quaterniond & q_last_in,
  const Eigen::Quaterniond & q_target_in,
  double max_step_rad)
{
  Eigen::Quaterniond q_last   = q_last_in.normalized();
  Eigen::Quaterniond q_target = q_target_in.normalized();

  Eigen::Quaterniond q_err = q_last.conjugate() * q_target;
  if (q_err.w() < 0.0) {
    q_target.coeffs() *= -1.0;
    q_err = q_last.conjugate() * q_target;
  }

  const double angle = quat_shortest_angle(q_err);
  if (angle <= max_step_rad || angle < 1e-12) { return q_target; }
  const double t = max_step_rad / angle;
  return q_last.slerp(t, q_target).normalized();
}

static Eigen::Quaterniond unity_relative_world_to_ros_relative(
  const geometry_msgs::msg::Quaternion & q_anchor_unity_msg,
  const geometry_msgs::msg::Quaternion & q_curr_unity_msg)
{
  const Eigen::Quaterniond q_anchor_u = msg_to_eigen(q_anchor_unity_msg).normalized();
  const Eigen::Quaterniond q_curr_u   = msg_to_eigen(q_curr_unity_msg).normalized();

  const Eigen::Matrix3d r_rel_u_world =
    q_curr_u.toRotationMatrix() * q_anchor_u.toRotationMatrix().transpose();

  const Eigen::Matrix3d c = unity_to_ros_basis();
  return Eigen::Quaterniond(c * r_rel_u_world * c.transpose()).normalized();
}

// ---------------------------------------------------------------------------
// RPY debug helper
// ---------------------------------------------------------------------------

struct RPY { double roll, pitch, yaw; };

static RPY quat_to_rpy(const geometry_msgs::msg::Quaternion & q)
{
  const double x = q.x, y = q.y, z = q.z, w = q.w;

  const double sinr_cosp = 2.0 * (w * x + y * z);
  const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
  const double roll = std::atan2(sinr_cosp, cosr_cosp);

  const double sinp = 2.0 * (w * y - z * x);
  const double pitch = (std::abs(sinp) >= 1.0)
    ? std::copysign(kPi / 2.0, sinp)
    : std::asin(sinp);

  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  const double yaw = std::atan2(siny_cosp, cosy_cosp);

  return {roll, pitch, yaw};
}

static double rad2deg(double x) { return x * 180.0 / kPi; }

// ---------------------------------------------------------------------------
// Node
// ---------------------------------------------------------------------------

class UnityAuthorityFilterServo : public rclcpp::Node
{
public:
  UnityAuthorityFilterServo()
  : Node("unity_authority_filter_servo")
  {
    // ------------------------------------------------------------------ //
    // Existing parameters (unchanged — serve as "far" baseline)           //
    // ------------------------------------------------------------------ //
    hand_topic_          = declare_parameter<std::string>("hand_topic",         "/unity/hand_pose");
    robot_ee_topic_      = declare_parameter<std::string>("robot_ee_topic",     "/robot/ee_pose");
    command_topic_       = declare_parameter<std::string>("command_topic",       "/unity/command_pose");
    teleop_enable_topic_ = declare_parameter<std::string>("teleop_enable_topic", "/unity/teleop_enabled");
    base_frame_          = declare_parameter<std::string>("base_frame",         "base_link");

    rate_hz_         = declare_parameter<double>("rate_hz",          60.0);
    position_scale_  = declare_parameter<double>("position_scale",   1.0);

    hand_deadband_m_    = declare_parameter<double>("hand_deadband_m",    0.003);
    max_total_delta_m_  = declare_parameter<double>("max_total_delta_m",  0.20);
    max_cmd_step_m_     = declare_parameter<double>("max_cmd_step_m",     0.01);

    x_min_ = declare_parameter<double>("x_min", 0.10);
    x_max_ = declare_parameter<double>("x_max", 0.80);
    y_min_ = declare_parameter<double>("y_min", -0.50);
    y_max_ = declare_parameter<double>("y_max",  0.50);
    z_min_ = declare_parameter<double>("z_min", 0.05);
    z_max_ = declare_parameter<double>("z_max", 0.70);

    orientation_mode_        = declare_parameter<std::string>("orientation_mode",        "full_relative");
    angular_deadband_deg_    = declare_parameter<double>("angular_deadband_deg",    1.5);
    max_angle_from_anchor_deg_ = declare_parameter<double>("max_angle_from_anchor_deg", 45.0);
    max_cmd_angle_step_deg_  = declare_parameter<double>("max_cmd_angle_step_deg",  5.0);

    debug_verbose_        = declare_parameter<bool>("debug_verbose",        false);
    debug_rpy_            = declare_parameter<bool>("debug_rpy",            false);
    debug_raw_hand_quat_  = declare_parameter<bool>("debug_raw_hand_quat",  false);
    debug_ee_forward_     = declare_parameter<bool>("debug_ee_forward",     false);

    angular_deadband_rad_      = angular_deadband_deg_      * kPi / 180.0;
    max_angle_from_anchor_rad_ = max_angle_from_anchor_deg_ * kPi / 180.0;
    max_cmd_angle_step_rad_    = max_cmd_angle_step_deg_    * kPi / 180.0;

    // Time without a hand message before treating Unity as disconnected.
    // Should be well above normal inter-message gap (1000/60Hz ≈ 17 ms).
    hand_timeout_ms_ = declare_parameter<double>("hand_timeout_ms", 500.0);

    // Ticks to hold position after teleop enables before latching anchor.
    // Gives Unity time to complete its hand-snap-to-EE before the anchor locks.
    // At 60 Hz, 6 ticks = 100 ms. Increase if the initial jump persists.
    anchor_settle_ticks_ = static_cast<int>(
      declare_parameter<double>("anchor_settle_ms", 150.0) / 1000.0 * std::max(rate_hz_, 1.0));

    // ------------------------------------------------------------------ //
    // Tuning profiles                                                      //
    // Four profiles: NORMAL×{far,near}, PRECISION×{far,near}              //
    // Proximity blends far→near; active_mode_ selects which pair to use.  //
    // ------------------------------------------------------------------ //

    // NORMAL far — reuse existing "far" param values loaded above
    normal_far_ = {
      hand_deadband_m_,
      max_cmd_step_m_,
      max_cmd_angle_step_rad_,
      position_scale_,
      position_scale_,   // tangential = position_scale when far from canvas
      position_scale_,   // normal     = position_scale when far from canvas
    };

    // NORMAL near — drawing-surface tightening (existing param names kept)
    {
      const double db  = declare_parameter<double>("hand_deadband_m_near",        0.008);
      const double stp = declare_parameter<double>("max_cmd_step_m_near",         0.004);
      const double ang = declare_parameter<double>("max_cmd_angle_step_deg_near",  3.0) * kPi / 180.0;
      const double ps  = declare_parameter<double>("position_scale_near",         0.4);
      const double ts  = declare_parameter<double>("tangential_scale_near",       0.6);
      const double ns  = declare_parameter<double>("normal_scale_near",           0.15);
      normal_near_ = { db, stp, ang, ps, ts, ns };
    }

    // PRECISION far — open-space, tighter than NORMAL far
    {
      const double db  = declare_parameter<double>("hand_deadband_m_precision",        0.005);
      const double stp = declare_parameter<double>("max_cmd_step_m_precision",         0.008);
      const double ang = declare_parameter<double>("max_cmd_angle_step_deg_precision",  4.0) * kPi / 180.0;
      const double ps  = declare_parameter<double>("position_scale_precision",         0.5);
      precision_far_ = { db, stp, ang, ps, ps, ps };
    }

    // PRECISION near — contact / detailed work
    {
      const double db  = declare_parameter<double>("hand_deadband_m_precision_near",        0.010);
      const double stp = declare_parameter<double>("max_cmd_step_m_precision_near",         0.003);
      const double ang = declare_parameter<double>("max_cmd_angle_step_deg_precision_near",  2.0) * kPi / 180.0;
      const double ps  = declare_parameter<double>("position_scale_precision_near",         0.3);
      const double ts  = declare_parameter<double>("tangential_scale_precision_near",       0.4);
      const double ns  = declare_parameter<double>("normal_scale_precision_near",           0.10);
      precision_near_ = { db, stp, ang, ps, ts, ns };
    }

    // Debug canvas state and blended values
    debug_canvas_ = declare_parameter<bool>("debug_canvas", false);

    // ------------------------------------------------------------------ //
    // Precision bounding box                                               //
    // Unity draws a box; its size drives position_scale in PRECISION mode. //
    // Smaller box = finer control (lower scale).                           //
    // ------------------------------------------------------------------ //
    precision_box_min_m_    = declare_parameter<double>("precision_box_min_m",    0.02);
    precision_box_max_m_    = declare_parameter<double>("precision_box_max_m",    0.05);
    precision_scale_at_min_ = declare_parameter<double>("precision_scale_at_min", 0.2);
    precision_scale_at_max_ = declare_parameter<double>("precision_scale_at_max", 0.5);

    // ------------------------------------------------------------------ //
    // NEW: mode manager subscriptions                                      //
    // ------------------------------------------------------------------ //
    const std::string mode_topic =
      declare_parameter<std::string>("mode_topic",          "/teleop_mode/active_mode");
    const std::string proximity_topic =
      declare_parameter<std::string>("proximity_topic",     "/teleop_mode/canvas_proximity");
    const std::string canvas_normal_topic =
      declare_parameter<std::string>("canvas_normal_topic", "/teleop_mode/canvas_normal");

    // Transient-local QoS matches what the mode manager publishes
    rclcpp::QoS latch_qos(1);
    latch_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    latch_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);

    sub_mode_ = create_subscription<std_msgs::msg::String>(
      mode_topic, latch_qos,
      [this](std_msgs::msg::String::SharedPtr msg) {
        active_mode_ = msg->data;
      });

    sub_proximity_ = create_subscription<std_msgs::msg::Float64>(
      proximity_topic, latch_qos,
      [this](std_msgs::msg::Float64::SharedPtr msg) {
        canvas_proximity_ = msg->data;
      });

    sub_canvas_normal_ = create_subscription<geometry_msgs::msg::Vector3>(
      canvas_normal_topic, latch_qos,
      [this](geometry_msgs::msg::Vector3::SharedPtr msg) {
        canvas_normal_ros_ = *msg;
        const double n = std::sqrt(
          msg->x * msg->x + msg->y * msg->y + msg->z * msg->z);
        have_canvas_normal_ = (n > 0.5);
      });

    // ------------------------------------------------------------------ //
    // Precision bounding box subscriptions                                 //
    // ------------------------------------------------------------------ //
    const std::string precision_box_topic =
      declare_parameter<std::string>("precision_box_topic", "/unity/precision_box");
    const std::string precision_box_size_topic =
      declare_parameter<std::string>("precision_box_size_topic", "/unity/precision_box_size");
    const std::string precision_exit_topic =
      declare_parameter<std::string>("precision_exit_topic", "/teleop_mode/precision_exit");

    sub_precision_box_ = create_subscription<PoseStamped>(
      precision_box_topic, 10,
      [this](PoseStamped::SharedPtr msg) {
        precision_box_center_unity_ = msg->pose.position;
        have_precision_box_ = true;
      });

    sub_precision_box_size_ = create_subscription<geometry_msgs::msg::Vector3>(
      precision_box_size_topic, 10,
      [this](geometry_msgs::msg::Vector3::SharedPtr msg) {
        precision_box_size_unity_ = *msg;
        // Geometric mean of x and y (the two planar dimensions) drives the scale.
        // Clamped to [precision_box_min_m_, precision_box_max_m_].
        const double sx = std::max(msg->x, 1e-6);
        const double sy = std::max(msg->y, 1e-6);
        const double effective = std::sqrt(sx * sy);
        const double t = clamp_scalar(
          (effective - precision_box_min_m_) /
          std::max(precision_box_max_m_ - precision_box_min_m_, 1e-9),
          0.0, 1.0);
        precision_box_scale_computed_ = lerp(precision_scale_at_min_, precision_scale_at_max_, t);
      });

    pub_precision_exit_ = create_publisher<std_msgs::msg::Bool>(precision_exit_topic, 10);

    // ------------------------------------------------------------------ //
    // Detachment / nudge mode                                              //
    // Active only when mode == PRECISION.                                  //
    // While detached, hand tracking is suspended; Unity sends discrete    //
    // nudge increments via nudge_cmd (Twist) which are applied once each. //
    //                                                                      //
    // linear  (Unity frame): position nudge direction (unit ±1.0 per axis)//
    // angular (ROS frame):   orientation nudge direction (unit ±1.0)       //
    // Step sizes are scaled by nudge_step_m_ and nudge_step_rad_.         //
    // ------------------------------------------------------------------ //
    nudge_step_m_   = declare_parameter<double>("nudge_step_m",   0.005);
    nudge_step_rad_ = declare_parameter<double>("nudge_step_deg", 2.0) * kPi / 180.0;

    sub_detach_mode_ = create_subscription<std_msgs::msg::Bool>(
      declare_parameter<std::string>("detach_mode_topic", "/unity/detach_mode"), 10,
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        const bool was = is_detached_;
        is_detached_ = msg->data;
        if (is_detached_ != was) {
          RCLCPP_INFO(get_logger(), "[DETACH] detach_mode=%d", is_detached_ ? 1 : 0);
        }
      });

    sub_nudge_cmd_ = create_subscription<geometry_msgs::msg::Twist>(
      declare_parameter<std::string>("nudge_cmd_topic", "/unity/nudge_cmd"), 10,
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        pending_nudge_ = *msg;
        have_nudge_    = true;
      });

    // ------------------------------------------------------------------ //
    // Canvas plane constraint                                              //
    // EE cannot pass canvas_stop_plane_x_ in base_link X during normal   //
    // operation. When pen_down is true, limit relaxes to                  //
    // canvas_touch_plane_x_, allowing the tool to press onto the surface. //
    // ------------------------------------------------------------------ //
    canvas_stop_plane_x_  = declare_parameter<double>("canvas_stop_plane_x",  0.42);
    canvas_touch_plane_x_ = declare_parameter<double>("canvas_touch_plane_x", 0.44);

    const std::string pen_down_topic =
      declare_parameter<std::string>("pen_down_topic", "/unity/pen_down");

    sub_pen_down_ = create_subscription<std_msgs::msg::Bool>(
      pen_down_topic, 10,
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        pen_down_ = msg->data;
      });

    // ------------------------------------------------------------------ //
    // Existing subscriptions and publisher                                 //
    // ------------------------------------------------------------------ //
    sub_hand_ = create_subscription<PoseStamped>(
      hand_topic_, 10,
      std::bind(&UnityAuthorityFilterServo::on_hand, this, std::placeholders::_1));

    sub_ee_ = create_subscription<PoseStamped>(
      robot_ee_topic_, 10,
      std::bind(&UnityAuthorityFilterServo::on_ee, this, std::placeholders::_1));

    sub_enable_ = create_subscription<std_msgs::msg::Bool>(
      teleop_enable_topic_, 10,
      std::bind(&UnityAuthorityFilterServo::on_enable, this, std::placeholders::_1));

    pub_command_ = create_publisher<PoseStamped>(command_topic_, 10);

    const double period = 1.0 / std::max(1.0, rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(period),
      std::bind(&UnityAuthorityFilterServo::on_timer, this));

    RCLCPP_INFO(get_logger(), "unity_authority_filter_servo started");
    RCLCPP_INFO(get_logger(), "orientation_mode: %s", orientation_mode_.c_str());
  }

private:
  // -------------------------------------------------------------------- //
  // Existing callbacks                                                     //
  // -------------------------------------------------------------------- //

  void on_hand(const PoseStamped::SharedPtr msg)
  {
    latest_hand_      = *msg;
    have_hand_        = true;
    last_hand_time_   = get_clock()->now();
  }

  void on_ee(const PoseStamped::SharedPtr msg)     { latest_ee_   = *msg; have_ee_   = true; }

  void on_enable(const std_msgs::msg::Bool::SharedPtr msg)
  {
    const bool old = teleop_enabled_;
    teleop_enabled_ = msg->data;
    if (teleop_enabled_ != old) {
      RCLCPP_INFO(get_logger(), "[ENABLE] teleop_enabled=%d", teleop_enabled_ ? 1 : 0);
    }
  }

  void clear_anchor()
  {
    anchor_latched_       = false;
    have_last_command_    = false;
    is_detached_          = false;
    have_nudge_           = false;
    canvas_plane_armed_   = false;
  }

  void latch_anchor()
  {
    hand_anchor_ = latest_hand_;
    ee_anchor_   = latest_ee_;
    anchor_latched_ = true;

    last_command_ = latest_ee_;
    last_command_.header.frame_id = base_frame_;
    have_last_command_ = true;

    RCLCPP_INFO(
      get_logger(),
      "[ANCHOR] latched hand=(%.3f, %.3f, %.3f) ee=(%.3f, %.3f, %.3f)",
      hand_anchor_.pose.position.x, hand_anchor_.pose.position.y, hand_anchor_.pose.position.z,
      ee_anchor_.pose.position.x,   ee_anchor_.pose.position.y,   ee_anchor_.pose.position.z);
  }

  geometry_msgs::msg::Point clamp_workspace(const geometry_msgs::msg::Point & p) const
  {
    return make_point(
      clamp_scalar(p.x, x_min_, x_max_),
      clamp_scalar(p.y, y_min_, y_max_),
      clamp_scalar(p.z, z_min_, z_max_));
  }

  // -------------------------------------------------------------------- //
  // Orientation command — angle_step_rad is blended by caller             //
  // -------------------------------------------------------------------- //

  geometry_msgs::msg::Quaternion compute_command_orientation(double angle_step_rad)
  {
    if (orientation_mode_ == "fixed") {
      return ee_anchor_.pose.orientation;
    }

    Eigen::Quaterniond q_rel = unity_relative_world_to_ros_relative(
      hand_anchor_.pose.orientation, latest_hand_.pose.orientation);

    q_rel = zero_small_relative_quat(q_rel, angular_deadband_rad_);
    q_rel = clamp_relative_quat_angle(q_rel, max_angle_from_anchor_rad_);

    const Eigen::Quaterniond q_ee_anchor =
      msg_to_eigen(ee_anchor_.pose.orientation).normalized();

    Eigen::Quaterniond q_target = (q_rel * q_ee_anchor).normalized();

    if (have_last_command_) {
      const Eigen::Quaterniond q_last =
        msg_to_eigen(last_command_.pose.orientation).normalized();
      q_target = step_clamp_world_quat(q_last, q_target, angle_step_rad);
    }

    return eigen_to_msg(q_target);
  }

  void publish_current_ee_as_command()
  {
    if (!have_ee_) { return; }

    PoseStamped cmd = latest_ee_;
    cmd.header.stamp = get_clock()->now();
    cmd.header.frame_id = base_frame_;
    pub_command_->publish(cmd);

    last_command_ = cmd;
    have_last_command_ = true;
  }

  // -------------------------------------------------------------------- //
  // Main timer                                                             //
  // -------------------------------------------------------------------- //

  void on_timer()
  {
    // ------------------------------------------------------------------ //
    // Disconnection detection                                              //
    // If no hand message arrives for hand_timeout_ms_, treat Unity as     //
    // disconnected: clear the anchor so reconnection starts fresh.        //
    // Without this, a stale anchor from the previous session causes the   //
    // robot to teleport on reconnect (teleop_enabled never goes false).   //
    // ------------------------------------------------------------------ //
    if (have_hand_) {
      const double age_ms =
        (get_clock()->now() - last_hand_time_).seconds() * 1000.0;
      if (age_ms > hand_timeout_ms_) {
        if (anchor_latched_ || teleop_enabled_) {
          RCLCPP_WARN(get_logger(),
            "[DISCONNECT] No hand pose for %.0f ms — clearing anchor and teleop state",
            age_ms);
          clear_anchor();
          teleop_enabled_      = false;
          prev_teleop_enabled_ = false;
        }
      }
    }

    if (!have_hand_ || !have_ee_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "[WAIT_INPUTS] hand=%d ee=%d teleop=%d",
        have_hand_ ? 1 : 0, have_ee_ ? 1 : 0, teleop_enabled_ ? 1 : 0);
      return;
    }

    // ------------------------------------------------------------------ //
    // Select profile pair by mode, blend by canvas proximity              //
    // prox = 0.0 → far profile unchanged; prox = 1.0 → full near profile //
    // ------------------------------------------------------------------ //
    const double prox = clamp_scalar(canvas_proximity_, 0.0, 1.0);

    const bool is_precision = (active_mode_ == "PRECISION");
    const TuningProfile & far_prof  = is_precision ? precision_far_  : normal_far_;
    const TuningProfile & near_prof = is_precision ? precision_near_ : normal_near_;
    TuningProfile active = blend_profiles(far_prof, near_prof, prox);

    // When a precision box is defined, replace position_scale at the far end
    // with the box-derived value.  Near-canvas end stays as configured.
    if (is_precision && have_precision_box_) {
      active.position_scale   = lerp(precision_box_scale_computed_, near_prof.position_scale,   prox);
      active.tangential_scale = lerp(precision_box_scale_computed_, near_prof.tangential_scale, prox);
    }

    const double blended_deadband   = active.hand_deadband_m;
    const double blended_max_step   = active.max_cmd_step_m;
    const double blended_angle_step = active.max_cmd_angle_step_rad;

    // ------------------------------------------------------------------ //
    // Teleop enable transitions                                            //
    // ------------------------------------------------------------------ //
    if (teleop_enabled_ != prev_teleop_enabled_) {
      if (teleop_enabled_) {
        clear_anchor();
        settle_ticks_remaining_ = anchor_settle_ticks_;
        RCLCPP_INFO(get_logger(),
          "[ENABLE] teleop on — holding %d ticks (%.0f ms) for hand snap to settle",
          anchor_settle_ticks_,
          anchor_settle_ticks_ * 1000.0 / std::max(rate_hz_, 1.0));
      } else {
        clear_anchor();
        publish_current_ee_as_command();
      }
      prev_teleop_enabled_ = teleop_enabled_;
    }

    if (!teleop_enabled_) {
      publish_current_ee_as_command();
      return;
    }

    // Hold position during settle window so any stale hand pose cannot drive motion.
    if (settle_ticks_remaining_ > 0) {
      --settle_ticks_remaining_;
      publish_current_ee_as_command();
      return;
    }

    if (!anchor_latched_) {
      latch_anchor();
    }

    // ------------------------------------------------------------------ //
    // Detachment mode: suspend hand tracking, apply discrete nudges       //
    // Only active when mode == PRECISION.                                  //
    // Each nudge message is consumed exactly once; position is held       //
    // between nudges.  Bypasses precision box bounds check.               //
    // ------------------------------------------------------------------ //
    if (is_precision && is_detached_) {
      if (have_nudge_ && have_last_command_) {
        have_nudge_ = false;

        // Position nudge: Unity linear → ROS delta, scaled by nudge_step_m_
        const geometry_msgs::msg::Point nudge_delta_unity = make_point(
          pending_nudge_.linear.x * nudge_step_m_,
          pending_nudge_.linear.y * nudge_step_m_,
          pending_nudge_.linear.z * nudge_step_m_);
        const geometry_msgs::msg::Point nudge_delta_ros = unity_to_ros_delta(nudge_delta_unity);
        last_command_.pose.position =
          clamp_workspace(point_add(last_command_.pose.position, nudge_delta_ros));

        // Orientation nudge: angular (ROS world frame), scaled by nudge_step_rad_
        // Pre-multiply = rotate in world frame.  angular.x/y/z → roll/pitch/yaw.
        const double rx = pending_nudge_.angular.x * nudge_step_rad_;
        const double ry = pending_nudge_.angular.y * nudge_step_rad_;
        const double rz = pending_nudge_.angular.z * nudge_step_rad_;
        if (std::abs(rx) + std::abs(ry) + std::abs(rz) > 1e-9) {
          const Eigen::Quaterniond q_nudge =
            (Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX())).normalized();
          const Eigen::Quaterniond q_cmd =
            msg_to_eigen(last_command_.pose.orientation).normalized();
          last_command_.pose.orientation = eigen_to_msg((q_nudge * q_cmd).normalized());
        }
      }

      // Publish held / nudged command
      if (!have_last_command_) {
        publish_current_ee_as_command();
      } else {
        PoseStamped cmd = last_command_;
        cmd.header.stamp    = get_clock()->now();
        cmd.header.frame_id = base_frame_;
        pub_command_->publish(cmd);
      }
      return;
    }

    // ------------------------------------------------------------------ //
    // Precision box: freeze command and signal exit when hand leaves box  //
    // ------------------------------------------------------------------ //
    if (is_precision && have_precision_box_) {
      const geometry_msgs::msg::Point & h = latest_hand_.pose.position;
      const bool in_box =
        std::abs(h.x - precision_box_center_unity_.x) <= precision_box_size_unity_.x * 0.5 &&
        std::abs(h.y - precision_box_center_unity_.y) <= precision_box_size_unity_.y * 0.5 &&
        std::abs(h.z - precision_box_center_unity_.z) <= precision_box_size_unity_.z * 0.5;

      std_msgs::msg::Bool exit_msg;
      exit_msg.data = !in_box;
      pub_precision_exit_->publish(exit_msg);

      if (!in_box) {
        publish_current_ee_as_command();
        return;
      }
    }

    // ------------------------------------------------------------------ //
    // Debug: raw hand quaternion                                           //
    // ------------------------------------------------------------------ //
    if (debug_raw_hand_quat_) {
      const auto & q_abs_u = latest_hand_.pose.orientation;

      Eigen::Quaterniond q_anchor_u = msg_to_eigen(hand_anchor_.pose.orientation).normalized();
      Eigen::Quaterniond q_curr_u   = msg_to_eigen(latest_hand_.pose.orientation).normalized();
      Eigen::Quaterniond q_rel_u_local = (q_anchor_u.conjugate() * q_curr_u).normalized();
      geometry_msgs::msg::Quaternion q_rel_u_msg = eigen_to_msg(q_rel_u_local);

      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 250,
        "[ROS RAW HAND] abs_u=(%.4f, %.4f, %.4f, %.4f) rel_u=(%.4f, %.4f, %.4f, %.4f) "
        "teleop=%d anchor=%d",
        q_abs_u.x, q_abs_u.y, q_abs_u.z, q_abs_u.w,
        q_rel_u_msg.x, q_rel_u_msg.y, q_rel_u_msg.z, q_rel_u_msg.w,
        teleop_enabled_ ? 1 : 0, anchor_latched_ ? 1 : 0);
    }

    // ------------------------------------------------------------------ //
    // Position delta: Unity → ROS, scale, deadband, clamp                 //
    // ------------------------------------------------------------------ //
    geometry_msgs::msg::Point hand_delta_unity = point_sub(
      latest_hand_.pose.position, hand_anchor_.pose.position);
    geometry_msgs::msg::Point hand_delta_ros = unity_to_ros_delta(hand_delta_unity);

    // Canvas-aware scaling from active profile:
    //   Canvas normal available: decompose into tangential + normal components.
    //   No canvas normal: uniform position_scale from active profile.
    if (have_canvas_normal_ && prox > 1e-6) {
      const Eigen::Vector3d n(
        canvas_normal_ros_.x, canvas_normal_ros_.y, canvas_normal_ros_.z);
      const Eigen::Vector3d delta(
        hand_delta_ros.x, hand_delta_ros.y, hand_delta_ros.z);

      const double n_comp        = delta.dot(n);
      const Eigen::Vector3d tang = delta - n_comp * n;

      const Eigen::Vector3d result =
        tang * active.tangential_scale + n * (n_comp * active.normal_scale);
      hand_delta_ros = make_point(result.x(), result.y(), result.z());
    } else {
      hand_delta_ros = point_scale(hand_delta_ros, active.position_scale);
    }

    const double raw_delta_norm = point_norm(hand_delta_ros);

    if (raw_delta_norm < blended_deadband) {
      hand_delta_ros = make_point(0.0, 0.0, 0.0);
    }

    hand_delta_ros = clamp_point_norm(hand_delta_ros, max_total_delta_m_);

    // ------------------------------------------------------------------ //
    // Command position: anchor + delta, workspace clamp, step clamp       //
    // ------------------------------------------------------------------ //
    geometry_msgs::msg::Point raw_cmd_pos_before_ws = point_add(
      ee_anchor_.pose.position, hand_delta_ros);
    geometry_msgs::msg::Point raw_cmd_pos = clamp_workspace(raw_cmd_pos_before_ws);

    geometry_msgs::msg::Point cmd_pos = raw_cmd_pos;
    double raw_step_norm = 0.0;
    if (have_last_command_) {
      const geometry_msgs::msg::Point step = point_sub(
        raw_cmd_pos, last_command_.pose.position);
      raw_step_norm = point_norm(step);
      const geometry_msgs::msg::Point step_clamped =
        clamp_point_norm(step, blended_max_step);
      cmd_pos = point_add(last_command_.pose.position, step_clamped);
    }

    // ------------------------------------------------------------------ //
    // Canvas plane clamp                                                   //
    // Stop plane holds position until pen_down; touch plane allows press. //
    // ------------------------------------------------------------------ //
    const double plane_limit = pen_down_ ? canvas_touch_plane_x_ : canvas_stop_plane_x_;
    if (cmd_pos.x < plane_limit)
      cmd_pos.x = plane_limit;

    // ------------------------------------------------------------------ //
    // Build and publish command                                            //
    // ------------------------------------------------------------------ //
    PoseStamped cmd;
    cmd.header.stamp    = get_clock()->now();
    cmd.header.frame_id = base_frame_;
    cmd.pose.position   = cmd_pos;
    cmd.pose.orientation = compute_command_orientation(blended_angle_step);

    pub_command_->publish(cmd);

    // ------------------------------------------------------------------ //
    // Debug: canvas / blended tuning values                               //
    // ------------------------------------------------------------------ //
    if (debug_canvas_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 250,
        "[CANVAS] mode=%s prox=%.3f | "
        "deadband=%.4f step=%.4f angle=%.2fdeg "
        "scale(pos=%.3f tang=%.3f norm=%.3f) | "
        "canvas_n=(%.2f,%.2f,%.2f)",
        active_mode_.c_str(), prox,
        blended_deadband, blended_max_step, rad2deg(blended_angle_step),
        active.position_scale, active.tangential_scale, active.normal_scale,
        canvas_normal_ros_.x, canvas_normal_ros_.y, canvas_normal_ros_.z);
    }

    // ------------------------------------------------------------------ //
    // Debug: RPY                                                           //
    // ------------------------------------------------------------------ //
    if (debug_rpy_) {
      const geometry_msgs::msg::Quaternion hand_ros_abs_q =
        unity_quat_to_ros_base(latest_hand_.pose.orientation);

      const Eigen::Quaterniond q_rel_dbg = unity_relative_world_to_ros_relative(
        hand_anchor_.pose.orientation, latest_hand_.pose.orientation);
      const geometry_msgs::msg::Quaternion hand_ros_rel_q = eigen_to_msg(q_rel_dbg);

      const RPY hand_abs_rpy = quat_to_rpy(hand_ros_abs_q);
      const RPY hand_rel_rpy = quat_to_rpy(hand_ros_rel_q);
      const RPY cmd_rpy      = quat_to_rpy(cmd.pose.orientation);
      const RPY ee_rpy       = quat_to_rpy(latest_ee_.pose.orientation);

      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 250,
        "[RPY deg] hand_abs=(%.1f %.1f %.1f) hand_rel=(%.1f %.1f %.1f) "
        "cmd=(%.1f %.1f %.1f) ee=(%.1f %.1f %.1f)",
        rad2deg(hand_abs_rpy.roll), rad2deg(hand_abs_rpy.pitch), rad2deg(hand_abs_rpy.yaw),
        rad2deg(hand_rel_rpy.roll), rad2deg(hand_rel_rpy.pitch), rad2deg(hand_rel_rpy.yaw),
        rad2deg(cmd_rpy.roll),      rad2deg(cmd_rpy.pitch),      rad2deg(cmd_rpy.yaw),
        rad2deg(ee_rpy.roll),       rad2deg(ee_rpy.pitch),       rad2deg(ee_rpy.yaw));
    }

    // ------------------------------------------------------------------ //
    // Debug: EE forward vector in base_link (+X axis of EE frame).       //
    // Compare against Unity HUD which prints hand forward remapped to ROS //
    // axes. Numbers should match when hand and EE point the same way.    //
    // ------------------------------------------------------------------ //
    if (debug_ee_forward_) {
      const Eigen::Quaterniond q_ee =
        msg_to_eigen(latest_ee_.pose.orientation).normalized();
      const Eigen::Vector3d ee_fwd = q_ee.toRotationMatrix() * Eigen::Vector3d(1.0, 0.0, 0.0);
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "[EE_FWD] (%.3f, %.3f, %.3f)",
        ee_fwd.x(), ee_fwd.y(), ee_fwd.z());
    }

    // ------------------------------------------------------------------ //
    // Debug: verbose state                                                 //
    // ------------------------------------------------------------------ //
    if (debug_verbose_) {
      const Eigen::Quaterniond q_rel_dbg = unity_relative_world_to_ros_relative(
        hand_anchor_.pose.orientation, latest_hand_.pose.orientation);
      const double rel_angle_deg = quat_shortest_angle(q_rel_dbg) * 180.0 / kPi;

      const bool pos_deadbanded     = (raw_delta_norm < blended_deadband);
      const bool total_delta_clamped = (raw_delta_norm > max_total_delta_m_);
      const bool workspace_clamped  =
        std::abs(raw_cmd_pos_before_ws.x - raw_cmd_pos.x) > 1e-6 ||
        std::abs(raw_cmd_pos_before_ws.y - raw_cmd_pos.y) > 1e-6 ||
        std::abs(raw_cmd_pos_before_ws.z - raw_cmd_pos.z) > 1e-6;
      const bool step_clamped = (raw_step_norm > blended_max_step + 1e-9);

      const auto ee_ros   = latest_ee_.pose.position;
      const auto ee_unity = ros_to_unity_position(ee_ros);
      const auto cmd_ros  = cmd.pose.position;
      const auto cmd_unity = ros_to_unity_position(cmd_ros);

      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 250,
        "[STATE] "
        "ee_r=(%.4f %.4f %.4f) ee_u=(%.4f %.4f %.4f) "
        "cmd_r=(%.4f %.4f %.4f) cmd_u=(%.4f %.4f %.4f) "
        "hand_du=(%.4f %.4f %.4f) hand_dr=(%.4f %.4f %.4f) "
        "raw_norm=%.4f rel_angle=%.2f "
        "db=%d total=%d ws=%d step=%d prox=%.3f",
        ee_ros.x,    ee_ros.y,    ee_ros.z,
        ee_unity.x,  ee_unity.y,  ee_unity.z,
        cmd_ros.x,   cmd_ros.y,   cmd_ros.z,
        cmd_unity.x, cmd_unity.y, cmd_unity.z,
        hand_delta_unity.x, hand_delta_unity.y, hand_delta_unity.z,
        hand_delta_ros.x,   hand_delta_ros.y,   hand_delta_ros.z,
        raw_delta_norm, rel_angle_deg,
        pos_deadbanded ? 1 : 0,
        total_delta_clamped ? 1 : 0,
        workspace_clamped ? 1 : 0,
        step_clamped ? 1 : 0,
        prox);
    }

    last_command_      = cmd;
    have_last_command_ = true;
  }

  // -------------------------------------------------------------------- //
  // Subscriptions                                                          //
  // -------------------------------------------------------------------- //
  rclcpp::Subscription<PoseStamped>::SharedPtr              sub_hand_;
  rclcpp::Subscription<PoseStamped>::SharedPtr              sub_ee_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr      sub_enable_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr    sub_mode_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr   sub_proximity_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_canvas_normal_;
  rclcpp::Subscription<PoseStamped>::SharedPtr              sub_precision_box_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_precision_box_size_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr      sub_detach_mode_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_nudge_cmd_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr      sub_pen_down_;

  rclcpp::Publisher<PoseStamped>::SharedPtr         pub_command_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_precision_exit_;
  rclcpp::TimerBase::SharedPtr                      timer_;

  // -------------------------------------------------------------------- //
  // Existing parameters                                                    //
  // -------------------------------------------------------------------- //
  std::string hand_topic_, robot_ee_topic_, command_topic_;
  std::string teleop_enable_topic_, base_frame_;
  std::string orientation_mode_;

  double rate_hz_, position_scale_;
  double hand_deadband_m_, max_total_delta_m_, max_cmd_step_m_;
  double x_min_, x_max_, y_min_, y_max_, z_min_, z_max_;
  double angular_deadband_deg_, angular_deadband_rad_;
  double max_angle_from_anchor_deg_, max_angle_from_anchor_rad_;
  double max_cmd_angle_step_deg_, max_cmd_angle_step_rad_;
  bool   debug_verbose_, debug_rpy_, debug_raw_hand_quat_, debug_ee_forward_;

  // -------------------------------------------------------------------- //
  // Tuning profiles                                                        //
  // -------------------------------------------------------------------- //
  TuningProfile normal_far_{};
  TuningProfile normal_near_{};
  TuningProfile precision_far_{};
  TuningProfile precision_near_{};
  bool debug_canvas_{false};

  // -------------------------------------------------------------------- //
  // NEW: mode manager runtime state                                        //
  // -------------------------------------------------------------------- //
  std::string                   active_mode_{"NORMAL"};
  double                        canvas_proximity_{0.0};
  geometry_msgs::msg::Vector3   canvas_normal_ros_{};
  bool                          have_canvas_normal_{false};

  // -------------------------------------------------------------------- //
  // Precision bounding box runtime state                                   //
  // -------------------------------------------------------------------- //
  bool                          have_precision_box_{false};
  geometry_msgs::msg::Point     precision_box_center_unity_{};
  geometry_msgs::msg::Vector3   precision_box_size_unity_{};
  double                        precision_box_scale_computed_{0.5};
  double                        precision_box_min_m_{0.02};
  double                        precision_box_max_m_{0.05};
  double                        precision_scale_at_min_{0.2};
  double                        precision_scale_at_max_{0.5};

  // -------------------------------------------------------------------- //
  // Detachment / nudge mode state                                          //
  // -------------------------------------------------------------------- //
  bool                          is_detached_{false};
  bool                          have_nudge_{false};
  bool                          pen_down_{false};
  bool                          canvas_plane_armed_{false};
  double                        canvas_stop_plane_x_{0.42};
  double                        canvas_touch_plane_x_{0.44};
  geometry_msgs::msg::Twist     pending_nudge_{};
  double                        nudge_step_m_{0.005};
  double                        nudge_step_rad_{0.0};

  // -------------------------------------------------------------------- //
  // Teleop runtime state                                                   //
  // -------------------------------------------------------------------- //
  bool have_hand_{false}, have_ee_{false};
  bool teleop_enabled_{false}, prev_teleop_enabled_{false};
  bool anchor_latched_{false}, have_last_command_{false};
  int  anchor_settle_ticks_{0};
  int  settle_ticks_remaining_{0};
  double hand_timeout_ms_{500.0};
  rclcpp::Time last_hand_time_{};

  PoseStamped latest_hand_, latest_ee_;
  PoseStamped hand_anchor_, ee_anchor_;
  PoseStamped last_command_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UnityAuthorityFilterServo>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
