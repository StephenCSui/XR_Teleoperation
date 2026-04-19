#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <string>

using geometry_msgs::msg::PoseStamped;

static constexpr double kPi = 3.14159265358979323846;

static double clamp_scalar(double x, double lo, double hi)
{
  return std::max(lo, std::min(hi, x));
}

static geometry_msgs::msg::Point make_point(double x, double y, double z)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
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
  const geometry_msgs::msg::Point & a,
  double s)
{
  return make_point(a.x * s, a.y * s, a.z * s);
}

static double point_norm(const geometry_msgs::msg::Point & p)
{
  return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

static geometry_msgs::msg::Point clamp_point_norm(
  const geometry_msgs::msg::Point & p,
  double max_norm)
{
  const double n = point_norm(p);
  if (n <= max_norm || n < 1e-12) {
    return p;
  }
  return point_scale(p, max_norm / n);
}

// Unity: +X right, +Y up, +Z forward
// ROS:   +X forward, +Y left, +Z up
static geometry_msgs::msg::Point unity_to_ros_delta(const geometry_msgs::msg::Point & u)
{
  return make_point(u.z, -u.x, u.y);
}

static geometry_msgs::msg::Point ros_to_unity_position(const geometry_msgs::msg::Point & r)
{
  return make_point(-r.y, r.z, r.x);
}

static Eigen::Matrix3d unity_to_ros_basis()
{
  Eigen::Matrix3d c;
  c <<  0.0,  0.0,  1.0,
       -1.0,  0.0,  0.0,
        0.0,  1.0,  0.0;
  return c;
}

static Eigen::Quaterniond msg_to_eigen(const geometry_msgs::msg::Quaternion & q)
{
  return Eigen::Quaterniond(q.w, q.x, q.y, q.z);
}

static geometry_msgs::msg::Quaternion eigen_to_msg(const Eigen::Quaterniond & q_in)
{
  Eigen::Quaterniond q = q_in.normalized();
  geometry_msgs::msg::Quaternion out;
  out.x = q.x();
  out.y = q.y();
  out.z = q.z();
  out.w = q.w();
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
  if (q.w() < 0.0) {
    q.coeffs() *= -1.0;
  }

  const double w = clamp_scalar(q.w(), -1.0, 1.0);
  return 2.0 * std::acos(w);
}

static Eigen::Quaterniond clamp_relative_quat_angle(
  const Eigen::Quaterniond & q_rel_in,
  double max_angle_rad)
{
  Eigen::Quaterniond q_rel = q_rel_in.normalized();
  if (q_rel.w() < 0.0) {
    q_rel.coeffs() *= -1.0;
  }

  const double angle = quat_shortest_angle(q_rel);
  if (angle <= max_angle_rad || angle < 1e-12) {
    return q_rel;
  }

  const double t = max_angle_rad / angle;
  return Eigen::Quaterniond::Identity().slerp(t, q_rel).normalized();
}

static Eigen::Quaterniond zero_small_relative_quat(
  const Eigen::Quaterniond & q_rel_in,
  double deadband_rad)
{
  Eigen::Quaterniond q_rel = q_rel_in.normalized();
  const double angle = quat_shortest_angle(q_rel);
  if (angle < deadband_rad) {
    return Eigen::Quaterniond::Identity();
  }
  return q_rel;
}

static Eigen::Quaterniond step_clamp_world_quat(
  const Eigen::Quaterniond & q_last_in,
  const Eigen::Quaterniond & q_target_in,
  double max_step_rad)
{
  Eigen::Quaterniond q_last = q_last_in.normalized();
  Eigen::Quaterniond q_target = q_target_in.normalized();

  Eigen::Quaterniond q_err = q_last.conjugate() * q_target;
  if (q_err.w() < 0.0) {
    q_target.coeffs() *= -1.0;
    q_err = q_last.conjugate() * q_target;
  }

  const double angle = quat_shortest_angle(q_err);
  if (angle <= max_step_rad || angle < 1e-12) {
    return q_target;
  }

  const double t = max_step_rad / angle;
  return q_last.slerp(t, q_target).normalized();
}

static Eigen::Quaterniond unity_relative_local_to_ros_relative(
  const geometry_msgs::msg::Quaternion & q_anchor_unity_msg,
  const geometry_msgs::msg::Quaternion & q_curr_unity_msg)
{
  const Eigen::Quaterniond q_anchor_u = msg_to_eigen(q_anchor_unity_msg).normalized();
  const Eigen::Quaterniond q_curr_u = msg_to_eigen(q_curr_unity_msg).normalized();

  const Eigen::Matrix3d r_anchor_u = q_anchor_u.toRotationMatrix();
  const Eigen::Matrix3d r_curr_u = q_curr_u.toRotationMatrix();

  // local-relative rotation in Unity
  const Eigen::Matrix3d r_rel_u_local = r_anchor_u.transpose() * r_curr_u;

  // convert same physical relative rotation into ROS basis
  const Eigen::Matrix3d c = unity_to_ros_basis();
  const Eigen::Matrix3d r_rel_r_local = c * r_rel_u_local * c.transpose();

  return Eigen::Quaterniond(r_rel_r_local).normalized();
}

struct RPY
{
  double roll;
  double pitch;
  double yaw;
};

static RPY quat_to_rpy(const geometry_msgs::msg::Quaternion & q)
{
  const double x = q.x;
  const double y = q.y;
  const double z = q.z;
  const double w = q.w;

  const double sinr_cosp = 2.0 * (w * x + y * z);
  const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
  const double roll = std::atan2(sinr_cosp, cosr_cosp);

  const double sinp = 2.0 * (w * y - z * x);
  double pitch;
  if (std::abs(sinp) >= 1.0) {
    pitch = std::copysign(kPi / 2.0, sinp);
  } else {
    pitch = std::asin(sinp);
  }

  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  const double yaw = std::atan2(siny_cosp, cosy_cosp);

  return {roll, pitch, yaw};
}

static double rad2deg(double x)
{
  return x * 180.0 / kPi;
}

class UnityAuthorityFilterServo : public rclcpp::Node
{
public:
  UnityAuthorityFilterServo()
  : Node("unity_authority_filter_servo")
  {
    hand_topic_ = declare_parameter<std::string>("hand_topic", "/unity/hand_pose");
    robot_ee_topic_ = declare_parameter<std::string>("robot_ee_topic", "/robot/ee_pose");
    command_topic_ = declare_parameter<std::string>("command_topic", "/unity/command_pose");
    teleop_enable_topic_ = declare_parameter<std::string>("teleop_enable_topic", "/unity/teleop_enabled");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

    // compatibility only
    joint_states_topic_ = declare_parameter<std::string>("joint_states_topic", "/joint_states");
    group_name_ = declare_parameter<std::string>("group_name", "ur_manipulator");
    ee_link_ = declare_parameter<std::string>("ee_link", "tool0");

    rate_hz_ = declare_parameter<double>("rate_hz", 60.0);
    position_scale_ = declare_parameter<double>("position_scale", 1.0);

    hand_deadband_m_ = declare_parameter<double>("hand_deadband_m", 0.003);
    max_total_delta_m_ = declare_parameter<double>("max_total_delta_m", 0.20);
    max_cmd_step_m_ = declare_parameter<double>("max_cmd_step_m", 0.01);

    x_min_ = declare_parameter<double>("x_min", 0.10);
    x_max_ = declare_parameter<double>("x_max", 0.80);
    y_min_ = declare_parameter<double>("y_min", -0.50);
    y_max_ = declare_parameter<double>("y_max", 0.50);
    z_min_ = declare_parameter<double>("z_min", 0.05);
    z_max_ = declare_parameter<double>("z_max", 0.70);

    orientation_mode_ = declare_parameter<std::string>("orientation_mode", "full_relative");
    angular_deadband_deg_ = declare_parameter<double>("angular_deadband_deg", 1.5);
    max_angle_from_anchor_deg_ = declare_parameter<double>("max_angle_from_anchor_deg", 45.0);
    max_cmd_angle_step_deg_ = declare_parameter<double>("max_cmd_angle_step_deg", 5.0);
    
    debug_verbose_ = declare_parameter<bool>("debug_verbose", true);
    debug_rpy_ = declare_parameter<bool>("debug_rpy", true);
    debug_raw_hand_quat_ = declare_parameter<bool>("debug_raw_hand_quat", true);

    angular_deadband_rad_ = angular_deadband_deg_ * kPi / 180.0;
    max_angle_from_anchor_rad_ = max_angle_from_anchor_deg_ * kPi / 180.0;
    max_cmd_angle_step_rad_ = max_cmd_angle_step_deg_ * kPi / 180.0;

    sub_hand_ = create_subscription<PoseStamped>(
      hand_topic_,
      10,
      std::bind(&UnityAuthorityFilterServo::on_hand, this, std::placeholders::_1));

    sub_ee_ = create_subscription<PoseStamped>(
      robot_ee_topic_,
      10,
      std::bind(&UnityAuthorityFilterServo::on_ee, this, std::placeholders::_1));

    sub_enable_ = create_subscription<std_msgs::msg::Bool>(
      teleop_enable_topic_,
      10,
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
  void on_hand(const PoseStamped::SharedPtr msg)
  {
    latest_hand_ = *msg;
    have_hand_ = true;
  }

  void on_ee(const PoseStamped::SharedPtr msg)
  {
    latest_ee_ = *msg;
    have_ee_ = true;
  }

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
    anchor_latched_ = false;
    have_last_command_ = false;
  }

  void latch_anchor()
  {
    hand_anchor_ = latest_hand_;
    ee_anchor_ = latest_ee_;
    anchor_latched_ = true;

    last_command_ = latest_ee_;
    last_command_.header.frame_id = base_frame_;
    have_last_command_ = true;

    RCLCPP_INFO(
      get_logger(),
      "[ANCHOR] latched hand=(%.3f, %.3f, %.3f) ee=(%.3f, %.3f, %.3f)",
      hand_anchor_.pose.position.x,
      hand_anchor_.pose.position.y,
      hand_anchor_.pose.position.z,
      ee_anchor_.pose.position.x,
      ee_anchor_.pose.position.y,
      ee_anchor_.pose.position.z);
  }

  geometry_msgs::msg::Point clamp_workspace(const geometry_msgs::msg::Point & p) const
  {
    return make_point(
      clamp_scalar(p.x, x_min_, x_max_),
      clamp_scalar(p.y, y_min_, y_max_),
      clamp_scalar(p.z, z_min_, z_max_));
  }

  geometry_msgs::msg::Quaternion compute_command_orientation()
  {
    if (orientation_mode_ == "fixed") {
      return ee_anchor_.pose.orientation;
    }

    Eigen::Quaterniond q_rel = unity_relative_local_to_ros_relative(
      hand_anchor_.pose.orientation,
      latest_hand_.pose.orientation);

    q_rel = zero_small_relative_quat(q_rel, angular_deadband_rad_);
    q_rel = clamp_relative_quat_angle(q_rel, max_angle_from_anchor_rad_);

    const Eigen::Quaterniond q_ee_anchor =
      msg_to_eigen(ee_anchor_.pose.orientation).normalized();

    // local-relative application: anchor * relative
    Eigen::Quaterniond q_target = (q_ee_anchor * q_rel).normalized();

    if (have_last_command_) {
      const Eigen::Quaterniond q_last =
        msg_to_eigen(last_command_.pose.orientation).normalized();
      q_target = step_clamp_world_quat(q_last, q_target, max_cmd_angle_step_rad_);
    }

    return eigen_to_msg(q_target);
  }

  void publish_current_ee_as_command()
  {
    if (!have_ee_) {
      return;
    }

    PoseStamped cmd = latest_ee_;
    cmd.header.stamp = get_clock()->now();
    cmd.header.frame_id = base_frame_;
    pub_command_->publish(cmd);

    last_command_ = cmd;
    have_last_command_ = true;
  }

  void on_timer()
  {
    if (!have_hand_ || !have_ee_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "[WAIT_INPUTS] hand=%d ee=%d teleop=%d",
        have_hand_ ? 1 : 0,
        have_ee_ ? 1 : 0,
        teleop_enabled_ ? 1 : 0);
      return;
    }

    if (teleop_enabled_ != prev_teleop_enabled_) {
      if (teleop_enabled_) {
        clear_anchor();
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

    if (!anchor_latched_) {
      latch_anchor();
    }

    if (debug_raw_hand_quat_) {
      const auto & q_abs_u = latest_hand_.pose.orientation;

      Eigen::Quaterniond q_anchor_u = msg_to_eigen(hand_anchor_.pose.orientation).normalized();
      Eigen::Quaterniond q_curr_u = msg_to_eigen(latest_hand_.pose.orientation).normalized();

      Eigen::Quaterniond q_rel_u_local = (q_anchor_u.conjugate() * q_curr_u).normalized();
      geometry_msgs::msg::Quaternion q_rel_u_msg = eigen_to_msg(q_rel_u_local);

      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 250,
        "[ROS RAW HAND] abs_u=(%.4f, %.4f, %.4f, %.4f) rel_u=(%.4f, %.4f, %.4f, %.4f) teleop=%d anchor=%d",
        q_abs_u.x, q_abs_u.y, q_abs_u.z, q_abs_u.w,
        q_rel_u_msg.x, q_rel_u_msg.y, q_rel_u_msg.z, q_rel_u_msg.w,
        teleop_enabled_ ? 1 : 0,
        anchor_latched_ ? 1 : 0
      );
    }

    geometry_msgs::msg::Point hand_delta_unity = point_sub(
      latest_hand_.pose.position, hand_anchor_.pose.position);

    geometry_msgs::msg::Point hand_delta_ros = unity_to_ros_delta(hand_delta_unity);
    hand_delta_ros = point_scale(hand_delta_ros, position_scale_);

    const double raw_delta_norm = point_norm(hand_delta_ros);

    if (raw_delta_norm < hand_deadband_m_) {
      hand_delta_ros = make_point(0.0, 0.0, 0.0);
    }

    hand_delta_ros = clamp_point_norm(hand_delta_ros, max_total_delta_m_);

    geometry_msgs::msg::Point raw_cmd_pos_before_ws = point_add(
      ee_anchor_.pose.position, hand_delta_ros);

    geometry_msgs::msg::Point raw_cmd_pos = clamp_workspace(raw_cmd_pos_before_ws);

    geometry_msgs::msg::Point cmd_pos = raw_cmd_pos;
    double raw_step_norm = 0.0;
    if (have_last_command_) {
      const geometry_msgs::msg::Point step = point_sub(
        raw_cmd_pos, last_command_.pose.position);
      raw_step_norm = point_norm(step);

      const geometry_msgs::msg::Point step_clamped = clamp_point_norm(
        step, max_cmd_step_m_);
      cmd_pos = point_add(last_command_.pose.position, step_clamped);
    }

    PoseStamped cmd;
    cmd.header.stamp = get_clock()->now();
    cmd.header.frame_id = base_frame_;
    cmd.pose.position = cmd_pos;
    cmd.pose.orientation = compute_command_orientation();

    pub_command_->publish(cmd);

    if (debug_rpy_) {
      const geometry_msgs::msg::Quaternion hand_ros_abs_q =
        unity_quat_to_ros_base(latest_hand_.pose.orientation);

      const Eigen::Quaterniond q_rel_dbg = unity_relative_local_to_ros_relative(
        hand_anchor_.pose.orientation,
        latest_hand_.pose.orientation);

      const geometry_msgs::msg::Quaternion hand_ros_rel_q =
        eigen_to_msg(q_rel_dbg);

      const RPY hand_abs_rpy = quat_to_rpy(hand_ros_abs_q);
      const RPY hand_rel_rpy = quat_to_rpy(hand_ros_rel_q);
      const RPY cmd_rpy = quat_to_rpy(cmd.pose.orientation);
      const RPY ee_rpy = quat_to_rpy(latest_ee_.pose.orientation);

      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 250,
        "[RPY deg] hand_abs=(%.1f %.1f %.1f) hand_rel=(%.1f %.1f %.1f) cmd=(%.1f %.1f %.1f) ee=(%.1f %.1f %.1f)",
        rad2deg(hand_abs_rpy.roll), rad2deg(hand_abs_rpy.pitch), rad2deg(hand_abs_rpy.yaw),
        rad2deg(hand_rel_rpy.roll), rad2deg(hand_rel_rpy.pitch), rad2deg(hand_rel_rpy.yaw),
        rad2deg(cmd_rpy.roll),      rad2deg(cmd_rpy.pitch),      rad2deg(cmd_rpy.yaw),
        rad2deg(ee_rpy.roll),       rad2deg(ee_rpy.pitch),       rad2deg(ee_rpy.yaw)
      );
    }

    if (debug_verbose_) {
      const Eigen::Quaterniond q_rel_dbg = unity_relative_local_to_ros_relative(
        hand_anchor_.pose.orientation,
        latest_hand_.pose.orientation);
      const double rel_angle_deg = quat_shortest_angle(q_rel_dbg) * 180.0 / kPi;

      const bool pos_deadbanded = (raw_delta_norm < hand_deadband_m_);
      const bool total_delta_clamped = (raw_delta_norm > max_total_delta_m_);
      const bool workspace_clamped =
        std::abs(raw_cmd_pos_before_ws.x - raw_cmd_pos.x) > 1e-6 ||
        std::abs(raw_cmd_pos_before_ws.y - raw_cmd_pos.y) > 1e-6 ||
        std::abs(raw_cmd_pos_before_ws.z - raw_cmd_pos.z) > 1e-6;
      const bool step_clamped = (raw_step_norm > max_cmd_step_m_ + 1e-9);

      const auto ee_ros = latest_ee_.pose.position;
      const auto ee_unity = ros_to_unity_position(ee_ros);

      const auto cmd_ros = cmd.pose.position;
      const auto cmd_unity = ros_to_unity_position(cmd_ros);

      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 250,
        "[STATE] "
        "ee_r=(%.4f %.4f %.4f) ee_u=(%.4f %.4f %.4f) "
        "cmd_r=(%.4f %.4f %.4f) cmd_u=(%.4f %.4f %.4f) "
        "hand_du=(%.4f %.4f %.4f) hand_dr=(%.4f %.4f %.4f) "
        "raw_norm=%.4f rel_angle=%.2f "
        "db=%d total=%d ws=%d step=%d",
        ee_ros.x, ee_ros.y, ee_ros.z,
        ee_unity.x, ee_unity.y, ee_unity.z,
        cmd_ros.x, cmd_ros.y, cmd_ros.z,
        cmd_unity.x, cmd_unity.y, cmd_unity.z,
        hand_delta_unity.x, hand_delta_unity.y, hand_delta_unity.z,
        hand_delta_ros.x, hand_delta_ros.y, hand_delta_ros.z,
        raw_delta_norm,
        rel_angle_deg,
        pos_deadbanded ? 1 : 0,
        total_delta_clamped ? 1 : 0,
        workspace_clamped ? 1 : 0,
        step_clamped ? 1 : 0
      );
    }

    last_command_ = cmd;
    have_last_command_ = true;
  }

private:
  rclcpp::Subscription<PoseStamped>::SharedPtr sub_hand_;
  rclcpp::Subscription<PoseStamped>::SharedPtr sub_ee_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_enable_;
  rclcpp::Publisher<PoseStamped>::SharedPtr pub_command_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string hand_topic_;
  std::string robot_ee_topic_;
  std::string command_topic_;
  std::string teleop_enable_topic_;
  std::string base_frame_;

  // compatibility only
  std::string joint_states_topic_;
  std::string group_name_;
  std::string ee_link_;

  std::string orientation_mode_;

  double rate_hz_;
  double position_scale_;

  double hand_deadband_m_;
  double max_total_delta_m_;
  double max_cmd_step_m_;

  double x_min_;
  double x_max_;
  double y_min_;
  double y_max_;
  double z_min_;
  double z_max_;

  double angular_deadband_deg_;
  double angular_deadband_rad_;
  double max_angle_from_anchor_deg_;
  double max_angle_from_anchor_rad_;
  double max_cmd_angle_step_deg_;
  double max_cmd_angle_step_rad_;

  bool debug_verbose_;
  bool debug_rpy_;
  bool debug_raw_hand_quat_;

  bool have_hand_{false};
  bool have_ee_{false};
  bool teleop_enabled_{false};
  bool prev_teleop_enabled_{false};
  bool anchor_latched_{false};
  bool have_last_command_{false};

  PoseStamped latest_hand_;
  PoseStamped latest_ee_;
  PoseStamped hand_anchor_;
  PoseStamped ee_anchor_;
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