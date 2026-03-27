// ========================================================
// Includes
// ========================================================

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <vector>

// ========================================================
// Aliases / constants
// ========================================================

using PoseStamped = geometry_msgs::msg::PoseStamped;

static constexpr double kPi = 3.14159265358979323846;

// ========================================================
// Basic point helpers
// ========================================================

static geometry_msgs::msg::Point point_add(
  const geometry_msgs::msg::Point& a,
  const geometry_msgs::msg::Point& b)
{
  geometry_msgs::msg::Point out;
  out.x = a.x + b.x;
  out.y = a.y + b.y;
  out.z = a.z + b.z;
  return out;
}

static geometry_msgs::msg::Point point_sub(
  const geometry_msgs::msg::Point& a,
  const geometry_msgs::msg::Point& b)
{
  geometry_msgs::msg::Point out;
  out.x = a.x - b.x;
  out.y = a.y - b.y;
  out.z = a.z - b.z;
  return out;
}

static geometry_msgs::msg::Point point_scale(
  const geometry_msgs::msg::Point& a,
  double s)
{
  geometry_msgs::msg::Point out;
  out.x = a.x * s;
  out.y = a.y * s;
  out.z = a.z * s;
  return out;
}

static double point_norm(const geometry_msgs::msg::Point& p)
{
  return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

// ========================================================
// Angle helpers
// ========================================================

static double wrap_pi(double a)
{
  while (a >  kPi) a -= 2.0 * kPi;
  while (a < -kPi) a += 2.0 * kPi;
  return a;
}

// ========================================================
// Unity -> ROS position conversion
// Unity: +X right, +Y up, +Z forward
// ROS:   +X forward, +Y left, +Z up
// ========================================================

static geometry_msgs::msg::Point unity_to_ros_base(const geometry_msgs::msg::Point& u)
{
  geometry_msgs::msg::Point r;
  r.x = u.z;
  r.y = -u.x;
  r.z = u.y;
  return r;
}

// ========================================================
// Quaternion conversion helpers
// ========================================================

static Eigen::Quaterniond msg_to_eigen(const geometry_msgs::msg::Quaternion& q)
{
  return Eigen::Quaterniond(q.w, q.x, q.y, q.z);
}

static geometry_msgs::msg::Quaternion eigen_to_msg(const Eigen::Quaterniond& q_in)
{
  const Eigen::Quaterniond q = q_in.normalized();

  geometry_msgs::msg::Quaternion out;
  out.x = q.x();
  out.y = q.y();
  out.z = q.z();
  out.w = q.w();
  return out;
}

static geometry_msgs::msg::Quaternion unity_quat_to_ros_base(
  const geometry_msgs::msg::Quaternion& q_unity_msg)
{
  const Eigen::Quaterniond q_unity = msg_to_eigen(q_unity_msg).normalized();
  const Eigen::Matrix3d R_unity = q_unity.toRotationMatrix();

  Eigen::Matrix3d C;
  C <<  0.0,  0.0,  1.0,
       -1.0,  0.0,  0.0,
        0.0,  1.0,  0.0;

  const Eigen::Matrix3d R_ros = C * R_unity * C.transpose();
  return eigen_to_msg(Eigen::Quaterniond(R_ros));
}

// ========================================================
// RPY helpers
// ========================================================

struct RPY
{
  double roll;
  double pitch;
  double yaw;
};

static RPY quat_to_rpy(const geometry_msgs::msg::Quaternion& q)
{
  const double x = q.x;
  const double y = q.y;
  const double z = q.z;
  const double w = q.w;

  const double sinr_cosp = 2.0 * (w * x + y * z);
  const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
  const double roll = std::atan2(sinr_cosp, cosr_cosp);

  const double sinp = 2.0 * (w * y - z * x);
  double pitch = 0.0;

  if (std::abs(sinp) >= 1.0)
  {
    pitch = std::copysign(kPi / 2.0, sinp);
  }
  else
  {
    pitch = std::asin(sinp);
  }

  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  const double yaw = std::atan2(siny_cosp, cosy_cosp);

  return {roll, pitch, yaw};
}

static geometry_msgs::msg::Quaternion rpy_to_quat(double roll, double pitch, double yaw)
{
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);

  geometry_msgs::msg::Quaternion q;
  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;
  return q;
}

// ========================================================
// UnityAuthorityFilter
// ========================================================

class UnityAuthorityFilter : public rclcpp::Node
{
public:
  UnityAuthorityFilter()
  : Node("unity_authority_filter_cpp")
  {
    // --------------------------------------------------------
    // Topics
    // --------------------------------------------------------
    hand_topic_ = declare_parameter<std::string>("hand_topic", "/unity/hand_pose");
    robot_ee_topic_ = declare_parameter<std::string>("robot_ee_topic", "/robot/ee_pose");
    joint_states_topic_ = declare_parameter<std::string>("joint_states_topic", "/joint_states");
    command_topic_ = declare_parameter<std::string>("command_topic", "/unity/command_pose");
    teleop_enable_topic_ = declare_parameter<std::string>("teleop_enable_topic", "/unity/teleop_enabled");

    // --------------------------------------------------------
    // Robot / frame params
    // --------------------------------------------------------
    group_name_ = declare_parameter<std::string>("group_name", "ur_manipulator");
    ee_link_ = declare_parameter<std::string>("ee_link", "tool0");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

    // --------------------------------------------------------
    // Timing / IK / Jacobian safety params
    // --------------------------------------------------------
    rate_hz_ = declare_parameter<double>("rate_hz", 10.0);
    ik_timeout_s_ = declare_parameter<double>("ik_timeout_s", 0.003);
    sigma_min_threshold_ = declare_parameter<double>("sigma_min_threshold", 0.01);

    // --------------------------------------------------------
    // Nearby fallback search params
    // --------------------------------------------------------
    pos_step_m_ = declare_parameter<double>("pos_step_m", 0.01);
    yaw_step_deg_ = declare_parameter<double>("yaw_step_deg", 8.0);
    max_layers_ = declare_parameter<int>("max_layers", 2);
    yaw_step_rad_ = yaw_step_deg_ * kPi / 180.0;

    // --------------------------------------------------------
    // Per-tick command clamp params
    // --------------------------------------------------------
    max_cmd_step_m_ = declare_parameter<double>("max_cmd_step_m", 0.02);
    max_cmd_yaw_step_deg_ = declare_parameter<double>("max_cmd_yaw_step_deg", 8.0);
    max_cmd_yaw_step_rad_ = max_cmd_yaw_step_deg_ * kPi / 180.0;

    // --------------------------------------------------------
    // Joint jump guard params
    // --------------------------------------------------------
    max_joint_jump_rad_ = declare_parameter<double>("max_joint_jump_rad", 0.8);

    // --------------------------------------------------------
    // Wrist_2 singularity guard params
    // --------------------------------------------------------
    wrist_2_joint_name_ = declare_parameter<std::string>("wrist_2_joint_name", "wrist_2_joint");
    wrist_2_min_dist_deg_ = declare_parameter<double>("wrist_2_min_dist_deg", 10.0);
    wrist_2_min_dist_rad_ = wrist_2_min_dist_deg_ * kPi / 180.0;

    // --------------------------------------------------------
    // Invalid recovery params
    // --------------------------------------------------------
    invalid_reanchor_ticks_ = declare_parameter<int>("invalid_reanchor_ticks", 3);

    // --------------------------------------------------------
    // Hand deadband params
    // --------------------------------------------------------
    hand_deadband_m_ = declare_parameter<double>("hand_deadband_m", 0.003);
    hand_yaw_deadband_deg_ = declare_parameter<double>("hand_yaw_deadband_deg", 1.5);
    hand_yaw_deadband_rad_ = hand_yaw_deadband_deg_ * kPi / 180.0;

    // --------------------------------------------------------
    // Orientation slack params
    // --------------------------------------------------------
    roll_search_max_deg_ = declare_parameter<double>("roll_search_max_deg", 10.0);
    pitch_search_max_deg_ = declare_parameter<double>("pitch_search_max_deg", 10.0);
    orientation_search_step_deg_ = declare_parameter<double>("orientation_search_step_deg", 10.0);

    roll_search_max_rad_ = roll_search_max_deg_ * kPi / 180.0;
    pitch_search_max_rad_ = pitch_search_max_deg_ * kPi / 180.0;
    orientation_search_step_rad_ = orientation_search_step_deg_ * kPi / 180.0;

    // --------------------------------------------------------
    // Debug params
    // --------------------------------------------------------
    debug_verbose_ = declare_parameter<bool>("debug_verbose", true);
    debug_log_every_n_ticks_ = declare_parameter<int>("debug_log_every_n_ticks", 1);


    // --------------------------------------------------------
    // Subscribers
    // --------------------------------------------------------
    sub_hand_ = create_subscription<PoseStamped>(
      hand_topic_,
      10,
      std::bind(&UnityAuthorityFilter::on_hand, this, std::placeholders::_1));

    sub_ee_ = create_subscription<PoseStamped>(
      robot_ee_topic_,
      10,
      std::bind(&UnityAuthorityFilter::on_robot_ee, this, std::placeholders::_1));

    sub_js_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic_,
      10,
      std::bind(&UnityAuthorityFilter::on_joint_states, this, std::placeholders::_1));

    sub_enable_ = create_subscription<std_msgs::msg::Bool>(
      teleop_enable_topic_,
      10,
      std::bind(&UnityAuthorityFilter::on_enable, this, std::placeholders::_1));

    // --------------------------------------------------------
    // Publisher
    // --------------------------------------------------------
    pub_cmd_ = create_publisher<PoseStamped>(command_topic_, 10);

    // --------------------------------------------------------
    // Timer
    // --------------------------------------------------------
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(1.0, rate_hz_)),
      std::bind(&UnityAuthorityFilter::tick, this));

    RCLCPP_INFO(get_logger(), "Authority filter constructed");
  }

  void init_moveit()
  {
    if (moveit_ready_) return;

    robot_model_loader::RobotModelLoader loader(shared_from_this(), "robot_description");
    robot_model_ = loader.getModel();

    if (!robot_model_)
    {
      throw std::runtime_error("Failed to load robot model from robot_description");
    }

    jmg_ = robot_model_->getJointModelGroup(group_name_);

    if (!jmg_)
    {
      throw std::runtime_error("JointModelGroup not found: " + group_name_);
    }

    if (!robot_model_->hasLinkModel(ee_link_))
    {
      throw std::runtime_error("EE link not found: " + ee_link_);
    }

    controlled_joint_names_ = jmg_->getVariableNames();

    wrist_2_index_ = -1;
    for (size_t i = 0; i < controlled_joint_names_.size(); ++i)
    {
      if (controlled_joint_names_[i] == wrist_2_joint_name_)
      {
        wrist_2_index_ = static_cast<int>(i);
        break;
      }
    }

    if (wrist_2_index_ >= 0)
    {
      RCLCPP_INFO(
        get_logger(),
        "Wrist_2 guard enabled. joint=%s index=%d min_dist_deg=%.1f",
        wrist_2_joint_name_.c_str(),
        wrist_2_index_,
        wrist_2_min_dist_deg_);
    }
    else
    {
      RCLCPP_WARN(
        get_logger(),
        "Wrist_2 guard NOT enabled. joint '%s' not found in joint group",
        wrist_2_joint_name_.c_str());
    }

    moveit_ready_ = true;

    RCLCPP_INFO(
      get_logger(),
      "MoveIt model loaded. hand=%s cmd=%s rate=%.1f group=%s ee=%s teleop_enable=%s",
      hand_topic_.c_str(),
      command_topic_.c_str(),
      rate_hz_,
      group_name_.c_str(),
      ee_link_.c_str(),
      teleop_enable_topic_.c_str());
  }

private:
  // ========================================================
  // Candidate / orientation structs
  // ========================================================

  struct OrientationOption
  {
    geometry_msgs::msg::Quaternion q;
    double rp_offset_abs{0.0};
  };

  struct CandidateChoice
  {
    PoseStamped pose;
    std::vector<double> joints;

    double wrist_margin{std::numeric_limits<double>::infinity()};
    double sigma_min{0.0};

    double joint_change_max{0.0};
    double joint_change_sum{0.0};

    double pos_offset_norm{0.0};
    double yaw_offset_abs{0.0};
    double rp_offset_abs{0.0};
  };

  enum class RejectReason
  {
    NONE = 0,
    IK_FAIL,
    JUMP_FAIL,
    WRIST_FAIL,
    JAC_FAIL
  };

  struct SearchDebugStats
  {
    int tried{0};
    int accepted{0};

    int ik_fail{0};
    int jump_fail{0};
    int wrist_fail{0};
    int jac_fail{0};
  };

  struct BuildDebugInfo
  {
    double raw_hand_delta_norm{0.0};
    double raw_hand_delta_yaw{0.0};

    bool pos_deadbanded{false};
    bool yaw_deadbanded{false};

    double raw_cmd_step{0.0};
    double raw_cmd_dyaw{0.0};

    bool pos_clamped{false};
    bool yaw_clamped{false};
  };

  

  // ========================================================
  // Input callbacks
  // ========================================================

  void on_hand(const PoseStamped::SharedPtr msg)
  {
    latest_hand_ = *msg;
  }

  void on_robot_ee(const PoseStamped::SharedPtr msg)
  {
    latest_ee_ = *msg;
  }

  void on_joint_states(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    latest_js_ = *msg;
  }

  void on_enable(const std_msgs::msg::Bool::SharedPtr msg)
  {
    teleop_enabled_ = msg->data;
    if (debug_verbose_)
    {
      RCLCPP_INFO(
        get_logger(),
        "[DEBUG][ENABLE_CB] teleop_enabled=%d",
        static_cast<int>(teleop_enabled_));
    }
  }

  // ========================================================
  // State checks
  // ========================================================

  bool have_inputs() const
  {
    return latest_hand_.has_value() &&
           latest_ee_.has_value() &&
           latest_js_.has_value();
  }

  // ========================================================
  // Nominal orientation latch
  // ========================================================

  void latch_nominal()
  {
    if (nominal_set_) return;

    const auto rpy = quat_to_rpy(latest_ee_->pose.orientation);
    nominal_roll_ = rpy.roll;
    nominal_pitch_ = rpy.pitch;
    nominal_set_ = true;

    RCLCPP_INFO(
      get_logger(),
      "Latched nominal roll/pitch. roll=%.3f pitch=%.3f",
      nominal_roll_,
      nominal_pitch_);
  }

  // ========================================================
  // Teleop anchor helpers
  // ========================================================

  void reset_anchor()
  {
    anchor_set_ = false;
  }

  void latch_anchor()
  {
    if (anchor_set_) return;

    hand_anchor_ros_ = unity_to_ros_base(latest_hand_->pose.position);
    robot_anchor_ros_ = latest_ee_->pose.position;

    const auto q_hand_ros = unity_quat_to_ros_base(latest_hand_->pose.orientation);
    hand_anchor_yaw_ = quat_to_rpy(q_hand_ros).yaw;
    robot_anchor_yaw_ = quat_to_rpy(latest_ee_->pose.orientation).yaw;

    anchor_set_ = true;

    RCLCPP_INFO(
      get_logger(),
      "Teleop anchor latched. hand=(%.3f, %.3f, %.3f) robot=(%.3f, %.3f, %.3f) hand_yaw=%.3f robot_yaw=%.3f",
      hand_anchor_ros_.x,
      hand_anchor_ros_.y,
      hand_anchor_ros_.z,
      robot_anchor_ros_.x,
      robot_anchor_ros_.y,
      robot_anchor_ros_.z,
      hand_anchor_yaw_,
      robot_anchor_yaw_);
  }

  // ========================================================
  // Hold / recovery helpers
  // ========================================================

  void publish_hold_pose()
  {
    if (!latest_ee_.has_value()) return;

    PoseStamped hold = *latest_ee_;
    hold.header.stamp = now();
    hold.header.frame_id = base_frame_;
    pub_cmd_->publish(hold);
  }

  void publish_last_valid_or_hold()
  {
    if (!latest_ee_.has_value()) return;

    PoseStamped out = last_valid_.has_value() ? *last_valid_ : *latest_ee_;
    out.header.stamp = now();
    out.header.frame_id = base_frame_;
    pub_cmd_->publish(out);
  }

  void reanchor_to_hold()
  {
    if (!latest_hand_.has_value() || !latest_ee_.has_value()) return;

    const PoseStamped& hold_pose = last_valid_.has_value() ? *last_valid_ : *latest_ee_;

    hand_anchor_ros_ = unity_to_ros_base(latest_hand_->pose.position);
    robot_anchor_ros_ = hold_pose.pose.position;

    const auto q_hand_ros = unity_quat_to_ros_base(latest_hand_->pose.orientation);
    hand_anchor_yaw_ = quat_to_rpy(q_hand_ros).yaw;
    robot_anchor_yaw_ = quat_to_rpy(hold_pose.pose.orientation).yaw;

    anchor_set_ = true;

    if (!last_valid_.has_value())
    {
      last_valid_ = hold_pose;
    }

    if (!has_last_valid_joints_)
    {
      capture_current_joint_reference();
    }

    RCLCPP_WARN(
      get_logger(),
      "Auto re-anchor after %d invalid ticks",
      invalid_reanchor_ticks_);
  }

  void handle_invalid_target()
  {
    ++invalid_streak_;

    publish_last_valid_or_hold();

    if (invalid_streak_ >= invalid_reanchor_ticks_)
    {
      reanchor_to_hold();
      invalid_streak_ = 0;
    }
  }

  // ========================================================
  // RobotState seeding
  // Prefer last accepted joint solution for continuity.
  // Fall back to current measured joint state if needed.
  // ========================================================

  void seed_state(moveit::core::RobotState& state) const
  {
    if (has_last_valid_joints_ &&
        last_valid_joints_.size() == controlled_joint_names_.size() &&
        !last_valid_joints_.empty())
    {
      state.setJointGroupPositions(jmg_, last_valid_joints_);
      state.update();
      return;
    }

    const auto& names = latest_js_->name;
    const auto& pos = latest_js_->position;

    for (size_t i = 0; i < names.size() && i < pos.size(); ++i)
    {
      state.setVariablePosition(names[i], pos[i]);
    }

    state.update();
  }

  // ========================================================
  // Joint jump / wrist guard / candidate evaluation
  // ========================================================

  void capture_current_joint_reference()
  {
    moveit::core::RobotState state(robot_model_);
    seed_state(state);

    last_valid_joints_.clear();
    state.copyJointGroupPositions(jmg_, last_valid_joints_);
    has_last_valid_joints_ = true;
  }

  double wrist_2_distance_to_singularity(double wrist_2_angle) const
  {
    return std::abs(std::remainder(wrist_2_angle, kPi));
  }

  bool joint_jump_metrics(
    const std::vector<double>& candidate_joints,
    double& joint_change_max,
    double& joint_change_sum) const
  {
    joint_change_max = 0.0;
    joint_change_sum = 0.0;

    if (!has_last_valid_joints_)
    {
      return true;
    }

    if (candidate_joints.size() != last_valid_joints_.size())
    {
      return false;
    }

    for (size_t i = 0; i < candidate_joints.size(); ++i)
    {
      const double d = std::abs(wrap_pi(candidate_joints[i] - last_valid_joints_[i]));
      joint_change_max = std::max(joint_change_max, d);
      joint_change_sum += d;
    }

    return joint_change_max <= max_joint_jump_rad_;
  }

  bool evaluate_candidate(
    const PoseStamped& target,
    double pos_offset_norm,
    double yaw_offset_abs,
    double rp_offset_abs,
    CandidateChoice& out,
    RejectReason* reject_reason = nullptr) const
  {
    if (reject_reason != nullptr)
    {
      *reject_reason = RejectReason::NONE;
    }

    moveit::core::RobotState state(robot_model_);
    seed_state(state);

    const bool ok_ik = state.setFromIK(jmg_, target.pose, ee_link_, ik_timeout_s_);
    if (!ok_ik)
    {
      if (reject_reason != nullptr) *reject_reason = RejectReason::IK_FAIL;
      return false;
    }

    state.update();

    std::vector<double> candidate_joints;
    state.copyJointGroupPositions(jmg_, candidate_joints);

    double joint_change_max = 0.0;
    double joint_change_sum = 0.0;

    if (!joint_jump_metrics(candidate_joints, joint_change_max, joint_change_sum))
    {
      if (reject_reason != nullptr) *reject_reason = RejectReason::JUMP_FAIL;
      return false;
    }

    double wrist_margin = std::numeric_limits<double>::infinity();

    if (wrist_2_index_ >= 0)
    {
      if (static_cast<size_t>(wrist_2_index_) >= candidate_joints.size())
      {
        if (reject_reason != nullptr) *reject_reason = RejectReason::WRIST_FAIL;
        return false;
      }

      const double wrist_2_angle = candidate_joints[static_cast<size_t>(wrist_2_index_)];
      wrist_margin = wrist_2_distance_to_singularity(wrist_2_angle);

      if (wrist_margin < wrist_2_min_dist_rad_)
      {
        if (reject_reason != nullptr) *reject_reason = RejectReason::WRIST_FAIL;
        return false;
      }
    }

    Eigen::Vector3d ref_point(0.0, 0.0, 0.0);
    Eigen::MatrixXd J;

    const bool jac_ok = state.getJacobian(
      jmg_,
      state.getLinkModel(ee_link_),
      ref_point,
      J,
      false);

    if (!jac_ok)
    {
      if (reject_reason != nullptr) *reject_reason = RejectReason::JAC_FAIL;
      return false;
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const auto& s = svd.singularValues();
    const double sigma_min = (s.size() > 0) ? s(s.size() - 1) : 0.0;

    if (sigma_min < sigma_min_threshold_)
    {
      if (reject_reason != nullptr) *reject_reason = RejectReason::JAC_FAIL;
      return false;
    }

    out.pose = target;
    out.joints = candidate_joints;
    out.wrist_margin = wrist_margin;
    out.sigma_min = sigma_min;
    out.joint_change_max = joint_change_max;
    out.joint_change_sum = joint_change_sum;
    out.pos_offset_norm = pos_offset_norm;
    out.yaw_offset_abs = yaw_offset_abs;
    out.rp_offset_abs = rp_offset_abs;

    return true;
  }

    
  bool candidate_better(const CandidateChoice& cand, const CandidateChoice& best) const
  {
    const double eps = 1e-9;

    // ------------------------------------------------------
    // First: follow the requested hand target as closely as possible
    // ------------------------------------------------------
    if (cand.pos_offset_norm + eps < best.pos_offset_norm) return true;
    if (cand.pos_offset_norm > best.pos_offset_norm + eps) return false;

    if (cand.yaw_offset_abs + eps < best.yaw_offset_abs) return true;
    if (cand.yaw_offset_abs > best.yaw_offset_abs + eps) return false;

    if (cand.rp_offset_abs + eps < best.rp_offset_abs) return true;
    if (cand.rp_offset_abs > best.rp_offset_abs + eps) return false;

    // ------------------------------------------------------
    // Then: prefer continuity / smaller joint motion
    // ------------------------------------------------------
    if (cand.joint_change_max + eps < best.joint_change_max) return true;
    if (cand.joint_change_max > best.joint_change_max + eps) return false;

    if (cand.joint_change_sum + eps < best.joint_change_sum) return true;
    if (cand.joint_change_sum > best.joint_change_sum + eps) return false;

    // ------------------------------------------------------
    // Finally: among equally close candidates, prefer healthier ones
    // ------------------------------------------------------
    if (cand.wrist_margin > best.wrist_margin + eps) return true;
    if (cand.wrist_margin + eps < best.wrist_margin) return false;

    if (cand.sigma_min > best.sigma_min + eps) return true;
    if (cand.sigma_min + eps < best.sigma_min) return false;

    return false;
  }

  // ========================================================
  // Command clamp
  // ========================================================

  void clamp_candidate_step(PoseStamped& cmd, double& yaw_cmd, BuildDebugInfo* dbg = nullptr) const
  {
    const PoseStamped& ref = last_valid_.has_value() ? *last_valid_ : *latest_ee_;

    geometry_msgs::msg::Point dp = point_sub(cmd.pose.position, ref.pose.position);
    const double dist = point_norm(dp);

    if (dbg != nullptr)
    {
      dbg->raw_cmd_step = dist;
    }

    if (dist > max_cmd_step_m_ && dist > 1e-9)
    {
      const double s = max_cmd_step_m_ / dist;
      dp = point_scale(dp, s);
      cmd.pose.position = point_add(ref.pose.position, dp);

      if (dbg != nullptr)
      {
        dbg->pos_clamped = true;
      }
    }

    const double ref_yaw = quat_to_rpy(ref.pose.orientation).yaw;
    double dyaw = wrap_pi(yaw_cmd - ref_yaw);

    if (dbg != nullptr)
    {
      dbg->raw_cmd_dyaw = dyaw;
    }

    if (dyaw > max_cmd_yaw_step_rad_)
    {
      dyaw = max_cmd_yaw_step_rad_;
      if (dbg != nullptr) dbg->yaw_clamped = true;
    }

    if (dyaw < -max_cmd_yaw_step_rad_)
    {
      dyaw = -max_cmd_yaw_step_rad_;
      if (dbg != nullptr) dbg->yaw_clamped = true;
    }

    yaw_cmd = wrap_pi(ref_yaw + dyaw);
    cmd.pose.orientation = rpy_to_quat(nominal_roll_, nominal_pitch_, yaw_cmd);
  }

  // ========================================================
  // Hand deadband
  // ========================================================

  void apply_hand_deadband(
    geometry_msgs::msg::Point& hand_delta,
    double& hand_delta_yaw,
    BuildDebugInfo* dbg = nullptr) const
  {
    const double pos_mag = point_norm(hand_delta);

    if (dbg != nullptr)
    {
      dbg->raw_hand_delta_norm = pos_mag;
      dbg->raw_hand_delta_yaw = hand_delta_yaw;
    }

    if (pos_mag < hand_deadband_m_)
    {
      hand_delta.x = 0.0;
      hand_delta.y = 0.0;
      hand_delta.z = 0.0;

      if (dbg != nullptr)
      {
        dbg->pos_deadbanded = true;
      }
    }

    if (std::abs(hand_delta_yaw) < hand_yaw_deadband_rad_)
    {
      hand_delta_yaw = 0.0;

      if (dbg != nullptr)
      {
        dbg->yaw_deadbanded = true;
      }
    }
}

  // ========================================================
  // Orientation slack search
  // ========================================================

  std::vector<OrientationOption> build_orientation_options(double yaw_cmd) const
  {
    std::vector<double> roll_offsets;
    std::vector<double> pitch_offsets;

    roll_offsets.push_back(0.0);
    pitch_offsets.push_back(0.0);

    for (double d = orientation_search_step_rad_;
         d <= roll_search_max_rad_ + 1e-9;
         d += orientation_search_step_rad_)
    {
      roll_offsets.push_back(+d);
      roll_offsets.push_back(-d);
    }

    for (double d = orientation_search_step_rad_;
         d <= pitch_search_max_rad_ + 1e-9;
         d += orientation_search_step_rad_)
    {
      pitch_offsets.push_back(+d);
      pitch_offsets.push_back(-d);
    }

    std::vector<OrientationOption> options;
    options.reserve(roll_offsets.size() * pitch_offsets.size());

    for (double ro : roll_offsets)
    {
      for (double po : pitch_offsets)
      {
        OrientationOption opt;
        opt.q = rpy_to_quat(nominal_roll_ + ro, nominal_pitch_ + po, yaw_cmd);
        opt.rp_offset_abs = std::abs(ro) + std::abs(po);
        options.push_back(opt);
      }
    }

    return options;
  }

  // ========================================================
  // Candidate build
  // ========================================================

  PoseStamped build_candidate(double& yaw_hand_out, BuildDebugInfo* dbg = nullptr) const
  {
    PoseStamped out;
    out.header.stamp = now();
    out.header.frame_id = base_frame_;

    const auto hand_now_ros = unity_to_ros_base(latest_hand_->pose.position);
    const auto q_hand_ros = unity_quat_to_ros_base(latest_hand_->pose.orientation);
    const auto hand_rpy_ros = quat_to_rpy(q_hand_ros);

    auto hand_delta = point_sub(hand_now_ros, hand_anchor_ros_);
    double hand_delta_yaw = wrap_pi(hand_rpy_ros.yaw - hand_anchor_yaw_);

    apply_hand_deadband(hand_delta, hand_delta_yaw, dbg);

    out.pose.position = point_add(robot_anchor_ros_, hand_delta);
    yaw_hand_out = wrap_pi(robot_anchor_yaw_ + hand_delta_yaw);

    out.pose.orientation = rpy_to_quat(nominal_roll_, nominal_pitch_, yaw_hand_out);

    clamp_candidate_step(out, yaw_hand_out, dbg);
    return out;
  }

  // ========================================================
  // Best-valid fallback search
  // ========================================================

  std::optional<CandidateChoice> find_best_valid(
    const PoseStamped& candidate,
    double yaw_hand,
    SearchDebugStats* dbg = nullptr) const
  {
    std::optional<CandidateChoice> best;

    const double x0 = candidate.pose.position.x;
    const double y0 = candidate.pose.position.y;
    const double z0 = candidate.pose.position.z;

    auto count_reject = [&](RejectReason rr)
    {
      if (dbg == nullptr) return;

      switch (rr)
      {
        case RejectReason::IK_FAIL:   ++dbg->ik_fail; break;
        case RejectReason::JUMP_FAIL: ++dbg->jump_fail; break;
        case RejectReason::WRIST_FAIL:++dbg->wrist_fail; break;
        case RejectReason::JAC_FAIL:  ++dbg->jac_fail; break;
        case RejectReason::NONE:      break;
      }
    };

    auto try_pose = [&](double x, double y, double z, double yaw_t, double pos_offset_norm, double yaw_offset_abs)
    {
      const auto q_options = build_orientation_options(yaw_t);

      for (const auto& qopt : q_options)
      {
        if (dbg != nullptr) ++dbg->tried;

        PoseStamped test = candidate;
        test.header.stamp = now();
        test.header.frame_id = base_frame_;
        test.pose.position.x = x;
        test.pose.position.y = y;
        test.pose.position.z = z;
        test.pose.orientation = qopt.q;

        CandidateChoice cand;
        RejectReason rr = RejectReason::NONE;

        if (!evaluate_candidate(test, pos_offset_norm, yaw_offset_abs, qopt.rp_offset_abs, cand, &rr))
        {
          count_reject(rr);
          continue;
        }

        if (dbg != nullptr) ++dbg->accepted;

        if (!best.has_value() || candidate_better(cand, *best))
        {
          best = cand;
        }
      }
    };

    // Exact target first
    try_pose(x0, y0, z0, yaw_hand, 0.0, 0.0);
    
    // If the exact target already works, stop immediately.
    // No need to search the whole tree.
    if (best.has_value() &&
        best->pos_offset_norm < 1e-9 &&
        best->yaw_offset_abs < 1e-9 &&
        best->rp_offset_abs < 1e-9)
    {
      return best;
    }

    for (int layer = 1; layer <= max_layers_; ++layer)
    {
      const double d = pos_step_m_ * layer;
      const double dyaw = yaw_step_rad_ * layer;

      const std::vector<std::array<double, 3>> offsets = {
        { d, 0, 0}, {-d, 0, 0}, {0,  d, 0}, {0, -d, 0}, {0, 0,  d}, {0, 0, -d},
        { d,  d, 0}, { d, -d, 0}, {-d,  d, 0}, {-d, -d, 0},
        { d, 0,  d}, { d, 0, -d}, {-d, 0,  d}, {-d, 0, -d},
        {0,  d,  d}, {0,  d, -d}, {0, -d,  d}, {0, -d, -d}
      };

      const std::vector<double> yaw_offsets = {0.0, dyaw, -dyaw};

      for (const auto& o : offsets)
      {
        const double pos_offset_norm =
          std::sqrt(o[0] * o[0] + o[1] * o[1] + o[2] * o[2]);

        for (double yo : yaw_offsets)
        {
          const double yaw_t = wrap_pi(yaw_hand + yo);
          const double yaw_offset_abs = std::abs(yo);

          try_pose(x0 + o[0], y0 + o[1], z0 + o[2], yaw_t, pos_offset_norm, yaw_offset_abs);
        }
      }
    }

    return best;
  }

  // ========================================================
  // Main tick
  // ========================================================

  void tick()
  {
    ++debug_tick_counter_;

    const bool log_now =
      debug_verbose_ &&
      (
        debug_log_every_n_ticks_ <= 1 ||
        (debug_tick_counter_ % static_cast<size_t>(debug_log_every_n_ticks_) == 0)
      );

    if (!moveit_ready_)
    {
      if (log_now)
      {
        RCLCPP_WARN(
          get_logger(),
          "[DEBUG][WAIT_MOVEIT] moveit_ready=0");
      }
      return;
    }

    if (!have_inputs())
    {
      if (log_now)
      {
        RCLCPP_WARN(
          get_logger(),
          "[DEBUG][WAIT_INPUTS] hand=%d ee=%d js=%d teleop=%d",
          static_cast<int>(latest_hand_.has_value()),
          static_cast<int>(latest_ee_.has_value()),
          static_cast<int>(latest_js_.has_value()),
          static_cast<int>(teleop_enabled_));
      }
      return;
    }

    if (log_now)
    {
      RCLCPP_INFO(
        get_logger(),
        "[DEBUG][TICK] teleop=%d anchor=%d last_valid=%d has_last_valid_joints=%d invalid_streak=%d",
        static_cast<int>(teleop_enabled_),
        static_cast<int>(anchor_set_),
        static_cast<int>(last_valid_.has_value()),
        static_cast<int>(has_last_valid_joints_),
        invalid_streak_);
    }

    latch_nominal();

    if (!teleop_enabled_)
    {
      if (log_now)
      {
        RCLCPP_INFO(
          get_logger(),
          "[DEBUG][DISABLED] publishing hold pose");
      }

      reset_anchor();
      last_valid_.reset();
      last_valid_joints_.clear();
      has_last_valid_joints_ = false;
      invalid_streak_ = 0;
      publish_hold_pose();
      teleop_prev_ = false;
      return;
    }

    if (!teleop_prev_ && teleop_enabled_)
    {
      reset_anchor();
      latch_anchor();
      last_valid_ = *latest_ee_;
      capture_current_joint_reference();
      invalid_streak_ = 0;
      publish_hold_pose();

      if (debug_verbose_)
      {
        RCLCPP_INFO(
          get_logger(),
          "[DEBUG][ENABLE_EDGE] anchor latched, publishing hold this tick");
      }

      teleop_prev_ = true;
      return;
    }

    double yaw_hand = 0.0;
    BuildDebugInfo build_dbg;
    PoseStamped candidate = build_candidate(yaw_hand, &build_dbg);

    SearchDebugStats search_dbg;
    auto chosen = find_best_valid(candidate, yaw_hand, &search_dbg);

    if (!chosen.has_value())
    {
      RCLCPP_WARN(
        get_logger(),
        "[DEBUG][NO_VALID] tried=%d accepted=%d ik_fail=%d jump_fail=%d wrist_fail=%d jac_fail=%d | hand_delta=%.4f m yaw_delta=%.2f deg deadband[pos=%d yaw=%d] clamp[pos=%d raw=%.4f m yaw=%d raw=%.2f deg] | invalid_streak=%d",
        search_dbg.tried,
        search_dbg.accepted,
        search_dbg.ik_fail,
        search_dbg.jump_fail,
        search_dbg.wrist_fail,
        search_dbg.jac_fail,
        build_dbg.raw_hand_delta_norm,
        build_dbg.raw_hand_delta_yaw * 180.0 / kPi,
        static_cast<int>(build_dbg.pos_deadbanded),
        static_cast<int>(build_dbg.yaw_deadbanded),
        static_cast<int>(build_dbg.pos_clamped),
        build_dbg.raw_cmd_step,
        static_cast<int>(build_dbg.yaw_clamped),
        build_dbg.raw_cmd_dyaw * 180.0 / kPi,
        invalid_streak_);

      handle_invalid_target();
      return;
    }

    last_valid_ = chosen->pose;
    last_valid_joints_ = chosen->joints;
    has_last_valid_joints_ = true;
    invalid_streak_ = 0;

    PoseStamped out = chosen->pose;
    out.header.stamp = now();
    out.header.frame_id = base_frame_;
    pub_cmd_->publish(out);

    RCLCPP_INFO(
      get_logger(),
      "[DEBUG][ACCEPT] tried=%d accepted=%d ik_fail=%d jump_fail=%d wrist_fail=%d jac_fail=%d | chosen[pos_off=%.4f m yaw_off=%.2f deg rp_off=%.2f deg joint_max=%.2f deg joint_sum=%.2f deg wrist_margin=%.2f deg sigma=%.4f] | hand_delta=%.4f m yaw_delta=%.2f deg deadband[pos=%d yaw=%d] clamp[pos=%d raw=%.4f m yaw=%d raw=%.2f deg]",
      search_dbg.tried,
      search_dbg.accepted,
      search_dbg.ik_fail,
      search_dbg.jump_fail,
      search_dbg.wrist_fail,
      search_dbg.jac_fail,
      chosen->pos_offset_norm,
      chosen->yaw_offset_abs * 180.0 / kPi,
      chosen->rp_offset_abs * 180.0 / kPi,
      chosen->joint_change_max * 180.0 / kPi,
      chosen->joint_change_sum * 180.0 / kPi,
      chosen->wrist_margin * 180.0 / kPi,
      chosen->sigma_min,
      build_dbg.raw_hand_delta_norm,
      build_dbg.raw_hand_delta_yaw * 180.0 / kPi,
      static_cast<int>(build_dbg.pos_deadbanded),
      static_cast<int>(build_dbg.yaw_deadbanded),
      static_cast<int>(build_dbg.pos_clamped),
      build_dbg.raw_cmd_step,
      static_cast<int>(build_dbg.yaw_clamped),
      build_dbg.raw_cmd_dyaw * 180.0 / kPi);

    teleop_prev_ = true;
  }
  // ========================================================
  // Parameters
  // ========================================================

  std::string hand_topic_;
  std::string robot_ee_topic_;
  std::string joint_states_topic_;
  std::string command_topic_;
  std::string teleop_enable_topic_;

  std::string group_name_;
  std::string ee_link_;
  std::string base_frame_;

  double rate_hz_{10.0};
  double ik_timeout_s_{0.003};
  double sigma_min_threshold_{0.01};

  double pos_step_m_{0.01};
  double yaw_step_deg_{8.0};
  double yaw_step_rad_{8.0 * kPi / 180.0};
  int max_layers_{2};

  double max_cmd_step_m_{0.02};
  double max_cmd_yaw_step_deg_{8.0};
  double max_cmd_yaw_step_rad_{8.0 * kPi / 180.0};

  double max_joint_jump_rad_{0.8};

  std::string wrist_2_joint_name_{"wrist_2_joint"};
  double wrist_2_min_dist_deg_{10.0};
  double wrist_2_min_dist_rad_{10.0 * kPi / 180.0};

  int invalid_reanchor_ticks_{3};
  int invalid_streak_{0};

  double hand_deadband_m_{0.003};
  double hand_yaw_deadband_deg_{1.5};
  double hand_yaw_deadband_rad_{1.5 * kPi / 180.0};

  double roll_search_max_deg_{10.0};
  double pitch_search_max_deg_{10.0};
  double orientation_search_step_deg_{10.0};

  double roll_search_max_rad_{10.0 * kPi / 180.0};
  double pitch_search_max_rad_{10.0 * kPi / 180.0};
  double orientation_search_step_rad_{10.0 * kPi / 180.0};

  bool debug_verbose_{true};
  int debug_log_every_n_ticks_{1};
  size_t debug_tick_counter_{0};
  // ========================================================
  // Latest inputs
  // ========================================================

  std::optional<PoseStamped> latest_hand_;
  std::optional<PoseStamped> latest_ee_;
  std::optional<sensor_msgs::msg::JointState> latest_js_;

  // ========================================================
  // Nominal orientation state
  // ========================================================

  bool nominal_set_{false};
  double nominal_roll_{0.0};
  double nominal_pitch_{0.0};

  // ========================================================
  // Teleop enable state
  // ========================================================

  bool teleop_enabled_{false};
  bool teleop_prev_{false};

  // ========================================================
  // Teleop anchor state
  // ========================================================

  bool anchor_set_{false};
  geometry_msgs::msg::Point hand_anchor_ros_;
  geometry_msgs::msg::Point robot_anchor_ros_;
  double hand_anchor_yaw_{0.0};
  double robot_anchor_yaw_{0.0};

  // ========================================================
  // Joint jump / wrist guard state
  // ========================================================

  std::vector<std::string> controlled_joint_names_;
  std::vector<double> last_valid_joints_;
  bool has_last_valid_joints_{false};
  int wrist_2_index_{-1};

  // ========================================================
  // Last valid output
  // ========================================================

  std::optional<PoseStamped> last_valid_;

  // ========================================================
  // MoveIt state
  // ========================================================

  bool moveit_ready_{false};
  moveit::core::RobotModelPtr robot_model_;
  const moveit::core::JointModelGroup* jmg_{nullptr};

  // ========================================================
  // ROS interfaces
  // ========================================================

  rclcpp::Subscription<PoseStamped>::SharedPtr sub_hand_;
  rclcpp::Subscription<PoseStamped>::SharedPtr sub_ee_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_enable_;
  rclcpp::Publisher<PoseStamped>::SharedPtr pub_cmd_;
  rclcpp::TimerBase::SharedPtr timer_;
};

// ========================================================
// Main
// ========================================================

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<UnityAuthorityFilter>();
  node->init_moveit();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}