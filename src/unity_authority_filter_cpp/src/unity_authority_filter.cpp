#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

#include <Eigen/Core>
#include <Eigen/SVD>

#include <cmath>
#include <optional>
#include <string>
#include <vector>
#include <array>

using PoseStamped = geometry_msgs::msg::PoseStamped;

static double wrap_pi(double a)
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

struct RPY { double r, p, y; };

static RPY quat_to_rpy(const geometry_msgs::msg::Quaternion& q)
{
  const double x = q.x, y = q.y, z = q.z, w = q.w;

  const double sinr_cosp = 2.0 * (w * x + y * z);
  const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
  const double roll = std::atan2(sinr_cosp, cosr_cosp);

  const double sinp = 2.0 * (w * y - z * x);
  double pitch;
  if (std::abs(sinp) >= 1.0) pitch = std::copysign(M_PI / 2.0, sinp);
  else pitch = std::asin(sinp);

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

class UnityAuthorityFilter : public rclcpp::Node
{
public:
  UnityAuthorityFilter() : Node("unity_authority_filter_cpp")
  {
    hand_topic_ = declare_parameter<std::string>("hand_topic", "/unity/hand_pose");
    robot_ee_topic_ = declare_parameter<std::string>("robot_ee_topic", "/robot/ee_pose");
    joint_states_topic_ = declare_parameter<std::string>("joint_states_topic", "/joint_states");
    command_topic_ = declare_parameter<std::string>("command_topic", "/unity/command_pose");

    group_name_ = declare_parameter<std::string>("group_name", "ur_manipulator");
    ee_link_ = declare_parameter<std::string>("ee_link", "tool0");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

    rate_hz_ = declare_parameter<double>("rate_hz", 20.0);
    ik_timeout_s_ = declare_parameter<double>("ik_timeout_s", 0.02);
    sigma_min_threshold_ = declare_parameter<double>("sigma_min_threshold", 0.02);

    pos_step_m_ = declare_parameter<double>("pos_step_m", 0.01);
    yaw_step_deg_ = declare_parameter<double>("yaw_step_deg", 5.0);
    max_layers_ = declare_parameter<int>("max_layers", 5);
    yaw_step_rad_ = yaw_step_deg_ * M_PI / 180.0;

    sub_hand_ = create_subscription<PoseStamped>(hand_topic_, 10,
      std::bind(&UnityAuthorityFilter::on_hand, this, std::placeholders::_1));
    sub_ee_ = create_subscription<PoseStamped>(robot_ee_topic_, 10,
      std::bind(&UnityAuthorityFilter::on_robot_ee, this, std::placeholders::_1));
    sub_js_ = create_subscription<sensor_msgs::msg::JointState>(joint_states_topic_, 10,
      std::bind(&UnityAuthorityFilter::on_joint_states, this, std::placeholders::_1));

    pub_cmd_ = create_publisher<PoseStamped>(command_topic_, 10);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(1.0, rate_hz_)),
      std::bind(&UnityAuthorityFilter::tick, this));

    RCLCPP_INFO(get_logger(), "Authority filter constructed (will init MoveIt model next).");
  }

  void init_moveit()
  {
    if (moveit_ready_) return;

    // Now shared_from_this() is valid because init_moveit() is called AFTER make_shared()
    robot_model_loader::RobotModelLoader loader(shared_from_this(), "robot_description");
    robot_model_ = loader.getModel();
    if (!robot_model_) throw std::runtime_error("Failed to load robot model from robot_description");

    jmg_ = robot_model_->getJointModelGroup(group_name_);
    if (!jmg_) throw std::runtime_error("JointModelGroup not found: " + group_name_);

    if (!robot_model_->hasLinkModel(ee_link_)) throw std::runtime_error("EE link not found: " + ee_link_);

    moveit_ready_ = true;

    RCLCPP_INFO(get_logger(), "MoveIt model loaded. hand=%s cmd=%s @ %.1f Hz (group=%s ee=%s)",
      hand_topic_.c_str(), command_topic_.c_str(), rate_hz_, group_name_.c_str(), ee_link_.c_str());
  }

private:
  void on_hand(const PoseStamped::SharedPtr msg) { latest_hand_ = *msg; }
  void on_robot_ee(const PoseStamped::SharedPtr msg) { latest_ee_ = *msg; }
  void on_joint_states(const sensor_msgs::msg::JointState::SharedPtr msg) { latest_js_ = *msg; }

  bool have_inputs() const
  {
    return latest_hand_.has_value() && latest_ee_.has_value() && latest_js_.has_value();
  }

  void latch_nominal()
  {
    if (nominal_set_) return;
    const auto rpy = quat_to_rpy(latest_ee_->pose.orientation);
    nominal_roll_ = rpy.r;
    nominal_pitch_ = rpy.p;
    nominal_set_ = true;
    RCLCPP_INFO(get_logger(), "Latched nominal roll/pitch: roll=%.3f pitch=%.3f", nominal_roll_, nominal_pitch_);
  }

  void seed_state(moveit::core::RobotState& state) const
  {
    const auto& names = latest_js_->name;
    const auto& pos = latest_js_->position;
    for (size_t i = 0; i < names.size() && i < pos.size(); ++i)
      state.setVariablePosition(names[i], pos[i]);
    state.update();
  }

  PoseStamped build_candidate(double& yaw_hand_out) const
  {
    PoseStamped out;
    out.header.stamp = now();
    out.header.frame_id = base_frame_;
    out.pose.position = latest_hand_->pose.position;

    const auto hand_rpy = quat_to_rpy(latest_hand_->pose.orientation);
    yaw_hand_out = hand_rpy.y;

    out.pose.orientation = rpy_to_quat(nominal_roll_, nominal_pitch_, yaw_hand_out);
    return out;
  }

  bool valid_ik_and_jacobian(const PoseStamped& target) const
  {
    moveit::core::RobotState state(robot_model_);
    seed_state(state);

    const bool ok_ik = state.setFromIK(jmg_, target.pose, ee_link_, ik_timeout_s_);
    if (!ok_ik) return false;

    state.update();

    Eigen::Vector3d ref_point(0.0, 0.0, 0.0);
    Eigen::MatrixXd J;

    const bool jac_ok = state.getJacobian(
      jmg_,
      state.getLinkModel(ee_link_),
      ref_point,
      J,
      false
    );
    if (!jac_ok) return false;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const auto& s = svd.singularValues();
    const double sigma_min = (s.size() > 0) ? s(s.size() - 1) : 0.0;

    return sigma_min >= sigma_min_threshold_;
  }

  std::optional<PoseStamped> find_nearest_valid(const PoseStamped& candidate, double yaw_hand) const
  {
    if (valid_ik_and_jacobian(candidate)) return candidate;

    const double x0 = candidate.pose.position.x;
    const double y0 = candidate.pose.position.y;
    const double z0 = candidate.pose.position.z;

    for (int layer = 1; layer <= max_layers_; ++layer)
    {
      const double d = pos_step_m_ * layer;
      const double dyaw = yaw_step_rad_ * layer;

      const std::vector<std::array<double,3>> offsets = {
        { d,0,0},{-d,0,0},{0, d,0},{0,-d,0},{0,0, d},{0,0,-d},
        { d, d,0},{ d,-d,0},{-d, d,0},{-d,-d,0},
        { d,0, d},{ d,0,-d},{-d,0, d},{-d,0,-d},
        {0, d, d},{0, d,-d},{0,-d, d},{0,-d,-d}
      };

      const std::vector<double> yaw_offsets = {0.0, dyaw, -dyaw};

      for (const auto& o : offsets)
      {
        PoseStamped test = candidate;
        test.header.stamp = now();
        test.header.frame_id = base_frame_;

        test.pose.position.x = x0 + o[0];
        test.pose.position.y = y0 + o[1];
        test.pose.position.z = z0 + o[2];

        for (double yo : yaw_offsets)
        {
          const double yaw_t = wrap_pi(yaw_hand + yo);
          test.pose.orientation = rpy_to_quat(nominal_roll_, nominal_pitch_, yaw_t);
          if (valid_ik_and_jacobian(test)) return test;
        }
      }
    }
    return std::nullopt;
  }

  void tick()
  {
    if (!moveit_ready_) return;
    if (!have_inputs()) return;

    latch_nominal();

    double yaw_hand = 0.0;
    PoseStamped candidate = build_candidate(yaw_hand);

    auto chosen = find_nearest_valid(candidate, yaw_hand);

    if (!chosen.has_value())
    {
      if (last_valid_.has_value())
      {
        PoseStamped out = *last_valid_;
        out.header.stamp = now();
        pub_cmd_->publish(out);
      }
      return;
    }

    last_valid_ = *chosen;

    PoseStamped out = *chosen;
    out.header.stamp = now();
    pub_cmd_->publish(out);
  }

private:
  std::string hand_topic_, robot_ee_topic_, joint_states_topic_, command_topic_;
  std::string group_name_, ee_link_, base_frame_;

  double rate_hz_{20.0};
  double ik_timeout_s_{0.02};
  double sigma_min_threshold_{0.02};

  double pos_step_m_{0.01};
  double yaw_step_deg_{5.0};
  double yaw_step_rad_{0.0872664626};
  int max_layers_{5};

  std::optional<PoseStamped> latest_hand_;
  std::optional<PoseStamped> latest_ee_;
  std::optional<sensor_msgs::msg::JointState> latest_js_;

  bool nominal_set_{false};
  double nominal_roll_{0.0};
  double nominal_pitch_{0.0};

  std::optional<PoseStamped> last_valid_;

  bool moveit_ready_{false};
  moveit::core::RobotModelPtr robot_model_;
  const moveit::core::JointModelGroup* jmg_{nullptr};

  rclcpp::Subscription<PoseStamped>::SharedPtr sub_hand_;
  rclcpp::Subscription<PoseStamped>::SharedPtr sub_ee_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
  rclcpp::Publisher<PoseStamped>::SharedPtr pub_cmd_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<UnityAuthorityFilter>();
  node->init_moveit();   // <-- critical: after make_shared(), shared_from_this() is safe

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}