#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

#include <Eigen/Core>

#include <cmath>
#include <optional>
#include <string>
#include <vector>

using PoseStamped = geometry_msgs::msg::PoseStamped;
using JointState = sensor_msgs::msg::JointState;
using FollowJT = control_msgs::action::FollowJointTrajectory;
using GoalHandleFJT = rclcpp_action::ClientGoalHandle<FollowJT>;

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

static double wrap_pi(double a)
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

class UnityGoalExecutor : public rclcpp::Node
{
public:
  UnityGoalExecutor() : Node("unity_goal_executor")
  {
    command_topic_ = declare_parameter<std::string>("command_topic", "/unity/command_pose");
    joint_states_topic_ = declare_parameter<std::string>("joint_states_topic", "/joint_states");

    group_name_ = declare_parameter<std::string>("group_name", "ur_manipulator");
    ee_link_ = declare_parameter<std::string>("ee_link", "tool0");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

    action_name_ = declare_parameter<std::string>(
      "follow_action", "/joint_trajectory_controller/follow_joint_trajectory"
    );

    exec_rate_hz_ = declare_parameter<double>("exec_rate_hz", 15.0);
    traj_duration_s_ = declare_parameter<double>("traj_duration_s", 0.25);
    ik_timeout_s_ = declare_parameter<double>("ik_timeout_s", 0.01);

    deadband_pos_m_ = declare_parameter<double>("deadband_pos_m", 0.004);
    deadband_yaw_rad_ = declare_parameter<double>("deadband_yaw_rad", 0.03);

    joint_names_ = declare_parameter<std::vector<std::string>>(
      "joint_names",
      std::vector<std::string>{
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint"
      }
    );

    sub_cmd_ = create_subscription<PoseStamped>(
      command_topic_, 10, std::bind(&UnityGoalExecutor::on_cmd, this, std::placeholders::_1)
    );
    sub_js_ = create_subscription<JointState>(
      joint_states_topic_, 10, std::bind(&UnityGoalExecutor::on_js, this, std::placeholders::_1)
    );

    action_client_ = rclcpp_action::create_client<FollowJT>(this, action_name_);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(1.0, exec_rate_hz_)),
      std::bind(&UnityGoalExecutor::tick, this)
    );

    RCLCPP_INFO(get_logger(), "Goal executor constructed (will init MoveIt model next).");
  }

  void init_moveit()
  {
    if (moveit_ready_) return;

    robot_model_loader::RobotModelLoader loader(shared_from_this(), "robot_description");
    robot_model_ = loader.getModel();
    if (!robot_model_) throw std::runtime_error("Failed to load robot model from robot_description");

    jmg_ = robot_model_->getJointModelGroup(group_name_);
    if (!jmg_) throw std::runtime_error("JointModelGroup not found: " + group_name_);
    if (!robot_model_->hasLinkModel(ee_link_)) throw std::runtime_error("EE link not found: " + ee_link_);

    if (!jmg_->canSetStateFromIK(ee_link_))
    {
      throw std::runtime_error(
        "No IK solver available for group '" + group_name_ + "' with tip '" + ee_link_ +
        "'. Ensure kinematics.yaml is passed as parameters to THIS node."
      );
    }

    moveit_ready_ = true;

    RCLCPP_INFO(get_logger(),
      "MoveIt model loaded. cmd=%s action=%s rate=%.1fHz traj=%.2fs deadband=%.3fm/%.3frad",
      command_topic_.c_str(), action_name_.c_str(), exec_rate_hz_, traj_duration_s_,
      deadband_pos_m_, deadband_yaw_rad_);
  }

private:
  void on_cmd(const PoseStamped::SharedPtr msg) { latest_cmd_ = *msg; }
  void on_js(const JointState::SharedPtr msg) { latest_js_ = *msg; }

  bool moved_enough(const PoseStamped& a, const PoseStamped& b) const
  {
    const double dx = a.pose.position.x - b.pose.position.x;
    const double dy = a.pose.position.y - b.pose.position.y;
    const double dz = a.pose.position.z - b.pose.position.z;
    const double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

    const double yaw_a = quat_to_rpy(a.pose.orientation).y;
    const double yaw_b = quat_to_rpy(b.pose.orientation).y;
    const double dyaw = std::abs(wrap_pi(yaw_a - yaw_b));

    return (dist >= deadband_pos_m_) || (dyaw >= deadband_yaw_rad_);
  }

  void seed_state(moveit::core::RobotState& state) const
  {
    const auto& names = latest_js_->name;
    const auto& pos = latest_js_->position;
    for (size_t i = 0; i < names.size() && i < pos.size(); ++i)
      state.setVariablePosition(names[i], pos[i]);
    state.update();
  }

  bool solve_ik(const PoseStamped& target, std::vector<double>& out_positions)
  {
    moveit::core::RobotState state(robot_model_);
    seed_state(state);

    if (!target.header.frame_id.empty() && target.header.frame_id != base_frame_)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "command_pose frame_id='%s' (expected '%s')",
        target.header.frame_id.c_str(), base_frame_.c_str());
    }

    const bool ok = state.setFromIK(jmg_, target.pose, ee_link_, ik_timeout_s_);
    if (!ok) return false;

    state.update();

    out_positions.clear();
    out_positions.reserve(joint_names_.size());
    for (const auto& jn : joint_names_)
      out_positions.push_back(state.getVariablePosition(jn));

    return true;
  }

  void send_trajectory_goal(const std::vector<double>& q_target)
  {
    if (!action_client_->wait_for_action_server(std::chrono::milliseconds(100)))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "FollowJointTrajectory action not available: %s", action_name_.c_str());
      return;
    }

    FollowJT::Goal goal;
    goal.trajectory.joint_names = joint_names_;

    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions = q_target;
    pt.time_from_start = rclcpp::Duration::from_seconds(traj_duration_s_);
    goal.trajectory.points.push_back(pt);

    auto send_opts = rclcpp_action::Client<FollowJT>::SendGoalOptions();

    send_opts.goal_response_callback =
      [this](std::shared_ptr<GoalHandleFJT> gh)
      {
        if (!gh)
        {
          RCLCPP_WARN(get_logger(), "Trajectory goal rejected.");
          return;
        }
        // We don't cancel old goals; latest-goal-wins by overwriting with new goals.
        active_goal_handle_ = gh;
      };

    send_opts.result_callback =
      [this](const GoalHandleFJT::WrappedResult& res)
      {
        (void)res;
      };

    action_client_->async_send_goal(goal, send_opts);
  }

  void tick()
  {
    if (!moveit_ready_) return;
    if (!latest_cmd_.has_value()) return;
    if (!latest_js_.has_value()) return;

    const PoseStamped& cmd = *latest_cmd_;

    if (last_sent_cmd_.has_value() && !moved_enough(cmd, *last_sent_cmd_))
      return;

    std::vector<double> q_target;
    if (!solve_ik(cmd, q_target))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "IK failed for command_pose; holding.");
      return;
    }

    send_trajectory_goal(q_target);
    last_sent_cmd_ = cmd;
  }

private:
  std::string command_topic_, joint_states_topic_;
  std::string group_name_, ee_link_, base_frame_;
  std::string action_name_;

  double exec_rate_hz_{15.0};
  double traj_duration_s_{0.25};
  double ik_timeout_s_{0.01};

  double deadband_pos_m_{0.004};
  double deadband_yaw_rad_{0.03};

  std::vector<std::string> joint_names_;

  bool moveit_ready_{false};
  moveit::core::RobotModelPtr robot_model_;
  const moveit::core::JointModelGroup* jmg_{nullptr};

  std::optional<PoseStamped> latest_cmd_;
  std::optional<JointState> latest_js_;
  std::optional<PoseStamped> last_sent_cmd_;

  rclcpp::Subscription<PoseStamped>::SharedPtr sub_cmd_;
  rclcpp::Subscription<JointState>::SharedPtr sub_js_;
  rclcpp_action::Client<FollowJT>::SharedPtr action_client_;
  std::shared_ptr<GoalHandleFJT> active_goal_handle_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<UnityGoalExecutor>();
  node->init_moveit();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}