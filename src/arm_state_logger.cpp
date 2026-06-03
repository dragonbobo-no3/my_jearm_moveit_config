#include <action_msgs/msg/goal_status.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using namespace std::chrono_literals;

class ArmStateLogger : public rclcpp::Node {
public:
  ArmStateLogger()
  : rclcpp::Node("arm_state_logger")
  {
    base_link_ = this->declare_parameter<std::string>("base_link", "base_link");
    ee_link_ = this->declare_parameter<std::string>("ee_link", "Link17");
    joint_state_topic_ = this->declare_parameter<std::string>("joint_state_topic", "/joint_states");
    action_status_topic_ = this->declare_parameter<std::string>(
      "action_status_topic", "/jearm_controller/follow_joint_trajectory/_action/status");
    configured_joint_names_ = this->declare_parameter<std::vector<std::string>>(
      "joint_names", std::vector<std::string>{});
    const double log_period_sec = this->declare_parameter<double>("log_period_sec", 5.0);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_,
      rclcpp::SystemDefaultsQoS(),
      std::bind(&ArmStateLogger::handleJointState, this, std::placeholders::_1));

    status_sub_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
      action_status_topic_,
      rclcpp::SystemDefaultsQoS(),
      std::bind(&ArmStateLogger::handleGoalStatus, this, std::placeholders::_1));

    periodic_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(std::max(log_period_sec, 0.1))),
      std::bind(&ArmStateLogger::handlePeriodicLog, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Arm state logger started: joint_states=%s, action_status=%s, TF=%s -> %s, period=%.2fs",
      joint_state_topic_.c_str(),
      action_status_topic_.c_str(),
      base_link_.c_str(),
      ee_link_.c_str(),
      log_period_sec);
  }

private:
  void handleJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    latest_joint_state_ = *msg;
    has_joint_state_ = !msg->name.empty() && msg->name.size() == msg->position.size();
  }

  void handleGoalStatus(const action_msgs::msg::GoalStatusArray::SharedPtr msg)
  {
    for (const auto & status : msg->status_list) {
      const std::string goal_id = formatGoalId(status.goal_info.goal_id.uuid);
      const int8_t previous_status = getPreviousStatus(goal_id);
      goal_status_by_id_[goal_id] = status.status;

      if (
        status.status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED &&
        previous_status != action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)
      {
        logCurrentState("goal_reached", goal_id);
      }
    }
  }

  void handlePeriodicLog()
  {
    logCurrentState("periodic");
  }

  void logCurrentState(const std::string & reason, const std::string & goal_id = std::string())
  {
    sensor_msgs::msg::JointState joint_state_snapshot;
    {
      std::lock_guard<std::mutex> lock(joint_state_mutex_);
      if (!has_joint_state_) {
        return;
      }
      joint_state_snapshot = latest_joint_state_;
    }

    const std::string joint_summary = formatJointPositions(joint_state_snapshot);
    const std::string pose_summary = formatEndEffectorPose();

    if (goal_id.empty()) {
      RCLCPP_INFO(
        this->get_logger(),
        "Arm state [%s]: joints=%s; ee_pose=%s",
        reason.c_str(),
        joint_summary.c_str(),
        pose_summary.c_str());
      return;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Arm state [%s] goal=%s: joints=%s; ee_pose=%s",
      reason.c_str(),
      goal_id.c_str(),
      joint_summary.c_str(),
      pose_summary.c_str());
  }

  int8_t getPreviousStatus(const std::string & goal_id) const
  {
    const auto it = goal_status_by_id_.find(goal_id);
    if (it == goal_status_by_id_.end()) {
      return action_msgs::msg::GoalStatus::STATUS_UNKNOWN;
    }
    return it->second;
  }

  std::string formatGoalId(const std::array<uint8_t, 16> & uuid) const
  {
    std::ostringstream stream;
    stream << std::hex << std::setfill('0');
    for (size_t index = 0; index < uuid.size(); ++index) {
      if (index == 4 || index == 6 || index == 8 || index == 10) {
        stream << '-';
      }
      stream << std::setw(2) << static_cast<int>(uuid[index]);
    }
    return stream.str();
  }

  std::string formatJointPositions(const sensor_msgs::msg::JointState & joint_state) const
  {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(3) << '[';

    std::unordered_map<std::string, double> joint_map;
    for (size_t index = 0; index < joint_state.name.size(); ++index) {
      joint_map[joint_state.name[index]] = joint_state.position[index];
    }

    std::unordered_set<std::string> emitted;
    bool first = true;

    for (const auto & joint_name : configured_joint_names_) {
      const auto it = joint_map.find(joint_name);
      if (it == joint_map.end()) {
        continue;
      }
      if (!first) {
        stream << ", ";
      }
      stream << joint_name << '=' << it->second;
      emitted.insert(joint_name);
      first = false;
    }

    for (size_t index = 0; index < joint_state.name.size(); ++index) {
      const auto & joint_name = joint_state.name[index];
      if (emitted.find(joint_name) != emitted.end()) {
        continue;
      }
      if (!first) {
        stream << ", ";
      }
      stream << joint_name << '=' << joint_state.position[index];
      first = false;
    }

    stream << ']';
    return stream.str();
  }

  std::string formatEndEffectorPose()
  {
    if (!tf_buffer_) {
      return "tf_unavailable";
    }

    try {
      const auto transform = tf_buffer_->lookupTransform(
        base_link_, ee_link_, tf2::TimePointZero, tf2::durationFromSec(0.1));
      const auto & t = transform.transform.translation;
      const auto & q = transform.transform.rotation;

      std::ostringstream stream;
      stream << std::fixed << std::setprecision(4)
             << "pos[x=" << t.x
             << ", y=" << t.y
             << ", z=" << t.z
             << "], quat[x=" << q.x
             << ", y=" << q.y
             << ", z=" << q.z
             << ", w=" << q.w
             << "]";
      return stream.str();
    } catch (const std::exception & ex) {
      return std::string("lookup_failed(") + ex.what() + ")";
    }
  }

  std::string base_link_;
  std::string ee_link_;
  std::string joint_state_topic_;
  std::string action_status_topic_;
  std::vector<std::string> configured_joint_names_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr status_sub_;
  rclcpp::TimerBase::SharedPtr periodic_timer_;

  sensor_msgs::msg::JointState latest_joint_state_;
  std::mutex joint_state_mutex_;
  bool has_joint_state_ = false;
  std::unordered_map<std::string, int8_t> goal_status_by_id_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmStateLogger>());
  rclcpp::shutdown();
  return 0;
}