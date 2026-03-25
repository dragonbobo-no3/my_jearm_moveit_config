/*
 * Fake Trajectory Executor for JEARM ARM
 * Simulates a joint trajectory controller without actual hardware
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <chrono>
#include <thread>
#include <cstdlib>
#include <mutex>
#include <cmath>
#include <sstream>
#include <iomanip>

class FakeTrajectoryExecutor : public rclcpp::Node {
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFollowJointTrajectory =
    rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

  FakeTrajectoryExecutor() : rclcpp::Node("fake_trajectory_executor") {
    base_link_ = this->declare_parameter<std::string>("base_link", "base_link");
    ee_link_ = this->declare_parameter<std::string>("ee_link", "Link17");
    const std::vector<std::string> default_joint_names = {
      "joint11", "joint12", "joint13", "joint14", "joint15", "joint16", "joint17",
      "joint21", "joint22", "joint23", "joint24", "joint25", "joint26", "joint27"};
    const auto configured_joint_names = this->declare_parameter<std::vector<std::string>>(
      "joint_names", default_joint_names);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    action_server_ =
      rclcpp_action::create_server<FollowJointTrajectory>(
      this,
      "jearm_controller/follow_joint_trajectory",
      std::bind(&FakeTrajectoryExecutor::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&FakeTrajectoryExecutor::handle_cancel, this, std::placeholders::_1),
      std::bind(&FakeTrajectoryExecutor::handle_accepted, this, std::placeholders::_1));

    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    {
      std::lock_guard<std::mutex> lock(joint_state_mutex_);
      current_joint_names_ = configured_joint_names;
      if (current_joint_names_.empty()) {
        current_joint_names_ = default_joint_names;
      }
      current_joint_positions_.assign(current_joint_names_.size(), 0.0);
      has_joint_state_ = true;
    }

    std::thread{std::bind(&FakeTrajectoryExecutor::publish_joint_states_loop, this)}.detach();

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    auto msg = std::make_unique<sensor_msgs::msg::JointState>();
    msg->header.stamp = this->now();
    msg->name = current_joint_names_;
    msg->position = current_joint_positions_;
    msg->velocity.resize(current_joint_positions_.size(), 0.0);
    msg->effort.resize(current_joint_positions_.size(), 0.0);
    joint_state_pub_->publish(std::move(msg));

    RCLCPP_INFO(
      this->get_logger(),
      "Fake Trajectory Executor initialized for jearm_controller (%zu joints, TF: %s -> %s)",
      current_joint_names_.size(), base_link_.c_str(), ee_link_.c_str());
  }

private:
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  std::vector<std::string> current_joint_names_;
  std::vector<double> current_joint_positions_;
  std::mutex joint_state_mutex_;
  bool has_joint_state_ = false;
  std::string base_link_;
  std::string ee_link_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp_action::GoalResponse handle_goal(
    const std::array<unsigned char, 16> & uuid,
    std::shared_ptr<const FollowJointTrajectory::Goal> goal)
  {
    (void)uuid;
    if (!goal) {
      RCLCPP_ERROR(this->get_logger(), "Received null goal");
      return rclcpp_action::GoalResponse::REJECT;
    }

    const auto & trajectory = goal->trajectory;
    RCLCPP_INFO(
      this->get_logger(),
      "Received new goal: %zu points, joints=[%s]",
      trajectory.points.size(),
      join_joint_names(trajectory.joint_names).c_str());

    if (!trajectory.points.empty()) {
      const auto to_sec = [](const builtin_interfaces::msg::Duration & duration_msg) {
          return duration_msg.sec + duration_msg.nanosec / 1e9;
        };

      const double first_time = to_sec(trajectory.points.front().time_from_start);
      const double last_time = to_sec(trajectory.points.back().time_from_start);
      bool is_monotonic = true;
      bool has_effective_timing = false;

      for (size_t i = 1; i < trajectory.points.size(); ++i) {
        const double prev_time = to_sec(trajectory.points[i - 1].time_from_start);
        const double curr_time = to_sec(trajectory.points[i].time_from_start);
        if (curr_time + 1e-9 < prev_time) {
          is_monotonic = false;
        }
        if (curr_time - prev_time > 1e-3) {
          has_effective_timing = true;
        }
      }

      RCLCPP_INFO(
        this->get_logger(),
        "Trajectory timing summary: first=%.3fs, last=%.3fs, duration=%.3fs, monotonic=%s, effective_timing=%s",
        first_time,
        last_time,
        std::max(0.0, last_time - first_time),
        is_monotonic ? "true" : "false",
        has_effective_timing ? "true" : "false");
    }

    for (size_t i = 0; i < trajectory.points.size(); ++i) {
      const auto & point = trajectory.points[i];
      const double point_time = point.time_from_start.sec + point.time_from_start.nanosec / 1e9;
      RCLCPP_DEBUG(
        this->get_logger(),
        "Path point[%zu] t=%.3fs %s",
        i,
        point_time,
        format_joint_positions(trajectory.joint_names, point.positions).c_str());
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Received cancel request");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
  {
    {
      std::lock_guard<std::mutex> lock(joint_state_mutex_);
      if (has_joint_state_ && !current_joint_names_.empty()) {
        auto msg = std::make_unique<sensor_msgs::msg::JointState>();
        msg->header.stamp = this->now();
        msg->name = current_joint_names_;
        msg->position = current_joint_positions_;
        msg->velocity.resize(current_joint_positions_.size(), 0.0);
        msg->effort.resize(current_joint_positions_.size(), 0.0);
        joint_state_pub_->publish(std::move(msg));
      }
    }

    std::thread{std::bind(&FakeTrajectoryExecutor::execute, this, goal_handle)}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "=== Executing trajectory with %lu points ===",
      goal_handle->get_goal()->trajectory.points.size());

    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<FollowJointTrajectory::Result>();

    if (!goal) {
      RCLCPP_ERROR(this->get_logger(), "Goal is null!");
      goal_handle->abort(result);
      return;
    }

    bool needs_timing = false;
    if (goal->trajectory.points.size() > 1) {
      const auto & first_point = goal->trajectory.points[0];
      const auto & second_point = goal->trajectory.points[1];
      double first_time = first_point.time_from_start.sec + first_point.time_from_start.nanosec / 1e9;
      double second_time = second_point.time_from_start.sec + second_point.time_from_start.nanosec / 1e9;

      if (std::abs(second_time - first_time) < 0.001) {
        needs_timing = true;
        RCLCPP_WARN(
          this->get_logger(),
          "Trajectory has no timing information - will compute based on joint velocities");
      }
    }

    double accumulated_time = 0.0;

    for (size_t i = 0; i < goal->trajectory.points.size(); ++i) {
      if (goal_handle->is_canceling()) {
        RCLCPP_INFO(this->get_logger(), "Trajectory cancelled at point %lu", i);
        goal_handle->canceled(result);
        return;
      }

      const auto & point = goal->trajectory.points[i];

      double point_time;
      if (needs_timing && i > 0) {
        const auto & prev_point = goal->trajectory.points[i - 1];
        double max_joint_delta = 0.0;
        for (size_t j = 0; j < point.positions.size(); ++j) {
          double delta = std::abs(point.positions[j] - prev_point.positions[j]);
          max_joint_delta = std::max(max_joint_delta, delta);
        }
        double segment_time = max_joint_delta / 1.0;
        accumulated_time += segment_time;
        point_time = accumulated_time;
      } else {
        point_time = point.time_from_start.sec + point.time_from_start.nanosec / 1e9;
      }

      std::string positions_str = "Point " + std::to_string(i) + " [";
      for (size_t j = 0; j < point.positions.size() && j < 3; ++j) {
        positions_str += std::to_string(point.positions[j]) + " ";
      }
      positions_str += "...] time=" + std::to_string(point_time) + "s";
      RCLCPP_DEBUG(this->get_logger(), "%s", positions_str.c_str());

      publish_joint_state(goal->trajectory.joint_names, point.positions);

      if (i < goal->trajectory.points.size() - 1) {
        double next_point_time;
        if (needs_timing) {
          const auto & next_point = goal->trajectory.points[i + 1];
          double max_joint_delta = 0.0;
          for (size_t j = 0; j < next_point.positions.size(); ++j) {
            double delta = std::abs(next_point.positions[j] - point.positions[j]);
            max_joint_delta = std::max(max_joint_delta, delta);
          }
          double segment_time = max_joint_delta / 1.0;
          next_point_time = point_time + segment_time;
        } else {
          const auto & next_point = goal->trajectory.points[i + 1];
          next_point_time = next_point.time_from_start.sec + next_point.time_from_start.nanosec / 1e9;
        }

        double speed_factor = 1.0;
        const char * env_factor = std::getenv("TRAJECTORY_SPEED_FACTOR");
        if (env_factor) {
          speed_factor = std::atof(env_factor);
          if (speed_factor <= 0) {
            speed_factor = 1.0;
          }
        }

        double wait_ms = (next_point_time - point_time) * 1000 * speed_factor;

        RCLCPP_DEBUG(
          this->get_logger(),
          "Waiting %.2f ms (current=%.3fs, next=%.3fs, speed_factor=%.2f)",
          wait_ms, point_time, next_point_time, speed_factor);

        if (wait_ms > 1) {
          std::this_thread::sleep_for(std::chrono::milliseconds((long)wait_ms));
        }
      }
    }

    const auto & last_point = goal->trajectory.points.back();
    publish_joint_state(goal->trajectory.joint_names, last_point.positions);

    std::vector<std::string> final_names;
    std::vector<double> final_positions;
    {
      std::lock_guard<std::mutex> lock(joint_state_mutex_);
      final_names = current_joint_names_;
      final_positions = current_joint_positions_;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Final arm state: %s",
      format_joint_positions(final_names, final_positions).c_str());

    log_end_effector_pose();

    RCLCPP_INFO(this->get_logger(), "=== Trajectory execution COMPLETED ===");
    goal_handle->succeed(result);
  }

  void log_end_effector_pose()
  {
    if (!tf_buffer_) {
      RCLCPP_WARN(this->get_logger(), "TF buffer not initialized, skip end-effector pose logging");
      return;
    }

    for (int attempt = 0; attempt < 5; ++attempt) {
      try {
        const auto transform = tf_buffer_->lookupTransform(
          base_link_, ee_link_, tf2::TimePointZero, tf2::durationFromSec(0.1));

        const auto & t = transform.transform.translation;
        const auto & q = transform.transform.rotation;
        RCLCPP_INFO(
          this->get_logger(),
          "Final EE pose (%s -> %s): pos[x=%.4f, y=%.4f, z=%.4f], quat[x=%.4f, y=%.4f, z=%.4f, w=%.4f]",
          base_link_.c_str(), ee_link_.c_str(),
          t.x, t.y, t.z,
          q.x, q.y, q.z, q.w);
        return;
      } catch (const std::exception & ex) {
        if (attempt == 4) {
          RCLCPP_WARN(
            this->get_logger(),
            "Failed to query final EE pose (%s -> %s): %s",
            base_link_.c_str(), ee_link_.c_str(), ex.what());
          return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
    }
  }

  std::string join_joint_names(const std::vector<std::string> & joint_names) const
  {
    std::ostringstream stream;
    for (size_t i = 0; i < joint_names.size(); ++i) {
      if (i > 0) {
        stream << ", ";
      }
      stream << joint_names[i];
    }
    return stream.str();
  }

  std::string format_joint_positions(
    const std::vector<std::string> & joint_names,
    const std::vector<double> & positions) const
  {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(3) << "[";
    const size_t count = std::min(joint_names.size(), positions.size());
    for (size_t i = 0; i < count; ++i) {
      if (i > 0) {
        stream << ", ";
      }
      stream << joint_names[i] << "=" << positions[i];
    }
    if (positions.size() > joint_names.size()) {
      if (count > 0) {
        stream << ", ";
      }
      stream << "extra_positions=" << (positions.size() - joint_names.size());
    }
    stream << "]";
    return stream.str();
  }

  void publish_joint_state(
    const std::vector<std::string> & joint_names,
    const std::vector<double> & positions)
  {
    if (joint_names.size() != positions.size()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Trajectory point mismatch: joint_names=%zu positions=%zu",
        joint_names.size(), positions.size());
      return;
    }

    {
      std::lock_guard<std::mutex> lock(joint_state_mutex_);
      for (size_t i = 0; i < joint_names.size(); ++i) {
        const auto it = std::find(current_joint_names_.begin(), current_joint_names_.end(), joint_names[i]);
        if (it == current_joint_names_.end()) {
          RCLCPP_WARN(
            this->get_logger(),
            "Ignoring unknown joint in trajectory: %s",
            joint_names[i].c_str());
          continue;
        }

        const size_t index = static_cast<size_t>(std::distance(current_joint_names_.begin(), it));
        if (index < current_joint_positions_.size()) {
          current_joint_positions_[index] = positions[i];
        }
      }
      has_joint_state_ = true;
    }

    auto msg = std::make_unique<sensor_msgs::msg::JointState>();
    msg->header.stamp = this->now();
    msg->name = current_joint_names_;
    msg->position = current_joint_positions_;
    msg->velocity.resize(current_joint_positions_.size(), 0.0);
    msg->effort.resize(current_joint_positions_.size(), 0.0);

    joint_state_pub_->publish(std::move(msg));
  }

  void publish_joint_states_loop()
  {
    rclcpp::Rate rate(50);
    while (rclcpp::ok()) {
      {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        if (has_joint_state_ && !current_joint_names_.empty()) {
          auto msg = std::make_unique<sensor_msgs::msg::JointState>();
          msg->header.stamp = this->now();
          msg->name = current_joint_names_;
          msg->position = current_joint_positions_;
          msg->velocity.resize(current_joint_positions_.size(), 0.0);
          msg->effort.resize(current_joint_positions_.size(), 0.0);
          joint_state_pub_->publish(std::move(msg));
        }
      }
      rate.sleep();
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeTrajectoryExecutor>());
  rclcpp::shutdown();
  return 0;
}
