/*
 * Fake Trajectory Executor for JEARM ARM
 * Simulates a joint trajectory controller without actual hardware
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <chrono>
#include <thread>
#include <cstdlib>
#include <mutex>
#include <cmath>

class FakeTrajectoryExecutor : public rclcpp::Node {
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFollowJointTrajectory =
    rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

  FakeTrajectoryExecutor() : rclcpp::Node("fake_trajectory_executor") {
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
      current_joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
      current_joint_positions_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      has_joint_state_ = true;
    }

    std::thread{std::bind(&FakeTrajectoryExecutor::publish_joint_states_loop, this)}.detach();

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    auto msg = std::make_unique<sensor_msgs::msg::JointState>();
    msg->header.stamp = this->now();
    msg->name = current_joint_names_;
    msg->position = current_joint_positions_;
    msg->velocity.resize(7, 0.0);
    msg->effort.resize(7, 0.0);
    joint_state_pub_->publish(std::move(msg));

    RCLCPP_INFO(
      this->get_logger(),
      "Fake Trajectory Executor initialized for jearm_controller (Home position: [0, 0, 0, 0, 0, 0, 0])");
  }

private:
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  std::vector<std::string> current_joint_names_;
  std::vector<double> current_joint_positions_;
  std::mutex joint_state_mutex_;
  bool has_joint_state_ = false;

  rclcpp_action::GoalResponse handle_goal(
    const std::array<unsigned char, 16> & uuid,
    std::shared_ptr<const FollowJointTrajectory::Goal> goal)
  {
    (void)uuid;
    (void)goal;
    RCLCPP_INFO(this->get_logger(), "Received new goal");
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

    RCLCPP_INFO(this->get_logger(), "=== Trajectory execution COMPLETED ===");
    goal_handle->succeed(result);
  }

  void publish_joint_state(
    const std::vector<std::string> & joint_names,
    const std::vector<double> & positions)
  {
    {
      std::lock_guard<std::mutex> lock(joint_state_mutex_);
      current_joint_names_ = joint_names;
      current_joint_positions_ = positions;
      has_joint_state_ = true;
    }

    auto msg = std::make_unique<sensor_msgs::msg::JointState>();
    msg->header.stamp = this->now();
    msg->name = joint_names;
    msg->position = positions;
    msg->velocity.resize(positions.size(), 0.0);
    msg->effort.resize(positions.size(), 0.0);

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
