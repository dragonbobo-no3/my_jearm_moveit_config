#pragma once

#include "je2_ros_goal_execution.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <functional>
#include <limits>
#include <stdexcept>
#include <thread>

namespace je2_experiment
{
// Test-only real FollowJointTrajectory endpoint. No SDK, claim, transport,
// timer, worker thread, install rule or default controller action name.
// Construct, spin callbacks, observe, poll and destroy on ONE thread, outside
// nested spins. The adapter/client must also be accessed on that same thread.
// Injected clock/send must be nonblocking and non-reentrant. Send means a single
// transport attempt, NOT a controller ACK. Failures are never retried.
class RosActionExecutor
{
public:
  using Action = control_msgs::action::FollowJointTrajectory;
  using Handle = rclcpp_action::ServerGoalHandle<Action>;
  using Clock = std::function<double()>;
  using Send = std::function<bool(const char *, std::size_t)>;

  RosActionExecutor(rclcpp::Node::SharedPtr node, const std::string & action_name,
    RosGoalAdapter & adapter, RosGoalExecution::Policy policy, Clock clock, Send send)
  : node_(std::move(node)), adapter_(adapter), execution_(adapter, policy),
    max_feedback_age_(policy.max_result_age), clock_(std::move(clock)), send_(std::move(send)),
    thread_(std::this_thread::get_id())
  {
    if (!node_ || action_name.empty() || !clock_ || !send_) throw std::invalid_argument("explicit action dependencies required");
    group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    server_ = rclcpp_action::create_server<Action>(node_, action_name,
      [this](const rclcpp_action::GoalUUID & id, std::shared_ptr<const Action::Goal> goal) {
        require_thread();
        if (stopped_ || ticket_ || handle_) return rclcpp_action::GoalResponse::REJECT;
        auto ticket = execution_.start(*goal, now());
        if (!ticket) return rclcpp_action::GoalResponse::REJECT;
        ticket_ = std::move(ticket);
        id_ = id;
        sent_points_ = 0;
        feedback_sequence_ = 0;
        cancel_accepted_ = false;
        // No frame is sent before the accepted callback associates this UUID.
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](std::shared_ptr<Handle> handle) {
        require_thread();
        if (stopped_ || !handle_ || handle != handle_ || handle->get_goal_id() != id_ || cancel_accepted_)
          return rclcpp_action::CancelResponse::REJECT;
        char wire[MK2_FRAME_MAX];
        const int size = execution_.request_cancel(ticket_, now(), wire, sizeof(wire));
        if (size < 0) return rclcpp_action::CancelResponse::REJECT;
        cancel_accepted_ = true;
        if (size > 0) send_once(wire, size);
        // Humble changes to CANCELING AFTER this callback returns. Never call
        // canceled/abort here, including on a failed Release send. poll does it.
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<Handle> handle) {
        require_thread();
        if (!ticket_ || handle_ || handle->get_goal_id() != id_)
          throw std::logic_error("action admission identity mismatch");
        handle_ = std::move(handle); // ACCEPT_AND_EXECUTE already executes it
      }, rcl_action_server_get_default_options(), group_);
  }

  RosActionExecutor(const RosActionExecutor &) = delete;
  RosActionExecutor & operator=(const RosActionExecutor &) = delete;

  ~RosActionExecutor()
  {
    // Explicit shutdown while the context is live is preferred. Never let the
    // framework's goal-handle destructor report CANCELED without our receipt.
    try { shutdown(); } catch (...) { /* teardown cannot certify an outcome */ }
    server_.reset(); // expire framework publication callbacks before handle
    handle_.reset();
  }

  int observe(const char * wire, std::size_t size)
  {
    require_thread();
    if (stopped_) return -1;
    return adapter_.observe(wire, size, now());
  }

  // Called independently of callbacks and observations by the serialized
  // executor. Absolute schedule remains anchored to admission, not first poll.
  void poll()
  {
    require_thread();
    if (stopped_) return;
    const double time = now();
    if (!handle_) { (void)adapter_.tick(time); return; }
    char wire[MK2_FRAME_MAX];
    const auto step = execution_.advance(ticket_, time, wire, sizeof(wire));
    if (step.output == RosGoalExecution::Output::Target || step.output == RosGoalExecution::Output::Release) {
      const bool sent = send_once(wire, step.size);
      if (sent && step.output == RosGoalExecution::Output::Target) ++sent_points_;
    }
    publish_feedback(time);
    publish_result();
  }

  // Local shutdown is ABORTED, not a physical stop, normal release or CANCELED.
  // No frame is emitted, no recovery is attempted. Stop spinning before destroy.
  void shutdown()
  {
    require_thread();
    if (stopped_) return;
    stopped_ = true;
    if (ticket_) (void)execution_.send_failed(ticket_);
    if (handle_ && handle_->is_active()) {
      auto result = std::make_shared<Action::Result>();
      result->error_code = Action::Result::PATH_TOLERANCE_VIOLATED;
      result->error_string = "JE2 experiment: executor_shutdown_without_committed_result";
      handle_->abort(result);
    }
    handle_.reset();
    ticket_.reset();
  }

private:
  void require_thread() const
  {
    if (std::this_thread::get_id() != thread_) throw std::logic_error("JE2 action executor requires its single owner thread");
  }
  double now() const
  {
    try { return clock_(); } catch (...) { return std::numeric_limits<double>::quiet_NaN(); }
  }
  bool send_once(const char * wire, int size)
  {
    bool sent = false;
    try { sent = size > 0 && send_(wire, static_cast<std::size_t>(size)); } catch (...) { sent = false; }
    if (!sent) (void)execution_.send_failed(ticket_);
    return sent;
  }
  void publish_feedback(double time)
  {
    // Only positions are observed. Do not invent actual velocity/acceleration,
    // interpolate a desired target, or republish duplicates as fresh samples.
    if (!handle_->is_executing() || sent_points_ == 0) return;
    RosGoalAdapter::Observation sample;
    if (!adapter_.observation(ticket_, sample) || sample.sequence == 0 || sample.sequence == feedback_sequence_ ||
        !std::isfinite(time) || sample.received_at > time || time - sample.received_at > max_feedback_age_) return;
    const auto & goal = handle_->get_goal()->trajectory;
    auto feedback = std::make_shared<Action::Feedback>();
    feedback->header.stamp = node_->now(); // publication ROS time, not vendor acquisition time
    feedback->joint_names = goal.joint_names;
    feedback->desired.positions = goal.points.at(sent_points_ - 1).positions;
    feedback->desired.time_from_start = goal.points.at(sent_points_ - 1).time_from_start;
    feedback->actual.positions.assign(sample.position.begin(), sample.position.end());
    for (std::size_t axis = 0; axis < MK2_AXES; ++axis)
      feedback->error.positions.push_back(feedback->desired.positions[axis] - sample.position[axis]);
    handle_->publish_feedback(feedback);
    feedback_sequence_ = sample.sequence;
  }
  static const char * reason(RosGoalExecution::Reason reason)
  {
    using R = RosGoalExecution::Reason;
    switch (reason) {
      case R::None: return "close_confirmed";
      case R::GoalTolerance: return "goal_tolerance";
      case R::ControlLost: return "control_lost";
      case R::ControlFault: return "control_fault";
      case R::ScheduleExpired: return "schedule_expired";
      case R::SendFailed: return "send_failed";
      case R::InvalidClock: return "invalid_clock";
      case R::StaleEvidence: return "stale_evidence";
    }
    return "unknown_failure";
  }
  void publish_result()
  {
    // Also guard the framework transition explicitly; owner-thread execution
    // ensures this cannot interleave with the cancel callback's return path.
    if (cancel_accepted_ && !handle_->is_canceling()) return;
    const auto result = execution_.take_result(ticket_);
    if (!result) return;
    auto message = std::make_shared<Action::Result>();
    message->error_string = std::string("JE2 experiment: ") + reason(result->reason);
    using State = RosGoalExecution::State;
    if (result->state == State::Succeeded || result->state == State::Canceled) {
      message->error_code = Action::Result::SUCCESSFUL;
    } else {
      // FJT has no CONTROL_FAILED constant. Keep a standard negative code,
      // with the actual experimental failure reason in error_string.
      message->error_code = result->reason == RosGoalExecution::Reason::GoalTolerance ?
        Action::Result::GOAL_TOLERANCE_VIOLATED : Action::Result::PATH_TOLERANCE_VIOLATED;
    }
    try {
      if (result->state == State::Succeeded) handle_->succeed(message);
      else if (result->state == State::Canceled) handle_->canceled(message);
      else handle_->abort(message);
    } catch (...) {
      stopped_ = true; // no retry, new admission or additional frames on publication failure
      throw;
    }
    handle_.reset();
    ticket_.reset();
  }

  rclcpp::Node::SharedPtr node_;
  RosGoalAdapter & adapter_;
  RosGoalExecution execution_;
  const double max_feedback_age_;
  Clock clock_;
  Send send_;
  const std::thread::id thread_;
  RosGoalAdapter::Ticket ticket_;
  rclcpp_action::GoalUUID id_{};
  std::shared_ptr<Handle> handle_;
  std::size_t sent_points_{};
  uint64_t feedback_sequence_{};
  bool cancel_accepted_{}, stopped_{};
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp_action::Server<Action>::SharedPtr server_;
};
}  // namespace je2_experiment