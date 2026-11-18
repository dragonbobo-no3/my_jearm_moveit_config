#pragma once

#include "je2_ros_goal_adapter.hpp"
#include <optional>

namespace je2_experiment
{
// Experimental, SDK-free goal lifecycle. Owns NO sockets, threads or ROS action
// server. The future serialized executor drives observations/ticks independently
// and immediately sends any returned frame (send_failed on failure, no retry).
// Results concern this explicit model contract, NOT certified physical safety,
// native stop provenance or production permission to start another Servo.
class RosGoalExecution
{
public:
  using Ticket = RosGoalAdapter::Ticket;
  using Goal = RosGoalAdapter::Goal;
  struct Policy
  {
    RosGoalAdapter::Axes position_tolerance{}; // explicit controller policy, not goal data
    double max_lateness{};
    double max_result_age{}; // seconds; positive, <= client 150ms observation limit
  };
  enum class State { Idle, Streaming, Closing, Succeeded, Canceled, Aborted };
  enum class Reason { None, GoalTolerance, ControlLost, ControlFault, ScheduleExpired, SendFailed, InvalidClock, StaleEvidence };
  enum class Output { Waiting, Target, Release, Terminal, Stale };
  struct Step { Output output; int size; };
  struct Result { State state; Reason reason; };

  RosGoalExecution(RosGoalAdapter & adapter, Policy policy)
  : adapter_(adapter), policy_(std::move(policy)) {}
  RosGoalExecution(const RosGoalExecution &) = delete;
  RosGoalExecution & operator=(const RosGoalExecution &) = delete;

  Ticket start(const Goal & goal, double now)
  {
    // No preemption, and don't overwrite an undelivered terminal result.
    if (ticket_ && (!terminal() || !result_taken_)) return {};
    if (!valid_policy() || !terminal_hold(goal) || !adapter_.close_supported()) return {};
    auto ticket = adapter_.bind_scheduled(goal, now, policy_.max_lateness);
    if (!ticket) return {};
    ticket_ = ticket;
    for (std::size_t i = 0; i < MK2_AXES; ++i) final_position_[i] = goal.trajectory.points.back().positions[i];
    state_ = State::Streaming;
    reason_ = Reason::None;
    cancel_requested_ = result_taken_ = false;
    last_now_ = now;
    return ticket_;
  }

  Step advance(const Ticket & ticket, double now, char * out, std::size_t size)
  {
    clear(out, size);
    if (!ticket_ || ticket != ticket_) return {Output::Stale, 0};
    if (terminal()) return {Output::Terminal, 0};
    if (!live_clock(now)) return {Output::Terminal, 0};
    if (state_ == State::Streaming) {
      const auto result = adapter_.dispatch(ticket_, now, out, size);
      switch (result.state) {
        case RosGoalAdapter::DispatchState::FrameReady: return {Output::Target, result.size};
        case RosGoalAdapter::DispatchState::WaitingDue:
        case RosGoalAdapter::DispatchState::WaitingInterval:
        case RosGoalAdapter::DispatchState::WaitingAck: return {Output::Waiting, 0};
        case RosGoalAdapter::DispatchState::Expired:
          finish(State::Aborted, Reason::ScheduleExpired); return {Output::Terminal, 0};
        case RosGoalAdapter::DispatchState::Rejected:
          control_failure(); return {Output::Terminal, 0};
        case RosGoalAdapter::DispatchState::Drained:
          state_ = State::Closing; break; // close explicitly, never infer success here
      }
    }
    return close_step(now, out, size);
  }

  // Accepted only before a terminal close receipt/result; freezes scheduling
  // immediately by requesting close, even when a target still awaits ACK.
  // >=0 accepted (positive is the one Release frame); -1 refused/aborted.
  // Do NOT call adapter.cancel first: that is local retirement, not this path.
  int request_cancel(const Ticket & ticket, double now, char * out, std::size_t size)
  {
    clear(out, size);
    if (!ticket_ || ticket != ticket_ || terminal() ||
        adapter_.close_state(ticket_) == RosGoalAdapter::CloseState::Confirmed) return -1;
    if (!live_clock(now)) return -1;
    cancel_requested_ = true;
    state_ = State::Closing;
    const auto result = close_step(now, out, size);
    return result.output == Output::Terminal ? -1 : result.size;
  }

  bool send_failed(const Ticket & ticket)
  {
    if (!ticket_ || ticket != ticket_ || terminal()) return false;
    if (!adapter_.send_failed(ticket_)) return false;
    finish(State::Aborted, Reason::SendFailed);
    return true;
  }

  State state(const Ticket & ticket) const { return ticket_ && ticket == ticket_ ? state_ : State::Idle; }
  std::optional<Result> take_result(const Ticket & ticket)
  {
    if (!ticket_ || ticket != ticket_ || !terminal() || result_taken_) return std::nullopt;
    result_taken_ = true;
    return Result{state_, reason_}; // exactly once; caller maps to its own action handle
  }

private:
  static void clear(char * out, std::size_t size) { if (out && size) out[0] = '\0'; }
  bool terminal() const
  {
    return state_ == State::Succeeded || state_ == State::Canceled || state_ == State::Aborted;
  }
  void finish(State state, Reason reason) { state_ = state; reason_ = reason; }
  void control_failure()
  {
    finish(State::Aborted, adapter_.session_phase(ticket_) == JE2_CLIENT_RETIRED ? Reason::ControlLost : Reason::ControlFault);
  }
  bool live_clock(double now)
  {
    const auto phase = adapter_.session_phase(ticket_);
    if (phase == JE2_CLIENT_RETIRED || phase == JE2_CLIENT_FAULT) { control_failure(); return false; }
    if (!std::isfinite(now) || now < last_now_) {
      // Identity was checked above. Never tick a NEW session from an old timer.
      (void)adapter_.tick(now);
      finish(State::Aborted, Reason::InvalidClock);
      return false;
    }
    last_now_ = now;
    return true;
  }
  Step close_step(double now, char * out, std::size_t size)
  {
    const auto close = adapter_.close_state(ticket_);
    if (close == RosGoalAdapter::CloseState::Confirmed) {
      RosGoalAdapter::Observation sample;
      if (!adapter_.observation(ticket_, sample) || sample.sequence == 0 ||
          sample.received_at > now || now - sample.received_at > policy_.max_result_age) {
        finish(State::Aborted, Reason::StaleEvidence);
      } else if (cancel_requested_) {
        finish(State::Canceled, Reason::None); // same receipt required, not a local cancel ACK
      } else {
        bool reached = true;
        for (std::size_t i = 0; i < MK2_AXES; ++i)
          reached = reached && std::abs(sample.position[i] - final_position_[i]) <= policy_.position_tolerance[i];
        finish(reached ? State::Succeeded : State::Aborted, reached ? Reason::None : Reason::GoalTolerance);
      }
      return {Output::Terminal, 0};
    }
    if (close == RosGoalAdapter::CloseState::Retired || close == RosGoalAdapter::CloseState::Fault) {
      control_failure(); return {Output::Terminal, 0};
    }
    const int n = adapter_.request_close(ticket_, now, out, size);
    if (n < 0) { control_failure(); return {Output::Terminal, 0}; }
    return {n > 0 ? Output::Release : Output::Waiting, n};
  }
  bool valid_policy() const
  {
    if (!std::isfinite(policy_.max_lateness) || policy_.max_lateness < 0 || policy_.max_lateness > .002 ||
        !std::isfinite(policy_.max_result_age) || policy_.max_result_age <= 0 || policy_.max_result_age > .15) return false;
    for (double tolerance : policy_.position_tolerance)
      if (!std::isfinite(tolerance) || tolerance <= 0) return false;
    return true;
  }
  static bool terminal_hold(const Goal & goal)
  {
    // Require an explicit final hold interval and zero terminal declared v/a.
    // Do not append a hold, clamp velocity or retime a moving endpoint.
    const auto & points = goal.trajectory.points;
    if (points.size() < 2) return false;
    const auto & last = points.back();
    const auto & before = points[points.size() - 2];
    if (last.positions.size() != MK2_AXES || before.positions.size() != MK2_AXES ||
        last.velocities.size() != MK2_AXES || (!last.accelerations.empty() && last.accelerations.size() != MK2_AXES)) return false;
    for (std::size_t i = 0; i < MK2_AXES; ++i) {
      if (!std::isfinite(last.positions[i]) || last.positions[i] != before.positions[i] || last.velocities[i] != 0 ||
          (!last.accelerations.empty() && last.accelerations[i] != 0)) return false;
    }
    return true;
  }

  RosGoalAdapter & adapter_;
  const Policy policy_;
  Ticket ticket_;
  RosGoalAdapter::Axes final_position_{};
  State state_{State::Idle};
  Reason reason_{Reason::None};
  double last_now_{};
  bool cancel_requested_{}, result_taken_{};
};
}  // namespace je2_experiment