#pragma once

#include <array>
#include <cmath>
#include <cstring>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include "je2_session_client.h"

namespace je2_experiment
{
// Build/test-only ROS message -> session frame boundary. NOT an action server,
// JTC replacement, interpolator or certified trajectory validator. No sockets,
// plugin registration, auto claim, hardware activation or install rule.
// All methods and access to client MUST be serialized by the future executor.
class RosGoalAdapter
{
public:
  using Goal = control_msgs::action::FollowJointTrajectory::Goal;
  using Names = std::array<std::string, MK2_AXES>;
  using Axes = std::array<double, MK2_AXES>;
  // Required explicit configuration from the future executor, never from a
  // goal. No implicit unrestricted/default limits or silent clamping.
  struct Limits
  {
    Axes lower{}, upper{}, velocity{}, acceleration{};
  };

  class Binding
  {
    friend class RosGoalAdapter;
    je2_client_lease lease_{};
    Goal goal_;
    double origin_{};
    double admitted_at_{}, lateness_{};
    bool scheduled_{};
  };
  // Callbacks retain this immutable object from admission. They must NEVER
  // obtain a new ticket/lease at callback execution time.
  using Ticket = std::shared_ptr<const Binding>;

  enum class DispatchState { WaitingDue, WaitingInterval, WaitingAck, FrameReady, Drained, Expired, Rejected };
  enum class CloseState { None, Pending, Confirmed, Retired, Fault };
  struct DispatchResult
  {
    DispatchState state;
    int size; // >0 ONLY for FrameReady; output is empty for all other states
  };
  struct Observation
  {
    Axes position{};
    uint64_t sequence{};
    double received_at{};
  };

  RosGoalAdapter(je2_client & client, Names names, Limits limits)
  : client_(client), names_(std::move(names)), limits_(std::move(limits)) {}
  RosGoalAdapter(const RosGoalAdapter &) = delete;
  RosGoalAdapter & operator=(const RosGoalAdapter &) = delete;

  Ticket bind(const Goal & goal, double now)
  {
    return bind_impl(goal, now, false, 0);
  }

  // Explicit experimental scheduling budget in seconds, fixed at admission.
  // At most 2ms (< half the minimum 5ms sample interval). Not a certified
  // hardware/network timing budget. Zero means no lateness is permitted.
  Ticket bind_scheduled(const Goal & goal, double now, double max_lateness)
  {
    if (!std::isfinite(max_lateness) || max_lateness < 0 || max_lateness > .002) return {};
    return bind_impl(goal, now, true, max_lateness);
  }

private:
  Ticket bind_impl(const Goal & goal, double now, bool scheduled, double lateness)
  {
    synchronize();
    if (je2_client_tick(&client_, now) != 0) { active_.reset(); return {}; }
    // No preemption/replacement behind the current goal. A later executor must
    // implement bounded cancellation and obtain a NEW explicit session.
    if (active_ || client_.phase != JE2_CLIENT_ACTIVE ||
        client_.wire_sequence != 2 || client_.acknowledged != 2 || !valid(goal) || !valid_limits()) return {};
    const double origin = client_.first_data_time + (now - client_.first_target_at);
    History initial;
    for (std::size_t i = 0; i < MK2_AXES; ++i) initial.position[i] = client_.first_target[i];
    initial.time = client_.first_data_time;
    initial.sequence = client_.wire_sequence;
    if (client_.last_data_time != initial.time || !valid_history(initial)) return {};
    // Validate every point before binding, including entry from the actual
    // first emitted hold. Do not discover a bad final point after partial motion.
    History preview = initial;
    for (const auto & point : goal.trajectory.points) {
      Axes velocity;
      const double time = origin + seconds(point.time_from_start);
      if (!valid_step(point, time, preview, velocity)) return {};
      commit(preview, point, time, velocity);
    }
    auto binding = std::make_shared<Binding>();
    if (je2_client_get_lease(&client_, &binding->lease_) != 0) return {};
    binding->goal_ = goal;  // bounded, immutable copy; caller cannot replace queued points
    binding->origin_ = origin;
    binding->admitted_at_ = now;
    binding->scheduled_ = scheduled;
    binding->lateness_ = lateness;
    active_ = binding;
    closing_.reset();
    history_ = initial;
    next_ = 0;
    last_dispatch_at_ = client_.first_target_at;
    return active_;
  }

public:
  int observe(const char * frame, std::size_t size, double now)
  {
    const int result = je2_client_observe(&client_, frame, size, now);
    synchronize();
    return result;
  }

  int tick(double now)
  {
    const int result = je2_client_tick(&client_, now);
    synchronize();
    return result;
  }

  // Submit exactly one already-sampled point. Does not interpolate or retime.
  // Positive = frame; zero = backpressure, index NOT consumed; negative = reject.
  // In particular this does NOT mark a ROS goal successful at its last point.
  int prepare(const Ticket & ticket, std::size_t index, double now, char * out, std::size_t size)
  {
    if (out && size) out[0] = '\0';
    synchronize();
    if (!active_ || ticket != active_ || active_->scheduled_) return -1;
    return prepare_point(ticket, index, now, out, size);
  }

  // One nonblocking scheduling step; no sleep, loop, resend, hold extension or
  // retiming. Caller must immediately attempt the returned frame send in the
  // same serialized executor and call send_failed on failure (never retry it).
  // Caller still independently drives observe/tick, including after Drained.
  // Drained ONLY means all goal frames consumed with healthy matching ACKs;
  // it is NOT physical motion completion, stopped ownership, or ROS success.
  DispatchResult dispatch(const Ticket & ticket, double now, char * out, std::size_t size)
  {
    if (out && size) out[0] = '\0';
    synchronize();
    if (!active_ || ticket != active_ || !active_->scheduled_) return {DispatchState::Rejected, 0};
    if (tick(now) != 0) return {DispatchState::Rejected, 0};
    if (!history_matches()) {
      reject_current();
      return {DispatchState::Rejected, 0};
    }
    if (next_ == active_->goal_.trajectory.points.size()) {
      return {client_.acknowledged == client_.wire_sequence ? DispatchState::Drained : DispatchState::WaitingAck, 0};
    }
    const double due = active_->admitted_at_ + seconds(active_->goal_.trajectory.points[next_].time_from_start);
    if (now > due + active_->lateness_) {
      reject_current(); // schedule deadline failure, NOT normal handover
      return {DispatchState::Expired, 0};
    }
    if (now < due) return {DispatchState::WaitingDue, 0};
    // Do not compress actual frame spacing after a late send. If waiting for
    // this spacing makes the next deadline expire, fail instead of retiming.
    if (now - last_dispatch_at_ < .005 - 1e-9) return {DispatchState::WaitingInterval, 0};
    const int result = prepare_point(ticket, next_, now, out, size);
    if (result > 0) {
      last_dispatch_at_ = now;
      return {DispatchState::FrameReady, result};
    }
    return {result == 0 ? DispatchState::WaitingAck : DispatchState::Rejected, 0};
  }

private:
  int prepare_point(const Ticket & ticket, std::size_t index, double now, char * out, std::size_t size)
  {
    if (out && size) out[0] = '\0';
    synchronize();
    if (!active_ || ticket != active_) return -1; // before clock or payload access
    if (index != next_ || index >= active_->goal_.trajectory.points.size()) return -1;
    const auto & point = active_->goal_.trajectory.points[index];
    const double time = active_->origin_ + seconds(point.time_from_start);
    // Another writer or untracked command must not silently change the finite
    // difference baseline. Feedback updates are NOT command history updates.
    Axes velocity;
    if (!history_matches() || !valid_step(point, time, history_, velocity)) {
      reject_current();
      return -1;
    }
    const int result = je2_client_target(&client_, &active_->lease_, now, time,
      point.positions.data(), out, size);
    if (result > 0) {
      commit(history_, point, time, velocity);
      history_.sequence = client_.wire_sequence;
      ++next_;
    }
    synchronize();
    return result;
  }

public:
  // Explicit session close for either end-of-stream or cancellation. This is
  // NOT a ROS result. Immediately fences further goal output, but does not
  // skip an outstanding frame. Poll with the SAME ticket; Release is formatted
  // at most once and must be sent immediately, with send_failed on failure.
  int request_close(const Ticket & ticket, double now, char * out, std::size_t size)
  {
    if (out && size) out[0] = '\0';
    synchronize();
    if (active_ && ticket == active_) {
      if (!history_matches()) { reject_current(); return -1; }
      closing_ = active_;
      active_.reset();
    }
    if (!closing_ || ticket != closing_) return -1;
    return je2_client_close(&client_, &closing_->lease_, now, out, size);
  }

  // Confirmed = historical engine barrier receipt for this release only;
  // no claim about goal tolerance, future ownership or current physical stop.
  CloseState close_state(const Ticket & ticket) const
  {
    if (!closing_ || ticket != closing_) return CloseState::None;
    const auto & lease = closing_->lease_;
    if (lease.generation != client_.generation ||
        std::memcmp(lease.boot, client_.boot, sizeof(lease.boot)) ||
        std::memcmp(lease.client, client_.client, sizeof(lease.client))) return CloseState::Retired;
    if (client_.phase == JE2_CLIENT_CLOSED) return CloseState::Confirmed;
    if (client_.phase == JE2_CLIENT_FAULT) return CloseState::Fault;
    if (client_.phase == JE2_CLIENT_DRAINING || client_.phase == JE2_CLIENT_CLOSING) return CloseState::Pending;
    return CloseState::Retired;
  }

  // Read-only views for the serialized goal lifecycle. A stale ticket must not
  // borrow either health or feedback from a newer client/session.
  je2_client_phase session_phase(const Ticket & ticket) const
  {
    if (!ticket) return JE2_CLIENT_RETIRED;
    const auto & lease = ticket->lease_;
    if (lease.generation != client_.generation ||
        std::memcmp(lease.boot, client_.boot, sizeof(lease.boot)) ||
        std::memcmp(lease.client, client_.client, sizeof(lease.client))) return JE2_CLIENT_RETIRED;
    return client_.phase;
  }
  bool close_supported() const { return client_.release_supported; }
  bool observation(const Ticket & ticket, Observation & out) const
  {
    out = {};
    if (!ticket || (ticket != active_ && ticket != closing_) || !client_.seen) return false;
    const auto phase = session_phase(ticket);
    if (phase != JE2_CLIENT_ACTIVE && phase != JE2_CLIENT_CLOSED) return false;
    if (!std::isfinite(client_.received_at)) return false;
    for (std::size_t i = 0; i < MK2_AXES; ++i) {
      if (!std::isfinite(client_.joint[i])) return false;
      out.position[i] = client_.joint[i];
    }
    out.sequence = client_.observation;
    out.received_at = client_.received_at;
    return true;
  }

  // Invalidate local output only; NOT a physical stop or completed ROS cancel.
  // A stale goal's cancellation must never retire a newer session.
  bool cancel(const Ticket & ticket)
  {
    synchronize();
    if (!active_ || ticket != active_) return false;
    je2_client_retire(&client_);
    active_.reset();
    return true;
  }

  bool send_failed(const Ticket & ticket)
  {
    synchronize();
    if (close_state(ticket) == CloseState::Pending) {
      je2_client_send_failed(&client_);
      return true;
    }
    if (!active_ || ticket != active_) return false;
    je2_client_send_failed(&client_);
    active_.reset();
    return true;
  }

  bool has_goal() { synchronize(); return static_cast<bool>(active_); }

private:
  bool history_matches() const
  {
    return client_.wire_sequence == history_.sequence && client_.last_data_time == history_.time;
  }
  void reject_current()
  {
    client_.phase = JE2_CLIENT_FAULT; // local validation failure, no SDK/stop frame
    active_.reset();
  }
  struct History
  {
    Axes position{}, velocity{};
    double time{};
    uint64_t sequence{};
  };
  bool valid_limits() const
  {
    for (std::size_t i = 0; i < MK2_AXES; ++i) {
      if (!std::isfinite(limits_.lower[i]) || !std::isfinite(limits_.upper[i]) ||
          limits_.lower[i] >= limits_.upper[i] ||
          !std::isfinite(limits_.velocity[i]) || limits_.velocity[i] <= 0 ||
          !std::isfinite(limits_.acceleration[i]) || limits_.acceleration[i] <= 0) return false;
    }
    return true;
  }
  bool valid_history(const History & history) const
  {
    if (!std::isfinite(history.time) || history.time <= 0) return false;
    for (std::size_t i = 0; i < MK2_AXES; ++i) {
      if (!std::isfinite(history.position[i]) || history.position[i] < limits_.lower[i] ||
          history.position[i] > limits_.upper[i] || !std::isfinite(history.velocity[i])) return false;
    }
    return true;
  }
  bool valid_step(const trajectory_msgs::msg::JointTrajectoryPoint & point, double time,
    const History & previous, Axes & velocity) const
  {
    const double dt = time - previous.time;
    if (!std::isfinite(time) || !std::isfinite(dt) || dt < .005 - 1e-9 || dt > .05 + 1e-9) return false;
    for (std::size_t i = 0; i < MK2_AXES; ++i) {
      const double q = point.positions[i];
      velocity[i] = (q - previous.position[i]) / dt;
      const double acceleration = (velocity[i] - previous.velocity[i]) / dt;
      if (!std::isfinite(q) || q < limits_.lower[i] || q > limits_.upper[i] ||
          !std::isfinite(velocity[i]) || std::abs(velocity[i]) > limits_.velocity[i] ||
          !std::isfinite(acceleration) || std::abs(acceleration) > limits_.acceleration[i] ||
          std::abs(point.velocities[i]) > limits_.velocity[i] ||
          (!point.accelerations.empty() && std::abs(point.accelerations[i]) > limits_.acceleration[i])) return false;
    }
    return true;
  }
  static void commit(History & history, const trajectory_msgs::msg::JointTrajectoryPoint & point,
    double time, const Axes & velocity)
  {
    for (std::size_t i = 0; i < MK2_AXES; ++i) history.position[i] = point.positions[i];
    history.velocity = velocity;
    history.time = time;
  }
  static double seconds(const builtin_interfaces::msg::Duration & duration)
  {
    return static_cast<double>(duration.sec) + duration.nanosec * 1e-9;
  }
  bool valid(const Goal & goal) const
  {
    const auto & trajectory = goal.trajectory;
    // Deliberately narrow fixture contract, not general FollowJointTrajectory.
    if (trajectory.header.stamp.sec != 0 || trajectory.header.stamp.nanosec != 0 ||
        !trajectory.header.frame_id.empty() || trajectory.joint_names.size() != MK2_AXES ||
        trajectory.points.empty() || trajectory.points.size() > 512 ||
        goal.multi_dof_trajectory.header.stamp.sec != 0 || goal.multi_dof_trajectory.header.stamp.nanosec != 0 ||
        !goal.multi_dof_trajectory.header.frame_id.empty() ||
        !goal.multi_dof_trajectory.joint_names.empty() || !goal.multi_dof_trajectory.points.empty() ||
        !goal.path_tolerance.empty() || !goal.goal_tolerance.empty() ||
        !goal.component_path_tolerance.empty() || !goal.component_goal_tolerance.empty() ||
        goal.goal_time_tolerance.sec != 0 || goal.goal_time_tolerance.nanosec != 0) return false;
    for (std::size_t i = 0; i < names_.size(); ++i) {
      if (names_[i].empty() || trajectory.joint_names[i] != names_[i]) return false;
      for (std::size_t j = 0; j < i; ++j) if (names_[i] == names_[j]) return false;
    }
    double previous = -1;
    for (const auto & point : trajectory.points) {
      if (point.time_from_start.sec < 0 || point.time_from_start.nanosec >= 1000000000u ||
          point.positions.size() != MK2_AXES || point.velocities.size() != MK2_AXES ||
          (!point.accelerations.empty() && point.accelerations.size() != MK2_AXES) ||
          !point.effort.empty()) return false;
      for (double q : point.positions) if (!std::isfinite(q)) return false;
      for (double v : point.velocities) if (!std::isfinite(v)) return false;
      for (double a : point.accelerations) if (!std::isfinite(a)) return false;
      const double time = seconds(point.time_from_start);
      if (previous >= 0 && (time - previous < .005 - 1e-9 || time - previous > .05 + 1e-9)) return false;
      if (previous < 0 && time > .05) return false;
      previous = time;
    }
    return true;
  }
  void synchronize()
  {
    if (!active_) return;
    const auto & lease = active_->lease_;
    if (client_.phase != JE2_CLIENT_ACTIVE || lease.generation != client_.generation ||
        std::memcmp(lease.boot, client_.boot, sizeof(lease.boot)) ||
        std::memcmp(lease.client, client_.client, sizeof(lease.client))) active_.reset();
  }

  je2_client & client_;
  Names names_;
  Limits limits_;
  History history_;
  Ticket active_;
  Ticket closing_;
  std::size_t next_{};
  double last_dispatch_at_{};
};
}  // namespace je2_experiment