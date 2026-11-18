// Actual ROS Goal -> client -> inproc ZMQ -> session parser/core -> memory IO
// -> shared Owner serializer -> inproc ZMQ -> client. No constructed ACTIVE,
// manual ACK, SDK/vendor header, PluginMain, ROS node, TCP or physical endpoint.
#include "je2_loopback_server.h"
#include "je2_ros_goal_adapter.hpp"
#include "je2_ros_goal_execution.hpp"
#include <gtest/gtest.h>
#include <zmq.h>
#include <limits>

namespace
{
using Adapter = je2_experiment::RosGoalAdapter;
using Dispatch = Adapter::DispatchState;
using Close = Adapter::CloseState;
using Execution = je2_experiment::RosGoalExecution;
Execution::Policy execution_policy()
{
  Execution::Policy policy;
  policy.position_tolerance.fill(.0001);
  policy.max_lateness = .002;
  policy.max_result_age = .05;
  return policy; // fixture contract, NOT physical commissioning values
}
const Adapter::Names names{"j1", "j2", "j3", "j4", "j5", "j6", "j7"};
Adapter::Limits limits()
{
  Adapter::Limits result;
  result.lower.fill(-1); result.upper.fill(1);
  result.velocity.fill(.2); result.acceleration.fill(.4);
  return result; // fixture profile only, NOT physical limits
}

class SessionLoopback : public ::testing::Test
{
protected:
  je2_loopback_server server{};
  je2_client client{};
  std::unique_ptr<Adapter> adapter;
  void * context{};
  void * command_tx{}, * command_rx{}, * owner_tx{}, * owner_rx{};

  void SetUp() override
  {
    ASSERT_EQ(je2_loopback_init(&server), 0);
    ASSERT_EQ(je2_client_init(&client), 0);
    adapter = std::make_unique<Adapter>(client, names, limits());
    context = zmq_ctx_new(); ASSERT_NE(context, nullptr);
    ASSERT_EQ(zmq_ctx_set(context, ZMQ_IO_THREADS, 0), 0); // inproc ONLY
    for (auto ** socket : {&command_tx, &command_rx, &owner_tx, &owner_rx}) {
      *socket = zmq_socket(context, ZMQ_PAIR); ASSERT_NE(*socket, nullptr);
      const int linger = 0, timeout = 200;
      ASSERT_EQ(zmq_setsockopt(*socket, ZMQ_LINGER, &linger, sizeof(linger)), 0);
      ASSERT_EQ(zmq_setsockopt(*socket, ZMQ_SNDTIMEO, &timeout, sizeof(timeout)), 0);
      ASSERT_EQ(zmq_setsockopt(*socket, ZMQ_RCVTIMEO, &timeout, sizeof(timeout)), 0);
    }
    ASSERT_EQ(zmq_bind(command_rx, "inproc://je2-command"), 0);
    ASSERT_EQ(zmq_connect(command_tx, "inproc://je2-command"), 0);
    ASSERT_EQ(zmq_bind(owner_rx, "inproc://je2-owner"), 0);
    ASSERT_EQ(zmq_connect(owner_tx, "inproc://je2-owner"), 0);
  }
  void TearDown() override
  {
    for (auto * socket : {command_tx, command_rx, owner_tx, owner_rx})
      if (socket) EXPECT_EQ(zmq_close(socket), 0);
    if (context) EXPECT_EQ(zmq_ctx_term(context), 0);
  }
  void queue(const std::string & wire, bool more = false)
  {
    ASSERT_FALSE(wire.empty());
    ASSERT_EQ(zmq_send(command_tx, wire.data(), wire.size(), more ? ZMQ_SNDMORE : 0),
      static_cast<int>(wire.size()));
  }
  int serve(double now)
  {
    char wire[MK2_FRAME_MAX];
    const int n = zmq_recv(command_rx, wire, sizeof(wire), 0);
    EXPECT_GT(n, 0); EXPECT_LE(n, MK2_FRAME_MAX);
    if (n <= 0 || n > MK2_FRAME_MAX) return -99;
    int more = 0; size_t size = sizeof(more);
    EXPECT_EQ(zmq_getsockopt(command_rx, ZMQ_RCVMORE, &more, &size), 0);
    return je2_loopback_receive(&server, wire, static_cast<size_t>(n), more != 0, now);
  }
  int deliver(const std::string & wire, double now) { queue(wire); return serve(now); }
  int observe(double now)
  {
    char wire[MK2_FRAME_MAX];
    const int n = je2_loopback_snapshot(&server, now, wire, sizeof(wire));
    EXPECT_GT(n, 0);
    if (n <= 0) return -99;
    EXPECT_EQ(zmq_send(owner_tx, wire, static_cast<size_t>(n), 0), n);
    const int received = zmq_recv(owner_rx, wire, sizeof(wire), 0);
    EXPECT_EQ(received, n);
    if (received != n) return -99;
    return adapter->observe(wire, static_cast<size_t>(received), now);
  }
  Adapter::Goal goal(std::size_t count = 3) const
  {
    Adapter::Goal result;
    result.trajectory.joint_names.assign(names.begin(), names.end());
    for (std::size_t i = 0; i < count; ++i) {
      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions.assign(client.first_target, client.first_target + MK2_AXES);
      for (auto & q : point.positions) q += .000005 * (i + 1) * (i + 2);
      point.velocities.assign(MK2_AXES, .001 * (i + 1));
      point.accelerations.assign(MK2_AXES, .1);
      point.time_from_start.nanosec = static_cast<uint32_t>(i * 10000000);
      result.trajectory.points.push_back(point);
    }
    return result;
  }
  Adapter::Goal execution_goal() const
  {
    auto result = goal(4);
    for (std::size_t i = 0; i < 4; ++i) {
      auto & point = result.trajectory.points[i];
      for (std::size_t axis = 0; axis < MK2_AXES; ++axis)
        point.positions[axis] = client.first_target[axis] + .00001 * (i == 3 ? 3 : i + 1);
      point.velocities.assign(MK2_AXES, i == 3 ? 0 : .001);
      point.accelerations.assign(MK2_AXES, 0);
    }
    return result; // caller supplies terminal hold; lifecycle never appends one
  }
  void stream_and_request_close(Execution & execution, const Adapter::Ticket & ticket,
    const Adapter::Goal & request, double & released_at, double start = 1.16)
  {
    char wire[MK2_FRAME_MAX];
    for (const auto & point : request.trajectory.points) {
      const double now = start + point.time_from_start.sec + point.time_from_start.nanosec * 1e-9;
      const auto result = execution.advance(ticket, now, wire, sizeof(wire));
      ASSERT_EQ(result.output, Execution::Output::Target); ASSERT_GT(result.size, 0);
      ASSERT_EQ(deliver(std::string(wire, result.size), now), 0);
      ASSERT_EQ(observe(now + .001), 0);
      ASSERT_FALSE(execution.take_result(ticket)); // final ACK still isn't success
      released_at = now + .002;
    }
    const auto result = execution.advance(ticket, released_at, wire, sizeof(wire));
    ASSERT_EQ(result.output, Execution::Output::Release); ASSERT_GT(result.size, 0);
    ASSERT_EQ(execution.state(ticket), Execution::State::Closing);
    ASSERT_EQ(deliver(std::string(wire, result.size), released_at), 0);
    ASSERT_EQ(observe(released_at + .001), 0);
  }
  void handshake(double base = 1.0)
  {
    ASSERT_EQ(observe(base), 0);
    ASSERT_EQ(client.phase, JE2_CLIENT_OBSERVING);
    ASSERT_FALSE(adapter->bind(goal(), base));
    char wire[MK2_FRAME_MAX];
    // Real repeated claim qualification, no assignment to phase/generation/ACK.
    for (int i = 0; i < 5; ++i) {
      const double now = base + .01 + .03 * i;
      ASSERT_GT(je2_client_claim(&client, now, wire, sizeof(wire)), 0);
      ASSERT_EQ(deliver(wire, now), i == 4 ? 0 : -1);
      ASSERT_EQ(observe(now + .001), 0);
      ASSERT_EQ(client.phase, i == 4 ? JE2_CLIENT_BOUND : JE2_CLIENT_CLAIMING);
      ASSERT_EQ(server.starts, 0u);
    }
    ASSERT_EQ(client.generation, 2u);
    // Changed pose must come from the post-Switch snapshot, not the old claim.
    server.sample.joint[0] = .25;
    ASSERT_GT(je2_client_switch(&client, base + .14, wire, sizeof(wire)), 0);
    ASSERT_EQ(deliver(wire, base + .14), 0);
    ASSERT_EQ(observe(base + .141), 0);
    ASSERT_EQ(client.phase, JE2_CLIENT_HOLD_READY);
    ASSERT_FALSE(adapter->bind(goal(), base + .142));
    ASSERT_EQ(server.starts, 0u);
    ASSERT_GT(je2_client_hold(&client, base + .15, .01, wire, sizeof(wire)), 0);
    ASSERT_EQ(deliver(wire, base + .15), 0);
    ASSERT_EQ(observe(base + .151), 0);
    ASSERT_EQ(client.phase, JE2_CLIENT_ACTIVE);
    ASSERT_EQ(client.acknowledged, 2u);
    ASSERT_DOUBLE_EQ(client.first_target[0], .25);
    ASSERT_DOUBLE_EQ(server.target[0], .25);
    ASSERT_EQ(server.starts, 1u); ASSERT_EQ(server.joints, 1u);
  }
  void finish_close(const Adapter::Ticket & ticket, double released_at, bool deliver_final = true)
  {
    je2_loopback_tick(&server, released_at + .01);
    ASSERT_EQ(observe(released_at + .011), 0);
    je2_loopback_tick(&server, released_at + .12);
    ASSERT_EQ(server.ends, 1u);
    ASSERT_EQ(server.engine.owner, JE2_EXIT_BARRIER);
    ASSERT_EQ(observe(released_at + .121), 0);
    ASSERT_EQ(adapter->close_state(ticket), Close::Pending); // End is NOT exit
    server.writer = JE2_WRITER_EXITED; // explicit fake instance evidence only
    je2_loopback_tick(&server, released_at + .13);
    ASSERT_EQ(observe(released_at + .131), 0);
    ASSERT_EQ(adapter->close_state(ticket), Close::Pending); // no pre-exit reuse
    je2_loopback_tick(&server, released_at + .14);
    ASSERT_EQ(observe(released_at + .141), 0);
    ASSERT_EQ(adapter->close_state(ticket), Close::Pending);
    je2_loopback_tick(&server, released_at + .25);
    if (deliver_final) {
      ASSERT_EQ(observe(released_at + .251), 0);
      ASSERT_EQ(adapter->close_state(ticket), Close::Confirmed);
      ASSERT_EQ(client.phase, JE2_CLIENT_CLOSED);
    }
    ASSERT_EQ(server.engine.owner, JE2_PENDANT);
    ASSERT_EQ(server.stops, 0u); ASSERT_EQ(server.jogs, 0u);
  }
};

TEST_F(SessionLoopback, FullHandshakeThenTwentyFiveUnmodifiedRosTargets)
{
  ASSERT_NO_FATAL_FAILURE(handshake());
  const auto request = goal(25);
  const auto ticket = adapter->bind(request, 1.16); ASSERT_TRUE(ticket);
  char wire[MK2_FRAME_MAX];
  for (std::size_t i = 0; i < request.trajectory.points.size(); ++i) {
    const double now = 1.16 + .01 * i;
    ASSERT_GT(adapter->prepare(ticket, i, now, wire, sizeof(wire)), 0);
    ASSERT_EQ(deliver(wire, now), 0);
    ASSERT_EQ(observe(now + .001), 0);
    ASSERT_EQ(client.acknowledged, i + 3);
    for (std::size_t axis = 0; axis < MK2_AXES; ++axis)
      EXPECT_DOUBLE_EQ(server.target[axis], request.trajectory.points[i].positions[axis]);
  }
  EXPECT_EQ(server.starts, 1u); EXPECT_EQ(server.joints, 26u);
  EXPECT_EQ(server.stops, 0u); EXPECT_EQ(server.ends, 0u);
  EXPECT_EQ(server.engine.core.fault, MK2_NO_FAULT);
  EXPECT_TRUE(adapter->has_goal()); // last ACK is NOT ROS goal completion
}

TEST_F(SessionLoopback, WithheldOwnerAckDoesNotAdvancePointOrHistory)
{
  ASSERT_NO_FATAL_FAILURE(handshake());
  const auto request = goal();
  const auto ticket = adapter->bind(request, 1.16); ASSERT_TRUE(ticket);
  char wire[MK2_FRAME_MAX];
  ASSERT_GT(adapter->prepare(ticket, 0, 1.16, wire, sizeof(wire)), 0);
  ASSERT_EQ(deliver(wire, 1.16), 0);
  ASSERT_EQ(server.session.last_sequence, 3u); ASSERT_EQ(client.acknowledged, 2u);
  EXPECT_EQ(adapter->prepare(ticket, 1, 1.17, wire, sizeof(wire)), 0);
  EXPECT_EQ(wire[0], '\0'); EXPECT_EQ(client.wire_sequence, 3u);
  EXPECT_EQ(server.joints, 2u);
  ASSERT_EQ(observe(1.171), 0);
  ASSERT_GT(adapter->prepare(ticket, 1, 1.172, wire, sizeof(wire)), 0);
  ASSERT_EQ(deliver(wire, 1.172), 0);
  EXPECT_DOUBLE_EQ(server.target[0], request.trajectory.points[1].positions[0]);
  EXPECT_EQ(server.joints, 3u);
}

TEST_F(SessionLoopback, NativeIntentRetiresClientAndFencesAlreadySerializedTarget)
{
  ASSERT_NO_FATAL_FAILURE(handshake());
  const auto ticket = adapter->bind(goal(), 1.16); ASSERT_TRUE(ticket);
  char wire[MK2_FRAME_MAX];
  ASSERT_GT(adapter->prepare(ticket, 0, 1.16, wire, sizeof(wire)), 0);
  const std::string late = wire;
  ASSERT_EQ(je2_loopback_button(&server, "tv49!", 5, 1.161), 0);
  ASSERT_EQ(server.engine.owner, JE2_YIELDING);
  ASSERT_EQ(observe(1.162), -1);
  ASSERT_EQ(client.phase, JE2_CLIENT_RETIRED);
  EXPECT_FALSE(adapter->has_goal());
  EXPECT_EQ(adapter->prepare(ticket, 1, 1.17, wire, sizeof(wire)), -1);
  EXPECT_EQ(wire[0], '\0');
  EXPECT_FALSE(adapter->cancel(ticket)); EXPECT_FALSE(adapter->send_failed(ticket));
  const unsigned reads = server.reads;
  EXPECT_EQ(deliver(late, 1.17), -1);
  EXPECT_EQ(server.reads, reads); EXPECT_EQ(server.joints, 1u);
  EXPECT_EQ(server.releases, 1u); EXPECT_EQ(server.stops, 0u);
  // End returning and idle state alone must NOT grant the pending jog.
  je2_loopback_tick(&server, 1.18); je2_loopback_tick(&server, 1.29);
  ASSERT_EQ(server.ends, 1u); ASSERT_EQ(server.jogs, 0u);
  ASSERT_EQ(server.engine.owner, JE2_EXIT_BARRIER);
  server.writer = JE2_WRITER_EXITED; // explicit fake writer evidence, not inferred
  je2_loopback_tick(&server, 1.30); je2_loopback_tick(&server, 1.31);
  EXPECT_EQ(server.jogs, 0u);
  je2_loopback_tick(&server, 1.42);
  ASSERT_EQ(server.jogs, 1u); ASSERT_EQ(server.engine.owner, JE2_PENDANT);
  const unsigned after = server.reads;
  EXPECT_EQ(deliver(late, 1.43), -1);
  EXPECT_EQ(server.reads, after); EXPECT_EQ(server.stops, 0u);
  EXPECT_EQ(observe(1.431), -1); EXPECT_EQ(client.phase, JE2_CLIENT_RETIRED);
}

TEST_F(SessionLoopback, LostTargetTimesOutEvenWithHealthyRealOwnerPublications)
{
  ASSERT_NO_FATAL_FAILURE(handshake());
  const auto ticket = adapter->bind(goal(), 1.16); ASSERT_TRUE(ticket);
  char wire[MK2_FRAME_MAX];
  ASSERT_GT(adapter->prepare(ticket, 0, 1.16, wire, sizeof(wire)), 0);
  // Deliberately drop this formatted frame; never resend it.
  for (double now : {1.19, 1.22, 1.25}) {
    je2_loopback_tick(&server, now);
    ASSERT_EQ(observe(now), 0);
    ASSERT_EQ(client.phase, JE2_CLIENT_ACTIVE);
    ASSERT_EQ(client.acknowledged, 2u);
  }
  je2_loopback_tick(&server, 1.261);
  ASSERT_EQ(server.engine.core.fault, MK2_NO_FAULT);
  ASSERT_EQ(observe(1.261), -1);
  EXPECT_EQ(client.phase, JE2_CLIENT_FAULT); EXPECT_FALSE(adapter->has_goal());
  EXPECT_EQ(adapter->prepare(ticket, 1, 1.262, wire, sizeof(wire)), -1);
  EXPECT_EQ(wire[0], '\0'); EXPECT_EQ(server.joints, 1u);
  EXPECT_EQ(observe(1.28), -1); // healthy late publications never reset the latch
}

TEST_F(SessionLoopback, LateOwnerAckCannotReviveExpiredTarget)
{
  ASSERT_NO_FATAL_FAILURE(handshake());
  const auto ticket = adapter->bind(goal(), 1.16); ASSERT_TRUE(ticket);
  char wire[MK2_FRAME_MAX];
  ASSERT_GT(adapter->prepare(ticket, 0, 1.16, wire, sizeof(wire)), 0);
  ASSERT_EQ(deliver(wire, 1.16), 0);
  ASSERT_EQ(server.session.last_sequence, 3u);
  ASSERT_EQ(server.engine.core.accepted_joints, 2u);
  ASSERT_EQ(client.acknowledged, 2u);
  // Withhold all subsequent Owner publications until after the fixed 100ms
  // target deadline, but before the 150ms observation/200ms server watchdogs.
  je2_loopback_tick(&server, 1.261);
  ASSERT_EQ(server.engine.core.fault, MK2_NO_FAULT);
  ASSERT_EQ(observe(1.261), -1); // includes the real successful consumption ACK
  EXPECT_EQ(client.phase, JE2_CLIENT_FAULT);
  EXPECT_EQ(client.acknowledged, 2u);
  EXPECT_FALSE(adapter->has_goal());
  EXPECT_EQ(adapter->prepare(ticket, 1, 1.262, wire, sizeof(wire)), -1);
  EXPECT_EQ(wire[0], '\0');
  EXPECT_EQ(observe(1.28), -1);
  EXPECT_EQ(server.joints, 2u);
  EXPECT_EQ(server.stops, 0u); EXPECT_EQ(server.ends, 0u);
}

TEST_F(SessionLoopback, ClientMotionCheckDoesNotReplaceServerFollowingGuard)
{
  ASSERT_NO_FATAL_FAILURE(handshake());
  const auto ticket = adapter->bind(goal(), 1.16); ASSERT_TRUE(ticket);
  char wire[MK2_FRAME_MAX];
  ASSERT_GT(adapter->prepare(ticket, 0, 1.16, wire, sizeof(wire)), 0);
  // Inject fake feedback disagreement AFTER local command-history validation.
  // Local bounds/derivatives cannot certify physical tracking or bypass core.
  server.sample.joint[0] = .30;
  ASSERT_EQ(deliver(wire, 1.16), -1);
  ASSERT_EQ(server.engine.core.fault, MK2_FOLLOWING);
  ASSERT_EQ(server.session.last_sequence, 3u);
  EXPECT_EQ(server.joints, 1u); // refused before another motion callback
  EXPECT_EQ(server.engine.core.accepted_joints, 1u);
  EXPECT_EQ(observe(1.161), -1);
  EXPECT_EQ(client.phase, JE2_CLIENT_FAULT);
  EXPECT_EQ(client.acknowledged, 2u);
  EXPECT_FALSE(adapter->has_goal());
  EXPECT_EQ(adapter->prepare(ticket, 1, 1.17, wire, sizeof(wire)), -1);
  EXPECT_EQ(wire[0], '\0');
  EXPECT_EQ(server.stops, 1u); EXPECT_EQ(server.ends, 1u);
}

TEST_F(SessionLoopback, ConsumedSequenceWithFailedCallbackIsNotSuccess)
{
  ASSERT_NO_FATAL_FAILURE(handshake());
  const auto ticket = adapter->bind(goal(), 1.16); ASSERT_TRUE(ticket);
  char wire[MK2_FRAME_MAX];
  ASSERT_GT(adapter->prepare(ticket, 0, 1.16, wire, sizeof(wire)), 0);
  server.joint_result = -42;
  ASSERT_EQ(deliver(wire, 1.16), -1);
  ASSERT_EQ(server.session.last_sequence, 3u); // consumed, NOT executed
  ASSERT_EQ(server.engine.core.accepted_joints, 1u);
  ASSERT_EQ(server.engine.core.fault, MK2_SDK_ERROR);
  ASSERT_EQ(observe(1.161), -1);
  EXPECT_EQ(client.phase, JE2_CLIENT_FAULT); EXPECT_EQ(client.acknowledged, 2u);
  EXPECT_EQ(adapter->prepare(ticket, 1, 1.17, wire, sizeof(wire)), -1);
  EXPECT_EQ(wire[0], '\0'); EXPECT_EQ(server.stops, 1u); EXPECT_EQ(server.ends, 1u);
}

TEST_F(SessionLoopback, StateSevenIsFaultNotNormalHandoverOrRecovery)
{
  ASSERT_NO_FATAL_FAILURE(handshake());
  const auto ticket = adapter->bind(goal(), 1.16); ASSERT_TRUE(ticket);
  server.sample.move_state = 7;
  je2_loopback_tick(&server, 1.161);
  ASSERT_EQ(server.engine.core.fault, MK2_STATE_ERROR);
  ASSERT_EQ(observe(1.162), -1);
  EXPECT_EQ(client.phase, JE2_CLIENT_FAULT);
  server.sample.move_state = 0;
  je2_loopback_tick(&server, 1.17);
  EXPECT_EQ(observe(1.171), -1);
  EXPECT_FALSE(adapter->bind(goal(), 1.18));
  EXPECT_EQ(client.phase, JE2_CLIENT_FAULT); EXPECT_EQ(server.joints, 1u);
  EXPECT_EQ(server.stops, 1u); EXPECT_EQ(server.ends, 1u); EXPECT_EQ(server.jogs, 0u);
}

TEST_F(SessionLoopback, NewBootAndExplicitNewClientNeverRelabelOldGoal)
{
  ASSERT_NO_FATAL_FAILURE(handshake());
  const auto old = adapter->bind(goal(), 1.16); ASSERT_TRUE(old);
  char wire[MK2_FRAME_MAX];
  ASSERT_GT(adapter->prepare(old, 0, 1.16, wire, sizeof(wire)), 0);
  const std::string late = wire, boot = server.session.boot;
  ASSERT_EQ(je2_loopback_init(&server), 0); // new TEST server lifetime, no real writer exists
  ASSERT_NE(boot, server.session.boot);
  EXPECT_EQ(deliver(late, 1.17), -1);
  EXPECT_EQ(server.reads, 0u); EXPECT_EQ(server.starts, 0u);
  ASSERT_EQ(observe(1.171), -1); ASSERT_EQ(client.phase, JE2_CLIENT_RETIRED);
  ASSERT_EQ(je2_client_init(&client), 0); // explicit new client, not automatic recovery
  ASSERT_NO_FATAL_FAILURE(handshake(2.0));
  const auto fresh = adapter->bind(goal(), 2.16); ASSERT_TRUE(fresh);
  EXPECT_EQ(client.generation, 2u); // identical number is insufficient identity
  const unsigned reads = server.reads;
  EXPECT_EQ(deliver(late, 2.161), -1); EXPECT_EQ(server.reads, reads);
  EXPECT_EQ(adapter->prepare(old, 1, std::numeric_limits<double>::quiet_NaN(), wire, sizeof(wire)), -1);
  EXPECT_FALSE(adapter->cancel(old)); EXPECT_FALSE(adapter->send_failed(old));
  ASSERT_EQ(client.phase, JE2_CLIENT_ACTIVE);
  ASSERT_GT(adapter->prepare(fresh, 0, 2.17, wire, sizeof(wire)), 0);
  ASSERT_EQ(deliver(wire, 2.17), 0);
  EXPECT_EQ(server.starts, 1u); EXPECT_EQ(server.joints, 2u);
}

TEST_F(SessionLoopback, ActualMultipartTailIsNotReinterpretedAsNewCommand)
{
  ASSERT_NO_FATAL_FAILURE(handshake());
  const auto ticket = adapter->bind(goal(), 1.16); ASSERT_TRUE(ticket);
  char wire[MK2_FRAME_MAX];
  ASSERT_GT(adapter->prepare(ticket, 0, 1.16, wire, sizeof(wire)), 0);
  queue("invalid-prefix", true); queue(wire);
  const unsigned reads = server.reads;
  EXPECT_EQ(serve(1.161), -1); EXPECT_TRUE(server.session.discarding_parts);
  EXPECT_EQ(serve(1.162), -1); EXPECT_FALSE(server.session.discarding_parts);
  EXPECT_EQ(server.reads, reads); EXPECT_EQ(server.session.last_sequence, 2u);
  // Parser injection only: a subsequent complete packet must still be usable.
  // The adapter has no resend API or autonomous recovery loop.
  ASSERT_EQ(deliver(wire, 1.17), 0);
  EXPECT_EQ(server.joints, 2u); EXPECT_EQ(observe(1.171), 0);
}

TEST_F(SessionLoopback, LocalCancellationIsNotAPhysicalStopAcknowledgement)
{
  ASSERT_NO_FATAL_FAILURE(handshake());
  const auto ticket = adapter->bind(goal(), 1.16); ASSERT_TRUE(ticket);
  ASSERT_TRUE(adapter->cancel(ticket));
  EXPECT_EQ(client.phase, JE2_CLIENT_RETIRED);
  EXPECT_EQ(server.engine.owner, JE2_PC); EXPECT_EQ(server.stops, 0u);
  char wire[MK2_FRAME_MAX] = "old";
  EXPECT_EQ(adapter->prepare(ticket, 0, 1.17, wire, sizeof(wire)), -1);
  EXPECT_EQ(wire[0], '\0');
  je2_loopback_tick(&server, 1.36);
  EXPECT_EQ(server.engine.core.fault, MK2_WATCHDOG);
  EXPECT_EQ(server.stops, 1u); EXPECT_EQ(server.ends, 1u); EXPECT_EQ(server.joints, 1u);
}
TEST_F(SessionLoopback, ScheduledTargetsWaitUntilDueAndDrainIsNotMotionCompletion)
{
  ASSERT_NO_FATAL_FAILURE(handshake());
  const auto request = goal(25);
  const auto ticket = adapter->bind_scheduled(request, 1.16, .002); ASSERT_TRUE(ticket);
  char wire[MK2_FRAME_MAX];
  for (std::size_t i = 0; i < request.trajectory.points.size(); ++i) {
    // Use the actual message duration. .01*i can round one ULP earlier than
    // nanosec*1e-9; that instant correctly remains WaitingDue (no early send).
    const double due = 1.16 + request.trajectory.points[i].time_from_start.nanosec * 1e-9;
    if (i) {
      EXPECT_EQ(adapter->dispatch(ticket, due - .002, wire, sizeof(wire)).state, Dispatch::WaitingDue);
      EXPECT_EQ(adapter->dispatch(ticket, std::nextafter(due, -std::numeric_limits<double>::infinity()),
        wire, sizeof(wire)).state, Dispatch::WaitingDue);
      EXPECT_EQ(wire[0], '\0'); EXPECT_EQ(server.joints, i + 1);
    }
    const auto result = adapter->dispatch(ticket, due, wire, sizeof(wire));
    ASSERT_EQ(result.state, Dispatch::FrameReady); ASSERT_GT(result.size, 0);
    ASSERT_EQ(deliver(std::string(wire, result.size), due), 0);
    for (std::size_t axis = 0; axis < MK2_AXES; ++axis)
      EXPECT_DOUBLE_EQ(server.target[axis], request.trajectory.points[i].positions[axis]);
    // Same wakeup can never drain another queued point or declare success.
    const auto again = adapter->dispatch(ticket, due, wire, sizeof(wire));
    EXPECT_EQ(again.state, i == 24 ? Dispatch::WaitingAck : Dispatch::WaitingDue);
    EXPECT_EQ(again.size, 0); EXPECT_EQ(wire[0], '\0');
    ASSERT_EQ(observe(due + .001), 0);
  }
  EXPECT_EQ(adapter->dispatch(ticket, 1.402, wire, sizeof(wire)).state, Dispatch::Drained);
  EXPECT_TRUE(adapter->has_goal()); EXPECT_EQ(server.engine.owner, JE2_PC);
  EXPECT_EQ(server.starts, 1u); EXPECT_EQ(server.joints, 26u); EXPECT_EQ(server.stops, 0u);
  // No automatic post-goal hold extension; existing watchdog still applies.
  je2_loopback_tick(&server, 1.61);
  EXPECT_EQ(server.engine.core.fault, MK2_WATCHDOG);
  EXPECT_EQ(observe(1.611), -1);
  EXPECT_EQ(adapter->dispatch(ticket, 1.612, wire, sizeof(wire)).state, Dispatch::Rejected);
  EXPECT_EQ(server.joints, 26u); EXPECT_EQ(server.stops, 1u);
}

TEST_F(SessionLoopback, ScheduledBackpressureWithinBudgetDoesNotChangeCommandTime)
{
  ASSERT_NO_FATAL_FAILURE(handshake());
  const auto request = goal();
  const auto ticket = adapter->bind_scheduled(request, 1.16, .002); ASSERT_TRUE(ticket);
  char wire[MK2_FRAME_MAX];
  ASSERT_EQ(adapter->dispatch(ticket, 1.16, wire, sizeof(wire)).state, Dispatch::FrameReady);
  ASSERT_EQ(deliver(wire, 1.16), 0);
  for (double now : {1.17, 1.1705}) {
    EXPECT_EQ(adapter->dispatch(ticket, now, wire, sizeof(wire)).state, Dispatch::WaitingAck);
    EXPECT_EQ(wire[0], '\0'); EXPECT_EQ(client.wire_sequence, 3u);
  }
  ASSERT_EQ(observe(1.171), 0);
  ASSERT_EQ(adapter->dispatch(ticket, 1.1715, wire, sizeof(wire)).state, Dispatch::FrameReady);
  ASSERT_EQ(deliver(wire, 1.1715), 0);
  EXPECT_DOUBLE_EQ(server.target[0], request.trajectory.points[1].positions[0]);
  EXPECT_NEAR(server.data_time, .03, 1e-12); // not .0315: time axis is never retimed
  EXPECT_EQ(server.joints, 3u);
}

TEST_F(SessionLoopback, ScheduledBackpressureExpiresBeforeAckWatchdogWithoutCatchup)
{
  ASSERT_NO_FATAL_FAILURE(handshake());
  const auto ticket = adapter->bind_scheduled(goal(), 1.16, .002); ASSERT_TRUE(ticket);
  char wire[MK2_FRAME_MAX];
  ASSERT_EQ(adapter->dispatch(ticket, 1.16, wire, sizeof(wire)).state, Dispatch::FrameReady);
  ASSERT_EQ(deliver(wire, 1.16), 0);
  EXPECT_EQ(adapter->dispatch(ticket, 1.17, wire, sizeof(wire)).state, Dispatch::WaitingAck);
  EXPECT_EQ(adapter->dispatch(ticket, 1.1721, wire, sizeof(wire)).state, Dispatch::Expired);
  EXPECT_EQ(client.phase, JE2_CLIENT_FAULT); EXPECT_FALSE(adapter->has_goal());
  EXPECT_EQ(wire[0], '\0'); EXPECT_EQ(client.wire_sequence, 3u);
  EXPECT_EQ(server.engine.core.fault, MK2_NO_FAULT); EXPECT_EQ(server.stops, 0u);
  EXPECT_EQ(observe(1.173), -1); // late ACK cannot restart scheduling
  EXPECT_EQ(adapter->dispatch(ticket, 1.18, wire, sizeof(wire)).state, Dispatch::Rejected);
  EXPECT_EQ(server.joints, 2u);
}

TEST_F(SessionLoopback, ScheduledFinalUnconfirmedFrameNeverReportsDrained)
{
  ASSERT_NO_FATAL_FAILURE(handshake());
  const auto ticket = adapter->bind_scheduled(goal(1), 1.16, .002); ASSERT_TRUE(ticket);
  char wire[MK2_FRAME_MAX];
  ASSERT_EQ(adapter->dispatch(ticket, 1.16, wire, sizeof(wire)).state, Dispatch::FrameReady);
  // Drop the only target, keep real Owner publications healthy and current.
  for (double now : {1.19, 1.22, 1.25}) {
    je2_loopback_tick(&server, now); ASSERT_EQ(observe(now), 0);
    EXPECT_EQ(adapter->dispatch(ticket, now, wire, sizeof(wire)).state, Dispatch::WaitingAck);
    EXPECT_EQ(wire[0], '\0');
  }
  EXPECT_EQ(adapter->dispatch(ticket, 1.261, wire, sizeof(wire)).state, Dispatch::Rejected);
  EXPECT_EQ(client.phase, JE2_CLIENT_FAULT); EXPECT_FALSE(adapter->has_goal());
  EXPECT_EQ(server.joints, 1u); EXPECT_EQ(server.stops, 0u);
}

TEST_F(SessionLoopback, ScheduledLateFirstWakeupSendsNothing)
{
  ASSERT_NO_FATAL_FAILURE(handshake());
  const auto ticket = adapter->bind_scheduled(goal(), 1.16, .002); ASSERT_TRUE(ticket);
  char wire[MK2_FRAME_MAX] = "old";
  EXPECT_EQ(adapter->dispatch(ticket, 1.163, wire, sizeof(wire)).state, Dispatch::Expired);
  EXPECT_EQ(wire[0], '\0'); EXPECT_EQ(client.wire_sequence, 2u);
  EXPECT_EQ(client.phase, JE2_CLIENT_FAULT); EXPECT_EQ(server.joints, 1u);
}

TEST_F(SessionLoopback, ScheduledActualFrameSpacingIsNotCompressedAfterLateSend)
{
  ASSERT_NO_FATAL_FAILURE(handshake());
  auto request = goal(2);
  for (std::size_t i = 0; i < 2; ++i) {
    auto & point = request.trajectory.points[i];
    point.time_from_start.nanosec = static_cast<uint32_t>(i * 5000000);
    point.positions.assign(client.first_target, client.first_target + MK2_AXES);
    point.velocities.assign(MK2_AXES, 0); point.accelerations.assign(MK2_AXES, 0);
  }
  const auto ticket = adapter->bind_scheduled(request, 1.16, .002); ASSERT_TRUE(ticket);
  char wire[MK2_FRAME_MAX];
  ASSERT_EQ(adapter->dispatch(ticket, 1.1615, wire, sizeof(wire)).state, Dispatch::FrameReady);
  ASSERT_EQ(deliver(wire, 1.1615), 0); ASSERT_EQ(observe(1.162), 0);
  EXPECT_EQ(adapter->dispatch(ticket, 1.165, wire, sizeof(wire)).state, Dispatch::WaitingInterval);
  EXPECT_EQ(wire[0], '\0'); EXPECT_EQ(client.wire_sequence, 3u);
  ASSERT_EQ(adapter->dispatch(ticket, 1.1665, wire, sizeof(wire)).state, Dispatch::FrameReady);
  ASSERT_EQ(deliver(wire, 1.1665), 0);
  EXPECT_NEAR(server.data_time, .025, 1e-12); EXPECT_EQ(server.joints, 3u);
}

TEST_F(SessionLoopback, ScheduledAdmissionRequiresBudgetAndCannotBypassDispatcher)
{
  ASSERT_NO_FATAL_FAILURE(handshake());
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (double budget : {-1.0, .0021, nan, std::numeric_limits<double>::infinity()}) {
    EXPECT_FALSE(adapter->bind_scheduled(goal(), 1.16, budget));
    EXPECT_EQ(client.phase, JE2_CLIENT_ACTIVE); EXPECT_EQ(client.wire_sequence, 2u);
  }
  auto request = goal();
  for (auto & point : request.trajectory.points) point.time_from_start.nanosec += 10000000;
  const auto ticket = adapter->bind_scheduled(request, 1.16, 0); ASSERT_TRUE(ticket);
  char wire[MK2_FRAME_MAX];
  // Low-level prepare cannot bypass a scheduled ticket, even with a bad clock.
  EXPECT_EQ(adapter->prepare(ticket, 0, nan, wire, sizeof(wire)), -1);
  EXPECT_EQ(client.phase, JE2_CLIENT_ACTIVE);
  EXPECT_EQ(adapter->dispatch(ticket, 1.16, wire, sizeof(wire)).state, Dispatch::WaitingDue);
  EXPECT_EQ(client.wire_sequence, 2u);
  ASSERT_EQ(adapter->dispatch(ticket, 1.17, wire, sizeof(wire)).state, Dispatch::FrameReady);
  EXPECT_EQ(deliver(wire, 1.17), 0);
}

TEST_F(SessionLoopback, ScheduledWaitingStillValidatesClock)
{
  ASSERT_NO_FATAL_FAILURE(handshake());
  auto request = goal();
  for (auto & point : request.trajectory.points) point.time_from_start.nanosec += 10000000;
  const auto ticket = adapter->bind_scheduled(request, 1.16, .002); ASSERT_TRUE(ticket);
  char wire[MK2_FRAME_MAX];
  EXPECT_EQ(adapter->dispatch(ticket, std::numeric_limits<double>::quiet_NaN(), wire, sizeof(wire)).state,
    Dispatch::Rejected);
  EXPECT_EQ(client.phase, JE2_CLIENT_FAULT); EXPECT_EQ(client.wire_sequence, 2u);
  EXPECT_EQ(wire[0], '\0'); EXPECT_FALSE(adapter->has_goal());
}

TEST_F(SessionLoopback, OldScheduledTimerCannotTouchExplicitNewSession)
{
  ASSERT_NO_FATAL_FAILURE(handshake());
  const auto old = adapter->bind_scheduled(goal(), 1.16, .002); ASSERT_TRUE(old);
  ASSERT_EQ(je2_loopback_init(&server), 0); // new fake server lifetime only
  ASSERT_EQ(observe(1.161), -1);
  ASSERT_EQ(je2_client_init(&client), 0);
  ASSERT_NO_FATAL_FAILURE(handshake(2.0));
  const auto fresh = adapter->bind_scheduled(goal(), 2.16, .002); ASSERT_TRUE(fresh);
  const auto before = client;
  char wire[MK2_FRAME_MAX];
  EXPECT_EQ(adapter->dispatch(old, std::numeric_limits<double>::quiet_NaN(), wire, sizeof(wire)).state,
    Dispatch::Rejected);
  EXPECT_FALSE(adapter->cancel(old)); EXPECT_FALSE(adapter->send_failed(old));
  EXPECT_EQ(std::memcmp(&before, &client, sizeof(client)), 0);
  EXPECT_EQ(wire[0], '\0');
  ASSERT_EQ(adapter->dispatch(fresh, 2.16, wire, sizeof(wire)).state, Dispatch::FrameReady);
  EXPECT_EQ(deliver(wire, 2.16), 0);
}

TEST_F(SessionLoopback, ScheduledCancelInvalidatesTimerWithoutStopAcknowledgement)
{
  ASSERT_NO_FATAL_FAILURE(handshake());
  const auto ticket = adapter->bind_scheduled(goal(), 1.16, .002); ASSERT_TRUE(ticket);
  ASSERT_TRUE(adapter->cancel(ticket));
  char wire[MK2_FRAME_MAX];
  EXPECT_EQ(adapter->dispatch(ticket, 1.16, wire, sizeof(wire)).state, Dispatch::Rejected);
  EXPECT_EQ(client.phase, JE2_CLIENT_RETIRED); EXPECT_EQ(wire[0], '\0');
  EXPECT_EQ(server.joints, 1u); EXPECT_EQ(server.stops, 0u); EXPECT_EQ(server.engine.owner, JE2_PC);
}

TEST_F(SessionLoopback, ScheduledSendFailureCannotRetryOrEmitCleanup)
{
  ASSERT_NO_FATAL_FAILURE(handshake());
  const auto ticket = adapter->bind_scheduled(goal(), 1.16, .002); ASSERT_TRUE(ticket);
  char wire[MK2_FRAME_MAX];
  ASSERT_EQ(adapter->dispatch(ticket, 1.16, wire, sizeof(wire)).state, Dispatch::FrameReady);
  // Model transport refusal without delivering or retrying the formatted frame.
  ASSERT_TRUE(adapter->send_failed(ticket));
  EXPECT_EQ(adapter->dispatch(ticket, 1.161, wire, sizeof(wire)).state, Dispatch::Rejected);
  EXPECT_EQ(client.phase, JE2_CLIENT_FAULT); EXPECT_EQ(wire[0], '\0');
  EXPECT_EQ(server.joints, 1u); EXPECT_EQ(server.stops, 0u); EXPECT_EQ(client.wire_sequence, 3u);
}
TEST_F(SessionLoopback, ExplicitCloseAfterDrainedNeedsCorrelatedExitBarrier)
{
  ASSERT_NO_FATAL_FAILURE(handshake());
  const auto ticket = adapter->bind_scheduled(goal(1), 1.16, .002); ASSERT_TRUE(ticket);
  char wire[MK2_FRAME_MAX];
  ASSERT_EQ(adapter->dispatch(ticket, 1.16, wire, sizeof(wire)).state, Dispatch::FrameReady);
  ASSERT_EQ(deliver(wire, 1.16), 0); ASSERT_EQ(observe(1.161), 0);
  ASSERT_EQ(adapter->dispatch(ticket, 1.162, wire, sizeof(wire)).state, Dispatch::Drained);
  ASSERT_GT(adapter->request_close(ticket, 1.163, wire, sizeof(wire)), 0);
  const std::string release = wire;
  ASSERT_EQ(deliver(release, 1.163), 0); ASSERT_EQ(observe(1.164), 0);
  EXPECT_EQ(server.session.release_sequence, 4u);
  EXPECT_EQ(adapter->close_state(ticket), Close::Pending);
  EXPECT_FALSE(adapter->has_goal());
  EXPECT_EQ(adapter->request_close(ticket, 1.165, wire, sizeof(wire)), 0);
  EXPECT_EQ(wire[0], '\0'); // never repeat the release
  ASSERT_NO_FATAL_FAILURE(finish_close(ticket, 1.163));
  EXPECT_EQ(server.releases, 1u); EXPECT_EQ(server.starts, 1u);
  const unsigned reads = server.reads;
  EXPECT_EQ(deliver(release, 1.43), -1); EXPECT_EQ(server.reads, reads);
  EXPECT_EQ(adapter->dispatch(ticket, 1.431, wire, sizeof(wire)).state, Dispatch::Rejected);
  EXPECT_FALSE(adapter->bind(goal(), 1.432)); // no automatic reuse of a closed session
  EXPECT_EQ(adapter->close_state(ticket), Close::Confirmed); // historical receipt only
}

TEST_F(SessionLoopback, CloseWaitsForOutstandingTargetAndFreezesNewOutput)
{
  ASSERT_NO_FATAL_FAILURE(handshake());
  const auto ticket = adapter->bind_scheduled(goal(), 1.16, .002); ASSERT_TRUE(ticket);
  je2_client_lease lease{}; ASSERT_EQ(je2_client_get_lease(&client, &lease), 0);
  char wire[MK2_FRAME_MAX];
  ASSERT_EQ(adapter->dispatch(ticket, 1.16, wire, sizeof(wire)).state, Dispatch::FrameReady);
  const std::string pending = wire;
  EXPECT_EQ(adapter->request_close(ticket, 1.161, wire, sizeof(wire)), 0);
  EXPECT_EQ(client.phase, JE2_CLIENT_DRAINING); EXPECT_EQ(wire[0], '\0');
  EXPECT_EQ(client.wire_sequence, 3u);
  EXPECT_EQ(adapter->dispatch(ticket, 1.162, wire, sizeof(wire)).state, Dispatch::Rejected);
  EXPECT_EQ(je2_client_target(&client, &lease, 1.162, .03, client.first_target, wire, sizeof(wire)), -1);
  ASSERT_EQ(deliver(pending, 1.162), 0); ASSERT_EQ(observe(1.163), 0);
  ASSERT_GT(adapter->request_close(ticket, 1.164, wire, sizeof(wire)), 0);
  ASSERT_EQ(client.wire_sequence, 4u);
  ASSERT_EQ(deliver(wire, 1.164), 0); ASSERT_EQ(observe(1.165), 0);
  ASSERT_NO_FATAL_FAILURE(finish_close(ticket, 1.164));
  EXPECT_EQ(server.joints, 2u); EXPECT_EQ(server.releases, 1u);
}

TEST_F(SessionLoopback, NativeIntentWinningCloseRaceDoesNotStopNewOwner)
{
  ASSERT_NO_FATAL_FAILURE(handshake());
  const auto ticket = adapter->bind(goal(), 1.16); ASSERT_TRUE(ticket);
  ASSERT_EQ(je2_loopback_button(&server, "tv49!", 5, 1.161), 0);
  char wire[MK2_FRAME_MAX];
  ASSERT_GT(adapter->request_close(ticket, 1.162, wire, sizeof(wire)), 0); // not yet observed yield
  const unsigned reads = server.reads;
  EXPECT_EQ(deliver(wire, 1.163), -1);
  EXPECT_EQ(server.reads, reads); EXPECT_EQ(server.releases, 1u); EXPECT_EQ(server.stops, 0u);
  EXPECT_EQ(server.session.release_sequence, 0u); // native yield is not our receipt
  EXPECT_EQ(observe(1.164), -1);
  EXPECT_EQ(adapter->close_state(ticket), Close::Retired);
  EXPECT_EQ(adapter->request_close(ticket, 1.165, wire, sizeof(wire)), -1);
  EXPECT_EQ(wire[0], '\0'); EXPECT_EQ(client.phase, JE2_CLIENT_RETIRED);
}

TEST_F(SessionLoopback, LostCloseFrameTimesOutWithoutRetry)
{
  ASSERT_NO_FATAL_FAILURE(handshake());
  const auto ticket = adapter->bind(goal(), 1.16); ASSERT_TRUE(ticket);
  char wire[MK2_FRAME_MAX];
  ASSERT_GT(adapter->request_close(ticket, 1.16, wire, sizeof(wire)), 0);
  for (double now : {1.18, 1.22, 1.25}) {
    ASSERT_EQ(observe(now), 0); // actual Owner: request not yet consumed
    EXPECT_EQ(adapter->request_close(ticket, now, wire, sizeof(wire)), 0);
    EXPECT_EQ(wire[0], '\0'); EXPECT_EQ(client.wire_sequence, 3u);
  }
  EXPECT_EQ(adapter->request_close(ticket, 1.261, wire, sizeof(wire)), -1);
  EXPECT_EQ(adapter->close_state(ticket), Close::Fault);
  EXPECT_EQ(server.releases, 0u); EXPECT_EQ(server.joints, 1u); EXPECT_EQ(server.stops, 0u);
}

TEST_F(SessionLoopback, CloseCannotSkipLostTargetOrExtendItsDeadline)
{
  ASSERT_NO_FATAL_FAILURE(handshake());
  const auto ticket = adapter->bind_scheduled(goal(), 1.16, .002); ASSERT_TRUE(ticket);
  char wire[MK2_FRAME_MAX];
  ASSERT_EQ(adapter->dispatch(ticket, 1.16, wire, sizeof(wire)).state, Dispatch::FrameReady);
  EXPECT_EQ(adapter->request_close(ticket, 1.161, wire, sizeof(wire)), 0);
  for (double now : {1.19, 1.22, 1.25}) {
    ASSERT_EQ(observe(now), 0);
    EXPECT_EQ(adapter->request_close(ticket, now, wire, sizeof(wire)), 0);
    EXPECT_EQ(client.wire_sequence, 3u); EXPECT_EQ(wire[0], '\0');
  }
  EXPECT_EQ(adapter->request_close(ticket, 1.261, wire, sizeof(wire)), -1);
  EXPECT_EQ(adapter->close_state(ticket), Close::Fault);
  EXPECT_EQ(server.releases, 0u); EXPECT_EQ(server.joints, 1u);
}

TEST_F(SessionLoopback, EndAndHealthyIdleCannotReplaceWriterExitReceipt)
{
  ASSERT_NO_FATAL_FAILURE(handshake());
  const auto ticket = adapter->bind(goal(), 1.16); ASSERT_TRUE(ticket);
  char wire[MK2_FRAME_MAX];
  ASSERT_GT(adapter->request_close(ticket, 1.16, wire, sizeof(wire)), 0);
  ASSERT_EQ(deliver(wire, 1.16), 0); ASSERT_EQ(observe(1.161), 0);
  je2_loopback_tick(&server, 1.17); ASSERT_EQ(observe(1.171), 0);
  for (double now : {1.28, 1.38, 1.48, 1.58, 1.65}) {
    je2_loopback_tick(&server, now); ASSERT_EQ(observe(now + .001), 0);
    EXPECT_EQ(adapter->request_close(ticket, now + .002, wire, sizeof(wire)), 0);
    EXPECT_EQ(adapter->close_state(ticket), Close::Pending);
    EXPECT_EQ(server.ends, 1u); EXPECT_EQ(server.sample.move_state, 0);
  }
  EXPECT_EQ(adapter->request_close(ticket, 1.661, wire, sizeof(wire)), -1);
  EXPECT_EQ(adapter->close_state(ticket), Close::Fault);
  EXPECT_EQ(server.engine.owner, JE2_EXIT_BARRIER); EXPECT_EQ(server.stops, 0u);
  EXPECT_EQ(wire[0], '\0');
}

TEST_F(SessionLoopback, StateSevenDuringCloseIsFaultNotConfirmation)
{
  ASSERT_NO_FATAL_FAILURE(handshake());
  const auto ticket = adapter->bind(goal(), 1.16); ASSERT_TRUE(ticket);
  char wire[MK2_FRAME_MAX];
  ASSERT_GT(adapter->request_close(ticket, 1.16, wire, sizeof(wire)), 0);
  ASSERT_EQ(deliver(wire, 1.16), 0); ASSERT_EQ(observe(1.161), 0);
  server.sample.move_state = 7; je2_loopback_tick(&server, 1.17);
  EXPECT_EQ(observe(1.171), -1); EXPECT_EQ(adapter->close_state(ticket), Close::Fault);
  server.sample.move_state = 0; je2_loopback_tick(&server, 1.18);
  EXPECT_EQ(observe(1.181), -1); EXPECT_EQ(adapter->close_state(ticket), Close::Fault);
  EXPECT_EQ(server.stops, 1u); EXPECT_EQ(server.ends, 1u); EXPECT_EQ(server.releases, 1u);
}

TEST_F(SessionLoopback, CloseTransportFailureAndOldCloseCallbackCannotAffectNewSession)
{
  ASSERT_NO_FATAL_FAILURE(handshake());
  const auto old = adapter->bind(goal(), 1.16); ASSERT_TRUE(old);
  char wire[MK2_FRAME_MAX];
  ASSERT_GT(adapter->request_close(old, 1.16, wire, sizeof(wire)), 0);
  ASSERT_TRUE(adapter->send_failed(old));
  EXPECT_EQ(adapter->close_state(old), Close::Fault);
  EXPECT_EQ(adapter->request_close(old, 1.17, wire, sizeof(wire)), -1);
  EXPECT_EQ(wire[0], '\0'); EXPECT_EQ(server.releases, 0u);
  ASSERT_EQ(je2_loopback_init(&server), 0); ASSERT_EQ(je2_client_init(&client), 0);
  ASSERT_NO_FATAL_FAILURE(handshake(2.0));
  const auto fresh = adapter->bind_scheduled(goal(), 2.16, .002); ASSERT_TRUE(fresh);
  const auto before = client;
  EXPECT_EQ(adapter->request_close(old, std::numeric_limits<double>::quiet_NaN(), wire, sizeof(wire)), -1);
  EXPECT_FALSE(adapter->send_failed(old)); EXPECT_FALSE(adapter->cancel(old));
  EXPECT_EQ(std::memcmp(&before, &client, sizeof(client)), 0);
  ASSERT_EQ(adapter->dispatch(fresh, 2.16, wire, sizeof(wire)).state, Dispatch::FrameReady);
  EXPECT_EQ(deliver(wire, 2.16), 0);
}
TEST_F(SessionLoopback, FailedReleaseCallbackDoesNotCreateSuccessReceipt)
{
  ASSERT_NO_FATAL_FAILURE(handshake());
  const auto ticket = adapter->bind(goal(), 1.16); ASSERT_TRUE(ticket);
  char wire[MK2_FRAME_MAX];
  ASSERT_GT(adapter->request_close(ticket, 1.16, wire, sizeof(wire)), 0);
  server.release_result = -42;
  EXPECT_EQ(deliver(wire, 1.16), -1);
  EXPECT_EQ(server.session.last_sequence, 3u);
  EXPECT_EQ(server.session.release_sequence, 0u);
  EXPECT_EQ(server.engine.core.fault, MK2_SDK_ERROR);
  EXPECT_EQ(observe(1.161), -1);
  EXPECT_EQ(adapter->close_state(ticket), Close::Fault);
  EXPECT_EQ(server.releases, 1u); EXPECT_EQ(server.stops, 1u); EXPECT_EQ(server.ends, 1u);
}

TEST_F(SessionLoopback, NewClaimBeforeCloseReceiptRetiresOldClientWithoutCleanup)
{
  ASSERT_NO_FATAL_FAILURE(handshake());
  const auto ticket = adapter->bind(goal(), 1.16); ASSERT_TRUE(ticket);
  char wire[MK2_FRAME_MAX];
  ASSERT_GT(adapter->request_close(ticket, 1.16, wire, sizeof(wire)), 0);
  ASSERT_EQ(deliver(wire, 1.16), 0); ASSERT_EQ(observe(1.161), 0);
  ASSERT_NO_FATAL_FAILURE(finish_close(ticket, 1.16, false));
  ASSERT_EQ(adapter->close_state(ticket), Close::Pending); // final receipt withheld
  je2_client peer{}; ASSERT_EQ(je2_client_init(&peer), 0);
  const int size = je2_loopback_snapshot(&server, 1.42, wire, sizeof(wire));
  ASSERT_GT(size, 0);
  ASSERT_EQ(je2_client_observe(&peer, wire, static_cast<size_t>(size), 1.42), 0);
  ASSERT_GT(je2_client_claim(&peer, 1.43, wire, sizeof(wire)), 0);
  ASSERT_EQ(deliver(wire, 1.43), 0); // new claim only, NOT a second ServoStart
  EXPECT_EQ(server.session.release_sequence, 0u);
  EXPECT_EQ(server.engine.generation, 4u);
  EXPECT_EQ(observe(1.431), -1);
  EXPECT_EQ(adapter->close_state(ticket), Close::Retired);
  EXPECT_EQ(adapter->request_close(ticket, 1.432, wire, sizeof(wire)), -1);
  EXPECT_EQ(wire[0], '\0');
  EXPECT_EQ(server.starts, 1u); EXPECT_EQ(server.releases, 1u); EXPECT_EQ(server.stops, 0u);
}
TEST_F(SessionLoopback, ExecutionSucceedsOnlyAfterFinalToleranceAndCloseWithOneResult)
{
  ASSERT_NO_FATAL_FAILURE(handshake());
  Execution execution(*adapter, execution_policy());
  const auto original = execution_goal();
  auto mutable_goal = original;
  const auto ticket = execution.start(mutable_goal, 1.16); ASSERT_TRUE(ticket);
  mutable_goal.trajectory.points.back().positions[0] += .1; // immutable admitted target
  double released_at = 0;
  ASSERT_NO_FATAL_FAILURE(stream_and_request_close(execution, ticket, original, released_at));
  EXPECT_FALSE(execution.take_result(ticket));
  ASSERT_NO_FATAL_FAILURE(finish_close(ticket, released_at));
  EXPECT_EQ(execution.state(ticket), Execution::State::Closing);
  char wire[MK2_FRAME_MAX];
  EXPECT_EQ(execution.request_cancel(ticket, released_at + .252, wire, sizeof(wire)), -1);
  EXPECT_EQ(execution.advance(ticket, released_at + .252, wire, sizeof(wire)).output, Execution::Output::Terminal);
  EXPECT_EQ(wire[0], '\0');
  EXPECT_FALSE(execution.start(original, std::numeric_limits<double>::quiet_NaN())); // don't overwrite result
  const auto result = execution.take_result(ticket); ASSERT_TRUE(result);
  EXPECT_EQ(result->state, Execution::State::Succeeded); EXPECT_EQ(result->reason, Execution::Reason::None);
  EXPECT_FALSE(execution.take_result(ticket)); EXPECT_FALSE(execution.send_failed(ticket));
  EXPECT_EQ(execution.advance(ticket, std::numeric_limits<double>::quiet_NaN(), wire, sizeof(wire)).output,
    Execution::Output::Terminal); // terminal result is immutable
  EXPECT_EQ(server.starts, 1u); EXPECT_EQ(server.joints, 5u); EXPECT_EQ(server.releases, 1u);
}

TEST_F(SessionLoopback, ExecutionClosedButOutsideGoalToleranceAborts)
{
  ASSERT_NO_FATAL_FAILURE(handshake());
  auto policy = execution_policy();
  Execution execution(*adapter, policy);
  policy.position_tolerance.fill(1); // changing caller policy must not loosen admitted checks
  const auto request = execution_goal();
  const auto ticket = execution.start(request, 1.16); ASSERT_TRUE(ticket);
  double released_at = 0;
  ASSERT_NO_FATAL_FAILURE(stream_and_request_close(execution, ticket, request, released_at));
  server.sample.joint[0] += .001; // healthy stable release pose, but NOT the goal tolerance
  ASSERT_NO_FATAL_FAILURE(finish_close(ticket, released_at));
  char wire[MK2_FRAME_MAX];
  EXPECT_EQ(execution.advance(ticket, released_at + .252, wire, sizeof(wire)).output, Execution::Output::Terminal);
  const auto result = execution.take_result(ticket); ASSERT_TRUE(result);
  EXPECT_EQ(result->state, Execution::State::Aborted);
  EXPECT_EQ(result->reason, Execution::Reason::GoalTolerance);
  EXPECT_EQ(client.phase, JE2_CLIENT_CLOSED); EXPECT_EQ(server.stops, 0u); // don't stop a later owner
}

TEST_F(SessionLoopback, ExecutionCancellationWaitsForOutstandingFrameAndReceipt)
{
  ASSERT_NO_FATAL_FAILURE(handshake());
  Execution execution(*adapter, execution_policy());
  const auto ticket = execution.start(execution_goal(), 1.16); ASSERT_TRUE(ticket);
  char wire[MK2_FRAME_MAX];
  ASSERT_EQ(execution.advance(ticket, 1.16, wire, sizeof(wire)).output, Execution::Output::Target);
  const std::string pending = wire;
  EXPECT_EQ(execution.request_cancel(ticket, 1.161, wire, sizeof(wire)), 0);
  EXPECT_EQ(client.phase, JE2_CLIENT_DRAINING); EXPECT_FALSE(execution.take_result(ticket));
  EXPECT_EQ(execution.advance(ticket, 1.162, wire, sizeof(wire)).output, Execution::Output::Waiting);
  EXPECT_EQ(wire[0], '\0');
  ASSERT_EQ(deliver(pending, 1.163), 0); ASSERT_EQ(observe(1.164), 0);
  ASSERT_EQ(execution.advance(ticket, 1.165, wire, sizeof(wire)).output, Execution::Output::Release);
  ASSERT_EQ(deliver(wire, 1.165), 0); ASSERT_EQ(observe(1.166), 0);
  ASSERT_NO_FATAL_FAILURE(finish_close(ticket, 1.165));
  EXPECT_EQ(execution.advance(ticket, 1.417, wire, sizeof(wire)).output, Execution::Output::Terminal);
  const auto result = execution.take_result(ticket); ASSERT_TRUE(result);
  EXPECT_EQ(result->state, Execution::State::Canceled); EXPECT_EQ(result->reason, Execution::Reason::None);
  EXPECT_EQ(server.joints, 2u); EXPECT_EQ(server.releases, 1u); EXPECT_EQ(server.stops, 0u);
}

TEST_F(SessionLoopback, ExecutionCancelDuringNormalCloseCannotTurnFaultIntoCanceled)
{
  ASSERT_NO_FATAL_FAILURE(handshake());
  Execution execution(*adapter, execution_policy());
  const auto request = execution_goal();
  const auto ticket = execution.start(request, 1.16); ASSERT_TRUE(ticket);
  double released_at = 0;
  ASSERT_NO_FATAL_FAILURE(stream_and_request_close(execution, ticket, request, released_at));
  char wire[MK2_FRAME_MAX];
  EXPECT_EQ(execution.request_cancel(ticket, released_at + .002, wire, sizeof(wire)), 0);
  EXPECT_EQ(wire[0], '\0'); EXPECT_EQ(server.releases, 1u);
  server.sample.move_state = 7; je2_loopback_tick(&server, released_at + .01);
  EXPECT_EQ(observe(released_at + .011), -1);
  EXPECT_EQ(execution.advance(ticket, released_at + .012, wire, sizeof(wire)).output, Execution::Output::Terminal);
  const auto result = execution.take_result(ticket); ASSERT_TRUE(result);
  EXPECT_EQ(result->state, Execution::State::Aborted); EXPECT_EQ(result->reason, Execution::Reason::ControlFault);
  EXPECT_EQ(execution.request_cancel(ticket, std::numeric_limits<double>::quiet_NaN(), wire, sizeof(wire)), -1);
}

TEST_F(SessionLoopback, ExecutionOwnershipLossAbortsWithoutRelease)
{
  ASSERT_NO_FATAL_FAILURE(handshake());
  Execution execution(*adapter, execution_policy());
  const auto ticket = execution.start(execution_goal(), 1.16); ASSERT_TRUE(ticket);
  ASSERT_EQ(je2_loopback_button(&server, "tv49!", 5, 1.161), 0); ASSERT_EQ(observe(1.162), -1);
  char wire[MK2_FRAME_MAX];
  EXPECT_EQ(execution.advance(ticket, 1.163, wire, sizeof(wire)).output, Execution::Output::Terminal);
  const auto result = execution.take_result(ticket); ASSERT_TRUE(result);
  EXPECT_EQ(result->state, Execution::State::Aborted); EXPECT_EQ(result->reason, Execution::Reason::ControlLost);
  EXPECT_EQ(wire[0], '\0'); EXPECT_EQ(server.releases, 1u); // only native yield, no stale release
  EXPECT_EQ(server.stops, 0u);
}

TEST_F(SessionLoopback, ExecutionExpiredScheduleAbortsWithoutAttemptingClose)
{
  ASSERT_NO_FATAL_FAILURE(handshake());
  Execution execution(*adapter, execution_policy());
  const auto ticket = execution.start(execution_goal(), 1.16); ASSERT_TRUE(ticket);
  char wire[MK2_FRAME_MAX];
  EXPECT_EQ(execution.advance(ticket, 1.163, wire, sizeof(wire)).output, Execution::Output::Terminal);
  const auto result = execution.take_result(ticket); ASSERT_TRUE(result);
  EXPECT_EQ(result->reason, Execution::Reason::ScheduleExpired);
  EXPECT_EQ(result->state, Execution::State::Aborted);
  EXPECT_EQ(server.releases, 0u); EXPECT_EQ(server.joints, 1u); EXPECT_EQ(wire[0], '\0');
}

TEST_F(SessionLoopback, ExecutionRejectsStaleCloseEvidenceInsteadOfSuccess)
{
  ASSERT_NO_FATAL_FAILURE(handshake());
  Execution execution(*adapter, execution_policy());
  const auto request = execution_goal();
  const auto ticket = execution.start(request, 1.16); ASSERT_TRUE(ticket);
  double released_at = 0;
  ASSERT_NO_FATAL_FAILURE(stream_and_request_close(execution, ticket, request, released_at));
  ASSERT_NO_FATAL_FAILURE(finish_close(ticket, released_at));
  char wire[MK2_FRAME_MAX];
  EXPECT_EQ(execution.advance(ticket, released_at + .4, wire, sizeof(wire)).output, Execution::Output::Terminal);
  const auto result = execution.take_result(ticket); ASSERT_TRUE(result);
  EXPECT_EQ(result->state, Execution::State::Aborted); EXPECT_EQ(result->reason, Execution::Reason::StaleEvidence);
  EXPECT_EQ(client.phase, JE2_CLIENT_CLOSED); EXPECT_EQ(server.stops, 0u);
}

TEST_F(SessionLoopback, ExecutionPolicyAndTerminalHoldAreRequiredWithoutPreemption)
{
  ASSERT_NO_FATAL_FAILURE(handshake());
  for (std::size_t axis = 0; axis < MK2_AXES; ++axis) {
    auto policy = execution_policy(); policy.position_tolerance[axis] = 0;
    Execution invalid(*adapter, policy);
    EXPECT_FALSE(invalid.start(execution_goal(), 1.16));
  }
  for (int kind = 0; kind < 5; ++kind) {
    auto policy = execution_policy();
    if (kind == 0) policy.max_result_age = 0;
    if (kind == 1) policy.max_result_age = .151;
    if (kind == 2) policy.position_tolerance[0] = std::numeric_limits<double>::quiet_NaN();
    if (kind == 3) policy.max_lateness = .003;
    if (kind == 4) policy.position_tolerance[0] = -1;
    Execution invalid(*adapter, policy);
    EXPECT_FALSE(invalid.start(execution_goal(), 1.16));
  }
  Execution execution(*adapter, execution_policy());
  EXPECT_FALSE(execution.start(goal(), 1.16)); // moving endpoint, never silently append hold
  auto invalid = execution_goal(); invalid.trajectory.points.back().velocities[0] = .001;
  EXPECT_FALSE(execution.start(invalid, 1.16));
  invalid = execution_goal(); invalid.trajectory.points.back().accelerations[0] = .1;
  EXPECT_FALSE(execution.start(invalid, 1.16));
  const auto ticket = execution.start(execution_goal(), 1.16); ASSERT_TRUE(ticket);
  const auto before = client;
  EXPECT_FALSE(execution.start(execution_goal(), std::numeric_limits<double>::quiet_NaN()));
  EXPECT_EQ(std::memcmp(&before, &client, sizeof(client)), 0); EXPECT_EQ(client.wire_sequence, 2u);
}

TEST_F(SessionLoopback, ExecutionOldCallbacksAndResultsCannotAffectNewGoal)
{
  ASSERT_NO_FATAL_FAILURE(handshake());
  Execution execution(*adapter, execution_policy());
  const auto old = execution.start(execution_goal(), 1.16); ASSERT_TRUE(old);
  char wire[MK2_FRAME_MAX];
  ASSERT_GT(execution.request_cancel(old, 1.16, wire, sizeof(wire)), 0);
  ASSERT_TRUE(execution.send_failed(old)); // release was NOT sent
  const auto failed = execution.take_result(old); ASSERT_TRUE(failed);
  EXPECT_EQ(failed->state, Execution::State::Aborted); EXPECT_EQ(failed->reason, Execution::Reason::SendFailed);
  ASSERT_EQ(je2_loopback_init(&server), 0); ASSERT_EQ(je2_client_init(&client), 0); // new fake lifetime
  ASSERT_NO_FATAL_FAILURE(handshake(2.0));
  const auto fresh = execution.start(execution_goal(), 2.16); ASSERT_TRUE(fresh);
  const auto before = client;
  EXPECT_EQ(execution.advance(old, std::numeric_limits<double>::quiet_NaN(), wire, sizeof(wire)).output,
    Execution::Output::Stale);
  EXPECT_EQ(execution.request_cancel(old, std::numeric_limits<double>::quiet_NaN(), wire, sizeof(wire)), -1);
  EXPECT_FALSE(execution.send_failed(old)); EXPECT_FALSE(execution.take_result(old));
  EXPECT_EQ(std::memcmp(&before, &client, sizeof(client)), 0);
  ASSERT_EQ(execution.advance(fresh, 2.16, wire, sizeof(wire)).output, Execution::Output::Target);
  EXPECT_EQ(deliver(wire, 2.16), 0);
}

TEST_F(SessionLoopback, ExecutionTargetSendFailureIsOneAbortedResult)
{
  ASSERT_NO_FATAL_FAILURE(handshake());
  Execution execution(*adapter, execution_policy());
  const auto ticket = execution.start(execution_goal(), 1.16); ASSERT_TRUE(ticket);
  char wire[MK2_FRAME_MAX];
  ASSERT_EQ(execution.advance(ticket, 1.16, wire, sizeof(wire)).output, Execution::Output::Target);
  ASSERT_TRUE(execution.send_failed(ticket));
  EXPECT_EQ(execution.advance(ticket, 1.161, wire, sizeof(wire)).output, Execution::Output::Terminal);
  const auto result = execution.take_result(ticket); ASSERT_TRUE(result);
  EXPECT_EQ(result->state, Execution::State::Aborted); EXPECT_EQ(result->reason, Execution::Reason::SendFailed);
  EXPECT_FALSE(execution.take_result(ticket)); EXPECT_EQ(wire[0], '\0');
  EXPECT_EQ(server.joints, 1u); EXPECT_EQ(server.releases, 0u);
}
}  // namespace