#include <gtest/gtest.h>
#include <json-c/json.h>
#include <limits>

#include "je2_ros_goal_adapter.hpp"

namespace
{
using Adapter = je2_experiment::RosGoalAdapter;
const Adapter::Names names{"j1", "j2", "j3", "j4", "j5", "j6", "j7"};

Adapter::Limits limits()
{
  Adapter::Limits result;
  result.lower.fill(-1.0); result.upper.fill(1.0);
  result.velocity.fill(.2); result.acceleration.fill(.4);
  return result; // test fixture only, not physical arm configuration
}

// Synthetic already-confirmed client for the ROS boundary unit tests only.
// The real handshake/transport is tested separately by the C/private sandbox.
void active_client(je2_client & client)
{
  ASSERT_EQ(je2_client_init(&client), 0);
  std::memcpy(client.boot, "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa", 33);
  client.phase = JE2_CLIENT_ACTIVE;
  client.generation = 2;
  client.wire_sequence = client.acknowledged = 2;
  client.first_data_time = client.last_data_time = .01;
  client.first_target_at = client.now = client.received_at = 1.01;
  client.seen = true;
}

Adapter::Goal goal()
{
  Adapter::Goal result;
  result.trajectory.joint_names.assign(names.begin(), names.end());
  for (int i = 0; i < 3; ++i) {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.assign(MK2_AXES, 0.0);
    point.velocities.assign(MK2_AXES, 0.0);
    point.time_from_start.nanosec = static_cast<uint32_t>(i * 10000000);
    result.trajectory.points.push_back(point);
  }
  return result;
}

TEST(Je2RosGoalAdapter, CannotAdmitBeforeFreshHoldConfirmation)
{
  je2_client client{};
  ASSERT_EQ(je2_client_init(&client), 0);
  Adapter adapter(client, names, limits());
  EXPECT_FALSE(adapter.bind(goal(), 1.0));
  active_client(client);
  EXPECT_TRUE(adapter.bind(goal(), 1.02));
  EXPECT_FALSE(adapter.bind(goal(), 1.02)); // no silent preemption
}

TEST(Je2RosGoalAdapter, BindsAnImmutableGoalAndDoesNotSkipOrReplayIndices)
{
  je2_client client{}; active_client(client);
  Adapter adapter(client, names, limits());
  auto source = goal();
  const auto ticket = adapter.bind(source, 1.02);
  ASSERT_TRUE(ticket);
  source.trajectory.points[0].positions[0] = 99;
  char wire[MK2_FRAME_MAX]{};
  EXPECT_EQ(adapter.prepare(ticket, 1, 1.02, wire, sizeof(wire)), -1);
  EXPECT_EQ(wire[0], '\0');
  ASSERT_GT(adapter.prepare(ticket, 0, 1.02, wire, sizeof(wire)), 0);
  const char * payload = std::strstr(wire, "Joint "); ASSERT_NE(payload, nullptr);
  auto * object = json_tokener_parse(payload + 6); ASSERT_NE(object, nullptr);
  json_object * robot = nullptr, * joints = nullptr;
  ASSERT_TRUE(json_object_object_get_ex(object, "Robot0", &robot));
  ASSERT_TRUE(json_object_object_get_ex(robot, "joint", &joints));
  EXPECT_DOUBLE_EQ(json_object_get_double(json_object_array_get_idx(joints, 0)), 0.0);
  json_object_put(object);
  EXPECT_EQ(adapter.prepare(ticket, 0, 1.025, wire, sizeof(wire)), -1);
  EXPECT_EQ(adapter.prepare(ticket, 1, 1.03, wire, sizeof(wire)), 0);
  EXPECT_EQ(client.wire_sequence, 3u);
  client.acknowledged = 3; // synthetic transport ACK, no SDK/network
  EXPECT_GT(adapter.prepare(ticket, 1, 1.03, wire, sizeof(wire)), 0);
  EXPECT_EQ(client.wire_sequence, 4u);
}

TEST(Je2RosGoalAdapter, OldCallbackCancelAndFailureCannotAffectNewSession)
{
  je2_client client{}; active_client(client);
  Adapter adapter(client, names, limits());
  const auto old = adapter.bind(goal(), 1.02); ASSERT_TRUE(old);
  active_client(client); // same boot/numeric generation, NEW random client ID
  const auto current = adapter.bind(goal(), 1.02); ASSERT_TRUE(current);
  char wire[MK2_FRAME_MAX]{};
  const auto before = client;
  EXPECT_EQ(adapter.prepare(old, 0, std::numeric_limits<double>::quiet_NaN(), wire, sizeof(wire)), -1);
  EXPECT_FALSE(adapter.cancel(old));
  EXPECT_FALSE(adapter.send_failed(old));
  EXPECT_EQ(std::memcmp(&before, &client, sizeof(client)), 0);
  EXPECT_TRUE(adapter.has_goal());
  EXPECT_GT(adapter.prepare(current, 0, 1.02, wire, sizeof(wire)), 0);
}

TEST(Je2RosGoalAdapter, CancellationAndTransportFailureInvalidateAllOutput)
{
  for (bool transport_failure : {false, true}) {
    je2_client client{}; active_client(client);
    Adapter adapter(client, names, limits());
    const auto ticket = adapter.bind(goal(), 1.02); ASSERT_TRUE(ticket);
    EXPECT_TRUE(transport_failure ? adapter.send_failed(ticket) : adapter.cancel(ticket));
    EXPECT_EQ(client.phase, transport_failure ? JE2_CLIENT_FAULT : JE2_CLIENT_RETIRED);
    EXPECT_FALSE(adapter.has_goal());
    char wire[MK2_FRAME_MAX] = "old";
    EXPECT_EQ(adapter.prepare(ticket, 0, 1.03, wire, sizeof(wire)), -1);
    EXPECT_EQ(wire[0], '\0');
    EXPECT_FALSE(adapter.bind(goal(), 1.04));
  }
}

TEST(Je2RosGoalAdapter, TimeoutClearsBindingWithoutEmittingCleanup)
{
  je2_client client{}; active_client(client);
  Adapter adapter(client, names, limits());
  const auto ticket = adapter.bind(goal(), 1.02); ASSERT_TRUE(ticket);
  EXPECT_EQ(adapter.tick(1.20), -1);
  EXPECT_FALSE(adapter.has_goal());
  char wire[MK2_FRAME_MAX] = "old";
  EXPECT_EQ(adapter.prepare(ticket, 0, 1.21, wire, sizeof(wire)), -1);
  EXPECT_EQ(wire[0], '\0');
}

TEST(Je2RosGoalAdapter, RejectsUnsupportedOrMalformedRosGoalsWithoutArming)
{
  for (int kind = 0; kind < 13; ++kind) {
    je2_client client{}; active_client(client);
    Adapter adapter(client, names, limits());
    auto request = goal();
    switch (kind) {
      case 0: request.trajectory.joint_names[0] = "other"; break;
      case 1: request.trajectory.points[0].positions.pop_back(); break;
      case 2: request.trajectory.points[0].velocities.clear(); break;
      case 3: request.trajectory.points[0].positions[0] = std::numeric_limits<double>::quiet_NaN(); break;
      case 4: request.trajectory.points[1].time_from_start.nanosec = 0; break;
      case 5: request.trajectory.header.stamp.sec = 1; break;
      case 6: request.trajectory.points[0].time_from_start.sec = -1; break;
      case 7: request.trajectory.points[0].time_from_start.nanosec = 1000000000u; break;
      case 8: request.trajectory.points.resize(513); break;
      case 9: request.component_path_tolerance.resize(1); break;
      case 10: request.goal_tolerance.resize(1); break;
      case 11: request.trajectory.points[0].effort.resize(MK2_AXES); break;
      default: request.multi_dof_trajectory.header.frame_id = "unsupported"; break;
    }
    EXPECT_FALSE(adapter.bind(request, 1.02)) << kind;
    EXPECT_FALSE(adapter.has_goal());
    EXPECT_EQ(client.phase, JE2_CLIENT_ACTIVE);
    EXPECT_EQ(client.wire_sequence, 2u);
  }
}
TEST(Je2RosGoalAdapter, RejectsMotionViolationsOnEveryAxisBeforeBinding)
{
  for (std::size_t axis = 0; axis < MK2_AXES; ++axis) {
    for (int kind = 0; kind < 7; ++kind) {
      je2_client client{}; active_client(client);
      Adapter adapter(client, names, limits());
      auto request = goal();
      auto & point = request.trajectory.points[1];
      switch (kind) {
        case 0: point.positions[axis] = 1.01; break;
        case 1: point.positions[axis] = -1.01; break;
        case 2: point.velocities[axis] = .21; break;
        case 3: point.accelerations.assign(MK2_AXES, 0); point.accelerations[axis] = .41; break;
        case 4: point.positions[axis] = .003; break; // fd velocity .3
        case 5: point.positions[axis] = .0001; break; // fd acceleration 1
        default: request.trajectory.points.back().positions[axis] = 2; break;
      }
      EXPECT_FALSE(adapter.bind(request, 1.02)) << axis << ":" << kind;
      EXPECT_FALSE(adapter.has_goal());
      EXPECT_EQ(client.phase, JE2_CLIENT_ACTIVE);
      EXPECT_EQ(client.wire_sequence, 2u); // zero frames, including bad final point
    }
  }
}

TEST(Je2RosGoalAdapter, RejectsInvalidLimitsWithoutAnUnrestrictedFallback)
{
  for (std::size_t axis = 0; axis < MK2_AXES; ++axis) {
    for (int kind = 0; kind < 8; ++kind) {
      je2_client client{}; active_client(client);
      auto configuration = limits();
      const double nan = std::numeric_limits<double>::quiet_NaN();
      const double inf = std::numeric_limits<double>::infinity();
      switch (kind) {
        case 0: configuration.lower[axis] = nan; break;
        case 1: configuration.upper[axis] = inf; break;
        case 2: configuration.lower[axis] = configuration.upper[axis]; break;
        case 3: configuration.velocity[axis] = 0; break;
        case 4: configuration.velocity[axis] = inf; break;
        case 5: configuration.acceleration[axis] = -1; break;
        case 6: configuration.acceleration[axis] = nan; break;
        default: configuration.upper[axis] = configuration.lower[axis] - 1; break;
      }
      Adapter adapter(client, names, configuration);
      EXPECT_FALSE(adapter.bind(goal(), 1.02)) << axis << ":" << kind;
      EXPECT_EQ(client.wire_sequence, 2u);
    }
  }
}

TEST(Je2RosGoalAdapter, UsesEmittedHoldRatherThanLaterMeasuredFeedback)
{
  je2_client client{}; active_client(client);
  client.first_target[0] = .125;
  client.joint[0] = .15; // feedback is not the command finite-difference seed
  Adapter adapter(client, names, limits());
  auto request = goal();
  for (auto & point : request.trajectory.points) point.positions[0] = .125;
  const auto ticket = adapter.bind(request, 1.02); ASSERT_TRUE(ticket);
  char wire[MK2_FRAME_MAX];
  ASSERT_GT(adapter.prepare(ticket, 0, 1.02, wire, sizeof(wire)), 0);
  client.acknowledged = 3;
  client.joint[0] = -.75;
  EXPECT_GT(adapter.prepare(ticket, 1, 1.03, wire, sizeof(wire)), 0);
  EXPECT_EQ(client.first_target[0], .125);
}

TEST(Je2RosGoalAdapter, ValidMotionPreservesTargetsAndBackpressureHistory)
{
  je2_client client{}; active_client(client);
  Adapter adapter(client, names, limits());
  auto request = goal();
  // fd velocities .001, .002, .003 and accelerations .1 rad/s^2.
  const double positions[] = {.00001, .00003, .00006};
  for (std::size_t i = 0; i < 3; ++i) {
    request.trajectory.points[i].positions[0] = positions[i];
    request.trajectory.points[i].velocities[0] = (i + 1) * .001;
  }
  const auto ticket = adapter.bind(request, 1.02); ASSERT_TRUE(ticket);
  char wire[MK2_FRAME_MAX];
  ASSERT_GT(adapter.prepare(ticket, 0, 1.02, wire, sizeof(wire)), 0);
  EXPECT_EQ(adapter.prepare(ticket, 1, 1.03, wire, sizeof(wire)), 0);
  EXPECT_EQ(adapter.prepare(ticket, 1, 1.03, wire, sizeof(wire)), 0);
  EXPECT_EQ(client.wire_sequence, 3u);
  for (std::size_t i = 1; i < 3; ++i) {
    client.acknowledged = client.wire_sequence;
    ASSERT_GT(adapter.prepare(ticket, i, 1.02 + .01 * i, wire, sizeof(wire)), 0);
    auto * object = json_tokener_parse(std::strstr(wire, "Joint ") + 6);
    ASSERT_NE(object, nullptr);
    json_object * robot = nullptr, * joints = nullptr;
    ASSERT_TRUE(json_object_object_get_ex(object, "Robot0", &robot));
    ASSERT_TRUE(json_object_object_get_ex(robot, "joint", &joints));
    EXPECT_DOUBLE_EQ(json_object_get_double(json_object_array_get_idx(joints, 0)), positions[i]);
    json_object_put(object);
  }
}

TEST(Je2RosGoalAdapter, DetectsUntrackedWriterAndNeverCatchesUpLatePoints)
{
  for (int kind = 0; kind < 3; ++kind) {
    je2_client client{}; active_client(client);
    Adapter adapter(client, names, limits());
    const auto ticket = adapter.bind(goal(), 1.02); ASSERT_TRUE(ticket);
    if (kind == 0) ++client.wire_sequence;
    if (kind == 1) client.last_data_time += .01;
    const uint64_t sequence = client.wire_sequence;
    char wire[MK2_FRAME_MAX] = "old";
    EXPECT_EQ(adapter.prepare(ticket, 0, kind == 2 ? 1.09 : 1.02, wire, sizeof(wire)), -1);
    EXPECT_EQ(wire[0], '\0');
    EXPECT_EQ(client.phase, JE2_CLIENT_FAULT);
    EXPECT_FALSE(adapter.has_goal());
    EXPECT_EQ(client.wire_sequence, sequence);
  }
}

TEST(Je2RosGoalAdapter, InvalidHoldSeedOrEntryTimingCannotBeHiddenByGoalData)
{
  for (int kind = 0; kind < 4; ++kind) {
    je2_client client{}; active_client(client);
    if (kind == 0) client.first_target[0] = std::numeric_limits<double>::quiet_NaN();
    if (kind == 1) client.first_target[0] = 2;
    Adapter adapter(client, names, limits());
    auto request = goal();
    if (kind == 2) request.trajectory.points[0].positions[0] = .0001; // bad entry acceleration
    EXPECT_FALSE(adapter.bind(request, kind == 3 ? 1.01 : 1.02)); // zero entry dt
    EXPECT_EQ(client.wire_sequence, 2u);
  }
}
}  // namespace