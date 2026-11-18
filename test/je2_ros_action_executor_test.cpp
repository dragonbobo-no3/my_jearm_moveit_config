// Real ROS action server/client on loopback-only DDS; real JE2 parser/engine
// with memory motion callbacks. Simulated control time, NOT SDK/physical motion
// or a scheduler latency benchmark. Controller delivery is an explicit fake
// queue (existing separate tests cover actual inproc ZMQ).
#include "je2_ros_action_executor.hpp"
#include "je2_loopback_server.h"
#include <gtest/gtest.h>
#include <rmw/rmw.h>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/rtps/transport/UDPv4TransportDescriptor.h>
#include <fastdds/rtps/transport/shared_mem/SharedMemTransportDescriptor.h>
#include <cstdlib>
#include <deque>
#include <unistd.h>

namespace
{
using namespace std::chrono_literals;
using Adapter = je2_experiment::RosGoalAdapter;
using Execution = je2_experiment::RosGoalExecution;
using Endpoint = je2_experiment::RosActionExecutor;
using Action = Endpoint::Action;
using ClientHandle = rclcpp_action::ClientGoalHandle<Action>;
using Code = rclcpp_action::ResultCode;
const Adapter::Names names{"j1", "j2", "j3", "j4", "j5", "j6", "j7"};

class ActionLoopback : public ::testing::Test
{
protected:
  je2_loopback_server controller{};
  je2_client session{};
  std::unique_ptr<Adapter> adapter;
  rclcpp::Context::SharedPtr context;
  rclcpp::Node::SharedPtr server_node, client_node;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> spin;
  std::unique_ptr<Endpoint> endpoint;
  rclcpp_action::Client<Action>::SharedPtr client;
  std::deque<std::string> wires;
  std::vector<Action::Feedback> feedback;
  double time{1.0};
  unsigned send_attempts{}, result_callbacks{};
  bool fail_send{};

  void check_loopback_transport(const eprosima::fastdds::dds::DomainParticipantQos & qos, bool runtime = false)
  {
    ASSERT_FALSE(qos.transport().use_builtin_transports);
    ASSERT_FALSE(qos.transport().user_transports.empty());
    if (!runtime) { ASSERT_EQ(qos.transport().user_transports.size(), 1u); }
    unsigned udp_count = 0;
    for (const auto & transport : qos.transport().user_transports) {
      if (const auto udp = std::dynamic_pointer_cast<eprosima::fastdds::rtps::UDPv4TransportDescriptor>(transport)) {
        ASSERT_EQ(udp->interfaceWhiteList, std::vector<std::string>{"127.0.0.1"});
        ++udp_count;
      } else {
        // Humble's localhost-only RMW adds its own loopback UDP and local SHM.
        // Validate EVERY transport; never allow unknown or unrestricted sockets.
        ASSERT_TRUE(runtime);
        ASSERT_TRUE(std::dynamic_pointer_cast<eprosima::fastdds::rtps::SharedMemTransportDescriptor>(transport));
      }
    }
    ASSERT_GT(udp_count, 0u);
    ASSERT_TRUE(qos.wire_protocol().builtin.metatrafficMulticastLocatorList.empty());
  }
  void SetUp() override
  {
    // Fail BEFORE creating any ROS context if CTest isolation was bypassed.
    ASSERT_STREQ(std::getenv("ROS_LOCALHOST_ONLY"), "1");
    ASSERT_STREQ(std::getenv("RMW_IMPLEMENTATION"), "rmw_fastrtps_cpp");
    ASSERT_STREQ(std::getenv("FASTRTPS_DEFAULT_PROFILES_FILE"), JE2_ACTION_TEST_PROFILE_PATH);
    // Fast DDS otherwise logs XML errors and silently uses default profiles.
    // Validate loading AND the parsed transport before creating ROS entities.
    auto * factory = eprosima::fastdds::dds::DomainParticipantFactory::get_instance();
    ASSERT_EQ(factory->load_XML_profiles_file(JE2_ACTION_TEST_PROFILE_PATH), ReturnCode_t::RETCODE_OK);
    eprosima::fastdds::dds::DomainParticipantQos profile;
    ASSERT_EQ(factory->get_participant_qos_from_profile("je2_action_test_only", profile), ReturnCode_t::RETCODE_OK);
    ASSERT_NO_FATAL_FAILURE(check_loopback_transport(profile));
    ASSERT_EQ(je2_loopback_init(&controller), 0);
    ASSERT_EQ(je2_client_init(&session), 0);
    Adapter::Limits limits;
    limits.lower.fill(-1); limits.upper.fill(1);
    limits.velocity.fill(.2); limits.acceleration.fill(.4);
    adapter = std::make_unique<Adapter>(session, names, limits);
    Execution::Policy policy;
    policy.position_tolerance.fill(.0001);
    policy.max_lateness = .002; policy.max_result_age = .05;
    context = std::make_shared<rclcpp::Context>();
    rclcpp::InitOptions init;
    init.shutdown_on_signal = false;
    init.auto_initialize_logging(false);
    init.set_domain_id(217); // explicit, independent of inherited ROS_DOMAIN_ID
    context->init(0, nullptr, init);
    ASSERT_STREQ(rmw_get_implementation_identifier(), "rmw_fastrtps_cpp");
    rclcpp::NodeOptions options;
    options.context(context).use_global_arguments(false).enable_rosout(false)
      .start_parameter_services(false).start_parameter_event_publisher(false);
    static unsigned instance = 0;
    const std::string ns = "/je2_action_test_" + std::to_string(getpid()) + "_" + std::to_string(++instance);
    server_node = std::make_shared<rclcpp::Node>("executor", ns, options);
    client_node = std::make_shared<rclcpp::Node>("client", ns, options);
    // Verify RMW did not replace the whitelist with builtin/default transports.
    const auto participants = factory->lookup_participants(217);
    ASSERT_FALSE(participants.empty());
    for (const auto * participant : participants)
      ASSERT_NO_FATAL_FAILURE(check_loopback_transport(participant->get_qos(), true));
    endpoint = std::make_unique<Endpoint>(server_node, "experimental_follow_joint_trajectory", *adapter,
      policy, [this] { return time; }, [this](const char * wire, std::size_t size) {
        ++send_attempts;
        if (fail_send) return false;
        wires.emplace_back(wire, size);
        return true; // transport enqueue, NOT simulated controller acknowledgement
      });
    client = rclcpp_action::create_client<Action>(client_node, "experimental_follow_joint_trajectory");
    rclcpp::ExecutorOptions executor_options; executor_options.context = context;
    spin = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(executor_options);
    spin->add_node(server_node); spin->add_node(client_node);
    ASSERT_TRUE(client->wait_for_action_server(2s));
  }
  void TearDown() override
  {
    if (endpoint) endpoint->shutdown();
    endpoint.reset();
    if (spin) spin->cancel();
    spin.reset(); client.reset(); client_node.reset(); server_node.reset();
    if (context && context->is_valid()) context->shutdown("test complete");
    context.reset();
  }
  int observe(double at)
  {
    time = at;
    char wire[MK2_FRAME_MAX];
    const int size = je2_loopback_snapshot(&controller, at, wire, sizeof(wire));
    EXPECT_GT(size, 0);
    return size > 0 ? endpoint->observe(wire, static_cast<std::size_t>(size)) : -99;
  }
  int deliver(double at)
  {
    EXPECT_FALSE(wires.empty());
    if (wires.empty()) return -99;
    const auto wire = wires.front(); wires.pop_front();
    return je2_loopback_receive(&controller, wire.data(), wire.size(), false, at);
  }
  void handshake(double base = 1.0)
  {
    ASSERT_EQ(observe(base), 0);
    char wire[MK2_FRAME_MAX];
    for (int i = 0; i < 5; ++i) {
      const double at = base + .01 + .03 * i;
      const int n = je2_client_claim(&session, at, wire, sizeof(wire));
      ASSERT_GT(n, 0);
      ASSERT_EQ(je2_loopback_receive(&controller, wire, n, false, at), i == 4 ? 0 : -1);
      ASSERT_EQ(observe(at + .001), 0);
    }
    controller.sample.joint[0] = .25;
    int n = je2_client_switch(&session, base + .14, wire, sizeof(wire)); ASSERT_GT(n, 0);
    ASSERT_EQ(je2_loopback_receive(&controller, wire, n, false, base + .14), 0);
    ASSERT_EQ(observe(base + .141), 0);
    n = je2_client_hold(&session, base + .15, .01, wire, sizeof(wire)); ASSERT_GT(n, 0);
    ASSERT_EQ(je2_loopback_receive(&controller, wire, n, false, base + .15), 0);
    ASSERT_EQ(observe(base + .151), 0);
    ASSERT_EQ(session.phase, JE2_CLIENT_ACTIVE); ASSERT_EQ(session.acknowledged, 2u);
    time = base + .16;
  }
  Action::Goal goal() const
  {
    Action::Goal goal;
    goal.trajectory.joint_names.assign(names.begin(), names.end());
    for (unsigned i = 0; i < 4; ++i) {
      trajectory_msgs::msg::JointTrajectoryPoint p;
      for (double q : session.first_target) p.positions.push_back(q + .00001 * (i == 3 ? 3 : i + 1));
      p.velocities.assign(MK2_AXES, i == 3 ? 0 : .001); p.accelerations.assign(MK2_AXES, 0);
      p.time_from_start.nanosec = i * 10000000;
      goal.trajectory.points.push_back(p);
    }
    return goal;
  }
  ClientHandle::SharedPtr submit(const Action::Goal & goal)
  {
    rclcpp_action::Client<Action>::SendGoalOptions options;
    options.feedback_callback = [this](ClientHandle::SharedPtr, std::shared_ptr<const Action::Feedback> f) {
      feedback.push_back(*f);
    };
    options.result_callback = [this](const ClientHandle::WrappedResult &) { ++result_callbacks; };
    auto future = client->async_send_goal(goal, options);
    EXPECT_EQ(spin->spin_until_future_complete(future, 2s), rclcpp::FutureReturnCode::SUCCESS);
    if (future.wait_for(0s) != std::future_status::ready) return {};
    return future.get();
  }
  void cancel(const ClientHandle::SharedPtr & handle, bool accepted = true)
  {
    auto future = client->async_cancel_goal(handle);
    ASSERT_EQ(spin->spin_until_future_complete(future, 2s), rclcpp::FutureReturnCode::SUCCESS);
    const auto response = future.get();
    if (accepted) {
      ASSERT_EQ(response->return_code, action_msgs::srv::CancelGoal::Response::ERROR_NONE);
      ASSERT_EQ(response->goals_canceling.size(), 1u);
      EXPECT_EQ(response->goals_canceling.front().goal_id.uuid, handle->get_goal_id());
    } else {
      EXPECT_TRUE(response->goals_canceling.empty());
    }
  }
  template<class Future>
  void pending(Future & future)
  {
    // Spin a bounded interval to prove the result was not simply left queued.
    EXPECT_EQ(spin->spin_until_future_complete(future, 5ms), rclcpp::FutureReturnCode::TIMEOUT);
    EXPECT_EQ(result_callbacks, 0u);
  }
  template<class Future>
  void result(Future & future, Code code, int error, const std::string & reason)
  {
    ASSERT_EQ(spin->spin_until_future_complete(future, 2s), rclcpp::FutureReturnCode::SUCCESS);
    const auto wrapped = future.get();
    EXPECT_EQ(wrapped.code, code); ASSERT_NE(wrapped.result, nullptr);
    EXPECT_EQ(wrapped.result->error_code, error);
    EXPECT_NE(wrapped.result->error_string.find(reason), std::string::npos);
    EXPECT_EQ(result_callbacks, 1u);
  }
  double stream()
  {
    const auto request = goal();
    for (const auto & p : request.trajectory.points) {
      time = 1.16 + p.time_from_start.nanosec * 1e-9;
      endpoint->poll();
      EXPECT_EQ(wires.size(), 1u);
      EXPECT_EQ(deliver(time), 0);
      EXPECT_EQ(observe(time + .001), 0);
    }
    time += .001;
    endpoint->poll();
    EXPECT_EQ(wires.size(), 1u);
    const double released = time;
    EXPECT_EQ(deliver(released), 0);
    EXPECT_EQ(observe(released + .001), 0);
    EXPECT_EQ(session.phase, JE2_CLIENT_CLOSING);
    return released;
  }
  template<class Future>
  void finish_close(double released, Future & future)
  {
    for (double offset : {.01, .12}) {
      je2_loopback_tick(&controller, released + offset);
      ASSERT_EQ(observe(released + offset + .001), 0);
      endpoint->poll();
      ASSERT_NO_FATAL_FAILURE(pending(future));
    }
    ASSERT_EQ(controller.ends, 1u);
    ASSERT_EQ(controller.engine.owner, JE2_EXIT_BARRIER); // End is NOT writer exit
    controller.writer = JE2_WRITER_EXITED; // explicit fake instance only
    for (double offset : {.13, .14, .25}) {
      je2_loopback_tick(&controller, released + offset);
      ASSERT_EQ(observe(released + offset + .001), 0);
      if (offset < .25) { endpoint->poll(); ASSERT_NO_FATAL_FAILURE(pending(future)); }
    }
    ASSERT_EQ(session.phase, JE2_CLIENT_CLOSED);
    ASSERT_EQ(controller.stops, 0u); ASSERT_EQ(controller.jogs, 0u);
  }
};

TEST_F(ActionLoopback, GoalFeedbackAndSuccessRequireCloseReceipt)
{
  ASSERT_NO_FATAL_FAILURE(handshake());
  auto handle = submit(goal()); ASSERT_TRUE(handle);
  auto future = client->async_get_result(handle);
  ASSERT_NO_FATAL_FAILURE(pending(future));
  const double released = stream();
  ASSERT_NO_FATAL_FAILURE(pending(future));
  ASSERT_FALSE(feedback.empty());
  for (const auto & f : feedback) {
    EXPECT_EQ(f.joint_names, goal().trajectory.joint_names);
    EXPECT_EQ(f.actual.positions.size(), 7u); EXPECT_EQ(f.error.positions.size(), 7u);
    EXPECT_TRUE(f.actual.velocities.empty()); EXPECT_TRUE(f.actual.accelerations.empty());
  }
  ASSERT_NO_FATAL_FAILURE(finish_close(released, future));
  endpoint->poll();
  ASSERT_NO_FATAL_FAILURE(result(future, Code::SUCCEEDED, Action::Result::SUCCESSFUL, "close_confirmed"));
  for (int i = 0; i < 3; ++i) endpoint->poll();
  spin->spin_some(5ms);
  EXPECT_EQ(result_callbacks, 1u); EXPECT_EQ(send_attempts, 5u);
  EXPECT_EQ(controller.joints, 5u); EXPECT_EQ(controller.releases, 1u);
  EXPECT_FALSE(submit(goal())); // CLOSED is not another execution session
}

TEST_F(ActionLoopback, NotReadyInvalidGoalAndBusyAreRejectedWithoutPreemption)
{
  EXPECT_FALSE(submit(goal())); EXPECT_EQ(send_attempts, 0u);
  ASSERT_NO_FATAL_FAILURE(handshake());
  auto invalid = goal(); invalid.trajectory.joint_names[0] = "wrong_joint";
  EXPECT_FALSE(submit(invalid));
  auto handle = submit(goal()); ASSERT_TRUE(handle);
  EXPECT_FALSE(submit(goal()));
  EXPECT_EQ(session.phase, JE2_CLIENT_ACTIVE); EXPECT_EQ(session.wire_sequence, 2u);
  EXPECT_EQ(send_attempts, 0u);
  auto future = client->async_get_result(handle);
  endpoint->shutdown();
  ASSERT_NO_FATAL_FAILURE(result(future, Code::ABORTED, Action::Result::PATH_TOLERANCE_VIOLATED, "shutdown"));
}

TEST_F(ActionLoopback, CancelWaitsForOutstandingAckAndWriterExit)
{
  ASSERT_NO_FATAL_FAILURE(handshake());
  auto handle = submit(goal()); ASSERT_TRUE(handle);
  auto future = client->async_get_result(handle);
  endpoint->poll(); ASSERT_EQ(wires.size(), 1u); // withhold target delivery
  time = 1.161; ASSERT_NO_FATAL_FAILURE(cancel(handle));
  ASSERT_EQ(session.phase, JE2_CLIENT_DRAINING); EXPECT_EQ(send_attempts, 1u);
  ASSERT_NO_FATAL_FAILURE(pending(future));
  ASSERT_EQ(deliver(1.162), 0); ASSERT_EQ(observe(1.163), 0);
  time = 1.164; endpoint->poll(); ASSERT_EQ(wires.size(), 1u);
  ASSERT_EQ(deliver(time), 0); ASSERT_EQ(observe(1.165), 0);
  ASSERT_NO_FATAL_FAILURE(finish_close(1.164, future));
  endpoint->poll();
  ASSERT_NO_FATAL_FAILURE(result(future, Code::CANCELED, Action::Result::SUCCESSFUL, "close_confirmed"));
  EXPECT_EQ(controller.joints, 2u); EXPECT_EQ(send_attempts, 2u);
}

TEST_F(ActionLoopback, FaultAfterAcceptedCancelIsAbortedNotCanceled)
{
  ASSERT_NO_FATAL_FAILURE(handshake());
  auto handle = submit(goal()); ASSERT_TRUE(handle);
  auto future = client->async_get_result(handle);
  ASSERT_NO_FATAL_FAILURE(cancel(handle));
  ASSERT_EQ(deliver(time), 0);
  controller.sample.move_state = 7;
  je2_loopback_tick(&controller, 1.161);
  ASSERT_EQ(observe(1.162), -1);
  endpoint->poll();
  ASSERT_NO_FATAL_FAILURE(result(future, Code::ABORTED, Action::Result::PATH_TOLERANCE_VIOLATED, "control_fault"));
  EXPECT_EQ(send_attempts, 1u); EXPECT_EQ(controller.joints, 1u);
}

TEST_F(ActionLoopback, OwnershipLossAbortsWithoutStaleRelease)
{
  ASSERT_NO_FATAL_FAILURE(handshake());
  auto handle = submit(goal()); ASSERT_TRUE(handle);
  auto future = client->async_get_result(handle);
  ASSERT_EQ(je2_loopback_button(&controller, "tv49!", 5, 1.161), 0);
  ASSERT_EQ(observe(1.162), -1);
  endpoint->poll();
  ASSERT_NO_FATAL_FAILURE(result(future, Code::ABORTED, Action::Result::PATH_TOLERANCE_VIOLATED, "control_lost"));
  EXPECT_EQ(send_attempts, 0u); EXPECT_TRUE(wires.empty());
}

TEST_F(ActionLoopback, ClosedOutsideGoalToleranceUsesStandardNegativeResult)
{
  ASSERT_NO_FATAL_FAILURE(handshake());
  auto handle = submit(goal()); ASSERT_TRUE(handle);
  auto future = client->async_get_result(handle);
  const double released = stream();
  controller.sample.joint[0] += .001;
  ASSERT_NO_FATAL_FAILURE(finish_close(released, future));
  endpoint->poll();
  ASSERT_NO_FATAL_FAILURE(result(future, Code::ABORTED, Action::Result::GOAL_TOLERANCE_VIOLATED, "goal_tolerance"));
}

TEST_F(ActionLoopback, FirstPollLateDoesNotRetimeGoal)
{
  ASSERT_NO_FATAL_FAILURE(handshake());
  auto handle = submit(goal()); ASSERT_TRUE(handle);
  auto future = client->async_get_result(handle);
  time = 1.163; endpoint->poll();
  ASSERT_NO_FATAL_FAILURE(result(future, Code::ABORTED, Action::Result::PATH_TOLERANCE_VIOLATED, "schedule_expired"));
  EXPECT_EQ(send_attempts, 0u); EXPECT_EQ(controller.joints, 1u);
}

TEST_F(ActionLoopback, TargetSendFailureAbortsExactlyOnceWithoutRetry)
{
  ASSERT_NO_FATAL_FAILURE(handshake());
  auto handle = submit(goal()); ASSERT_TRUE(handle);
  auto future = client->async_get_result(handle);
  fail_send = true; endpoint->poll();
  ASSERT_NO_FATAL_FAILURE(result(future, Code::ABORTED, Action::Result::PATH_TOLERANCE_VIOLATED, "send_failed"));
  endpoint->poll(); EXPECT_EQ(send_attempts, 1u); EXPECT_TRUE(wires.empty());
}

TEST_F(ActionLoopback, CancelReleaseSendFailureAbortsAfterFrameworkTransition)
{
  ASSERT_NO_FATAL_FAILURE(handshake());
  auto handle = submit(goal()); ASSERT_TRUE(handle);
  auto future = client->async_get_result(handle);
  fail_send = true; ASSERT_NO_FATAL_FAILURE(cancel(handle));
  ASSERT_NO_FATAL_FAILURE(pending(future));
  endpoint->poll();
  ASSERT_NO_FATAL_FAILURE(result(future, Code::ABORTED, Action::Result::PATH_TOLERANCE_VIOLATED, "send_failed"));
  EXPECT_EQ(send_attempts, 1u); EXPECT_EQ(controller.releases, 0u);
}

TEST_F(ActionLoopback, CancelAfterCloseReceiptCannotRewriteSuccess)
{
  ASSERT_NO_FATAL_FAILURE(handshake());
  auto handle = submit(goal()); ASSERT_TRUE(handle);
  auto future = client->async_get_result(handle);
  const double released = stream();
  ASSERT_NO_FATAL_FAILURE(finish_close(released, future));
  ASSERT_NO_FATAL_FAILURE(cancel(handle, false));
  endpoint->poll();
  ASSERT_NO_FATAL_FAILURE(result(future, Code::SUCCEEDED, Action::Result::SUCCESSFUL, "close_confirmed"));
  EXPECT_EQ(send_attempts, 5u);
}

TEST_F(ActionLoopback, ShutdownDuringAcceptedCancelIsNotFakeCanceled)
{
  ASSERT_NO_FATAL_FAILURE(handshake());
  auto handle = submit(goal()); ASSERT_TRUE(handle);
  auto future = client->async_get_result(handle);
  ASSERT_NO_FATAL_FAILURE(cancel(handle));
  endpoint->shutdown();
  ASSERT_NO_FATAL_FAILURE(result(future, Code::ABORTED, Action::Result::PATH_TOLERANCE_VIOLATED, "shutdown"));
  EXPECT_EQ(send_attempts, 1u); EXPECT_EQ(controller.ends, 0u);
}

TEST_F(ActionLoopback, ForeignThreadIsRejectedBeforeClientMutation)
{
  ASSERT_NO_FATAL_FAILURE(handshake());
  const auto before = session.wire_sequence;
  bool rejected = false;
  std::thread foreign([&] { try { endpoint->poll(); } catch (const std::logic_error &) { rejected = true; } });
  foreign.join();
  EXPECT_TRUE(rejected); EXPECT_EQ(session.wire_sequence, before); EXPECT_EQ(send_attempts, 0u);
}

TEST_F(ActionLoopback, OldCancelUuidCannotCancelNewSessionGoal)
{
  ASSERT_NO_FATAL_FAILURE(handshake());
  auto old = submit(goal()); ASSERT_TRUE(old);
  auto old_result = client->async_get_result(old);
  fail_send = true; endpoint->poll();
  ASSERT_NO_FATAL_FAILURE(result(old_result, Code::ABORTED, Action::Result::PATH_TOLERANCE_VIOLATED, "send_failed"));
  // New fake worker lifetime, NOT a second ServoStart on the existing worker.
  ASSERT_EQ(je2_loopback_init(&controller), 0);
  ASSERT_EQ(je2_client_init(&session), 0);
  fail_send = false; result_callbacks = 0;
  ASSERT_NO_FATAL_FAILURE(handshake(2.0));
  auto current = submit(goal()); ASSERT_TRUE(current);
  ASSERT_NE(current->get_goal_id(), old->get_goal_id());
  auto current_result = client->async_get_result(current);
  // Send the stale UUID over the real cancel service, bypassing only the
  // client's local terminal-handle cache, not the server's UUID checks.
  auto cancel_client = client_node->create_client<action_msgs::srv::CancelGoal>(
    "experimental_follow_joint_trajectory/_action/cancel_goal");
  ASSERT_TRUE(cancel_client->wait_for_service(2s));
  auto request = std::make_shared<action_msgs::srv::CancelGoal::Request>();
  request->goal_info.goal_id.uuid = old->get_goal_id();
  auto response = cancel_client->async_send_request(request);
  ASSERT_EQ(spin->spin_until_future_complete(response, 2s), rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_TRUE(response.get()->goals_canceling.empty());
  EXPECT_EQ(session.phase, JE2_CLIENT_ACTIVE); EXPECT_EQ(session.wire_sequence, 2u);
  EXPECT_EQ(send_attempts, 1u); // only the old failed send
  ASSERT_NO_FATAL_FAILURE(pending(current_result));
  endpoint->poll(); ASSERT_EQ(wires.size(), 1u);
  ASSERT_EQ(deliver(time), 0); ASSERT_EQ(observe(time + .001), 0);
  EXPECT_EQ(controller.joints, 2u);
  endpoint->shutdown();
  ASSERT_NO_FATAL_FAILURE(result(current_result, Code::ABORTED, Action::Result::PATH_TOLERANCE_VIOLATED, "shutdown"));
}

TEST_F(ActionLoopback, DuplicatePollDoesNotRepublishObservationOrTarget)
{
  ASSERT_NO_FATAL_FAILURE(handshake());
  auto handle = submit(goal()); ASSERT_TRUE(handle);
  auto future = client->async_get_result(handle);
  endpoint->poll(); ASSERT_EQ(deliver(time), 0); ASSERT_EQ(observe(1.161), 0);
  endpoint->poll();
  ASSERT_NO_FATAL_FAILURE(pending(future));
  ASSERT_FALSE(feedback.empty());
  const auto count = feedback.size();
  for (int i = 0; i < 3; ++i) endpoint->poll();
  ASSERT_NO_FATAL_FAILURE(pending(future));
  EXPECT_EQ(feedback.size(), count); EXPECT_EQ(send_attempts, 1u);
  endpoint->shutdown();
  ASSERT_NO_FATAL_FAILURE(result(future, Code::ABORTED, Action::Result::PATH_TOLERANCE_VIOLATED, "shutdown"));
}
}  // namespace