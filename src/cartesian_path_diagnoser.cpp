#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <future>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/contact_information.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit_msgs/srv/get_cartesian_path.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <moveit_msgs/srv/get_state_validity.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class CartesianPathDiagnoser : public rclcpp::Node {
public:
  CartesianPathDiagnoser()
  : rclcpp::Node("cartesian_path_diagnoser")
  {
    service_name_ = this->declare_parameter<std::string>("service_name", "/compute_cartesian_path");
    raw_service_name_ = this->declare_parameter<std::string>("raw_service_name", "/compute_cartesian_path_raw");
    ik_service_name_ = this->declare_parameter<std::string>("ik_service_name", "/compute_ik");
    state_validity_service_name_ = this->declare_parameter<std::string>(
      "state_validity_service_name", "/check_state_validity");
    service_timeout_sec_ = this->declare_parameter<double>("service_timeout_sec", 3.0);
    ik_timeout_sec_ = this->declare_parameter<double>("ik_timeout_sec", 0.05);

    service_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    client_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    ik_client_ = this->create_client<moveit_msgs::srv::GetPositionIK>(
      ik_service_name_,
      rmw_qos_profile_services_default,
      client_callback_group_);
    state_validity_client_ =
      this->create_client<moveit_msgs::srv::GetStateValidity>(
      state_validity_service_name_,
      rmw_qos_profile_services_default,
      client_callback_group_);
    cartesian_client_ = this->create_client<moveit_msgs::srv::GetCartesianPath>(
      raw_service_name_,
      rmw_qos_profile_services_default,
      client_callback_group_);

    service_ = this->create_service<moveit_msgs::srv::GetCartesianPath>(
      service_name_,
      std::bind(
        &CartesianPathDiagnoser::handleRequest,
        this,
        std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default,
      service_callback_group_);

    RCLCPP_INFO(
      this->get_logger(),
      "Cartesian path diagnoser proxy ready: %s -> %s",
      service_name_.c_str(),
      raw_service_name_.c_str());
  }

private:
  struct IkAttempt
  {
    bool success = false;
    moveit_msgs::msg::RobotState solution;
    int32_t error_code = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
  };

  struct StateCheck
  {
    bool valid = false;
    std::vector<moveit_msgs::msg::ContactInformation> contacts;
    std::vector<moveit_msgs::msg::ConstraintEvalResult> constraint_results;
  };

  void handleRequest(
    const std::shared_ptr<moveit_msgs::srv::GetCartesianPath::Request> request,
    std::shared_ptr<moveit_msgs::srv::GetCartesianPath::Response> response)
  {
    if (!waitForService(cartesian_client_, raw_service_name_) ||
      !waitForService(ik_client_, ik_service_name_) ||
      !waitForService(state_validity_client_, state_validity_service_name_))
    {
      response->error_code.val = moveit_msgs::msg::MoveItErrorCodes::COMMUNICATION_FAILURE;
      response->fraction = 0.0;
      return;
    }

    auto forwarded_request = std::make_shared<moveit_msgs::srv::GetCartesianPath::Request>(*request);
    auto future = cartesian_client_->async_send_request(forwarded_request);
    if (future.wait_for(serviceTimeout()) != std::future_status::ready) {
      RCLCPP_ERROR(this->get_logger(), "Timed out waiting for %s", raw_service_name_.c_str());
      response->error_code.val = moveit_msgs::msg::MoveItErrorCodes::TIMED_OUT;
      response->fraction = 0.0;
      return;
    }

    *response = *future.get();
    if (response->fraction >= 0.999999 || request->waypoints.empty()) {
      return;
    }

    diagnoseFailure(*request, *response);
  }

  template <typename ServiceT>
  bool waitForService(const std::shared_ptr<rclcpp::Client<ServiceT>> & client, const std::string & name)
  {
    if (client->wait_for_service(serviceTimeout())) {
      return true;
    }

    RCLCPP_ERROR(this->get_logger(), "Service %s is unavailable", name.c_str());
    return false;
  }

  std::chrono::milliseconds serviceTimeout() const
  {
    return std::chrono::milliseconds(static_cast<int>(service_timeout_sec_ * 1000.0));
  }

  builtin_interfaces::msg::Duration ikTimeoutMsg() const
  {
    builtin_interfaces::msg::Duration timeout;
    timeout.sec = static_cast<int32_t>(ik_timeout_sec_);
    timeout.nanosec = static_cast<uint32_t>((ik_timeout_sec_ - timeout.sec) * 1e9);
    return timeout;
  }

  void diagnoseFailure(
    const moveit_msgs::srv::GetCartesianPath::Request & request,
    const moveit_msgs::srv::GetCartesianPath::Response & response)
  {
    const size_t waypoint_count = request.waypoints.size();
    const double raw_index = std::max(0.0, response.fraction) * static_cast<double>(waypoint_count);
    const size_t failing_waypoint_index = std::min(
      waypoint_count - 1,
      static_cast<size_t>(std::floor(raw_index)));

    RCLCPP_WARN(
      this->get_logger(),
      "Cartesian path stopped at %.2f%%. First failing requested waypoint is likely index %zu/%zu for link %s.",
      response.fraction * 100.0,
      failing_waypoint_index,
      waypoint_count - 1,
      request.link_name.empty() ? "<group tip>" : request.link_name.c_str());

    moveit_msgs::msg::RobotState seed_state = request.start_state;
    for (size_t index = 0; index < failing_waypoint_index; ++index) {
      const auto attempt = solveIk(seed_state, request, request.waypoints[index], moveit_msgs::msg::Constraints{}, false);
      if (!attempt.success) {
        RCLCPP_WARN(
          this->get_logger(),
          "Could not reconstruct seed state through waypoint %zu before diagnosis: %s",
          index,
          errorCodeToString(attempt.error_code).c_str());
        break;
      }
      seed_state = attempt.solution;
    }

    if (failing_waypoint_index == 0) {
      diagnoseSample(seed_state, request, request.waypoints.front(), 0, -1, 1);
      return;
    }

    const auto & previous_waypoint = request.waypoints[failing_waypoint_index - 1];
    const auto & failing_waypoint = request.waypoints[failing_waypoint_index];
    const double distance = euclideanDistance(previous_waypoint, failing_waypoint);
    const size_t samples = std::max<size_t>(1, static_cast<size_t>(std::ceil(distance / request.max_step)));

    RCLCPP_WARN(
      this->get_logger(),
      "Diagnosing segment %zu -> %zu using %zu interpolated samples (distance=%.4fm, max_step=%.4fm)",
      failing_waypoint_index - 1,
      failing_waypoint_index,
      samples,
      distance,
      request.max_step);

    moveit_msgs::msg::RobotState sample_seed = seed_state;
    for (size_t sample = 1; sample <= samples; ++sample) {
      const double ratio = static_cast<double>(sample) / static_cast<double>(samples);
      const auto pose = interpolatePose(previous_waypoint, failing_waypoint, ratio);
      const bool success = diagnoseSample(
        sample_seed,
        request,
        pose,
        failing_waypoint_index,
        static_cast<int>(sample),
        static_cast<int>(samples));
      if (!success) {
        return;
      }

      const auto constrained_attempt = solveIk(
        sample_seed,
        request,
        pose,
        request.path_constraints,
        request.avoid_collisions);
      if (constrained_attempt.success) {
        sample_seed = constrained_attempt.solution;
      }
    }

    RCLCPP_WARN(
      this->get_logger(),
      "The sampled segment did not reproduce a concrete failure. The truncation may depend on MoveIt's internal start pose interpolation or a different IK seed.");
  }

  bool diagnoseSample(
    const moveit_msgs::msg::RobotState & seed_state,
    const moveit_msgs::srv::GetCartesianPath::Request & request,
    const geometry_msgs::msg::Pose & pose,
    size_t waypoint_index,
    int sample_index,
    int sample_count)
  {
    const auto unconstrained = solveIk(seed_state, request, pose, moveit_msgs::msg::Constraints{}, false);
    if (!unconstrained.success) {
      RCLCPP_WARN(
        this->get_logger(),
        "Failure reason near waypoint %zu sample %d/%d: IK failed before collision checking (%s)",
        waypoint_index,
        sample_index,
        sample_count,
        errorCodeToString(unconstrained.error_code).c_str());
      logPose(pose);
      return false;
    }

    const auto collision_state = checkState(unconstrained.solution, request.group_name, moveit_msgs::msg::Constraints{});
    const auto constrained_state = checkState(unconstrained.solution, request.group_name, request.path_constraints);
    const auto requested_attempt = solveIk(
      seed_state,
      request,
      pose,
      request.path_constraints,
      request.avoid_collisions);

    if (request.avoid_collisions && !collision_state.valid && !collision_state.contacts.empty()) {
      RCLCPP_WARN(
        this->get_logger(),
        "Failure reason near waypoint %zu sample %d/%d: collision detected (%s)",
        waypoint_index,
        sample_index,
        sample_count,
        summarizeContacts(collision_state.contacts).c_str());
      logPose(pose);
      return false;
    }

    if (hasConstraintViolation(constrained_state)) {
      RCLCPP_WARN(
        this->get_logger(),
        "Failure reason near waypoint %zu sample %d/%d: path constraints violated",
        waypoint_index,
        sample_index,
        sample_count);
      logPose(pose);
      return false;
    }

    if (!requested_attempt.success) {
      const std::string qualifier = (!collision_state.valid && collision_state.contacts.empty())
        ? "state invalid without reported contacts; likely joint bounds or a solver-side validity check"
        : errorCodeToString(requested_attempt.error_code);
      RCLCPP_WARN(
        this->get_logger(),
        "Failure reason near waypoint %zu sample %d/%d: %s",
        waypoint_index,
        sample_index,
        sample_count,
        qualifier.c_str());
      logPose(pose);
      return false;
    }

    return true;
  }

  IkAttempt solveIk(
    const moveit_msgs::msg::RobotState & seed_state,
    const moveit_msgs::srv::GetCartesianPath::Request & request,
    const geometry_msgs::msg::Pose & pose,
    const moveit_msgs::msg::Constraints & constraints,
    bool avoid_collisions)
  {
    IkAttempt result;

    auto ik_request = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
    ik_request->ik_request.group_name = request.group_name;
    ik_request->ik_request.robot_state = seed_state;
    ik_request->ik_request.constraints = constraints;
    ik_request->ik_request.avoid_collisions = avoid_collisions;
    ik_request->ik_request.ik_link_name = request.link_name;
    ik_request->ik_request.pose_stamped.header = request.header;
    ik_request->ik_request.pose_stamped.pose = pose;
    ik_request->ik_request.timeout = ikTimeoutMsg();

    auto future = ik_client_->async_send_request(ik_request);
    if (future.wait_for(serviceTimeout()) != std::future_status::ready) {
      result.error_code = moveit_msgs::msg::MoveItErrorCodes::TIMED_OUT;
      return result;
    }

    const auto response = future.get();
    result.error_code = response->error_code.val;
    result.success = (response->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
    result.solution = response->solution;
    return result;
  }

  StateCheck checkState(
    const moveit_msgs::msg::RobotState & state,
    const std::string & group_name,
    const moveit_msgs::msg::Constraints & constraints)
  {
    StateCheck result;

    auto request = std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
    request->robot_state = state;
    request->group_name = group_name;
    request->constraints = constraints;

    auto future = state_validity_client_->async_send_request(request);
    if (future.wait_for(serviceTimeout()) != std::future_status::ready) {
      return result;
    }

    const auto response = future.get();
    result.valid = response->valid;
    result.contacts = response->contacts;
    result.constraint_results = response->constraint_result;
    return result;
  }

  static bool hasConstraintViolation(const StateCheck & state)
  {
    return std::any_of(
      state.constraint_results.begin(),
      state.constraint_results.end(),
      [](const auto & item) {
        return !item.result;
      });
  }

  static double euclideanDistance(
    const geometry_msgs::msg::Pose & from,
    const geometry_msgs::msg::Pose & to)
  {
    const double dx = to.position.x - from.position.x;
    const double dy = to.position.y - from.position.y;
    const double dz = to.position.z - from.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  static geometry_msgs::msg::Pose interpolatePose(
    const geometry_msgs::msg::Pose & from,
    const geometry_msgs::msg::Pose & to,
    double ratio)
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = from.position.x + (to.position.x - from.position.x) * ratio;
    pose.position.y = from.position.y + (to.position.y - from.position.y) * ratio;
    pose.position.z = from.position.z + (to.position.z - from.position.z) * ratio;
    pose.orientation = slerp(from.orientation, to.orientation, ratio);
    return pose;
  }

  static geometry_msgs::msg::Quaternion slerp(
    geometry_msgs::msg::Quaternion from,
    geometry_msgs::msg::Quaternion to,
    double ratio)
  {
    normalize(from);
    normalize(to);

    double dot = from.x * to.x + from.y * to.y + from.z * to.z + from.w * to.w;
    if (dot < 0.0) {
      dot = -dot;
      to.x = -to.x;
      to.y = -to.y;
      to.z = -to.z;
      to.w = -to.w;
    }

    geometry_msgs::msg::Quaternion result;
    if (dot > 0.9995) {
      result.x = from.x + ratio * (to.x - from.x);
      result.y = from.y + ratio * (to.y - from.y);
      result.z = from.z + ratio * (to.z - from.z);
      result.w = from.w + ratio * (to.w - from.w);
      normalize(result);
      return result;
    }

    const double theta_0 = std::acos(dot);
    const double theta = theta_0 * ratio;
    const double sin_theta = std::sin(theta);
    const double sin_theta_0 = std::sin(theta_0);
    const double scale_from = std::cos(theta) - dot * sin_theta / sin_theta_0;
    const double scale_to = sin_theta / sin_theta_0;

    result.x = scale_from * from.x + scale_to * to.x;
    result.y = scale_from * from.y + scale_to * to.y;
    result.z = scale_from * from.z + scale_to * to.z;
    result.w = scale_from * from.w + scale_to * to.w;
    normalize(result);
    return result;
  }

  static void normalize(geometry_msgs::msg::Quaternion & quaternion)
  {
    const double norm = std::sqrt(
      quaternion.x * quaternion.x +
      quaternion.y * quaternion.y +
      quaternion.z * quaternion.z +
      quaternion.w * quaternion.w);
    if (norm <= 1e-12) {
      quaternion.x = 0.0;
      quaternion.y = 0.0;
      quaternion.z = 0.0;
      quaternion.w = 1.0;
      return;
    }

    quaternion.x /= norm;
    quaternion.y /= norm;
    quaternion.z /= norm;
    quaternion.w /= norm;
  }

  static std::string summarizeContacts(const std::vector<moveit_msgs::msg::ContactInformation> & contacts)
  {
    if (contacts.empty()) {
      return "no contact details";
    }

    std::ostringstream stream;
    const size_t max_contacts = std::min<size_t>(3, contacts.size());
    for (size_t index = 0; index < max_contacts; ++index) {
      if (index > 0) {
        stream << "; ";
      }
      stream << contacts[index].contact_body_1 << " <-> " << contacts[index].contact_body_2;
    }
    if (contacts.size() > max_contacts) {
      stream << " (" << contacts.size() << " contacts)";
    }
    return stream.str();
  }

  void logPose(const geometry_msgs::msg::Pose & pose)
  {
    RCLCPP_WARN(
      this->get_logger(),
      "Failing pose: pos[%.4f, %.4f, %.4f] quat[%.4f, %.4f, %.4f, %.4f]",
      pose.position.x,
      pose.position.y,
      pose.position.z,
      pose.orientation.x,
      pose.orientation.y,
      pose.orientation.z,
      pose.orientation.w);
  }

  static std::string errorCodeToString(int32_t error_code)
  {
    switch (error_code) {
      case moveit_msgs::msg::MoveItErrorCodes::SUCCESS:
        return "SUCCESS";
      case moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED:
        return "PLANNING_FAILED";
      case moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN:
        return "INVALID_MOTION_PLAN";
      case moveit_msgs::msg::MoveItErrorCodes::TIMED_OUT:
        return "TIMED_OUT";
      case moveit_msgs::msg::MoveItErrorCodes::START_STATE_IN_COLLISION:
        return "START_STATE_IN_COLLISION";
      case moveit_msgs::msg::MoveItErrorCodes::START_STATE_VIOLATES_PATH_CONSTRAINTS:
        return "START_STATE_VIOLATES_PATH_CONSTRAINTS";
      case moveit_msgs::msg::MoveItErrorCodes::GOAL_IN_COLLISION:
        return "GOAL_IN_COLLISION";
      case moveit_msgs::msg::MoveItErrorCodes::GOAL_VIOLATES_PATH_CONSTRAINTS:
        return "GOAL_VIOLATES_PATH_CONSTRAINTS";
      case moveit_msgs::msg::MoveItErrorCodes::GOAL_CONSTRAINTS_VIOLATED:
        return "GOAL_CONSTRAINTS_VIOLATED";
      case moveit_msgs::msg::MoveItErrorCodes::GOAL_STATE_INVALID:
        return "GOAL_STATE_INVALID";
      case moveit_msgs::msg::MoveItErrorCodes::INVALID_GROUP_NAME:
        return "INVALID_GROUP_NAME";
      case moveit_msgs::msg::MoveItErrorCodes::INVALID_ROBOT_STATE:
        return "INVALID_ROBOT_STATE";
      case moveit_msgs::msg::MoveItErrorCodes::INVALID_LINK_NAME:
        return "INVALID_LINK_NAME";
      case moveit_msgs::msg::MoveItErrorCodes::FRAME_TRANSFORM_FAILURE:
        return "FRAME_TRANSFORM_FAILURE";
      case moveit_msgs::msg::MoveItErrorCodes::COMMUNICATION_FAILURE:
        return "COMMUNICATION_FAILURE";
      case moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION:
        return "NO_IK_SOLUTION";
      default:
        return "ERROR_CODE_" + std::to_string(error_code);
    }
  }

  std::string service_name_;
  std::string raw_service_name_;
  std::string ik_service_name_;
  std::string state_validity_service_name_;
  double service_timeout_sec_;
  double ik_timeout_sec_;

  rclcpp::Client<moveit_msgs::srv::GetCartesianPath>::SharedPtr cartesian_client_;
  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr ik_client_;
  rclcpp::Client<moveit_msgs::srv::GetStateValidity>::SharedPtr state_validity_client_;
  rclcpp::Service<moveit_msgs::srv::GetCartesianPath>::SharedPtr service_;
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  rclcpp::CallbackGroup::SharedPtr client_callback_group_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CartesianPathDiagnoser>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}