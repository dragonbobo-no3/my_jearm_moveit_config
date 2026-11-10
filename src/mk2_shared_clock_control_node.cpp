#include <atomic>
#include <chrono>
#include <cstdlib>
#include <exception>
#include <memory>
#include <stdexcept>
#include <thread>
#include <vector>

#include <controller_manager/controller_manager.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_helpers.hpp>

#include "mk2_shared_clock_cycle.hpp"

// MK2 launch-only entry point. Uses the installed controller manager, JTC and
// hardware plugins without modifying their limits or lifecycle. Unlike Humble's
// stock entry point, all three stages receive ONE sampled ROS timestamp.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto manager = std::make_shared<controller_manager::ControllerManager>(
      executor, "controller_manager");
    if (manager->get_parameter_or<bool>("use_sim_time", false)) {
      throw std::runtime_error("MK2 SDK transport requires wall-time ROS clock; use_sim_time is forbidden");
    }
    const auto rate = manager->get_update_rate();
    if (rate != 100) {
      throw std::runtime_error("MK2 shared-clock entry requires the configured 100 Hz update rate");
    }
    if (manager->get_parameter_or<bool>("lock_memory", realtime_tools::has_realtime_kernel())) {
      const auto locked = realtime_tools::lock_memory();
      if (!locked.first) {
        RCLCPP_WARN(manager->get_logger(), "Memory lock unavailable: %s", locked.second.c_str());
      }
    }
    executor->add_node(manager);
    std::atomic<bool> failed{false};
    std::thread control_thread([&]() {
      try {
        rclcpp::Parameter affinity;
        if (manager->get_parameter("cpu_affinity", affinity)) {
          std::vector<int> cpus;
          if (affinity.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
            cpus.push_back(static_cast<int>(affinity.as_int()));
          } else if (affinity.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
            for (const auto cpu : affinity.as_integer_array()) cpus.push_back(static_cast<int>(cpu));
          } else {
            throw std::runtime_error("invalid cpu_affinity parameter type");
          }
          const auto pinned = realtime_tools::set_current_thread_affinity(cpus);
          if (!pinned.first) {
            RCLCPP_WARN(manager->get_logger(), "CPU affinity unavailable: %s", pinned.second.c_str());
          }
        }
        const auto priority = manager->get_parameter_or<int>("thread_priority", 50);
        if (!realtime_tools::configure_sched_fifo(priority)) {
          RCLCPP_WARN(manager->get_logger(),
            "FIFO priority %d unavailable; timing/velocity/acceleration guards remain enforced", priority);
        }
        RCLCPP_INFO(manager->get_logger(),
          "MK2 shared sample clock enabled: read/update/write use identical time, 100 Hz; guards unchanged");
        const auto nominal = std::chrono::nanoseconds(1'000'000'000 / rate);
        auto deadline = std::chrono::steady_clock::now();
        auto previous = manager->now();
        bool first = true;
        while (rclcpp::ok()) {
          const auto sample_time = manager->now();
          const auto elapsed = first ? rclcpp::Duration(nominal) : sample_time - previous;
          // Do not feed stale/backward samples to JTC or invent a fixed period
          // after an overrun. Same interval window as the hardware adapter.
          if (elapsed.nanoseconds() < 5'000'000 || elapsed.nanoseconds() > 50'000'000) {
            throw std::runtime_error("MK2 shared sample interval outside 5..50 ms; stopping control process");
          }
          previous = sample_time;
          first = false;
          mk2_control::run_cycle(*manager, sample_time, elapsed);
          deadline += nominal;
          const auto now = std::chrono::steady_clock::now();
          // No rapid catch-up burst after a missed deadline. Native wall-clock
          // skew and watchdog remain authoritative even with sample-time sharing.
          if (deadline <= now) deadline = now + nominal;
          std::this_thread::sleep_until(deadline);
        }
      } catch (const std::exception & error) {
        failed.store(true);
        RCLCPP_FATAL(manager->get_logger(), "MK2 shared-clock loop stopped: %s", error.what());
        rclcpp::shutdown();
      }
    });
    try {
      executor->spin();
    } catch (const std::exception & error) {
      failed.store(true);
      RCLCPP_FATAL(manager->get_logger(), "MK2 executor stopped: %s", error.what());
    }
    rclcpp::shutdown();
    control_thread.join();
    return failed.load() ? EXIT_FAILURE : EXIT_SUCCESS;
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("mk2_shared_clock_control"), "Startup refused: %s", error.what());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }
}