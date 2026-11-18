cmake_minimum_required(VERSION 3.10)
project(my_jearm_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(fake_trajectory_executor
  src/fake_trajectory_executor.cpp
)

add_executable(cartesian_path_diagnoser
  src/cartesian_path_diagnoser.cpp
)

add_executable(arm_state_logger
  src/arm_state_logger.cpp
)

add_executable(mk2_shared_clock_control_node
  src/mk2_shared_clock_control_node.cpp
)
target_compile_features(mk2_shared_clock_control_node PRIVATE cxx_std_17)
ament_target_dependencies(mk2_shared_clock_control_node
  controller_manager
  controller_manager_msgs
  hardware_interface
  lifecycle_msgs
  rclcpp
  realtime_tools
)

ament_target_dependencies(fake_trajectory_executor
  rclcpp
  rclcpp_action
  control_msgs
  sensor_msgs
  tf2_ros
)

ament_target_dependencies(arm_state_logger
  action_msgs
  rclcpp
  sensor_msgs
  tf2_ros
)

ament_target_dependencies(cartesian_path_diagnoser
  geometry_msgs
  moveit_msgs
  rclcpp
)

# Install config files
install(DIRECTORY 
  config 
  launch 
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  arm_state_logger
  cartesian_path_diagnoser
  fake_trajectory_executor
  mk2_shared_clock_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/mk2_state_bridge.py scripts/mk2_control_owner.py
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES scripts/mk2_ownership.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(JE2_JSON_C REQUIRED IMPORTED_TARGET json-c)

  # Test-only ROS/session boundary: no install, plugin export, socket or SDK.
  add_library(je2_session_client_test_support STATIC controller_bridge/je2_session_client.c)
  target_compile_features(je2_session_client_test_support PRIVATE c_std_11)
  target_include_directories(je2_session_client_test_support PUBLIC controller_bridge)
  target_link_libraries(je2_session_client_test_support PUBLIC PkgConfig::JE2_JSON_C m)
  ament_add_gtest(je2_ros_goal_adapter_test test/je2_ros_goal_adapter_test.cpp)
  target_include_directories(je2_ros_goal_adapter_test PRIVATE src)
  target_compile_features(je2_ros_goal_adapter_test PRIVATE cxx_std_17)
  target_link_libraries(je2_ros_goal_adapter_test je2_session_client_test_support)
  ament_target_dependencies(je2_ros_goal_adapter_test control_msgs)

  # In-process ZMQ integration: actual protocol/engine/Owner formatter, memory
  # motion callbacks ONLY. No SDK, TCP endpoints, ROS node, plugin or install.
  pkg_check_modules(JE2_ZMQ REQUIRED IMPORTED_TARGET libzmq)
  add_library(je2_loopback_server_test_support STATIC
    test/je2_loopback_server.c controller_bridge/jeserver_v2.c controller_bridge/mk2_control_core.c)
  target_compile_features(je2_loopback_server_test_support PRIVATE c_std_11)
  target_include_directories(je2_loopback_server_test_support PUBLIC controller_bridge test)
  target_compile_options(je2_loopback_server_test_support PRIVATE -Wall -Wextra -Werror -pedantic)
  target_link_libraries(je2_loopback_server_test_support PUBLIC PkgConfig::JE2_JSON_C m)
  ament_add_gtest(je2_ros_session_loopback_test test/je2_ros_session_loopback_test.cpp TIMEOUT 30)
  target_include_directories(je2_ros_session_loopback_test PRIVATE src)
  target_compile_features(je2_ros_session_loopback_test PRIVATE cxx_std_17)
  target_link_libraries(je2_ros_session_loopback_test je2_loopback_server_test_support
    je2_session_client_test_support PkgConfig::JE2_ZMQ)
  ament_target_dependencies(je2_ros_session_loopback_test control_msgs)

  # Real action API, isolated loopback-only DDS, explicit fake controller queue.
  # Never install this endpoint or expose a production controller action name.
  find_package(rmw REQUIRED)
  find_package(fastrtps REQUIRED)
  ament_add_gtest(je2_ros_action_executor_test test/je2_ros_action_executor_test.cpp TIMEOUT 90
    APPEND_LIBRARY_DIRS "$<TARGET_FILE_DIR:rclcpp::rclcpp>")
  target_include_directories(je2_ros_action_executor_test PRIVATE src)
  target_compile_features(je2_ros_action_executor_test PRIVATE cxx_std_17)
  target_compile_options(je2_ros_action_executor_test PRIVATE -Wall -Wextra -Werror)
  target_compile_definitions(je2_ros_action_executor_test PRIVATE
    JE2_ACTION_TEST_PROFILE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/je2_action_loopback_dds.xml")
  target_link_libraries(je2_ros_action_executor_test je2_loopback_server_test_support je2_session_client_test_support fastrtps)
  ament_target_dependencies(je2_ros_action_executor_test rclcpp rclcpp_action control_msgs action_msgs rmw)
  set_tests_properties(je2_ros_action_executor_test PROPERTIES RUN_SERIAL TRUE ENVIRONMENT
    "ROS_LOCALHOST_ONLY=1;RMW_IMPLEMENTATION=rmw_fastrtps_cpp;FASTRTPS_DEFAULT_PROFILES_FILE=${CMAKE_CURRENT_SOURCE_DIR}/test/je2_action_loopback_dds.xml;FASTDDS_DEFAULT_PROFILES_FILE=${CMAKE_CURRENT_SOURCE_DIR}/test/je2_action_loopback_dds.xml;ROS_DISCOVERY_SERVER=;SKIP_DEFAULT_XML=0")

  add_executable(je2_owner_legacy_test test/je2_owner_legacy_test.c)
  target_include_directories(je2_owner_legacy_test PRIVATE controller_bridge)
  target_compile_features(je2_owner_legacy_test PRIVATE c_std_11)
  target_compile_options(je2_owner_legacy_test PRIVATE -UNDEBUG -Wall -Wextra -Werror -pedantic)
  add_test(NAME je2_owner_legacy_test COMMAND je2_owner_legacy_test)
  set_tests_properties(je2_owner_legacy_test PROPERTIES TIMEOUT 30)

  # Reuse the protocol regression through CTest as well as its standalone runner.
  # Keep assert-based checks enabled even in Release/RelWithDebInfo builds.
  add_executable(je2_session_client_protocol_test controller_bridge/tests/je2_session_client_test.c)
  target_link_libraries(je2_session_client_protocol_test PRIVATE je2_session_client_test_support)
  target_compile_features(je2_session_client_protocol_test PRIVATE c_std_11)
  target_compile_options(je2_session_client_protocol_test PRIVATE -UNDEBUG)
  add_test(NAME je2_session_client_protocol_test COMMAND je2_session_client_protocol_test)
  set_tests_properties(je2_session_client_protocol_test PROPERTIES TIMEOUT 30)
  if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    add_executable(je2_session_client_sanitized_test
      controller_bridge/je2_session_client.c controller_bridge/tests/je2_session_client_test.c)
    target_compile_features(je2_session_client_sanitized_test PRIVATE c_std_11)
    target_compile_options(je2_session_client_sanitized_test PRIVATE
      -UNDEBUG -fsanitize=address,undefined -fno-omit-frame-pointer -fno-pie)
    target_link_libraries(je2_session_client_sanitized_test PRIVATE PkgConfig::JE2_JSON_C m)
    set_target_properties(je2_session_client_sanitized_test PROPERTIES
      LINK_FLAGS "-fsanitize=address,undefined -no-pie")
    add_test(NAME je2_session_client_sanitized_test COMMAND je2_session_client_sanitized_test)
    set_tests_properties(je2_session_client_sanitized_test PROPERTIES TIMEOUT 30
      ENVIRONMENT "ASAN_OPTIONS=detect_leaks=1:halt_on_error=1;UBSAN_OPTIONS=halt_on_error=1")
  endif()
endif()

ament_package()
