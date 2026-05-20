cmake_minimum_required(VERSION 3.10)
project(my_jearm_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(fake_trajectory_executor
  src/fake_trajectory_executor.cpp
)

add_executable(cartesian_path_diagnoser
  src/cartesian_path_diagnoser.cpp
)

ament_target_dependencies(fake_trajectory_executor
  rclcpp
  rclcpp_action
  control_msgs
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
  cartesian_path_diagnoser
  fake_trajectory_executor
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
