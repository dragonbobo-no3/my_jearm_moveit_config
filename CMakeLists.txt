cmake_minimum_required(VERSION 3.10)
project(my_jearm_moveit_config)

find_package(ament_cmake REQUIRED)

# Install config files
install(DIRECTORY 
  config 
  launch 
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
