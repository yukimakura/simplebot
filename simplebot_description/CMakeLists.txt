cmake_minimum_required(VERSION 3.5.0)
project(simplebot_description)



find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)

install(DIRECTORY
  launch
  config
  urdf
  dae
  rviz
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")


ament_package()