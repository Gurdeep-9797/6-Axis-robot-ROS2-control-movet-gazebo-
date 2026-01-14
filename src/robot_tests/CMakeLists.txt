cmake_minimum_required(VERSION 3.8)
project(robot_tests)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(motion_stack_test_node src/motion_stack_test_node.cpp)
ament_target_dependencies(motion_stack_test_node
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS
  motion_stack_test_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
