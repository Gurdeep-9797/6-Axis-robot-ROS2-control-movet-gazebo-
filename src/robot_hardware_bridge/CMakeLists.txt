cmake_minimum_required(VERSION 3.8)
project(robot_hardware_bridge)

# Default to C++17 for modern features
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(robot_msgs REQUIRED)

# Include directories
include_directories(include)

# =============================================================================
# C++ Bridge Node
# =============================================================================
add_executable(bridge_node_cpp
  src/bridge_node.cpp
)

ament_target_dependencies(bridge_node_cpp
  rclcpp
  std_msgs
  sensor_msgs
  trajectory_msgs
  robot_msgs
)

# Install C++ executables
install(TARGETS
  bridge_node_cpp
  DESTINATION lib/${PROJECT_NAME}
)

# Install C++ headers
install(DIRECTORY include/
  DESTINATION include
)

# =============================================================================
# Python Nodes (existing)
# =============================================================================
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  robot_hardware_bridge/bridge_node.py
  robot_hardware_bridge/sim_backend.py
  robot_hardware_bridge/real_backend.py
  robot_hardware_bridge/moveit_adapter_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# =============================================================================
# Launch files
# =============================================================================
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# =============================================================================
# Testing
# =============================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
