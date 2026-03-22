#!/bin/bash
cd /mnt/d/Projects/ROBOT/6-Axis-robot-ROS2-control-movet-gazebo-
source /opt/ros/humble/setup.bash

# Ensure workspace is fully built
colcon build --symlink-install --packages-ignore robot_tests robot_hardware_bridge
source install/setup.bash

echo "Launching Gazebo Harmonic Headless in Background..."
# Launch Simulation in background, timeout after 45s
timeout 45 ros2 launch robot_gazebo gazebo.launch.py headless:=true > gazebo_test.log 2>&1 &

# Give it 25 seconds to spin up Harmonic, spawn the robot, and initialize gz_ros2_control plugins
sleep 25

echo "Scraping /joint_states to verify bridged pipeline..."
# Attempt to read from the topic once to prove the pipeline is flowing
timeout 10 ros2 topic echo /joint_states --once > topic_test.log 2>&1

echo "Test complete."
