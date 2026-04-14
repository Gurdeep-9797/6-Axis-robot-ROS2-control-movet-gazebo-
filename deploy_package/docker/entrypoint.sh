#!/bin/bash
set -e

# Source ROS2
source /opt/ros/humble/setup.bash
cd /ros_ws
colcon build --symlink-install
source install/setup.bash

echo "🖥️ Starting Fluxbox window manager..."
fluxbox &

echo "⏳ Waiting for fluxbox to start..."
sleep 2

echo "📺 Starting VNC server on :1..."
# Start VNC with proper geometry and settings
vncserver :1 -geometry 1920x1080 -depth 24 -localhost no &
VNC_PID=$!

echo "⏳ Waiting 3 seconds for VNC to initialize..."
sleep 3

echo "🌐 Starting noVNC web viewer on port 6080..."
# VNC server runs on port 5901 (display :1)
websockify --web=/usr/share/novnc 6080 localhost:5901 &

echo "⏳ Waiting 5 seconds for VNC to initialize..."
sleep 5

echo "🚀 Launching Gazebo GUI..."
# Export DISPLAY for Gazebo
export DISPLAY=:1
export HOME=/root

# Launch Gazebo with GUI
exec ros2 launch robot_gazebo gazebo.launch.py headless:=false
