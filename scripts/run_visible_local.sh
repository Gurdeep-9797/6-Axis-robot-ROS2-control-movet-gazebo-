# Visible Simulation Run - WSL2 Local Execution Script
# =====================================================
# This script runs the simulation locally in WSL2 for GUI visibility
# since Docker X11 forwarding is blocked on Windows.

echo "==================================================="
echo "VISIBLE SIMULATION RUN - WSL2 LOCAL MODE"  
echo "==================================================="

# Prerequisites check
echo "[CHECK] Verifying ROS 2 Humble installation..."
source /opt/ros/humble/setup.bash || { echo "[ERROR] ROS 2 Humble not installed"; exit 1; }

# Navigate to workspace  
cd ~/robot_ws || cd /ros_ws || { echo "[ERROR] Workspace not found"; exit 1; }

echo "[BUILD] Building workspace..."
colcon build --packages-ignore robot_tests --symlink-install

echo "[SOURCE] Sourcing workspace..."
source install/setup.bash

# Set display for local X11
export DISPLAY=:0
export QT_X11_NO_MITSHM=1

echo "[LAUNCH] Starting Gazebo + Robot..."
echo "Gazebo window should appear on your desktop."
echo ""
echo "Once Gazebo is running, open a new terminal and run:"
echo "  source /opt/ros/humble/setup.bash"
echo "  source ~/robot_ws/install/setup.bash"  
echo "  ros2 launch robot_moveit_config move_group.launch.py"
echo ""

ros2 launch robot_gazebo gazebo.launch.py headless:=false
