#!/bin/bash
# ROS 2 Humble + MoveIt 2 + Gazebo Setup Script
# Run inside Ubuntu 22.04 WSL

set -e

echo "=== STEP 4: ROS 2 Humble + MoveIt + Gazebo Installation ==="

# Update system
echo "[1/8] Updating system..."
sudo apt-get update && sudo apt-get upgrade -y

# Add ROS 2 repository
echo "[2/8] Adding ROS 2 repository..."
sudo apt-get install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt-get update

# Install ROS 2 Humble Desktop
echo "[3/8] Installing ROS 2 Humble Desktop..."
sudo apt-get install -y ros-humble-desktop

# Install MoveIt 2
echo "[4/8] Installing MoveIt 2..."
sudo apt-get install -y ros-humble-moveit ros-humble-moveit-ros-planning-interface

# Install Gazebo ROS packages (Fortress)
echo "[5/8] Installing Gazebo ROS packages..."
sudo apt-get install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control

# Install ros2_control and controllers
echo "[6/8] Installing ros2_control..."
sudo apt-get install -y ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-controller-manager

# Install build tools
echo "[7/8] Installing build tools..."
sudo apt-get install -y python3-colcon-common-extensions python3-vcstool build-essential python3-rosdep

# Initialize rosdep
echo "[8/8] Initializing rosdep..."
sudo rosdep init || true
rosdep update

# Add ROS sourcing to bashrc
echo "Adding ROS to .bashrc..."
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi

echo "=== INSTALLATION COMPLETE ==="
echo "Please run: source ~/.bashrc"
