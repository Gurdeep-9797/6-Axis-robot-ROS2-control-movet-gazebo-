#!/bin/bash
set -e

# Setup Environment
source /opt/ros/humble/setup.bash
if [ -f ~/robot_ws/install/setup.bash ]; then
    source ~/robot_ws/install/setup.bash
else
    echo "Error: Workspace not built or setup.bash not found."
    exit 1
fi

export ROBOT_MODE=SIM
export CONTROLLER_TYPE=GAZEBO
export ROS_DOMAIN_ID=0

echo "=== STARTING ROBOT SIMULATION ==="

cleanup() {
    echo "Shutting down..."
    kill $(jobs -p) 2>/dev/null
    exit
}
trap cleanup SIGINT SIGTERM

# 1. Launch Hardware Bridge (Bridge Node) using DIRECT PATH
echo "[1/4] Starting Hardware Bridge..."
~/robot_ws/install/robot_hardware_bridge/bin/bridge_node --ros-args -p robot_mode:=SIM -p controller_type:=GAZEBO &
BRIDGE_PID=$!
sleep 1

# 1b. MoveIt Adapter
echo "[1b/4] Starting MoveIt Adapter..."
~/robot_ws/install/robot_hardware_bridge/bin/moveit_adapter &
ADAPTER_PID=$!
sleep 1

# 2. Launch Gazebo (The Plant)
echo "[2/4] Starting Gazebo..."
if [ "$HEADLESS" = "true" ]; then
    GUI_ARG="false"
else
    GUI_ARG="true"
fi
ros2 launch robot_gazebo gazebo.launch.py gui:=$GUI_ARG &
GAZEBO_PID=$!
sleep 5

# 3. Launch MoveIt (Planner)
echo "[3/4] Starting MoveIt..."
ros2 launch robot_moveit_config move_group.launch.py &
MOVEIT_PID=$!
sleep 5

# 4. Launch RViz (Visualizer)
if [ "$HEADLESS" != "true" ]; then
    echo "[4/4] Starting RViz..."
    RVIZ_CONFIG=$(ros2 pkg prefix robot_moveit_config)/share/robot_moveit_config/config/moveit.rviz
    if [ -f "$RVIZ_CONFIG" ]; then
        rviz2 -d "$RVIZ_CONFIG" &
    else
        echo "Warning: moveit.rviz not found, launching empty rviz2"
        rviz2 &
    fi
    RVIZ_PID=$!
else
    echo "[4/4] Skipping RViz (Headless Mode)"
fi

echo "=== SYSTEM RUNNING ==="
echo "Bridge: $BRIDGE_PID"
echo "Gazebo: $GAZEBO_PID"
echo "MoveIt: $MOVEIT_PID"
echo "Press Ctrl+C to stop"

wait $BRIDGE_PID
