#!/bin/bash
# Start ROS services for RoboForge pipeline
# Run from: wsl -d Ubuntu-22.04 -- bash /mnt/d/Projects/ROBOT/6-Axis-robot-ROS2-control-movet-gazebo-/start_ros_services.sh

set -e
WORKSPACE="/mnt/d/Projects/ROBOT/6-Axis-robot-ROS2-control-movet-gazebo-"
cd "$WORKSPACE"

echo "=== RoboForge ROS Services ==="

# Source ROS
source /opt/ros/humble/setup.bash
if [ -f "$WORKSPACE/install/setup.bash" ]; then
    source "$WORKSPACE/install/setup.bash"
    echo "[OK] Workspace sourced"
else
    echo "[WARN] No install/setup.bash found, using base ROS only"
fi

# Start ROS Bridge (background)
echo "[1/2] Starting ROS Bridge WebSocket on ws://0.0.0.0:9090 ..."
ros2 launch rosbridge_server rosbridge_websocket_launch.xml > "$WORKSPACE/bridge.log" 2>&1 &
BRIDGE_PID=$!
echo "      PID: $BRIDGE_PID"

sleep 3

# Check if bridge started
if kill -0 $BRIDGE_PID 2>/dev/null; then
    echo "[OK] ROS Bridge is running"
else
    echo "[FAIL] ROS Bridge failed to start. Check bridge.log"
    cat "$WORKSPACE/bridge.log"
    exit 1
fi

# Start Gazebo (background)  
echo "[2/2] Starting Gazebo Harmonic (headless)..."
ros2 launch robot_gazebo gazebo.launch.py headless:=true > "$WORKSPACE/gazebo.log" 2>&1 &
GAZEBO_PID=$!
echo "      PID: $GAZEBO_PID"

sleep 5

if kill -0 $GAZEBO_PID 2>/dev/null; then
    echo "[OK] Gazebo is running"
else
    echo "[WARN] Gazebo may not have started. Check gazebo.log"
fi

echo ""
echo "=== All services started ==="
echo "  ROS Bridge: PID $BRIDGE_PID (ws://localhost:9090)"
echo "  Gazebo:     PID $GAZEBO_PID"
echo ""
echo "Press Ctrl+C to stop all services"

# Wait for either to exit
wait $BRIDGE_PID $GAZEBO_PID
