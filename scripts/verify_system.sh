#!/bin/bash
set -e

# Source ROS 2 environment
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash
export HEADLESS=true

echo "Environment Setup Complete"
echo "PYTHONPATH: $PYTHONPATH"
which python3

# Clean up any lingering processes
killall -9 gzserver gzclient bridge_node moveit_adapter > /dev/null 2>&1 || true

# Launch Simulation in Background
echo "Launching Simulation..."
./scripts/run_simulation.sh > simulation.log 2>&1 &
SIM_PID=$!
echo "Simulation launched with PID: $SIM_PID"

# Wait for startup
echo "Waiting 60s for simulation startup..."
sleep 60

# Debug Environment
echo "DEBUG: Testing python environment..."
python3 -c "import sys; print('SYS.PATH:', sys.path); import rclpy; print('RCLPY IMPORT OK')" || echo "DEBUG: Import failed"

# Run Verification
echo "Running Automated Logic Test..."
python3 scripts/automated_logic_test.py > test_results.log 2>&1
TEST_EXIT_CODE=$?
echo "Test script exited with code: $TEST_EXIT_CODE"

# Cleanup
echo "Terminating Simulation..."
kill $SIM_PID || true
# Ensure children are dead
killall -9 gzserver gzclient bridge_node moveit_adapter > /dev/null 2>&1 || true

exit $TEST_EXIT_CODE
