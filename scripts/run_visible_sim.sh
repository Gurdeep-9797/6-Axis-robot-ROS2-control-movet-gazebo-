#!/bin/bash
# ============================================================
# VISIBLE ROBOT SIMULATION - One-Click Launcher
# ============================================================
# Run this in WSL2 Ubuntu with ROS 2 Humble installed.
#
# What you will see:
#   - RViz window opens with a 6-axis robot arm
#   - Robot joints move continuously
#   - Motion is controlled by Antigravity (this script)
#
# Requirements:
#   - ROS 2 Humble installed in WSL2
#   - X11 display working (WSLg or VcXsrv)
# ============================================================

echo "============================================================"
echo "ANTIGRAVITY VISIBLE ROBOT SIMULATION"
echo "============================================================"
echo ""

# Source ROS 2
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "[OK] ROS 2 Humble sourced"
else
    echo "[ERROR] ROS 2 Humble not found at /opt/ros/humble"
    echo "Please install ROS 2 Humble first."
    exit 1
fi

# Set display
export DISPLAY=${DISPLAY:-:0}
export QT_X11_NO_MITSHM=1
echo "[OK] DISPLAY set to $DISPLAY"

# Find the script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEMO_SCRIPT="$SCRIPT_DIR/visible_robot_demo.py"

# Check if demo script exists
if [ ! -f "$DEMO_SCRIPT" ]; then
    # Try parent directory
    DEMO_SCRIPT="$(dirname "$SCRIPT_DIR")/visible_robot_demo.py"
fi

if [ ! -f "$DEMO_SCRIPT" ]; then
    echo "[ERROR] visible_robot_demo.py not found"
    exit 1
fi

echo "[OK] Found demo script: $DEMO_SCRIPT"
echo ""
echo "Starting visible simulation..."
echo "An RViz window should appear with a moving robot arm."
echo "Press Ctrl+C to stop."
echo ""

# Run the Python demo
python3 "$DEMO_SCRIPT"
