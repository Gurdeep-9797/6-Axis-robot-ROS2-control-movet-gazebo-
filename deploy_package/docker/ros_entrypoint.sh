#!/bin/bash
set -e

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Source workspace if built
echo "Building workspace..."
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
if [ -f /ros_ws/install/setup.bash ]; then
    source /ros_ws/install/setup.bash
fi

# Set RMW implementation
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}

# Execute command
exec "$@"
