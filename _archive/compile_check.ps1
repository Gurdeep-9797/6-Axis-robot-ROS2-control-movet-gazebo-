$ErrorActionPreference = "Stop"
Write-Host "Rebuilding WPF..."
dotnet build RoboForge_WPF\RoboForge_WPF.csproj > build_wpf.log 2>&1
Write-Host "WPF Done."
Write-Host "Rebuilding ROS Bridge Node..."
wsl -d Ubuntu-22.04 --cd /mnt/d/Projects/ROBOT/6-Axis-robot-ROS2-control-movet-gazebo- -e bash -c "source /opt/ros/humble/setup.bash && colcon build --packages-select robot_hardware_bridge" > build_ros.log 2>&1
Write-Host "ROS Done."
