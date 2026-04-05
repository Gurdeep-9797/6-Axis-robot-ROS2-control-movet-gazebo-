$ErrorActionPreference = "Stop"

Write-Host "=============================================" -ForegroundColor Cyan
Write-Host "   Pinnacle Simulation Pipeline (Native WSL2)  " -ForegroundColor Cyan
Write-Host "=============================================" -ForegroundColor Cyan

# 0. Compile ROS Workspace
Write-Host "[0/4] Natively compiling URDF and Robot Models in WSL2..." -ForegroundColor Yellow
Start-Process wsl.exe -ArgumentList "-d Ubuntu-22.04 -- bash -c `"cd /mnt/d/Projects/ROBOT/6-Axis-robot-ROS2-control-movet-gazebo- && rm -rf build install log && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-ignore robot_tests robot_hardware_bridge`"" -NoNewWindow -Wait

# 1. Start Rosbridge
Write-Host "[1/4] Spinning up RosBridge WebSocket..." -ForegroundColor Yellow
Start-Process wsl.exe -ArgumentList "-d Ubuntu-22.04 -- bash -c `"cd /mnt/d/Projects/ROBOT/6-Axis-robot-ROS2-control-movet-gazebo- && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch rosbridge_server rosbridge_websocket_launch.xml`""

Start-Sleep -Seconds 4

# 2. Start Gazebo (Native WSLg)
Write-Host "[2/4] Summoning Gazebo Graphic Engine..." -ForegroundColor Yellow
Start-Process wsl.exe -ArgumentList "-d Ubuntu-22.04 -- bash -c `"cd /mnt/d/Projects/ROBOT/6-Axis-robot-ROS2-control-movet-gazebo- && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch robot_gazebo gazebo.launch.py`""

Start-Sleep -Seconds 8

# 3. Start WPF Software
Write-Host "[3/4] Launching C# Visual Interface..." -ForegroundColor Yellow
Start-Process dotnet.exe -ArgumentList "run --project d:\Projects\ROBOT\6-Axis-robot-ROS2-control-movet-gazebo-\TeachPendant_WPF"

Write-Host "All channels deployed successfully." -ForegroundColor Green
Start-Sleep -Seconds 3
