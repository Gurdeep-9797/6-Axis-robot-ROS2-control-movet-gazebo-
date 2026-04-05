$ErrorActionPreference = "Continue"

Write-Host "Cleaning up old processes..."
wsl -d Ubuntu-22.04 -- bash -c "pkill -f ros2; pkill -f gz"
Start-Sleep -Seconds 2

Write-Host "[1/4] Compiling workspace..."
wsl -d Ubuntu-22.04 -- bash -c "cd /mnt/d/Projects/ROBOT/6-Axis-robot-ROS2-control-movet-gazebo- && source /opt/ros/humble/setup.bash && colcon build"

Write-Host "[2/4] Starting ROS Bridge (Background)..."
Start-Process wsl.exe -ArgumentList "-d Ubuntu-22.04 -- bash -c `"cd /mnt/d/Projects/ROBOT/6-Axis-robot-ROS2-control-movet-gazebo- && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch rosbridge_server rosbridge_websocket_launch.xml > bridge.log 2>&1`"" -WindowStyle Hidden

Write-Host "[3/4] Starting Gazebo Harmonic with UI (Background)..."
# Not running headless so the user can see the 3D physics!
Start-Process wsl.exe -ArgumentList "-d Ubuntu-22.04 -- bash -c `"cd /mnt/d/Projects/ROBOT/6-Axis-robot-ROS2-control-movet-gazebo- && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch robot_gazebo gazebo.launch.py > gazebo.log 2>&1`"" -WindowStyle Normal

Write-Host "Waiting 15 seconds for Gazebo to load completely..."
Start-Sleep -Seconds 15

Write-Host "[4/4] Opening WPF Application..."
Write-Host "The application is now opening on your screen! Move the sliders to test the robot."
$exePath = "d:\Projects\ROBOT\6-Axis-robot-ROS2-control-movet-gazebo-\TeachPendant_WPF\bin\x64\Debug\net8.0-windows\TeachPendant_WPF.exe"
if (!(Test-Path $exePath)) {
    dotnet build "d:\Projects\ROBOT\6-Axis-robot-ROS2-control-movet-gazebo-\TeachPendant_WPF"
}
$wpfProc = Start-Process $exePath -Wait -WindowStyle Normal

Write-Host "WPF Closed. Cleaning up ROS 2 processes..."
wsl -d Ubuntu-22.04 -- bash -c "pkill -f ros2"
wsl -d Ubuntu-22.04 -- bash -c "pkill -f gz"
Write-Host "Done!"
