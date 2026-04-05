$ErrorActionPreference = "Stop"

Write-Host "[1/4] Compiling workspace..."
wsl -d Ubuntu-22.04 -- bash -c "cd /mnt/d/Projects/ROBOT/6-Axis-robot-ROS2-control-movet-gazebo- && source /opt/ros/humble/setup.bash && colcon build"

Write-Host "[2/4] Starting ROS Bridge (Background)..."
$bridgeProc = Start-Process wsl.exe -ArgumentList "-d Ubuntu-22.04 -- bash -c `"cd /mnt/d/Projects/ROBOT/6-Axis-robot-ROS2-control-movet-gazebo- && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch rosbridge_server rosbridge_websocket_launch.xml > bridge.log 2>&1`"" -PassThru -WindowStyle Hidden

Write-Host "[3/4] Starting Gazebo Harmonic Headless (Background)..."
$gazeboProc = Start-Process wsl.exe -ArgumentList "-d Ubuntu-22.04 -- bash -c `"cd /mnt/d/Projects/ROBOT/6-Axis-robot-ROS2-control-movet-gazebo- && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch robot_gazebo gazebo.launch.py headless:=true > gazebo.log 2>&1`"" -PassThru -WindowStyle Hidden

Write-Host "Waiting 25 seconds for Gazebo to spin up..."
Start-Sleep -Seconds 25

Write-Host "[4/4] Starting WPF Simulator..."
# Using dotnet run redirects
$wpfProc = Start-Process dotnet.exe -ArgumentList "run --project d:\Projects\ROBOT\6-Axis-robot-ROS2-control-movet-gazebo-\TeachPendant_WPF" -PassThru -RedirectStandardOutput "d:\Projects\ROBOT\6-Axis-robot-ROS2-control-movet-gazebo-\wpf.log" -RedirectStandardError "d:\Projects\ROBOT\6-Axis-robot-ROS2-control-movet-gazebo-\wpf_err.log" -WindowStyle Hidden

Write-Host "Giving WPF Simulator 20 seconds to connect to RosBridge..."
Start-Sleep -Seconds 20

Write-Host "Stopping all background processes..."
Stop-Process -Id $wpfProc.Id -Force -ErrorAction SilentlyContinue
wsl -d Ubuntu-22.04 -- bash -c "pkill -f ros2"
wsl -d Ubuntu-22.04 -- bash -c "pkill -f gz"
Write-Host "Done! You can now check wpf.log, bridge.log, and gazebo.log for telemetry flow."
