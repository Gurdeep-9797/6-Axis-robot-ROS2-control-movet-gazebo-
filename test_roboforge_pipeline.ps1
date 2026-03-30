$ErrorActionPreference = "Stop"

Write-Host "=============================================" -ForegroundColor Cyan
Write-Host "   RoboForge Automated Pipeline Verification   " -ForegroundColor Cyan
Write-Host "=============================================" -ForegroundColor Cyan

Write-Host "[1/4] Compiling workspace (WSL) and WPF..."
wsl -d Ubuntu-22.04 -- bash -c "cd /mnt/d/Projects/ROBOT/6-Axis-robot-ROS2-control-movet-gazebo- && source /opt/ros/humble/setup.bash && colcon build"
dotnet build d:\Projects\ROBOT\6-Axis-robot-ROS2-control-movet-gazebo-\RoboForge_WPF

Write-Host "[2/4] Starting ROS Bridge (Background)..."
$bridgeProc = Start-Process wsl.exe -ArgumentList "-d Ubuntu-22.04 -- bash -c `"cd /mnt/d/Projects/ROBOT/6-Axis-robot-ROS2-control-movet-gazebo- && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch rosbridge_server rosbridge_websocket_launch.xml > bridge.log 2>&1`"" -PassThru -WindowStyle Hidden

Write-Host "[3/4] Starting Gazebo Harmonic Headless (Background)..."
$gazeboProc = Start-Process wsl.exe -ArgumentList "-d Ubuntu-22.04 -- bash -c `"cd /mnt/d/Projects/ROBOT/6-Axis-robot-ROS2-control-movet-gazebo- && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch robot_gazebo gazebo.launch.py headless:=true > gazebo.log 2>&1`"" -PassThru -WindowStyle Hidden

Write-Host "Waiting 25 seconds for Gazebo to spin up..."
Start-Sleep -Seconds 25

Write-Host "[4/4] Starting RoboForge WPF (--autorun)"
$wpfProc = Start-Process dotnet.exe -ArgumentList "run --project d:\Projects\ROBOT\6-Axis-robot-ROS2-control-movet-gazebo-\RoboForge_WPF -- --autorun" -PassThru -RedirectStandardOutput "d:\Projects\ROBOT\6-Axis-robot-ROS2-control-movet-gazebo-\wpf_autorun.log" -RedirectStandardError "d:\Projects\ROBOT\6-Axis-robot-ROS2-control-movet-gazebo-\wpf_autorun_err.log" -WindowStyle Hidden

Write-Host "Waiting 15 seconds for WPF to inject points and MoveIt to plan..."
Start-Sleep -Seconds 15

Write-Host "Stopping all background processes..."
Stop-Process -Id $wpfProc.Id -Force -ErrorAction SilentlyContinue
wsl -d Ubuntu-22.04 -- bash -c "pkill -f ros2"
wsl -d Ubuntu-22.04 -- bash -c "pkill -f gz"

Write-Host "✅ Verification Complete! Check wpf_autorun.log and gazebo.log for joint_trajectory commands matching your blocks." -ForegroundColor Green
