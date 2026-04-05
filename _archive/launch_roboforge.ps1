$ErrorActionPreference = "Stop"

Write-Host "=============================================" -ForegroundColor Cyan
Write-Host "   RoboForge WPF + ROS2 + Gazebo Launch      " -ForegroundColor Cyan
Write-Host "=============================================" -ForegroundColor Cyan

# 1. Start ROS services in WSL (background window)
Write-Host "[1/2] Starting ROS services in WSL..." -ForegroundColor Yellow
$rosProc = Start-Process wsl.exe -ArgumentList "-d Ubuntu-22.04 -- bash /mnt/d/Projects/ROBOT/6-Axis-robot-ROS2-control-movet-gazebo-/start_ros_services.sh" -PassThru
Write-Host "  WSL PID: $($rosProc.Id)" -ForegroundColor DarkGray

Write-Host "  Waiting 15s for services to initialize..." -ForegroundColor DarkGray
Start-Sleep -Seconds 15

# 2. Launch WPF UI
Write-Host "[2/2] Launching RoboForge WPF UI..." -ForegroundColor Yellow
Start-Process "d:\Projects\ROBOT\6-Axis-robot-ROS2-control-movet-gazebo-\RoboForge_WPF\bin\x64\Debug\net8.0-windows\RoboForge_WPF.exe"

Write-Host ""
Write-Host "=============================================" -ForegroundColor Green
Write-Host "  All systems launched!                      " -ForegroundColor Green
Write-Host "  ROS Bridge: ws://localhost:9090            " -ForegroundColor Green
Write-Host "  CLI API:    http://127.0.0.1:5050/api/    " -ForegroundColor Green
Write-Host "=============================================" -ForegroundColor Green
Write-Host ""
Write-Host "To verify pipeline, run:" -ForegroundColor DarkGray
Write-Host "  .\verify_pipeline.ps1" -ForegroundColor White
Write-Host ""
Write-Host "To stop everything later:" -ForegroundColor DarkGray
Write-Host '  taskkill /F /IM RoboForge_WPF.exe' -ForegroundColor White
Write-Host '  wsl -d Ubuntu-22.04 -- bash -c "pkill -f ros2; pkill -f gz"' -ForegroundColor White
