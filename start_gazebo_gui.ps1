# RoboForge v8.2 — Gazebo GUI Launcher (Windows + Docker WSL2)
# ─────────────────────────────────────────────────────────────
# This script sets up X11 forwarding for Gazebo GUI display

Write-Host "🤖 Starting Gazebo GUI for RoboForge v8.2..." -ForegroundColor Cyan

# 1. Check for X Server
$xServerFound = $false

# Check for VcXsrv
if (Test-Path "C:\Program Files\VcXsrv\vcxsrv.exe") {
    Write-Host "✅ VcXsrv found" -ForegroundColor Green
    $xServerFound = $true
    $xServerPath = "C:\Program Files\VcXsrv\vcxsrv.exe"
}
# Check for X410
elseif (Get-AppxPackage -Name "X410" -ErrorAction SilentlyContinue) {
    Write-Host "✅ X410 found" -ForegroundColor Green
    $xServerFound = $true
}
# Check for WSLg (Windows 11 built-in)
elseif ($env:WSL_DISTRO_NAME -or (wsl -l -v 2>&1 | Select-String "WSL 2")) {
    Write-Host "✅ WSLg detected (Windows 11 built-in X server)" -ForegroundColor Green
    $xServerFound = $true
    $useWSLg = $true
}

if (-not $xServerFound) {
    Write-Host ""
    Write-Host "⚠️  No X Server detected!" -ForegroundColor Yellow
    Write-Host ""
    Write-Host "To display Gazebo GUI on Windows, you need one of:" -ForegroundColor White
    Write-Host "  1. VcXsrv (Recommended): https://sourceforge.net/projects/vcxsrv/" -ForegroundColor Gray
    Write-Host "  2. X410 (Microsoft Store): https://apps.microsoft.com/detail/x410" -ForegroundColor Gray
    Write-Host "  3. Windows 11 WSLg (Built-in, no installation needed)" -ForegroundColor Gray
    Write-Host ""
    Write-Host "Options:" -ForegroundColor Yellow
    Write-Host "  A) Install VcXsrv and run this script again" -ForegroundColor Gray
    Write-Host "  B) Use web-based monitoring at http://localhost:3000 (already running)" -ForegroundColor Gray
    Write-Host "  C) Start Gazebo headless + use pipeline verification script" -ForegroundColor Gray
    Write-Host ""
    
    $choice = Read-Host "Choose (A/B/C)"
    
    if ($choice -eq "B") {
        Write-Host "🌐 Opening RoboForge IDE..." -ForegroundColor Cyan
        Start-Process "http://localhost:3000"
        exit
    }
    elseif ($choice -eq "C") {
        Write-Host "📊 Starting pipeline verification..." -ForegroundColor Cyan
        & "$PSScriptRoot\verify_pipeline.ps1"
        exit
    }
    else {
        Write-Host "ℹ️  Please install VcXsrv and re-run this script." -ForegroundColor Yellow
        exit
    }
}

# 2. Get WSL2 host IP
Write-Host "🔍 Detecting network configuration..." -ForegroundColor Gray
$wslHostIP = wsl hostname -I 2>$null
if ($wslHostIP) {
    $wslHostIP = $wslHostIP.Trim()
    Write-Host "   WSL2 IP: $wslHostIP" -ForegroundColor Gray
}

# 3. Start Gazebo with X11 forwarding
Write-Host "🚀 Starting Gazebo with GUI..." -ForegroundColor Cyan

if ($useWSLg) {
    # Windows 11 WSLg - direct display
    docker run -d --name roboforge_gazebo `
        --network 6-axis-robot-ros2-control-movet-gazebo-_robot_net `
        -e ROS_DOMAIN_ID=42 `
        -e DISPLAY=:0 `
        -v /tmp/.X11-unix:/tmp/.X11-unix `
        -v /d/Projects/ROBOT/6-Axis-robot-ROS2-control-movet-gazebo-/src:/ros_ws/src `
        robotics_base:latest `
        bash -c "source /opt/ros/humble/setup.bash && cd /ros_ws && colcon build --symlink-install && source install/setup.bash && ros2 launch robot_gazebo gazebo.launch.py headless:=false"
}
else {
    # VcXsrv/X410 - use host IP
    $hostIP = (Get-NetIPAddress -AddressFamily IPv4 | Where-Object { $_.InterfaceAlias -notlike "*Loopback*" } | Select-Object -First 1).IPAddress
    
    docker run -d --name roboforge_gazebo `
        --network 6-axis-robot-ros2-control-movet-gazebo-_robot_net `
        -e ROS_DOMAIN_ID=42 `
        -e DISPLAY=$hostIP`:0.0 `
        -v /d/Projects/ROBOT/6-Axis-robot-ROS2-control-movet-gazebo-/src:/ros_ws/src `
        robotics_base:latest `
        bash -c "source /opt/ros/humble/setup.bash && cd /ros_ws && colcon build --symlink-install && source install/setup.bash && ros2 launch robot_gazebo gazebo.launch.py headless:=false"
}

Write-Host "⏳ Waiting for Gazebo to start..." -ForegroundColor Gray
Start-Sleep -Seconds 10

# Check container status
$status = docker inspect roboforge_gazebo --format '{{.State.Status}}' 2>$null
if ($status -eq "running") {
    Write-Host "✅ Gazebo GUI is starting!" -ForegroundColor Green
    Write-Host ""
    Write-Host "📺 The Gazebo window should appear shortly..." -ForegroundColor Cyan
    Write-Host "   If not, check that your X server is running and accessible." -ForegroundColor Gray
}
else {
    Write-Host "❌ Gazebo failed to start" -ForegroundColor Red
    docker logs roboforge_gazebo --tail 20
}
