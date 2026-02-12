Write-Host "=== 6-AXIS DIGITAL TWIN LAUNCHER ===" -ForegroundColor Cyan

# 1. Check Docker
Write-Host "Checking Docker..."
$dockerCheck = docker ps 2>&1
if ($LASTEXITCODE -ne 0) {
    Write-Host "Warning: Docker is not running or not installed." -ForegroundColor Yellow
    Write-Host "You can still run in Simulation Mode (Manual) or Physical Serial Mode."
}
else {
    Write-Host "Docker is running." -ForegroundColor Green
    $response = Read-Host "Start ROS 2 Simulation Stack? (y/n)"
    if ($response -eq 'y') {
        docker compose -f docker/docker-compose.sim.yml up -d
        Write-Host "ROS 2 Stack Started." -ForegroundColor Green
    }
}

# 2. Check for Controller
Write-Host "Scanning for Controllers..."
$ports = [System.IO.Ports.SerialPort]::GetPortNames()
if ($ports.Count -gt 0) {
    Write-Host "Found devices: $ports" -ForegroundColor Green
}
else {
    Write-Host "No Serial Controllers found (safe for Sim Mode)." -ForegroundColor Gray
}

# 3. Launch Simulator
Write-Host "Launching Simulator..." -ForegroundColor Cyan
Start-Process "d:\Projects\ROBOT\6-Axis-robot-ROS2-control-movet-gazebo-\RobotSimulator\bin\Release\net8.0-windows\RobotSimulator.exe"

Write-Host "Done. Console will close in 5 seconds."
Start-Sleep -Seconds 5
