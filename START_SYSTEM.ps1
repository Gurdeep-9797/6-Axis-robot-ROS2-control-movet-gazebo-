Write-Host "=== 6-AXIS DIGITAL TWIN: AUTOMATED LAUNCHER ===" -ForegroundColor Cyan
Write-Host "This script will start the Docker backend and the Simulator Client." -ForegroundColor Gray

# 1. Check Docker
Write-Host "`n[1/3] Checking Docker Environment..."
$dockerCheck = docker ps 2>&1
if ($LASTEXITCODE -ne 0) {
    Write-Host "Error: Docker is not running!" -ForegroundColor Red
    Write-Host "Please start Docker Desktop and try again."
    Read-Host "Press Enter to exit..."
    exit
}

# 2. Start ROS 2 Stack
Write-Host "`n[2/3] Starting ROS 2 Simulation Stack (may take a moment)..."
docker compose -f docker/docker-compose.sim.yml up -d

Write-Host "Waiting for ROS Bridge (Port 9090)..." -ForegroundColor Yellow
$retries = 0
while ($retries -lt 30) {
    $conn = Test-NetConnection -ComputerName localhost -Port 9090 -InformationLevel Quiet
    if ($conn) {
        Write-Host "ROS Bridge is Ready!" -ForegroundColor Green
        break
    }
    Start-Sleep -Seconds 1
    $retries++
}

if (-not $conn) {
    Write-Host "Warning: ROS Bridge did not become ready in time. Simulator might not connect automatically." -ForegroundColor Magenta
}

# 3. Launch Simulator
Write-Host "`n[3/3] Launching Simulator Client..." -ForegroundColor Cyan
Start-Process "d:\Projects\ROBOT\6-Axis-robot-ROS2-control-movet-gazebo-\RobotSimulator\bin\Release\net8.0-windows\RobotSimulator.exe" -ArgumentList "--auto-connect"

Write-Host "`nSystem Started Successfully." -ForegroundColor Green
Write-Host "Closing launcher in 5 seconds..."
Start-Sleep -Seconds 5
