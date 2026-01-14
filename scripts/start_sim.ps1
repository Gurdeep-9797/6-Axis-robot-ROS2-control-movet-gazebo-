# Startup Scripts (PowerShell)
# Helper scripts for Windows users to launch the system easily

$ErrorActionPreference = "Stop"

Write-Host "Starting Robot System in SIMULATION MODE..." -ForegroundColor Green
Write-Host "This will launch: ROS Core, MoveIt, Gazebo, Bridge (SIM), and UI" -ForegroundColor Gray

# Load environment
if (Test-Path ".env") {
    Get-Content ".env" | ForEach-Object {
        if ($_ -match "^([^#=]+)=(.*)") {
            [Environment]::SetEnvironmentVariable($matches[1], $matches[2], "Process")
        }
    }
}

# Override for SIM
[Environment]::SetEnvironmentVariable("ROBOT_MODE", "SIM", "Process")
[Environment]::SetEnvironmentVariable("CONTROLLER_TYPE", "FAKE", "Process")

# Launch Docker Compose
docker compose -f docker-compose.yml -f docker-compose.sim.yml up -d

Write-Host "System started! Access UI at http://localhost:8080" -ForegroundColor Green
