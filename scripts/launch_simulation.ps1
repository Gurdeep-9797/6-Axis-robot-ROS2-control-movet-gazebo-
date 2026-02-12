<#
.SYNOPSIS
    One-Click Launch for the ROS 2 Simulation Environment.
.DESCRIPTION
    Starts ROS Core, MoveIt, Gazebo, and Hardware Bridge containers.
    Ensures network and volume mounts are correct.
#>
$ErrorActionPreference = "Stop"

Write-Host "Starting ROS 2 Simulation Environment..." -ForegroundColor Green

# Check for X Server (Optional but recommended for GUI)
$Display = $env:DISPLAY
if (-not $Display) {
    Write-Warning "DISPLAY environment variable not set. GUI apps (Gazebo/RViz) may not appear."
    Write-Warning "If using VcXsrv, set DISPLAY=host.docker.internal:0"
    # Attempt a default
    $env:DISPLAY = "host.docker.internal:0"
}

# Set Project Root for Volume Mounting
$env:PROJECT_ROOT = (Get-Item .).FullName
Write-Host "Mounting Project Root: $env:PROJECT_ROOT"

# Run Docker Compose
docker compose -f docker-compose.sim.yml up -d

Write-Host "Containers Launched:"
docker compose -f docker-compose.sim.yml ps

Write-Host "Environment is READY."
Write-Host "To view logs: docker compose -f docker-compose.sim.yml logs -f"
Write-Host "To stop: docker compose -f docker-compose.sim.yml down"
