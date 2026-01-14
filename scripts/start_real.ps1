# Startup Scripts (PowerShell)
# Helper scripts for Windows users to launch the system easily

$ErrorActionPreference = "Stop"

Write-Host "Starting Robot System in REAL HARDWARE MODE..." -ForegroundColor Red
Write-Host "WARNING: This connects to real hardware. Ensure E-STOP is accessible." -ForegroundColor Yellow

# Load environment
if (Test-Path ".env") {
    Get-Content ".env" | ForEach-Object {
        if ($_ -match "^([^#=]+)=(.*)") {
            [Environment]::SetEnvironmentVariable($matches[1], $matches[2], "Process")
        }
    }
}

# Override for REAL
[Environment]::SetEnvironmentVariable("ROBOT_MODE", "REAL", "Process")
[Environment]::SetEnvironmentVariable("CONTROLLER_TYPE", "REAL", "Process")

# Launch Docker Compose
docker compose -f docker-compose.yml -f docker-compose.real.yml up -d

Write-Host "System started in REAL mode." -ForegroundColor Green
