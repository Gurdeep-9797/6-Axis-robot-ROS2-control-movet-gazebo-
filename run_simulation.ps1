<#
.SYNOPSIS
    Launch the full robot simulation pipeline.
    Builds C++ simulator, starts Docker containers, launches simulator.

.PARAMETER Config
    Build configuration: Release (default) or Debug.

.PARAMETER NoBuild
    Skip the build step (assumes previously built).

.PARAMETER NoDocker
    Skip Docker containers (run simulator standalone).

.EXAMPLE
    .\run_simulation.ps1
    .\run_simulation.ps1 -NoBuild
    .\run_simulation.ps1 -Config Debug -NoDocker
#>
param(
    [ValidateSet("Release", "Debug")]
    [string]$Config = "Release",
    [switch]$NoBuild,
    [switch]$NoDocker
)

$ErrorActionPreference = "Stop"
$ProjectRoot = Split-Path -Parent $MyInvocation.MyCommand.Path

Write-Host ""
Write-Host "======================================================" -ForegroundColor Magenta
Write-Host "  Robot Simulator - Full Pipeline Launch               " -ForegroundColor Magenta
Write-Host "======================================================" -ForegroundColor Magenta
Write-Host ""

# ===========================================================
#  STEP 1: BUILD
# ===========================================================
if (-not $NoBuild) {
    Write-Host "=== [1/4] Building Simulator ===" -ForegroundColor Yellow
    & "$ProjectRoot\build.ps1" -Config $Config -SkipTests
    if ($LASTEXITCODE -ne 0) {
        Write-Host "  [FATAL] Build failed. Aborting." -ForegroundColor Red
        exit 1
    }
}
else {
    Write-Host "=== [1/4] Build skipped (-NoBuild) ===" -ForegroundColor Gray
}

# ===========================================================
#  STEP 2: DOCKER (ROS2 + Gazebo)
# ===========================================================
if (-not $NoDocker) {
    Write-Host ""
    Write-Host "=== [2/4] Starting Docker Containers ===" -ForegroundColor Yellow

    # Check Docker is running
    docker info 2>$null | Out-Null
    if ($LASTEXITCODE -ne 0) {
        Write-Host "  [FATAL] Docker is not running. Start Docker Desktop first." -ForegroundColor Red
        exit 1
    }
    Write-Host "  [OK] Docker running" -ForegroundColor Green

    # Set PROJECT_ROOT for docker-compose
    $env:PROJECT_ROOT = $ProjectRoot

    # Start containers
    $composeFile = Join-Path $ProjectRoot "docker\docker-compose.sim.yml"
    docker compose -f $composeFile up -d

    if ($LASTEXITCODE -ne 0) {
        Write-Host "  [ERROR] Docker Compose failed" -ForegroundColor Red
        exit 1
    }

    # List running containers
    Write-Host ""
    Write-Host "  Running containers:" -ForegroundColor Gray
    docker compose -f $composeFile ps --format "table {{.Name}}`t{{.Status}}"
    Write-Host ""

    # ===========================================================
    #  STEP 3: WAIT FOR ROS BRIDGE
    # ===========================================================
    Write-Host "=== [3/4] Waiting for ROS Bridge (port 9090) ===" -ForegroundColor Yellow
    $maxRetries = 60
    $retries = 0
    $ready = $false

    while ($retries -lt $maxRetries) {
        try {
            $tcp = New-Object System.Net.Sockets.TcpClient
            $tcp.Connect("localhost", 9090)
            $tcp.Close()
            $ready = $true
            break
        }
        catch {
            $retries++
            Write-Host "  Waiting... ($retries/$maxRetries)" -ForegroundColor Gray -NoNewline
            Write-Host "`r" -NoNewline
            Start-Sleep -Seconds 2
        }
    }

    if ($ready) {
        Write-Host "  [OK] ROS Bridge ready on port 9090      " -ForegroundColor Green
    }
    else {
        Write-Host "  [WARN] ROS Bridge not ready after $maxRetries attempts" -ForegroundColor Yellow
        Write-Host "  Continuing anyway - simulator can connect later" -ForegroundColor Gray
    }
}
else {
    Write-Host ""
    Write-Host "=== [2/4] Docker skipped (-NoDocker) ===" -ForegroundColor Gray
    Write-Host "=== [3/4] ROS Bridge check skipped ===" -ForegroundColor Gray
}

# ===========================================================
#  STEP 4: LAUNCH SIMULATOR
# ===========================================================
Write-Host ""
Write-Host "=== [4/4] Launching Simulator ===" -ForegroundColor Yellow

$exePath = Join-Path $ProjectRoot "build\RobotSimulator_CPP.exe"
if (-not (Test-Path $exePath)) {
    # Fallback: check Release subdirectory
    $exePath = Join-Path $ProjectRoot "build\Release\RobotSimulator_CPP.exe"
}

if (Test-Path $exePath) {
    Write-Host "  Executable: $exePath" -ForegroundColor White
    Start-Process $exePath
    Write-Host "  [OK] Simulator launched" -ForegroundColor Green
}
else {
    Write-Host "  [ERROR] Executable not found" -ForegroundColor Red
    Write-Host "  Expected: $exePath" -ForegroundColor Gray
    exit 1
}

Write-Host ""
Write-Host "======================================================" -ForegroundColor Green
Write-Host "  PIPELINE RUNNING                                     " -ForegroundColor Green
Write-Host "======================================================" -ForegroundColor Green
if (-not $NoDocker) {
    $stopCmd = "docker compose -f docker/docker-compose.sim.yml down"
    Write-Host "  Stop containers:  $stopCmd" -ForegroundColor Gray
}
Write-Host ""
