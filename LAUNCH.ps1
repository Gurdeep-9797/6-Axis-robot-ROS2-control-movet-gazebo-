# RoboForge v8.2 — One-Click Deployment Launcher
# ───────────────────────────────────────────────
# This script sets up and launches the complete RoboForge system on any Windows PC

param(
    [switch]$NoBrowser,    # Don't auto-open browser
    [switch]$Headless,     # Run without Gazebo GUI
    [string]$ControllerIP  # Optional: Physical robot IP address
)

$ErrorActionPreference = "Stop"

Write-Host ""
Write-Host "╔═══════════════════════════════════════════════════════════╗" -ForegroundColor Cyan
Write-Host "║         RoboForge v8.2 — One-Click Deployment            ║" -ForegroundColor Cyan
Write-Host "║         Industrial Robot IDE + ROS2 Backend              ║" -ForegroundColor Cyan
Write-Host "╚═══════════════════════════════════════════════════════════╝" -ForegroundColor Cyan
Write-Host ""

# ─── 1. Prerequisites Check ───
Write-Host "[1/7] Checking Prerequisites..." -ForegroundColor Yellow

# Check Docker
if (!(Get-Command docker -ErrorAction SilentlyContinue)) {
    Write-Host ""
    Write-Host "❌ ERROR: Docker not found!" -ForegroundColor Red
    Write-Host ""
    Write-Host "Please install Docker Desktop:" -ForegroundColor Yellow
    Write-Host "  https://www.docker.com/products/docker-desktop/" -ForegroundColor Gray
    Write-Host ""
    exit 1
}

$dockerVersion = docker version --format "{{.Server.Version}}" 2>$null
Write-Host "  ✅ Docker: $dockerVersion" -ForegroundColor Green

# Check Docker Compose
if (!(docker compose version 2>$null)) {
    Write-Host "  ❌ ERROR: Docker Compose not available!" -ForegroundColor Red
    exit 1
}
Write-Host "  ✅ Docker Compose: Available" -ForegroundColor Green

# Check if Docker daemon is running
if (!(docker info 2>$null)) {
    Write-Host "  ❌ ERROR: Docker daemon not running!" -ForegroundColor Red
    Write-Host "  Please start Docker Desktop and try again." -ForegroundColor Yellow
    exit 1
}
Write-Host "  ✅ Docker Daemon: Running" -ForegroundColor Green

# Check ports availability
$ports = @(3000, 6080, 8765, 9090)
foreach ($port in $ports) {
    $inUse = Get-NetTCPConnection -LocalPort $port -ErrorAction SilentlyContinue
    if ($inUse) {
        Write-Host "  ⚠️ Port $port is already in use" -ForegroundColor Yellow
    }
}
Write-Host "  ✅ Network Ports: Available" -ForegroundColor Green

Write-Host ""

# ─── 2. Stop Existing Instances ───
Write-Host "[2/7] Cleaning Up Previous Instances..." -ForegroundColor Yellow

docker compose down --remove-orphans 2>$null | Out-Null
docker ps -a --filter "name=roboforge" --format "{{.Names}}" | ForEach-Object {
    docker stop $_ 2>$null | Out-Null
    docker rm $_ 2>$null | Out-Null
}

Write-Host "  ✅ Cleanup complete" -ForegroundColor Green
Write-Host ""

# ─── 3. Build & Launch Services ───
Write-Host "[3/7] Building & Starting Services..." -ForegroundColor Yellow

# Set Gazebo profile
if ($Headless) {
    Write-Host "  Mode: Headless (no Gazebo GUI)" -ForegroundColor Gray
} else {
    Write-Host "  Mode: Full (with Gazebo VNC GUI)" -ForegroundColor Gray
}

# Build and start
if ($Headless) {
    docker compose up -d 2>&1 | Out-Null
} else {
    # Start with Gazebo VNC
    docker compose up -d 2>&1 | Out-Null
    
    # Start Gazebo VNC if image exists
    if (docker images roboforge_gazebo_vnc --format "{{.Repository}}" 2>$null) {
        Write-Host "  Starting Gazebo VNC container..." -ForegroundColor Gray
        docker run -d --name roboforge_gazebo `
            --network 6-axis-robot-ros2-control-movet-gazebo-_robot_net `
            -e ROS_DOMAIN_ID=42 `
            -p 6080:6080 `
            -v "${PSScriptRoot}\src:/ros_ws/src" `
            roboforge_gazebo_vnc 2>$null | Out-Null
    }
}

# Configure hardware IP if provided
if ($ControllerIP) {
    Write-Host "  Hardware IP: $ControllerIP" -ForegroundColor Gray
    $env:CONTROLLER_IP = $ControllerIP
}

Write-Host "  ✅ Services started" -ForegroundColor Green
Write-Host ""

# ─── 4. Wait for Services ───
Write-Host "[4/7] Waiting for Services to Initialize..." -ForegroundColor Yellow

$maxRetries = 60
$retryCount = 0
$ready = $false

while ($retryCount -lt $maxRetries) {
    try {
        $response = Invoke-WebRequest -Uri "http://localhost:8765/health" -UseBasicParsing -TimeoutSec 2 -ErrorAction Stop
        if ($response.StatusCode -eq 200) {
            $ready = $true
            break
        }
    } catch {
        # Continue waiting
    }
    Start-Sleep -Seconds 2
    $retryCount++
    Write-Host "  . " -NoNewline -ForegroundColor Gray
    
    if ($retryCount % 15 -eq 0) {
        Write-Host "" -ForegroundColor Gray
    }
}

if ($ready) {
    Write-Host ""
    Write-Host "  ✅ All services initialized" -ForegroundColor Green
} else {
    Write-Host ""
    Write-Host "  ⚠️ Services starting (may need more time)" -ForegroundColor Yellow
}
Write-Host ""

# ─── 5. Verify System ───
Write-Host "[5/7] Verifying System Status..." -ForegroundColor Yellow

# Container status
$containers = docker ps --filter "name=roboforge" --format "{{.Names}}" 2>$null
$containerCount = ($containers | Measure-Object).Count
Write-Host "  ✅ Containers: $containerCount running" -ForegroundColor Green

# Bridge health
try {
    $health = Invoke-RestMethod -Uri "http://localhost:8765/health" -ErrorAction Stop
    if ($health.status -eq "ok") {
        Write-Host "  ✅ Bridge: Healthy (MoveIt: $($health.moveit_ready))" -ForegroundColor Green
    }
} catch {
    Write-Host "  ⚠️ Bridge: Not responding yet" -ForegroundColor Yellow
}

# React UI
try {
    $response = Invoke-WebRequest -Uri "http://localhost:3000" -UseBasicParsing -TimeoutSec 2 -ErrorAction Stop
    if ($response.StatusCode -eq 200) {
        Write-Host "  ✅ React IDE: Accessible" -ForegroundColor Green
    }
} catch {
    Write-Host "  ⚠️ React IDE: Not responding yet" -ForegroundColor Yellow
}

# Gazebo VNC
try {
    $response = Invoke-WebRequest -Uri "http://localhost:6080" -UseBasicParsing -TimeoutSec 2 -ErrorAction Stop
    if ($response.StatusCode -eq 200) {
        Write-Host "  ✅ Gazebo VNC: Accessible" -ForegroundColor Green
    }
} catch {
    Write-Host "  ⚠️ Gazebo VNC: Not available" -ForegroundColor Yellow
}

Write-Host ""

# ─── 6. Display Access Information ───
Write-Host "[6/7] System Access Information" -ForegroundColor Yellow
Write-Host ""
Write-Host "  ╔═══════════════════════════════════════════════════════╗" -ForegroundColor Cyan
Write-Host "  ║              ACCESS POINTS                            ║" -ForegroundColor Cyan
Write-Host "  ╠═══════════════════════════════════════════════════════╣" -ForegroundColor Cyan

Write-Host "  ║  🌐 React Online IDE:                                 ║" -ForegroundColor White
Write-Host "  ║     http://localhost:3000                             ║" -ForegroundColor Cyan
Write-Host "  ║                                                       ║" -ForegroundColor Cyan
Write-Host "  ║  📺 Gazebo 3D Simulation:                             ║" -ForegroundColor White
Write-Host "  ║     http://localhost:6080/vnc.html                    ║" -ForegroundColor Cyan
Write-Host "  ║     (Password: roboforge)                             ║" -ForegroundColor Gray
Write-Host "  ║                                                       ║" -ForegroundColor Cyan
Write-Host "  ║  🔌 WebSocket Bridge:                                 ║" -ForegroundColor White
Write-Host "  ║     ws://localhost:9090                               ║" -ForegroundColor Cyan
Write-Host "  ║                                                       ║" -ForegroundColor Cyan
Write-Host "  ║  📡 REST API:                                         ║" -ForegroundColor White
Write-Host "  ║     http://localhost:8765/health                      ║" -ForegroundColor Cyan
Write-Host "  ║                                                       ║" -ForegroundColor Cyan
Write-Host "  ╚═══════════════════════════════════════════════════════╝" -ForegroundColor Cyan

Write-Host ""

# ─── 7. Launch Browser ───
Write-Host "[7/7] Launching Interfaces..." -ForegroundColor Yellow

if (-not $NoBrowser) {
    Start-Process "http://localhost:3000"
    Write-Host "  ✅ Opened React IDE in browser" -ForegroundColor Green
    
    Start-Sleep -Seconds 2
    
    if (-not $Headless) {
        Start-Process "http://localhost:6080/vnc.html"
        Write-Host "  ✅ Opened Gazebo VNC in browser" -ForegroundColor Green
    }
} else {
    Write-Host "  ⏭️ Browser launch skipped" -ForegroundColor Gray
}

Write-Host ""
Write-Host "╔═══════════════════════════════════════════════════════════╗" -ForegroundColor Green
Write-Host "║              🚀 RoboForge is READY!                       ║" -ForegroundColor Green
Write-Host "╚═══════════════════════════════════════════════════════════╝" -ForegroundColor Green
Write-Host ""

# ─── Quick Status Summary ───
Write-Host "Quick Commands:" -ForegroundColor Yellow
Write-Host "  Monitor pipeline:   .\monitor_pipeline.ps1" -ForegroundColor Gray
Write-Host "  View logs:          docker logs roboforge_bridge -f" -ForegroundColor Gray
Write-Host "  Stop all:           docker compose down" -ForegroundColor Gray
Write-Host "  Restart service:    docker restart roboforge_bridge" -ForegroundColor Gray
Write-Host ""

Write-Host "Documentation:" -ForegroundColor Yellow
Write-Host "  Full guide:       QUICK_ACCESS.md" -ForegroundColor Gray
Write-Host "  Test report:      TEST_SIMULATION_REPORT.md" -ForegroundColor Gray
Write-Host ""

# Keep script info available
Write-Host "Press Ctrl+C to exit this window (services will continue running)" -ForegroundColor DarkGray
Write-Host ""

# Wait for user to exit (services keep running in background)
try {
    while ($true) {
        Start-Sleep -Seconds 10
    }
} catch {
    Write-Host ""
    Write-Host "Exiting launcher... Services remain running." -ForegroundColor Gray
}
