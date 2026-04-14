# RoboForge v8.2 — Portable Deployment Package Creator
# ───────────────────────────────────────────────────
# Creates a complete deployment package for transfer to another PC

$ErrorActionPreference = "Stop"

Write-Host ""
Write-Host "╔═══════════════════════════════════════════════════════════╗" -ForegroundColor Cyan
Write-Host "║      RoboForge v8.2 — Deployment Package Creator         ║" -ForegroundColor Cyan
Write-Host "╚═══════════════════════════════════════════════════════════╝" -ForegroundColor Cyan
Write-Host ""

# Get project root
$ProjectRoot = $PSScriptRoot
$DeployDir = Join-Path $ProjectRoot "deploy_package"

Write-Host "[1/5] Preparing Deployment Package..." -ForegroundColor Yellow
Write-Host "  Project Root: $ProjectRoot" -ForegroundColor Gray
Write-Host "  Deploy Dir: $DeployDir" -ForegroundColor Gray
Write-Host ""

# Clean previous package
if (Test-Path $DeployDir) {
    Write-Host "  Removing previous package..." -ForegroundColor Gray
    Remove-Item $DeployDir -Recurse -Force
}

# Create directory structure
Write-Host "[2/5] Creating Package Structure..." -ForegroundColor Yellow

$dirs = @(
    "deploy_package",
    "deploy_package\src",
    "deploy_package\NEW_UI\remix-of-roboflow-studio",
    "deploy_package\tools",
    "deploy_package\scripts",
    "deploy_package\docs",
    "deploy_package\docker"
)

foreach ($dir in $dirs) {
    $path = Join-Path $ProjectRoot $dir
    if (!(Test-Path $path)) {
        New-Item -Path $path -ItemType Directory -Force | Out-Null
    }
}

Write-Host "  ✅ Directory structure created" -ForegroundColor Green
Write-Host ""

# ─── Copy Core Files ───
Write-Host "[3/5] Copying Core Files..." -ForegroundColor Yellow

# Docker Compose
Copy-Item "docker-compose.yml" "deploy_package\" -Force
Write-Host "  ✅ docker-compose.yml" -ForegroundColor Green

# Launch script
Copy-Item "LAUNCH.ps1" "deploy_package\" -Force
Write-Host "  ✅ LAUNCH.ps1 (one-click launcher)" -ForegroundColor Green

# Documentation
@("README.md", "QUICK_ACCESS.md", "TEST_SIMULATION_REPORT.md", "API_AND_CONNECTIONS.md") | ForEach-Object {
    if (Test-Path $_) {
        Copy-Item $_ "deploy_package\docs\" -Force
        Write-Host "  ✅ $_" -ForegroundColor Green
    }
}

Write-Host ""

# ─── Copy Source Code ───
Write-Host "[4/5] Copying Source Code..." -ForegroundColor Yellow

# ROS2 Source (src directory)
$srcItems = @(
    "robot_description",
    "robot_moveit_config",
    "robot_gazebo",
    "roboforge_bridge",
    "robot_msgs",
    "robot_hardware_bridge",
    "robot_analysis"
)

foreach ($item in $srcItems) {
    $srcPath = Join-Path "src" $item
    $destPath = Join-Path "deploy_package\src" $item
    if (Test-Path $srcPath) {
        if (Test-Path $destPath) {
            Remove-Item $destPath -Recurse -Force
        }
        Copy-Item $srcPath $destPath -Recurse -Force
        Write-Host "  ✅ src/$item" -ForegroundColor Green
    }
}

# React Online IDE
$reactFiles = @(
    "package.json",
    "package-lock.json",
    "Dockerfile",
    "vite.config.ts",
    "tsconfig.json",
    "index.html",
    "tailwind.config.ts",
    "postcss.config.js",
    "eslint.config.js",
    ".gitignore",
    "src",
    "public"
)

foreach ($file in $reactFiles) {
    $srcPath = Join-Path "NEW_UI\remix-of-roboflow-studio" $file
    $destPath = Join-Path "deploy_package\NEW_UI\remix-of-roboflow-studio" $file
    if (Test-Path $srcPath) {
        if (Test-Path $destPath) {
            Remove-Item $destPath -Recurse -Force
        }
        Copy-Item $srcPath $destPath -Recurse -Force
    }
}
Write-Host "  ✅ React Online IDE (full source)" -ForegroundColor Green

# Docker files
Copy-Item "docker\*" "deploy_package\docker\" -Force -ErrorAction SilentlyContinue
Write-Host "  ✅ Docker files (Gazebo VNC)" -ForegroundColor Green

# Tools
Copy-Item "tools\parse_joints.py" "deploy_package\tools\" -Force -ErrorAction SilentlyContinue
Write-Host "  ✅ Tools (joint parser)" -ForegroundColor Green

Write-Host ""

# ─── Create README for Package ───
Write-Host "[5/5] Creating Package README..." -ForegroundColor Yellow

$readme = @"
# RoboForge v8.2 — Portable Deployment Package

## Quick Start

### Prerequisites
1. **Windows 10/11** with Docker Desktop installed
2. **Docker Desktop** running (https://www.docker.com/products/docker-desktop/)
3. **Git** (optional, for version control)

### One-Click Launch

1. **Extract** this package to any location
2. **Open PowerShell** in the extracted directory
3. **Run**: ``.\LAUNCH.ps1``
4. **Wait** ~60 seconds for all services to start
5. **Browser opens automatically** with React IDE and Gazebo GUI

### Manual Launch

````powershell
# Start all services
docker compose up -d

# Wait for services
Start-Sleep -Seconds 30

# Open interfaces
start http://localhost:3000          # React Online IDE
start http://localhost:6080/vnc.html # Gazebo 3D GUI (VNC)
````

### Access Points

| Interface | URL | Description |
|-----------|-----|-------------|
| **React Online IDE** | http://localhost:3000 | Full robot programming IDE |
| **Gazebo 3D GUI** | http://localhost:6080/vnc.html | 3D simulation with VNC |
| **Bridge REST API** | http://localhost:8765/health | System health check |
| **WebSocket Bridge** | ws://localhost:9090 | rosbridge-compatible |

### With Physical Robot

````powershell
# Launch with robot IP
.\LAUNCH.ps1 -ControllerIP 192.168.1.100
````

### Headless Mode (No Gazebo GUI)

````powershell
.\LAUNCH.ps1 -Headless
````

## System Architecture

````
┌─────────────────────────────────────────────────┐
│              FRONTEND LAYER                       │
│                                                   │
│  React Online IDE (Vite + React 18 + Three.js)  │
│    → ws://localhost:9090                         │
│    → http://localhost:8765                       │
└────────────────┬────────────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────────────────┐
│              BACKEND (Docker)                     │
│                                                   │
│  ROS2 Humble + MoveIt 2 + Gazebo Harmonic       │
│                                                   │
│  ┌──────────────┐    ┌──────────────┐           │
│  │ roboforge    │◄──►│ roboforge    │           │
│  │ bridge       │    │ moveit       │           │
│  │ WS:9090      │    │ IK/FK        │           │
│  │ REST:8765    │    │              │           │
│  └──────┬───────┘    └──────┬───────┘           │
│         │                   │                    │
│         ▼                   ▼                    │
│  ┌──────────────┐    ┌──────────────┐           │
│  │ pseudo_hw    │    │ ros_core     │           │
│  │ 250Hz sim    │    │ robot_state  │           │
│  │              │    │ publisher    │           │
│  └──────────────┘    └──────────────┘           │
│                                                   │
│  gazebo (VNC): 3D physics simulation             │
└─────────────────────────────────────────────────┘
````

## Monitoring & Troubleshooting

### Check System Status
````powershell
# View all containers
docker ps --filter "name=roboforge"

# Check bridge health
curl http://localhost:8765/health

# View logs
docker logs roboforge_bridge -f
docker logs roboforge_gazebo -f
````

### Restart Services
````powershell
# Restart specific service
docker restart roboforge_bridge

# Restart all
docker compose restart
````

### Stop All Services
````powershell
docker compose down
````

## Package Contents

````
deploy_package/
├── LAUNCH.ps1                    # One-click launcher
├── docker-compose.yml            # Service orchestration
├── docs/                         # Documentation
│   ├── README.md
│   ├── QUICK_ACCESS.md
│   ├── TEST_SIMULATION_REPORT.md
│   └── API_AND_CONNECTIONS.md
├── src/                          # ROS2 source code
│   ├── robot_description/        # URDF robot model
│   ├── robot_moveit_config/      # MoveIt 2 configuration
│   ├── robot_gazebo/             # Gazebo simulation
│   ├── roboforge_bridge/         # WebSocket + REST bridge
│   ├── robot_msgs/               # Custom ROS2 messages
│   └── robot_hardware_bridge/    # Hardware integration
├── NEW_UI/remix-of-roboflow-studio/  # React Online IDE
├── docker/                       # Docker files (Gazebo VNC)
└── tools/                        # Utility scripts
````

## System Requirements

- **OS**: Windows 10/11 (Pro/Enterprise for Docker)
- **RAM**: 8GB minimum, 16GB recommended
- **CPU**: 4+ cores recommended
- **Disk**: 10GB free space
- **Docker**: Docker Desktop with WSL2 backend

## Known Issues

1. **WPF Offline Client**: Requires HelixToolkit.SharpDX fixes (see main repo)
2. **Gazebo GUI**: Uses VNC web viewer (no X server needed)
3. **First Launch**: Takes 60-90 seconds to build and start

## Support

- Full documentation: docs/ directory
- API reference: docs/API_AND_CONNECTIONS.md
- Test report: docs/TEST_SIMULATION_REPORT.md

---

**Version**: v8.2  
**Build Date**: $(Get-Date -Format "yyyy-MM-dd")  
**Docker Image**: robotics_base:latest
"@

$readme | Out-File -FilePath "deploy_package\README.md" -Encoding UTF8

Write-Host "  ✅ README.md created" -ForegroundColor Green
Write-Host ""

# Calculate package size
$packageSize = (Get-ChildItem $DeployDir -Recurse | Measure-Object -Property Length -Sum).Sum / 1MB
Write-Host "Package Size: $([math]::Round($packageSize, 2)) MB" -ForegroundColor Yellow
Write-Host ""

Write-Host "╔═══════════════════════════════════════════════════════════╗" -ForegroundColor Green
Write-Host "║          ✅ DEPLOYMENT PACKAGE READY!                     ║" -ForegroundColor Green
Write-Host "╚═══════════════════════════════════════════════════════════╝" -ForegroundColor Green
Write-Host ""
Write-Host "Location: $DeployDir" -ForegroundColor Cyan
Write-Host ""
Write-Host "To deploy on another PC:" -ForegroundColor Yellow
Write-Host "  1. Copy the 'deploy_package' folder to USB/network" -ForegroundColor Gray
Write-Host "  2. On target PC, ensure Docker Desktop is installed" -ForegroundColor Gray
Write-Host "  3. Run: .\LAUNCH.ps1" -ForegroundColor Gray
Write-Host ""
