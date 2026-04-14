# RoboForge v8.2 — Portable Deployment Package

## Quick Start

### Prerequisites
1. **Windows 10/11** with Docker Desktop installed
2. **Docker Desktop** running (https://www.docker.com/products/docker-desktop/)
3. **Git** (optional, for version control)

### One-Click Launch

1. **Extract** this package to any location
2. **Open PowerShell** in the extracted directory
3. **Run**: `.\LAUNCH.ps1`
4. **Wait** ~60 seconds for all services to start
5. **Browser opens automatically** with React IDE and Gazebo GUI

### Manual Launch

``powershell
# Start all services
docker compose up -d

# Wait for services
Start-Sleep -Seconds 30

# Open interfaces
start http://localhost:3000          # React Online IDE
start http://localhost:6080/vnc.html # Gazebo 3D GUI (VNC)
``

### Access Points

| Interface | URL | Description |
|-----------|-----|-------------|
| **React Online IDE** | http://localhost:3000 | Full robot programming IDE |
| **Gazebo 3D GUI** | http://localhost:6080/vnc.html | 3D simulation with VNC |
| **Bridge REST API** | http://localhost:8765/health | System health check |
| **WebSocket Bridge** | ws://localhost:9090 | rosbridge-compatible |

### With Physical Robot

``powershell
# Launch with robot IP
.\LAUNCH.ps1 -ControllerIP 192.168.1.100
``

### Headless Mode (No Gazebo GUI)

``powershell
.\LAUNCH.ps1 -Headless
``

## System Architecture

``
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
``

## Monitoring & Troubleshooting

### Check System Status
``powershell
# View all containers
docker ps --filter "name=roboforge"

# Check bridge health
curl http://localhost:8765/health

# View logs
docker logs roboforge_bridge -f
docker logs roboforge_gazebo -f
``

### Restart Services
``powershell
# Restart specific service
docker restart roboforge_bridge

# Restart all
docker compose restart
``

### Stop All Services
``powershell
docker compose down
``

## Package Contents

``
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
``

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
**Build Date**: 2026-04-14  
**Docker Image**: robotics_base:latest
