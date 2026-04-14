# RoboForge v8.2 — Online System Guide

> **Browser-based robot programming with ROS2/MoveIt/Gazebo backend**

---

## 🚀 Quick Start

### One-Command Launch

```powershell
# Start entire online system
docker compose up -d --profile sim

# Open browser
start http://localhost:3000        # React IDE
start http://localhost:6080/vnc.html  # Gazebo 3D GUI
```

---

## 🌐 Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    FRONTEND LAYER                                │
│                                                                  │
│  React Online IDE (Vite + React 18 + Three.js)                 │
│    → ws://localhost:9090 (BackendConnector.ts)                  │
│    → http://localhost:8765 (REST health checks)                 │
└──────────────────────────────┬──────────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────────┐
│                    BACKEND (Docker Containers)                    │
│  ROS_DOMAIN_ID=42  │  Network: robot_net                         │
│                                                                  │
│  ┌──────────────────────┐    ┌─────────────────────────────┐   │
│  │ roboforge_bridge     │◄──►│ roboforge_moveit            │   │
│  │  WS 0.0.0.0:9090     │    │  MoveGroup: initialized      │   │
│  │  REST 0.0.0.0:8765   │    │  IK: /compute_ik (responding)│   │
│  │  Kinematics: ✅       │    │  KDL kinematics plugin       │   │
│  └──────────┬───────────┘    └──────────────┬──────────────┘   │
│             │                                │                  │
│             ▼                                ▼                  │
│  ┌──────────────────────┐    ┌─────────────────────────────┐   │
│  │ pseudo_hardware      │    │ roboforge_core              │   │
│  │  250Hz sim loop      │◄──►│ robot_state_publisher       │   │
│  │  Cubic Hermite       │    │ /robot_description          │   │
│  │  Home: [0,-0.3,0.2,  │    │ /tf, /tf_static             │   │
│  │        0,-0.5,0]     │    │                             │   │
│  └──────────────────────┘    └─────────────────────────────┘   │
│                                                                  │
│  gazebo: Gazebo Harmonic + gz_ros2_control                      │
└──────────────────────────────┬──────────────────────────────────┘
                               │
                    (Live Mode)│ TCP 192.168.1.100:5000
                               ▼
┌─────────────────────────────────────────────────────────────────┐
│                    PHYSICAL HARDWARE                              │
│  ESP32 wifi_controller → PCA9685 PWM → 6 Servos                  │
│  ESP32 serial_control → PID DC motors + encoders                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## 📦 Docker Services

| Service | Container | Port | Health |
|---------|-----------|------|--------|
| **React IDE** | roboforge_frontend | 3000 | ✅ HTTP 200 |
| **Bridge** | roboforge_bridge | 9090, 8765 | ✅ Healthy |
| **MoveIt** | roboforge_moveit | — | ✅ IK Ready |
| **Gazebo** | roboforge_gazebo | 6080 | ✅ VNC Active |
| **Pseudo HW** | roboforge_pseudo_hw | — | ✅ 250Hz |
| **ROS Core** | roboforge_core | — | ✅ Healthy |

---

## 🔗 Access Points

| Interface | URL | Purpose |
|-----------|-----|---------|
| **React IDE** | http://localhost:3000 | Full robot programming IDE |
| **Gazebo GUI** | http://localhost:6080/vnc.html | 3D simulation with VNC |
| **REST API** | http://localhost:8765/health | System health check |
| **WebSocket** | ws://localhost:9090 | rosbridge-compatible |

---

## 🛠️ Monitoring & Troubleshooting

### Check System Status
```powershell
# View all containers
docker ps --filter "name=roboforge"

# Check bridge health
curl http://localhost:8765/health

# View logs
docker logs roboforge_bridge -f
docker logs roboforge_gazebo -f
```

### Restart Services
```powershell
# Restart specific service
docker restart roboforge_bridge

# Restart all
docker compose restart
```

### Stop All Services
```powershell
docker compose down
```

---

## 📡 ROS2 System

### Active Nodes (8 total)
- `/move_group` — MoveIt motion planning
- `/pseudo_hardware_node` — 250Hz joint simulation
- `/roboforge_bridge` — WebSocket + REST bridge
- `/robot_state_publisher` (x2) — URDF processing
- `/transform_listener` — TF2 transforms

### Active Topics (20 total)
**Critical:**
- `/joint_states` @ 250Hz — Joint positions
- `/joint_trajectory_command` — Movement commands
- `/planned_trajectory` — MoveIt planned paths
- `/robot_description` — URDF model
- `/tf` — Coordinate transforms

### Services (45+ total)
**Critical:**
- `/compute_ik` — Inverse kinematics
- `/compute_fk` — Forward kinematics
- `/roboforge/health_check` — System health

---

## 🌍 Deployment to Another PC

### Prerequisites
- Windows 10/11 with Docker Desktop
- 8GB RAM minimum (16GB recommended)
- 10GB free disk space

### Steps
1. Copy entire project folder to target PC
2. Open PowerShell in project root
3. Run: `docker compose up -d --profile sim`
4. Wait 60-90 seconds for services to initialize
5. Open browser: http://localhost:3000

### Using Deploy Package
```powershell
# On source PC
.\CREATE_DEPLOY_PACKAGE.ps1

# Copy deploy_package folder to target PC
# On target PC
cd deploy_package
.\LAUNCH.ps1
```

---

## 🔌 Hardware Integration

### Connect Physical Robot
```powershell
# Set robot IP
$env:CONTROLLER_IP=192.168.1.100

# Restart bridge with hardware config
docker restart roboforge_bridge
```

### ESP32 Firmware
- **WiFi Controller**: TCP binary protocol (port 5000)
- **Serial Controller**: USB-serial text protocol
- Flash with PlatformIO: `pio run -e wifi_controller -t upload`

---

## 📊 Performance Metrics

| Metric | Value | Status |
|--------|-------|--------|
| Joint state rate | ~250Hz | ✅ Target met |
| IK response time | < 5s | ✅ Acceptable |
| WebSocket latency | < 100ms | ✅ Good |
| REST API response | < 50ms | ✅ Excellent |
| React UI load | < 2s | ✅ Fast |
| Gazebo VNC FPS | 30fps | ✅ Smooth |

---

*Online System v8.2 | Last Updated: 2026-04-14*
