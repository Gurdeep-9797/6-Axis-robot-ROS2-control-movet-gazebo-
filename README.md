# RoboForge — Industrial 6-Axis Robot Arm IDE

[![Build & Release](https://github.com/Gurdeep-9797/6-Axis-robot-ROS2-control-movet-gazebo-/actions/workflows/build.yml/badge.svg)](https://github.com/Gurdeep-9797/6-Axis-robot-ROS2-control-movet-gazebo-/actions/workflows/build.yml)

A full-stack industrial robot control system featuring a **browser-based React IDE** with 3D visualization, **ROS 2 Humble / MoveIt 2** motion planning backend, **Gazebo Harmonic** physics simulation, and **ESP32 real-time hardware bridge** for physical robot control.

---

## Table of Contents

- [Architecture](#architecture)
- [Quick Start](#quick-start)
- [Project Structure](#project-structure)
- [Features](#features)
- [Docker Services](#docker-services)
- [API Reference](#api-reference)
- [Firmware & Hardware](#firmware--hardware)
- [Troubleshooting](#troubleshooting)
- [Tech Stack](#tech-stack)
- [Contributing](#contributing)
- [License](#license)

---

## Architecture

```
┌──────────────────────────────────────────────────────────────────────┐
│                         FRONTEND LAYER                                │
│                                                                       │
│  React Online IDE (Vite + React 18 + Three.js + Glassmorphism UI)    │
│    → ws://localhost:9090  (WebSocket — rosbridge-compatible)         │
│    → http://localhost:8765 (REST API — health checks)                │
└────────────────────────┬─────────────────────────────────────────────┘
                         │
                         ▼
┌──────────────────────────────────────────────────────────────────────┐
│                    BACKEND LAYER (Docker Containers)                   │
│  ROS_DOMAIN_ID=42  │  Network: robot_net                              │
│                                                                       │
│  ┌─────────────────────┐  ROS 2 Topics  ┌──────────────────────────┐│
│  │ roboforge_bridge    │◄──────────────►│ roboforge_moveit          ││
│  │  WS:9090, REST:8765 │  /compute_ik   │  MoveGroup: initialized   ││
│  │  Kinematics: ✅      │  /compute_fk   │  IK: KDL plugin           ││
│  └──────────┬──────────┘                └──────────┬───────────────┘│
│             │                                      │                 │
│             │ /joint_states (250Hz)                │                 │
│             │ /joint_trajectory_cmd                │                 │
│             ▼                                      ▼                 │
│  ┌─────────────────────┐         ┌────────────────────────────────┐ │
│  │ pseudo_hardware     │         │ roboforge_core                 │ │
│  │  250Hz sim loop     │◄───────►│  robot_state_publisher         │ │
│  │  Cubic Hermite      │         │  /robot_description, /tf       │ │
│  └─────────────────────┘         └────────────────────────────────┘ │
│                                                                       │
│  gazebo (profile=sim): Gazebo Harmonic + gz_ros2_control             │
└────────────────────────┬─────────────────────────────────────────────┘
                         │
              (Live Mode)│ TCP 192.168.1.100:5000 or USB-Serial
                         ▼
┌──────────────────────────────────────────────────────────────────────┐
│                      PHYSICAL HARDWARE                                │
│  ESP32 wifi_controller  → PCA9685 PWM → 6 Servos                    │
│  ESP32 serial_control   → PID DC motors + quadrature encoders        │
└──────────────────────────────────────────────────────────────────────┘
```

---

## Quick Start

### Prerequisites

| Tool | Version | Purpose |
|------|---------|---------|
| Docker Desktop | 24+ | Container runtime for ROS 2 / MoveIt / Gazebo |
| Node.js | 20+ | React IDE dev server (if running outside Docker) |
| PlatformIO | Latest | ESP32 firmware flashing |

### Launch the Full Stack

```bash
# 1. Start all services (ROS 2 Backend + MoveIt + Bridge + React UI)
docker compose up -d

# 2. Open the IDE in your browser
start http://localhost:8080
```

Wait ~30 seconds for all services to initialize. The system verifier container will
automatically test the pipeline health on startup.

### With Gazebo Physics Simulation

```bash
docker compose --profile sim up -d
```

---

## Project Structure

```
Robot-software/
│
├── NEW_UI/                            ← React Online IDE (Frontend)
│   └── remix-of-roboflow-studio/
│       ├── src/
│       │   ├── components/robot/      # 3D viewport, robot controls
│       │   ├── store/AppState.tsx     # State management
│       │   ├── services/              # BackendConnector (WebSocket)
│       │   └── engine/                # IKSolver, TrajectoryPlanner
│       ├── package.json
│       └── vite.config.ts
│
├── src/                               ← ROS 2 Workspace Source
│   ├── roboforge_bridge/              # WebSocket (9090) + REST (8765) bridge node
│   ├── robot_description/             # URDF/Xacro robot model
│   ├── robot_moveit_config/           # MoveIt 2 config (IK, OMPL, controllers)
│   ├── robot_gazebo/                  # Gazebo Harmonic simulation launch
│   ├── robot_hardware_bridge/         # Hardware relay (SIM + REAL backends)
│   ├── robot_msgs/                    # Custom ROS 2 messages
│   ├── robot_analysis/                # Accuracy analysis nodes
│   └── robot_tests/                   # Motion stack integration tests
│
├── firmware/                          ← ESP32 Firmware
│   ├── hardware_config.h              # Pin assignments & hardware constants
│   ├── wifi_controller/               # TCP binary protocol (port 5000)
│   └── serial_control/                # USB-serial text protocol
│
├── docker/                            ← Docker Configurations
│   ├── Dockerfile.base                # ROS 2 Humble base image
│   ├── Dockerfile.gazebo              # Gazebo Harmonic
│   ├── Dockerfile.roboforge_bridge    # Bridge node
│   ├── Dockerfile.ui                  # React UI
│   ├── docker-compose.real.yml        # Real hardware mode
│   ├── docker-compose.sim.yml         # Simulation mode
│   └── entrypoint.sh
│
├── config/                            ← Robot Parameters
│   ├── encoder_config.yaml            # Encoder settings per joint
│   └── robot_params.yaml              # Global robot parameters
│
├── controller/                        ← Hardware Mapping
│   └── hardware_map.yaml              # ESP32 + PCA9685 pin layout
│
├── tools/                             ← Utility Scripts
│   ├── SolidWorksExporter/            # SolidWorks URDF export plugin
│   ├── urdf_post_processing/          # URDF validation & processing
│   ├── solidworks_urdf_pipeline.py    # Full SW-to-URDF pipeline
│   ├── test_pipeline.py               # End-to-end pipeline verification
│   └── ...
│
├── tests/                             ← Test Suites
│   ├── pipeline_check.py
│   ├── sim_validation_suite.py
│   └── ...
│
├── docs/                              ← Documentation
│   ├── API_AND_CONNECTIONS.md         # Full API & protocol reference
│   └── USAGE.md                       # Hardware setup & usage guide
│
├── deploy/                            ← Deployment
│   └── README.md                      # Deployment instructions
│
├── .github/                           ← CI/CD
│   ├── workflows/build.yml
│   └── ISSUE_TEMPLATE/
│
├── docker-compose.yml                 # Main service orchestration
├── platformio.ini                     # ESP32 build configuration
├── .gitignore
└── README.md                          # ← You are here
```

---

## Features

### 🌐 Online IDE (React + Three.js)

- **3D Robot Viewport** — Three.js with `@react-three/fiber`, orbit controls, gizmo manipulation
- **Program Tree Editor** — Nested block-based robot programming (MoveJ, MoveL, SetDO, Wait, Gripper, If, While, For)
- **Scene Outliner** — Add/manage 3D objects (boxes, cylinders, spheres) as obstacles
- **Waypoint Management** — Click in 3D to place waypoints, drag to reposition
- **IK Mode Toggle** — MoveIt 2 (online) ↔ Offline Analytical ↔ Offline Numerical DLS
- **Motor PWM Control** — 5–100% speed slider, publishes to all 6 joints
- **Connection Status** — Live / Connecting / Offline indicator with auto-reconnect
- **Console** — Real-time execution log with color-coded messages
- **Health Check Modal** — Pre-flight validation before switching to Live mode

### ⚙️ Backend (ROS 2 + MoveIt 2)

- **MoveIt 2 IK Service** — KDL kinematics plugin, 5 attempts, 1.0s timeout
- **Trajectory Planning** — OMPL planners (RRTConnect, RRTstar, PRM)
- **250Hz Pseudo-Hardware Simulation** — Cubic Hermite spline interpolation with joint limit enforcement
- **REST API** — Health check, hardware port listing, data log endpoints
- **WebSocket Bridge** — rosbridge-compatible protocol for frontend communication
- **Automated Verifier** — Pipeline health check runs on `docker compose up`

### 🔌 Hardware Integration

- **ESP32 WiFi Controller** — TCP binary protocol for wireless servo control
- **ESP32 Serial Controller** — USB text protocol with PID closed-loop DC motor control
- **PCA9685 PWM Driver** — 16-channel PWM for servo control via I2C
- **Quadrature Encoders** — 4096 PPR (J1–J3), 2048 PPR (J4–J6)

---

## Docker Services

| Service | Container | Port | Purpose |
|---------|-----------|------|---------|
| **ROS Core** | `roboforge_core` | — | `robot_state_publisher`, TF tree |
| **MoveIt** | `roboforge_moveit` | — | MoveGroup, IK/FK services |
| **Pseudo HW** | `roboforge_pseudo_hw` | — | 250Hz simulated joint states |
| **Bridge** | `roboforge_bridge` | 9090 (WS), 8765 (REST) | WebSocket + REST API bridge |
| **Frontend** | `roboforge_frontend` | 8080 | React Online IDE |
| **Gazebo** | `roboforge_gazebo` | — | Physics simulation (optional, `--profile sim`) |
| **Verifier** | `roboforge_verifier` | — | Automated pipeline health check |

### Common Commands

```bash
# Start all services
docker compose up -d

# Start with Gazebo simulation
docker compose --profile sim up -d

# Check container status
docker ps --filter "name=roboforge"

# View bridge logs
docker logs roboforge_bridge -f

# Check health
curl http://localhost:8765/health

# Restart a specific service
docker restart roboforge_bridge

# Stop everything
docker compose down
```

---

## API Reference

### REST API (Port 8765)

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/health` | GET | System health check — returns MoveIt readiness, kinematics status |
| `/api/hardware/ports` | GET | List connected USB serial ports |
| `/api/logs?n=200` | GET | Last N entries from the data logger |

### WebSocket API (Port 9090)

The bridge uses a rosbridge-compatible JSON protocol over WebSocket. Key operations:

| Operation | Direction | Description |
|-----------|-----------|-------------|
| `robot_state` | Bridge → UI | Joint positions, velocities, TCP pose (250Hz) |
| `call_service /compute_ik` | UI → Bridge | Inverse kinematics via MoveIt |
| `call_service /compute_fk` | UI → Bridge | Forward kinematics via MoveIt |
| `roboforge/execute_program` | UI → Bridge | Execute compiled robot program |
| `roboforge/health` | UI → Bridge | Query system health |

> 📖 Full protocol documentation: [docs/API_AND_CONNECTIONS.md](docs/API_AND_CONNECTIONS.md)

### ROS 2 Topics

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/joint_states` | `sensor_msgs/JointState` | 250Hz | Joint positions + velocities |
| `/joint_trajectory_command` | `trajectory_msgs/JointTrajectory` | On-demand | Movement commands |
| `/robot_description` | `std_msgs/String` | Latched | URDF model |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | Variable | Coordinate transforms |

### ROS 2 Services

| Service | Type | Description |
|---------|------|-------------|
| `/compute_ik` | `moveit_msgs/GetPositionIK` | Inverse kinematics solver |
| `/compute_fk` | `moveit_msgs/GetPositionFK` | Forward kinematics solver |
| `/roboforge/health_check` | `std_srvs/Trigger` | System health check |

---

## Firmware & Hardware

### Wiring Diagram

| Component | Signal | ESP32 Pin | PCA9685 Channel |
|-----------|--------|-----------|-----------------|
| **PCA9685** | SDA / SCL | GPIO 21 / 22 | — |
| **Joint 1** | Servo / Encoder A,B | — / GPIO 34,35 | Ch 0 |
| **Joint 2** | Servo / Encoder A,B | — / GPIO 32,33 | Ch 1 |
| **Joint 3** | Servo / Encoder A,B | — / GPIO 25,26 | Ch 2 |
| **Joint 4** | Servo / Encoder A,B | — / GPIO 27,14 | Ch 3 |
| **Joint 5** | Servo / Encoder A,B | — / GPIO 12,13 | Ch 4 |
| **Joint 6** | Servo / Encoder A,B | — / GPIO 4,16 | Ch 5 |

> ⚠️ **NEVER** power servos from the ESP32. Use an external **5V 6A PSU** on PCA9685 V+.

### Flashing Firmware

```bash
# WiFi mode (TCP binary protocol, port 5000)
pio run -e wifi_controller -t upload

# Serial mode (USB text protocol, 115200 baud)
pio run -e serial_control -t upload

# Serial mode with DC encoder support
pio run -e serial_control_encoder -t upload
```

### Connecting Physical Hardware

```powershell
# Set ESP32 IP address
$env:CONTROLLER_IP = "192.168.1.100"

# Start the stack
docker compose up -d

# The bridge will auto-connect to the ESP32 at the configured IP
```

### Serial Protocol Format

```
<J0:90.50,J1:45.00,J2:30.00,J3:0.00,J4:15.00,J5:0.00>
```

> 📖 Full hardware guide: [docs/USAGE.md](docs/USAGE.md)

---

## Troubleshooting

| Problem | Cause | Fix |
|---------|-------|-----|
| UI shows blank white page | JavaScript error | Check browser console (F12) for errors |
| WebSocket connection fails | Bridge not running | `docker ps` — verify `roboforge_bridge` is Up |
| MoveIt IK returns error -31 | Target pose unreachable | Try reachable poses within robot workspace |
| Joint states not updating | Pseudo hardware crashed | `docker logs roboforge_pseudo_hw` |
| Servos are jittery | Insufficient power supply | Use 5V 6A PSU, ensure shared ground |
| COM port unavailable | Port in use | Close other serial monitor applications |

### Pipeline Verification

```bash
# Run end-to-end test
python tools/test_pipeline.py
```

Expected output:
```
[1] REST API:       ✅ PASS  ({"moveit_ready":true})
[2] IK via MoveIt:  ✅ PASS  (responses in 16-47ms)
[3] Joint States:   ✅ PASS  (250Hz from pseudo_hardware_node)
```

---

## Tech Stack

| Layer | Technology |
|-------|------------|
| **Frontend** | React 18, TypeScript, Vite, Three.js, TailwindCSS, Shadcn UI |
| **3D Engine** | `@react-three/fiber` + `@react-three/drei` |
| **State** | Zustand |
| **Backend** | ROS 2 Humble, Python `rclpy`, `websockets`, `aiohttp` |
| **Motion Planning** | MoveIt 2 (KDL kinematics, OMPL planners) |
| **Simulation** | Gazebo Harmonic, `gz_ros2_control` |
| **Firmware** | Arduino (ESP32), PlatformIO |
| **Hardware** | ESP32, PCA9685, Quadrature Encoders |
| **Containers** | Docker, Docker Compose |
| **CI/CD** | GitHub Actions |

---

## Contributing

1. Fork the repository
2. Create a feature branch: `git checkout -b feature/my-feature`
3. Commit your changes: `git commit -m "Add my feature"`
4. Push to the branch: `git push origin feature/my-feature`
5. Open a Pull Request

### Branch Naming

- `feature/` — New features
- `fix/` — Bug fixes
- `docs/` — Documentation changes
- `refactor/` — Code restructuring

---

## License

This project is part of an academic/personal robotics research project.

---

*Built with ROS 2 Humble, React Three Fiber, MoveIt 2, and Gazebo Harmonic.*
