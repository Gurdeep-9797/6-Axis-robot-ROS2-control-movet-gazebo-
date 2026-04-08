# RoboForge v8.2 — System Overview

> **Complete file inventory and system summary for the 6-Axis Robot ROS2 Control project.**
> Generated: 2026-04-08

---

## 1. Project Purpose

A unified robotics development platform for programming, simulating, and deploying a 6-axis industrial robot arm. Supports **simulation mode** (Gazebo + pseudo-hardware) and **real hardware mode** (ESP32 motor controller).

**Key capabilities**:
- Block-based robot programming IDE (React web + WPF desktop)
- Collision-aware motion planning via MoveIt 2 (OMPL + TRAC-IK)
- Real-time 3D visualization (Three.js / HelixToolkit)
- 250Hz simulated hardware loop with joint limit enforcement
- Physical robot control via ESP32 + PCA9685 + encoders
- WebSocket bridge (rosbridge-compatible) + REST API
- Accuracy analysis (planned vs actual trajectory RMSE)

---

## 2. Architecture Layers

```
┌─────────────────────────────────────────────────────┐
│  PRESENTATION LAYER (User Interface)                 │
│                                                      │
│  React Online IDE (Vite + Three.js + roslib)        │
│  WPF Offline Client (.NET 8 + HelixToolkit)         │
└──────────────────────┬──────────────────────────────┘
                       │ WebSocket ws://localhost:9090
                       ▼
┌─────────────────────────────────────────────────────┐
│  MIDDLEWARE LAYER (ROS2 Humble in Docker)            │
│                                                      │
│  roboforge_bridge: WebSocket server + REST API      │
│    - rosbridge v2 protocol + custom ops              │
│    - URDF-based FK + manipulability computation      │
│    - MoveIt IK proxy (/compute_ik, /compute_fk)      │
│    - Trajectory relay + program execution            │
│    - Data logger (JSONL, 2000-entry ring buffer)     │
│                                                      │
│  pseudo_hardware: 250Hz simulated robot              │
│    - Cubic Hermite spline interpolation              │
│    - Joint limit + velocity enforcement              │
│    - Publishes /joint_states at 250Hz                │
│                                                      │
│  robot_core: robot_state_publisher                   │
│    - Publishes /robot_description from URDF           │
│    - TF tree management                              │
│                                                      │
│  moveit: MoveIt 2 move_group                         │
│    - OMPL planners: RRTConnect, RRTstar, PRM          │
│    - TRAC-IK kinematics solver                       │
│    - Collision checking + trajectory optimization    │
└──────────────────────┬──────────────────────────────┘
                       │
           ┌───────────┴───────────┐
           ▼                       ▼
┌─────────────────────┐   ┌─────────────────────┐
│  SIMULATION MODE    │   │  REAL HARDWARE MODE  │
│                      │   │                     │
│  Gazebo Harmonic    │   │  ESP32 Controller    │
│  gz_ros2_control    │   │  PCA9685 PWM Driver  │
│  Physics engine     │   │  Quadrature Encoders │
│  1000Hz control     │   │  50Hz control loop   │
└─────────────────────┘   └─────────────────────┘
```

---

## 3. Complete File Inventory

### 3.1 ROS2 Packages (`src/`)

| Package | Type | Files | Status | Description |
|---------|------|-------|--------|-------------|
| `robot_description` | ament_cmake | 11 | ✅ Working | URDF/xacro models, joint limits, launch files |
| `robot_gazebo` | ament_cmake | 5 | ✅ Working | Gazebo Harmonic launch, controllers config |
| `robot_moveit_config` | ament_cmake | 9 | ✅ Working | MoveIt 2 config (OMPL, TRAC-IK, SRDF) |
| `roboforge_bridge` | ament_python | 9 | ✅ Working | WebSocket bridge, REST API, kinematics, nodes |
| `robot_hardware_bridge` | hybrid | 10 | ⚠️ Partial | Hardware relay, MoveIt adapter, SIM/REAL backends |
| `robot_msgs` | rosidl | 4 | ✅ Working | Custom messages (TrajectoryAck, ExecutionState) |
| `robot_analysis` | ament_python | 5 | ✅ Working | Accuracy analysis, RMSE computation |
| `robot_tests` | ament_cmake | 3 | ✅ Working | Motion stack integration tests (C++) |

### 3.2 Frontend Applications

| Application | Path | Tech Stack | Status | Description |
|------------|------|-----------|--------|-------------|
| React Online IDE | `NEW_UI/remix-of-roboflow-studio/` | Vite + React 18 + TS + Three.js | ✅ Working | Full robot programming IDE with 3D viewport |
| WPF Offline Client | `src/RoboForge.Wpf/` | .NET 8 WPF + HelixToolkit | ⚠️ Shell complete | Dark-themed UI shell, logic stubbed |

### 3.3 Firmware

| Firmware | Path | Target | Status | Description |
|----------|------|--------|--------|-------------|
| WiFi Controller | `firmware/wifi_controller/` | ESP32 | ✅ Working (open-loop) | TCP server, binary protocol, PCA9685 + encoders |
| Serial Controller | `firmware/serial_control/` | ESP32 | ⚠️ Preprocessor bug | Text protocol, PID DC motor control |

### 3.4 Configuration

| File | Purpose |
|------|---------|
| `docker-compose.yml` | Docker service orchestration (5 services) |
| `platformio.ini` | ESP32 firmware build configuration |
| `controller/hardware_map.yaml` | ESP32 pin mapping, I2C, servo/encoder config |
| `config/encoder_config.yaml` | Encoder calibration parameters |
| `config/robot_params.yaml` | Global system parameters |
| `version-lock.yaml` | Pinned versions for toolchain, vcpkg, ROS2 |

### 3.5 Tools & Utilities

| Script | Language | Purpose |
|--------|----------|---------|
| `tools/pipeline_diagnostics.py` | Python | End-to-end ROS2 pipeline health check |
| `tools/audit_moveit_config.py` | Python | Strict MoveIt config validation |
| `tools/select_ik_solver.py` | Python | KDL vs TRAC-IK solver selection |
| `tools/solidworks_urdf_pipeline.py` | Python | SolidWorks URDF export stub |
| `tools/test_e2e_compilation_flow.py` | Python | Cross-platform compilation verification |
| `tools/deploy.ps1` | PowerShell | C++ binary release packaging |
| `tools/import_urdf.ps1` | PowerShell | URDF import with Docker validation |
| `tools/install_vs_tools.ps1` | PowerShell | VS2022 + .NET 8 + CMake installer |
| `tools/retool_xaml.py` | Python | WPF MainWindow layout refactoring |
| `tools/urdf_post_processing/validate_and_process.py` | Python | URDF XML validation + path normalization |
| `tools/SolidWorksExporter/` | C# | SolidWorks add-in for URDF export |

### 3.6 Documentation

| File | Status | Description |
|------|--------|-------------|
| `API_AND_CONNECTIONS.md` | ✅ Updated 2026-04-08 | Complete API reference for all endpoints, protocols, data flows |
| `PHYSICAL_READ_MODE.md` | ✅ Updated 2026-04-08 | Physical hardware integration guide, pin maps, protocols |
| `README.md` | Original | Project overview |
| `ROBOFORGE_UNIFIED_ARCHITECTURE.md` | Original | Architecture documentation |
| `docs/USAGE.md` | Original | Setup and usage guide |
| `.qwen/TASKS.md` | ✅ New | Qwen task tracking (done/remaining) |
| `.qwen/SYSTEM_OVERVIEW.md` | ✅ New | This file |

### 3.7 Model Files

| File | Format | Description |
|------|--------|-------------|
| `robot.urdf` | URDF (processed) | 6-DOF industrial robot, ready for parsing |
| `test.sdf` | SDF 1.11 | Gazebo-compatible model definition |
| `urdf.xml` | Empty | Placeholder |

### 3.8 Archived Projects (`_archive/`)

| Project | Notes |
|---------|-------|
| `RoboForge_WPF/` | Legacy WPF with DSL compiler, kinematics engine, scene graph |
| `TeachPendant_WPF/` | Older teach pendant with EF Core, trajectory engine, simulation driver |

### 3.9 Build & CI

| File | Purpose |
|------|---------|
| `build_wpf.ps1` | WPF build script |
| `run_roboforge.ps1` | Master Docker launch script |
| `6-Axis-robot-ROS2-control-movet-gazebo-.sln` | Visual Studio solution (4 projects) |
| `.gitignore` | Git ignore rules |
| `.github/` | GitHub Actions workflows |

---

## 4. Technology Stack Summary

| Layer | Technology | Version |
|-------|-----------|---------|
| **ROS 2** | Humble Hawksbill | 2022 |
| **Motion Planning** | MoveIt 2 + OMPL | Latest |
| **IK Solver** | TRAC-IK | Configured |
| **Physics Sim** | Gazebo Harmonic | Latest |
| **Backend Bridge** | Python + websockets + aiohttp | 3.11+ |
| **React UI** | React 18 + TypeScript + Vite + Three.js | Latest |
| **WPF UI** | .NET 8 WPF + CommunityToolkit.Mvvm | 8.0 |
| **3D Desktop** | HelixToolkit.Wpf.SharpDX | 2.24.0 |
| **Firmware** | ESP32 Arduino + PlatformIO | Latest |
| **Motor Driver** | PCA9685 (I2C 16-channel PWM) | 50Hz |
| **Container** | Docker Compose | 3.8 |
| **CI/CD** | GitHub Actions | YAML |

---

## 5. Port Map

| Port | Protocol | Service | Purpose |
|------|----------|---------|---------|
| 3000 | HTTP | React UI | Online IDE (Vite dev server) |
| 8765 | HTTP | REST API | Health, hardware ports, logs |
| 9090 | WebSocket | rosbridge | IK/FK, trajectory, joint states |
| 5000 | TCP | ESP32 server | Binary protocol (real hardware) |
| 115200 | Serial | ESP32 USB | Text protocol (serial control) |

---

## 6. ROS2 Topic Summary

### Publishers

| Node | Publishes | Hz |
|------|-----------|-----|
| `pseudo_hardware_node` | `/joint_states` | 250 |
| `pseudo_hardware_node` | `/roboforge/hw_status` | 10 |
| `roboforge_bridge` | `/joint_states` (enriched) | 250 |
| `roboforge_bridge` | `/diagnostics` | 1 |
| `robot_state_publisher` | `/robot_description` | Latched |
| `robot_state_publisher` | `/tf`, `/tf_static` | Variable |
| `gazebo_error_node` | `/roboforge/tracking_alert` | On-demand |
| `encoder_interface_node` | `/encoder/raw` | 250 |
| `motor_controller_node` | `/roboforge/motor_config` (sub) | — |

### Subscribers

| Node | Subscribes | Purpose |
|------|-----------|---------|
| `roboforge_bridge` | `/joint_states` | Enrich with FK + manipulability |
| `roboforge_bridge` | `/planned_trajectory` | Execute trajectories |
| `roboforge_bridge` | `/roboforge/tracking_alert` | Halt on critical errors |
| `pseudo_hardware_node` | `/joint_trajectory_command` | Interpolate and simulate |
| `motor_controller_node` | `/joint_trajectory_command` | PI/FOC motor control |
| `motor_controller_node` | `/roboforge/motor_config` | Per-joint configuration |
| `encoder_interface_node` | `/roboforge/encoder_config` | Per-joint encoder config |
| `moveit_adapter_node` | `/bridge_controller/follow_joint_trajectory` | MoveIt trajectory relay |

### Services

| Service | Type | Server | Client |
|---------|------|--------|--------|
| `/compute_ik` | `moveit_msgs/GetPositionIK` | MoveIt | Bridge (proxy), UI |
| `/compute_fk` | `moveit_msgs/GetPositionFK` | MoveIt | Bridge (proxy) |
| `/roboforge/health_check` | `std_srvs/Trigger` | Bridge | UI, diagnostics |

---

## 7. Data Flow Diagrams

### 7.1 IK Request Flow

```
UI (React/WPF)
  │  WS: call_service /compute_ik
  ▼
roboforge_bridge
  │  ROS2: call /compute_ik
  ▼
MoveIt move_group
  │  TRAC-IK solver → collision-free IK solution
  ▼
roboforge_bridge
  │  WS: service_response (joint angles or error)
  ▼
UI (React/WPF)
```

### 7.2 Trajectory Execution Flow

```
UI: Run Program
  │  Compile blocks → TrajectorySegment[]
  │  WS: roboforge/execute_program
  ▼
roboforge_bridge
  │  For each segment:
  │    1. Publish to /joint_trajectory_command
  │    2. WS: roboforge/execution_progress (pc, total)
  │    3. Sleep(duration_s)
  ▼
pseudo_hardware_node
  │  Cubic Hermite spline interpolation @ 250Hz
  │  Publish /joint_states with positions + velocities
  ▼
roboforge_bridge
  │  Subscribe /joint_states → enrich with FK + manipulability
  │  WS: joint_states data stream → UI
  ▼
UI: Update 3D robot model in real-time
```

### 7.3 Physical Robot Flow

```
UI: Run Program
  │  WS: publish /joint_trajectory_command
  ▼
roboforge_bridge
  │  Route to RealControllerBackend
  ▼
TCP 192.168.1.100:5000
  │  Binary protocol (MSG_TRAJECTORY 0x01)
  ▼
ESP32 wifi_controller
  │  50Hz control loop:
  │    - Read encoders
  │    - Write servos via PCA9685
  │    - Publish joint states (MSG_JOINT_STATE 0x10)
  ▼
Servo motors move + Encoder feedback
  │
  ▼
ESP32 → TCP → RealControllerBackend → /joint_states → Bridge → UI
```

---

## 8. Current System Health (2026-04-08 01:00)

| Component | Status | Details |
|-----------|--------|---------|
| Docker Containers | ✅ All 5 running | core(healthy), moveit, pseudo_hw, bridge, frontend |
| ROS2 Topics | ✅ 11 active | /joint_states, /joint_trajectory_command, etc. |
| ROS2 Services | ✅ 3 key services | /compute_ik, /compute_fk, /roboforge/health_check |
| WebSocket Bridge | ✅ Port 9090 | Kinematics loaded, URDF fixed |
| REST API | ✅ Port 8765 | /health returns ok, kinematics_loaded: true |
| React UI | ✅ Port 3000 | Full IDE serving |
| WPF Client | ✅ Builds (0 errors) | UI shell complete, 10/11 ViewModels stubbed |
| Pseudo Hardware | ✅ 250Hz loop | Home: [0, -0.3, 0.2, 0, -0.5, 0] |
| MoveIt | ⚠️ Starting | May need 30-60s to become ready |
| URDF/Kinematics | ✅ Fixed | Processed URDF loaded, FK working |

---

## 9. Quick Start Commands

### Start Full Stack (Simulation)
```bash
docker compose up -d
# Wait 60s for all services
# Open http://localhost:3000 in browser
```

### Start with Gazebo Physics
```bash
docker compose --profile sim up -d
```

### Build WPF Client
```powershell
.\build_wpf.ps1
# Or: dotnet build src/RoboForge.Wpf/RoboForge.Wpf.csproj
```

### Run Pipeline Diagnostics
```bash
python tools/pipeline_diagnostics.py
# Outputs: pipeline_diagnostic_report.json
```

### Flash ESP32 Firmware
```bash
pio run -e wifi_controller -t upload
# Or for serial: pio run -e serial_control -t upload
```

### Check System Health
```bash
curl http://localhost:8765/health
# {"status":"ok","kinematics_loaded":true,"robot_loaded":true}
```

---

## 10. Known Issues & TODOs

See `.qwen/TASKS.md` for the complete task list with 20 remaining items.

**Top 5 priorities**:
1. **R1**: Fix WPF `EnumToBoolConverter` (5 min)
2. **R2**: Fix firmware/serial_control preprocessor bug (10 min)
3. **R3**: Create missing `safety_watchdog.py` (30 min)
4. **R4**: Remove orphaned root-level duplicate directories (5 min)
5. **R5**: Fix version string consistency (v7.0 → v8.2) (2 min)
