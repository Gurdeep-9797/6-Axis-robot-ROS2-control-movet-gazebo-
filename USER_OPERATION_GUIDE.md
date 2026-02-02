# USER OPERATION GUIDE — Industrial Robot Platform

## Table of Contents

1. [One-Time Setup](#1-one-time-setup)
2. [Folder Structure](#2-folder-structure)
3. [Environment Variables](#3-environment-variables)
4. [Docker Build & Run](#4-docker-build--run)
5. [Running SIM Mode](#5-running-sim-mode)
6. [Running REAL Mode](#6-running-real-mode)
7. [Using Simulator as Development Platform](#7-using-simulator-as-development-platform)
8. [Optional UI Usage](#8-optional-ui-usage)
9. [Enabling Analysis Modules](#9-enabling-analysis-modules)
10. [Logs & Debugging](#10-logs--debugging)
11. [Deployment & Updates](#11-deployment--updates)
12. [Quick Reference Commands](#12-quick-reference-commands)

---

## Terminology (CRITICAL)

| Term | Definition |
|------|------------|
| **USER CONTROLLER** | What the user interacts with: Simulator UI, Web UI, Desktop UI, Joystick. |
| **SIMULATOR** | Gazebo or any physics simulator. Non-real-time. Visualization + physics ONLY. |
| **RT CONTROLLER** | ESP32 (or RT Linux MCU). Owns GPIO, PWM, PID, Safety. The ONLY real-time executor. |
| **ROS DOMAIN** | ROS 2 + MoveIt. Planning, IK, FK, smoothing. NEVER real-time. |
| **HARDWARE BRIDGE** | Transport + schema validation ONLY. No logic, no compensation. |

---

## 1. One-Time Setup

### 1.1 Prerequisites

| Requirement          | Version / Spec                          | Purpose                    |
|----------------------|-----------------------------------------|----------------------------|
| Windows 10/11        | 64-bit, Build 19041+                    | Host OS                    |
| Docker Desktop       | 4.0+ with WSL2 backend                  | Container runtime          |
| VS Code              | Latest                                  | Development IDE            |
| Git                  | 2.30+                                   | Version control            |
| X Server (optional)  | VcXsrv or similar                       | GUI forwarding (Simulator) |

### 1.2 Install Docker Desktop

1. Download Docker Desktop from https://docker.com
2. Run installer
3. Enable WSL2 backend during installation
4. Restart computer when prompted
5. Verify installation:
   ```powershell
   docker --version
   docker compose version
   ```

### 1.3 Configure WSL2 Memory (Recommended)

Create or edit `%USERPROFILE%\.wslconfig`:
```ini
[wsl2]
memory=8GB
processors=4
swap=2GB
```

Restart WSL:
```powershell
wsl --shutdown
```

### 1.4 Clone Repository

```powershell
cd D:\
git clone <repository-url> robot_system
cd robot_system
```

### 1.5 First-Time Environment Setup

Copy the example environment file:
```powershell
copy .env.example .env
```

Edit `.env` with your configuration (see Section 3).

---

## 2. Folder Structure

```
D:\robot_system\
│
├── .env                          # Environment configuration (USER EDITS THIS)
├── .env.example                  # Template for .env
├── docker-compose.yml            # Main compose file
├── docker-compose.sim.yml        # SIM mode overrides
├── docker-compose.real.yml       # REAL mode overrides
│
├── docker/                       # Docker build files
│   ├── Dockerfile.ros            # ROS 2 base image
│   ├── Dockerfile.gazebo         # Simulator image
│   └── Dockerfile.ui             # Optional UI image
│
├── src/                          # ROS 2 workspace source
│   ├── robot_description/        # URDF, meshes, configs
│   ├── robot_moveit_config/      # MoveIt configuration (IK, FK, planning)
│   ├── robot_hardware_bridge/    # Hardware Bridge node (transport ONLY)
│   ├── robot_gazebo/             # Simulator config (SIM mode execution)
│   └── robot_analysis/           # Accuracy comparison nodes
│
├── firmware/                     # RT CONTROLLER code (ESP32)
│   └── esp32_robot_controller/   # PID, PWM, Encoder, Safety logic
│
├── controller/                   # RT CONTROLLER configuration
│   └── hardware_map.yaml         # GPIO and Servo channel mappings
│
├── config/                       # Shared configuration
├── logs/                         # Runtime logs (auto-created)
├── scripts/                      # Helper scripts
└── docs/                         # Documentation
```

---

## 3. Environment Variables

All configuration is managed through the `.env` file. **Never edit code to change settings.**

### 3.1 Core Variables

| Variable            | Values              | Description                              |
|---------------------|---------------------|------------------------------------------|
| `ROBOT_MODE`        | `SIM` / `REAL`      | Execution mode selection                 |
| `ROBOT_NAME`        | String              | Identifier for this robot instance       |
| `ROS_DOMAIN_ID`     | 0-232               | ROS 2 domain isolation                   |

### 3.2 RT Controller Variables

These variables configure the connection to the **RT Controller** (ESP32 hardware).

| Variable              | Values              | Description                              |
|-----------------------|---------------------|------------------------------------------|
| `RT_CONTROLLER_TYPE`  | `FAKE` / `REAL`     | RT Controller backend selection          |
| `RT_CONTROLLER_IP`    | IP address          | RT Controller network address            |
| `RT_CONTROLLER_PORT`  | Port number         | RT Controller communication port         |
| `RT_CONTROLLER_PROTO` | `SERIAL`/`TCP`/`UDP`| Communication protocol                   |

### 3.3 UI Variables (User Controller)

| Variable            | Values              | Description                              |
|---------------------|---------------------|------------------------------------------|
| `ENABLE_UI`         | `true` / `false`    | Enable/disable UI container              |
| `UI_TYPE`           | `web` / `desktop`   | Type of User Controller UI to launch     |
| `UI_PORT`           | Port number         | Web UI port (if web type)                |

### 3.4 Simulation Variables

| Variable            | Values              | Description                              |
|---------------------|---------------------|------------------------------------------|
| `GAZEBO_HEADLESS`   | `true` / `false`    | Run Simulator without GUI                |
| `GAZEBO_WORLD`      | Filename            | World file to load                       |
| `SIM_REAL_TIME`     | `true` / `false`    | Attempt real-time simulation             |

### 3.5 Analysis Variables

| Variable                | Values          | Description                          |
|-------------------------|-----------------|--------------------------------------|
| `ENABLE_ANALYSIS`       | `true` / `false`| Enable analysis nodes                |
| `ENABLE_ACCURACY_ANALYSIS`| `true`/`false`| Enable planned vs actual comparison  |
| `LOG_LEVEL`             | `DEBUG`/`INFO`/`WARN`/`ERROR` | Log verbosity    |
| `LOG_TO_FILE`           | `true` / `false`| Write logs to files                  |

### 3.6 Example .env File

```bash
# ========================================
# ROBOT SYSTEM CONFIGURATION
# ========================================

# Core Settings
ROBOT_MODE=SIM
ROBOT_NAME=robot_arm_01
ROS_DOMAIN_ID=42

# RT Controller Settings (ESP32 Hardware)
RT_CONTROLLER_TYPE=FAKE
RT_CONTROLLER_IP=192.168.1.100
RT_CONTROLLER_PORT=5000
RT_CONTROLLER_PROTO=TCP

# User Controller (UI) Settings
ENABLE_UI=true
UI_TYPE=web
UI_PORT=8080

# Simulator Settings
GAZEBO_HEADLESS=false
GAZEBO_WORLD=default.world
SIM_REAL_TIME=true

# Analysis Settings
ENABLE_ANALYSIS=false
ENABLE_ACCURACY_ANALYSIS=false
LOG_LEVEL=INFO
LOG_TO_FILE=true

# Display (for GUI forwarding)
DISPLAY=host.docker.internal:0.0
```

---

## 4. Docker Build & Run

### 4.1 Build All Images

```powershell
cd D:\robot_system
docker compose build
```

This builds all images with cached layers. First build takes 10-20 minutes.

### 4.2 Start System

**SIM Mode:**
```powershell
docker compose -f docker-compose.yml -f docker-compose.sim.yml up -d
```

**REAL Mode:**
```powershell
docker compose -f docker-compose.yml -f docker-compose.real.yml up -d
```

### 4.3 Stop System

```powershell
docker compose down
```

---

## 5. Running SIM Mode

SIM mode uses the **Simulator (Gazebo)** as a physics engine. No real hardware or RT Controller is required.

### 5.1 Prerequisites

- Docker Desktop running
- X Server running (for Simulator GUI)
- `.env` configured with `ROBOT_MODE=SIM`

### 5.2 Start SIM Mode

```powershell
.\scripts\start_sim.ps1
```

### 5.3 Verify SIM Mode Running

```powershell
docker compose ps
# Expected: ros_core, moveit, gazebo, bridge containers running
```

### 5.4 Access Simulator GUI

Gazebo (Simulator) window should appear automatically. If not:
- Verify X Server is running
- Check DISPLAY environment variable
- Check logs: `docker compose logs gazebo`

### 5.5 Stop SIM Mode

```powershell
docker compose down
```

---

## 6. Running REAL Mode

REAL mode connects to the **RT Controller (ESP32)** which owns motor control. Ensure safety procedures are followed.

### 6.1 Prerequisites

- RT Controller (ESP32) powered and connected
- Network connectivity verified
- Emergency stop accessible
- Clear workspace around robot
- `.env` configured with `ROBOT_MODE=REAL`

### 6.2 Pre-Flight Checklist

| Check                               | Status |
|-------------------------------------|--------|
| RT Controller powered               | ☐      |
| E-STOP not engaged                  | ☐      |
| Network cable connected             | ☐      |
| RT Controller IP reachable          | ☐      |
| Workspace clear of obstructions     | ☐      |
| Personnel aware robot will move     | ☐      |

### 6.3 Verify Network Connection

```powershell
# Ping RT Controller
ping 192.168.1.100
```

### 6.4 Start REAL Mode

```powershell
.\scripts\start_real.ps1
```

### 6.5 Verify Connection to RT Controller

```powershell
docker compose logs -f robot_bridge
# Look for: "Connected to RT Controller at 192.168.1.100:5000"
```

### 6.6 Emergency Stop Procedure

**At any time:**
1. Press hardware E-STOP button
2. Robot will halt immediately (RT Controller cuts power)
3. To resume: Clear fault, release E-STOP, restart system

### 6.7 Stop REAL Mode

```powershell
docker compose down
```

---

## 7. Using Simulator as Development Platform

The **Simulator (Gazebo)** can act as a development platform without any UI. The simulation physics replaces the RT Controller for testing purposes.

> **IMPORTANT**: In REAL mode, the Simulator is VISUALIZATION ONLY and NEVER authoritative. This section applies to SIM mode only.

### 7.1 Configuration

Edit `.env`:
```bash
ROBOT_MODE=SIM
RT_CONTROLLER_TYPE=FAKE
ENABLE_UI=false
GAZEBO_HEADLESS=false
```

### 7.2 Start Simulator-Only

```powershell
docker compose -f docker-compose.yml -f docker-compose.sim.yml up -d ros_core moveit gazebo bridge
```

### 7.3 Interact via ROS CLI

```powershell
docker compose exec ros_core bash
ros2 topic list
ros2 service list
```

### 7.4 Headless Simulator (No GUI)

For CI/CD or remote servers:
```bash
GAZEBO_HEADLESS=true
```

---

## 8. Optional UI Usage (User Controller)

The **User Controller (UI)** is entirely optional. System functions without it.

### 8.1 Enable UI

Edit `.env`:
```bash
ENABLE_UI=true
UI_TYPE=web
UI_PORT=8080
```

### 8.2 Access Web UI

Open browser: `http://localhost:8080`

### 8.3 UI Features

| Feature               | Description                              |
|-----------------------|------------------------------------------|
| Goal Selection        | Click to set target pose                 |
| Plan Visualization    | View planned trajectory before execution |
| Execute Button        | Trigger trajectory execution             |
| Joint State Display   | Real-time joint positions                |
| Mode Indicator        | Shows SIM/REAL mode                      |
| Fault Display         | Shows any active faults                  |

### 8.4 UI Limitations (By Design)

The User Controller (UI) cannot:
- Directly command motors (only RT Controller can)
- Bypass safety checks
- Execute without MoveIt validation
- Override RT Controller decisions

---

## 9. Enabling Analysis Modules

Analysis modules provide debugging and performance insights.

### 9.1 Enable Analysis

Edit `.env`:
```bash
ENABLE_ANALYSIS=true
ENABLE_ACCURACY_ANALYSIS=true
LOG_LEVEL=DEBUG
```

### 9.2 Accuracy Comparison

When enabled, compares planned (MoveIt) vs actual (encoder) trajectories:

```powershell
docker compose exec ros_core bash
ros2 topic echo /analysis/accuracy_metrics
# Output includes: position_error_rms, max_deviation
```

---

## 10. Logs & Debugging

### 10.1 Log Locations

| Container    | Log Path                          | Contents                    |
|--------------|-----------------------------------|-----------------------------|
| ros_core     | `logs/ros/ros_core.log`           | ROS 2 node logs             |
| moveit       | `logs/ros/moveit.log`             | Planning logs (IK/FK)       |
| gazebo       | `logs/gazebo/gazebo.log`          | Simulator logs              |
| bridge       | `logs/ros/bridge.log`             | Communication logs          |
| analysis     | `logs/analysis/`                  | Analysis data               |

### 10.2 View Live Logs

```powershell
docker compose logs -f
docker compose logs -f ros_core
```

### 10.3 Debugging Common Issues

**Issue: Simulator window does not appear**
```powershell
# Verify X Server running, check DISPLAY variable
docker compose logs gazebo
```

**Issue: No joint states published**
```powershell
# Check bridge connection to RT Controller
docker compose logs bridge
docker compose exec bridge ping $RT_CONTROLLER_IP
```

**Issue: MoveIt planning fails**
```powershell
# Check collision geometry and URDF
docker compose logs moveit
ros2 param get /robot_state_publisher robot_description
```

---

## 11. Deployment & Updates

### 11.1 Update Code

```powershell
cd D:\robot_system
docker compose down
git pull origin main
docker compose build
docker compose up -d
```

### 11.2 Update RT Controller Firmware

1. Stop robot system: `docker compose down`
2. Follow ESP32 firmware update procedure (see `docs/ESP32_PCA9685_WIRING_AND_SETUP.md`)
3. Verify RT Controller version matches expectations
4. Restart: `docker compose up -d`

---

## 12. Quick Reference Commands

### 12.1 Startup Commands

| Action                    | Command                                                      |
|---------------------------|--------------------------------------------------------------|
| Start SIM mode            | `docker compose -f docker-compose.yml -f docker-compose.sim.yml up -d` |
| Start REAL mode           | `docker compose -f docker-compose.yml -f docker-compose.real.yml up -d` |
| Stop all                  | `docker compose down`                                        |
| Restart all               | `docker compose restart`                                     |

### 12.2 Monitoring Commands

| Action                    | Command                                                      |
|---------------------------|--------------------------------------------------------------|
| View all logs             | `docker compose logs -f`                                     |
| View specific log         | `docker compose logs -f ros_core`                            |
| List containers           | `docker compose ps`                                          |
| Enter container           | `docker compose exec ros_core bash`                          |

### 12.3 Troubleshooting

| Issue                     | Command                                                      |
|---------------------------|--------------------------------------------------------------|
| Check ROS topics          | `docker compose exec ros_core ros2 topic list`               |
| Check RT Controller ping  | `docker compose exec ros_core ping $RT_CONTROLLER_IP`        |
| Force rebuild             | `docker compose build --no-cache`                            |
