# USER OPERATION GUIDE — Industrial Robot Platform

## Table of Contents

1. [One-Time Setup](#1-one-time-setup)
2. [Folder Structure](#2-folder-structure)
3. [Environment Variables](#3-environment-variables)
4. [Docker Build & Run](#4-docker-build--run)
5. [Running SIM Mode](#5-running-sim-mode)
6. [Running REAL Mode](#6-running-real-mode)
7. [Using Gazebo as Controller](#7-using-gazebo-as-controller)
8. [Optional UI Usage](#8-optional-ui-usage)
9. [Enabling Analysis Modules](#9-enabling-analysis-modules)
10. [Logs & Debugging](#10-logs--debugging)
11. [Deployment & Updates](#11-deployment--updates)
12. [Quick Reference Commands](#12-quick-reference-commands)

---

## 1. One-Time Setup

### 1.1 Prerequisites

| Requirement          | Version / Spec                          | Purpose                    |
|----------------------|-----------------------------------------|----------------------------|
| Windows 10/11        | 64-bit, Build 19041+                    | Host OS                    |
| Docker Desktop       | 4.0+ with WSL2 backend                  | Container runtime          |
| VS Code              | Latest                                  | Development IDE            |
| Git                  | 2.30+                                   | Version control            |
| X Server (optional)  | VcXsrv or similar                       | GUI forwarding (Gazebo)    |

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
│   ├── Dockerfile.gazebo         # Gazebo simulation
│   └── Dockerfile.ui             # Optional UI image
│
├── src/                          # ROS 2 workspace source
│   ├── robot_description/        # URDF, meshes, configs
│   │   ├── urdf/
│   │   │   └── robot.urdf.xacro
│   │   ├── meshes/
│   │   └── config/
│   │       └── joint_limits.yaml
│   │
│   ├── robot_moveit_config/      # MoveIt configuration
│   │   ├── config/
│   │   │   ├── kinematics.yaml
│   │   │   ├── joint_limits.yaml
│   │   │   └── controllers.yaml
│   │   └── launch/
│   │       └── move_group.launch.py
│   │
│   ├── robot_hardware_bridge/    # Hardware interface node
│   │   ├── robot_hardware_bridge/
│   │   │   ├── __init__.py
│   │   │   ├── bridge_node.py
│   │   │   ├── real_backend.py
│   │   │   └── sim_backend.py
│   │   └── launch/
│   │       └── bridge.launch.py
│   │
│   ├── robot_gazebo/             # Gazebo simulation
│   │   ├── worlds/
│   │   ├── launch/
│   │   └── config/
│   │
│   └── robot_analysis/           # Optional analysis tools
│       ├── robot_analysis/
│       │   ├── accuracy_node.py
│       │   └── logger_node.py
│       └── launch/
│
├── controller/                   # Real-time controller code
│   ├── firmware/                 # MCU firmware (if applicable)
│   └── rt_linux/                 # RT Linux code (if applicable)
│
├── config/                       # Shared configuration
│   ├── robot_params.yaml         # Robot-specific parameters
│   └── network.yaml              # Network configuration
│
├── logs/                         # Runtime logs (auto-created)
│   ├── ros/
│   ├── gazebo/
│   └── analysis/
│
├── scripts/                      # Helper scripts
│   ├── start_sim.ps1
│   ├── start_real.ps1
│   ├── stop_all.ps1
│   └── view_logs.ps1
│
└── docs/                         # Documentation
    ├── SYSTEM_LOGIC_AND_FLOW.md
    └── USER_OPERATION_GUIDE.md
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

### 3.2 Controller Variables

| Variable            | Values              | Description                              |
|---------------------|---------------------|------------------------------------------|
| `CONTROLLER_TYPE`   | `FAKE` / `REAL`     | Controller backend selection             |
| `CONTROLLER_IP`     | IP address          | Real controller network address          |
| `CONTROLLER_PORT`   | Port number         | Controller communication port            |
| `CONTROLLER_PROTO`  | `SERIAL`/`TCP`/`UDP`| Communication protocol                   |

### 3.3 UI Variables

| Variable            | Values              | Description                              |
|---------------------|---------------------|------------------------------------------|
| `ENABLE_UI`         | `true` / `false`    | Enable/disable UI container              |
| `UI_TYPE`           | `web` / `desktop`   | Type of UI to launch                     |
| `UI_PORT`           | Port number         | Web UI port (if web type)                |

### 3.4 Simulation Variables

| Variable            | Values              | Description                              |
|---------------------|---------------------|------------------------------------------|
| `GAZEBO_HEADLESS`   | `true` / `false`    | Run Gazebo without GUI                   |
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

# Controller Settings
CONTROLLER_TYPE=FAKE
CONTROLLER_IP=192.168.1.100
CONTROLLER_PORT=5000
CONTROLLER_PROTO=TCP

# UI Settings
ENABLE_UI=true
UI_TYPE=web
UI_PORT=8080

# Simulation Settings
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

### 4.2 Build Specific Image

```powershell
docker compose build ros_core
docker compose build gazebo
docker compose build ui
```

### 4.3 Force Rebuild (No Cache)

```powershell
docker compose build --no-cache
```

### 4.4 Start System

**SIM Mode:**
```powershell
docker compose -f docker-compose.yml -f docker-compose.sim.yml up -d
```

**REAL Mode:**
```powershell
docker compose -f docker-compose.yml -f docker-compose.real.yml up -d
```

### 4.5 Stop System

```powershell
docker compose down
```

### 4.6 View Running Containers

```powershell
docker compose ps
```

### 4.7 View Container Logs

```powershell
# All containers
docker compose logs -f

# Specific container
docker compose logs -f ros_core
docker compose logs -f gazebo
```

---

## 5. Running SIM Mode

SIM mode uses Gazebo as a physics simulator. No real hardware is required.

### 5.1 Prerequisites

- Docker Desktop running
- X Server running (for Gazebo GUI)
- `.env` configured with `ROBOT_MODE=SIM`

### 5.2 Start X Server (Windows)

1. Launch VcXsrv
2. Select "Multiple windows"
3. Set display number to 0
4. Check "Disable access control"
5. Finish

### 5.3 Start SIM Mode

**Option A: Using Script**
```powershell
.\scripts\start_sim.ps1
```

**Option B: Manual**
```powershell
# Ensure .env has ROBOT_MODE=SIM
docker compose -f docker-compose.yml -f docker-compose.sim.yml up -d
```

### 5.4 Verify SIM Mode Running

```powershell
# Check all containers are running
docker compose ps

# Expected output:
# NAME                STATUS
# robot_ros_core      running
# robot_moveit        running
# robot_gazebo        running
# robot_bridge        running
```

### 5.5 Access Gazebo GUI

Gazebo window should appear automatically. If not:
- Verify X Server is running
- Check DISPLAY environment variable
- Check container logs: `docker compose logs gazebo`

### 5.6 Send Test Motion (SIM)

```powershell
# Enter ROS container
docker compose exec ros_core bash

# Inside container, publish test goal
ros2 topic pub /goal_pose geometry_msgs/Pose "{position: {x: 0.5, y: 0.0, z: 0.5}}" --once

# Exit container
exit
```

### 5.7 Stop SIM Mode

```powershell
docker compose down
```

---

## 6. Running REAL Mode

REAL mode connects to actual hardware. Ensure safety procedures are followed.

### 6.1 Prerequisites

- Real-time controller powered and connected
- Network connectivity verified
- Emergency stop accessible
- Clear workspace around robot
- `.env` configured with `ROBOT_MODE=REAL`

### 6.2 Pre-Flight Checklist

| Check                               | Status |
|-------------------------------------|--------|
| Controller powered                 | ☐      |
| E-STOP not engaged                  | ☐      |
| Network cable connected             | ☐      |
| Controller IP reachable             | ☐      |
| Workspace clear of obstructions     | ☐      |
| Personnel aware robot will move     | ☐      |

### 6.3 Verify Network Connection

```powershell
# Ping controller
ping 192.168.1.100
```

### 6.4 Start REAL Mode

**Option A: Using Script**
```powershell
.\scripts\start_real.ps1
```

**Option B: Manual**
```powershell
# Ensure .env has ROBOT_MODE=REAL
docker compose -f docker-compose.yml -f docker-compose.real.yml up -d
```

### 6.5 Verify Connection to Controller

```powershell
# Check bridge container logs
docker compose logs -f robot_bridge

# Look for:
# "Connected to controller at 192.168.1.100:5000"
# "Receiving joint states"
```

### 6.6 Verify Joint States

```powershell
docker compose exec ros_core bash

# Inside container
ros2 topic echo /joint_states

# Should see live encoder data
exit
```

### 6.7 Emergency Stop Procedure

**At any time:**
1. Press hardware E-STOP button
2. Robot will halt immediately
3. To resume:
   - Clear fault conditions
   - Release E-STOP
   - Restart system if necessary

### 6.8 Stop REAL Mode

```powershell
docker compose down
```

---

## 7. Using Gazebo as Controller

Gazebo can act as a development platform without any UI. The simulation physics replaces the real controller.

### 7.1 Configuration

Edit `.env`:
```bash
ROBOT_MODE=SIM
CONTROLLER_TYPE=FAKE
ENABLE_UI=false
GAZEBO_HEADLESS=false
```

### 7.2 Start Gazebo-Only

```powershell
docker compose -f docker-compose.yml -f docker-compose.sim.yml up -d ros_core moveit gazebo bridge
```

### 7.3 Interact via ROS CLI

```powershell
docker compose exec ros_core bash

# List available topics
ros2 topic list

# List available services
ros2 service list

# Send motion command
ros2 action send_goal /move_group/move moveit_msgs/action/MoveGroup "{...}"
```

### 7.4 Headless Gazebo (No GUI)

For CI/CD or remote servers:
```bash
GAZEBO_HEADLESS=true
```

---

## 8. Optional UI Usage

The UI is entirely optional. System functions without it.

### 8.1 Enable UI

Edit `.env`:
```bash
ENABLE_UI=true
UI_TYPE=web
UI_PORT=8080
```

### 8.2 Start with UI

```powershell
docker compose -f docker-compose.yml -f docker-compose.sim.yml up -d
```

### 8.3 Access Web UI

Open browser: `http://localhost:8080`

### 8.4 UI Features

| Feature               | Description                              |
|-----------------------|------------------------------------------|
| Goal Selection        | Click to set target pose                 |
| Plan Visualization    | View planned trajectory before execution |
| Execute Button        | Trigger trajectory execution             |
| Joint State Display   | Real-time joint positions                |
| Mode Indicator        | Shows SIM/REAL mode                      |
| Fault Display         | Shows any active faults                  |

### 8.5 Disable UI

Edit `.env`:
```bash
ENABLE_UI=false
```

Or simply do not start the UI container:
```powershell
docker compose up -d ros_core moveit gazebo bridge
```

### 8.6 UI Limitations (By Design)

The UI cannot:
- Directly control motors
- Bypass safety checks
- Execute without planner validation
- Override controller decisions

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

### 9.2 Start with Analysis

```powershell
docker compose -f docker-compose.yml -f docker-compose.sim.yml up -d
```

Analysis container will start automatically if `ENABLE_ANALYSIS=true`.

### 9.3 Accuracy Comparison

When enabled, compares planned vs actual trajectories:

```powershell
docker compose exec ros_core bash

# View accuracy metrics
ros2 topic echo /analysis/accuracy_metrics

# Output includes:
# - position_error_rms
# - velocity_error_rms
# - max_deviation
# - timing_error
```

### 9.4 View Analysis Logs

```powershell
# Real-time
docker compose logs -f analysis

# Or from file
Get-Content -Wait D:\robot_system\logs\analysis\accuracy.log
```

### 9.5 Export Analysis Data

```powershell
docker compose exec analysis bash

# Inside container
ros2 run robot_analysis export_data --output /logs/analysis/export.csv

exit

# File available at D:\robot_system\logs\analysis\export.csv
```

### 9.6 Disable Analysis

Edit `.env`:
```bash
ENABLE_ANALYSIS=false
```

System runs faster without analysis overhead.

---

## 10. Logs & Debugging

### 10.1 Log Locations

| Container    | Log Path                          | Contents                    |
|--------------|-----------------------------------|-----------------------------|
| ros_core     | `logs/ros/ros_core.log`           | ROS 2 node logs             |
| moveit       | `logs/ros/moveit.log`             | Planning logs               |
| gazebo       | `logs/gazebo/gazebo.log`          | Simulation logs             |
| bridge       | `logs/ros/bridge.log`             | Communication logs          |
| analysis     | `logs/analysis/`                  | Analysis data               |

### 10.2 View Live Logs

```powershell
# All containers
docker compose logs -f

# Single container
docker compose logs -f ros_core

# With timestamps
docker compose logs -f -t ros_core
```

### 10.3 Log Levels

Set in `.env`:
```bash
LOG_LEVEL=DEBUG   # Maximum verbosity
LOG_LEVEL=INFO    # Normal operation
LOG_LEVEL=WARN    # Warnings and errors only
LOG_LEVEL=ERROR   # Errors only
```

### 10.4 Debugging Connectivity

```powershell
docker compose exec ros_core bash

# Check ROS 2 communication
ros2 node list
ros2 topic list
ros2 topic info /joint_states

# Check network
ping controller_ip

exit
```

### 10.5 Debugging Common Issues

**Issue: Gazebo window does not appear**
```powershell
# Verify X Server running
# Verify DISPLAY variable in .env
# Check: docker compose logs gazebo
```

**Issue: No joint states published**
```powershell
# Check bridge connection
docker compose logs bridge

# Verify controller reachable
docker compose exec bridge ping $CONTROLLER_IP
```

**Issue: MoveIt planning fails**
```powershell
# Check collision geometry
docker compose logs moveit

# Verify URDF loaded
ros2 param get /robot_state_publisher robot_description
```

### 10.6 Reset Logs

```powershell
# Stop system
docker compose down

# Clear logs
Remove-Item -Recurse -Force D:\robot_system\logs\*

# Restart
docker compose up -d
```

---

## 11. Deployment & Updates

### 11.1 Update Code

```powershell
cd D:\robot_system

# Stop system
docker compose down

# Pull latest
git pull origin main

# Rebuild if Dockerfiles changed
docker compose build

# Restart
docker compose up -d
```

### 11.2 Update Controller Firmware

1. Stop robot system: `docker compose down`
2. Follow controller-specific firmware update procedure
3. Verify controller version matches expectations
4. Restart: `docker compose up -d`

### 11.3 Backup Configuration

```powershell
# Backup .env and configs
Copy-Item .env .env.backup
Copy-Item -Recurse config\ config_backup\
```

### 11.4 Deploy to New Machine

1. Install prerequisites (Section 1)
2. Clone repository
3. Copy `.env` from backup or configure new
4. Build images: `docker compose build`
5. Start: `docker compose up -d`

### 11.5 Version Management

```powershell
# Check current version
git describe --tags

# Switch to specific version
git checkout v1.2.0
docker compose build
docker compose up -d
```

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
| Container resources       | `docker stats`                                               |

### 12.3 ROS Commands (Inside Container)

```powershell
docker compose exec ros_core bash
```

| Action                    | Command                                                      |
|---------------------------|--------------------------------------------------------------|
| List nodes                | `ros2 node list`                                             |
| List topics               | `ros2 topic list`                                            |
| Echo topic                | `ros2 topic echo /joint_states`                              |
| List services             | `ros2 service list`                                          |
| Call service              | `ros2 service call /service_name std_srvs/srv/Trigger`       |
| Check TF tree             | `ros2 run tf2_tools view_frames`                             |

### 12.4 Troubleshooting Commands

| Issue                     | Command                                                      |
|---------------------------|--------------------------------------------------------------|
| Container not starting    | `docker compose logs container_name`                         |
| Network issues            | `docker compose exec ros_core ping controller_ip`            |
| Rebuild single container  | `docker compose build container_name`                        |
| Full reset                | `docker compose down -v && docker compose up -d`             |

---

## Support Contacts

For issues not covered in this guide:

- Check `logs/` directory for error details
- Review `SYSTEM_LOGIC_AND_FLOW.md` for architecture questions
- Consult controller-specific documentation for hardware issues

---

*Document Version: 1.0*
*Generated for: Plug-and-Play Industrial Robot Platform*
*Root Directory: D:\robot_system*
