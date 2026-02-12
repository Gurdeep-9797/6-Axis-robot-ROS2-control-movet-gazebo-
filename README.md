# 6-Axis Industrial Robot Platform

A full-stack robot control system: **C# WPF Simulator** → **ROS 2 / MoveIt Planning** → **ESP32 Hardware**.

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│  Simulator (C# WPF)     ──── rosbridge WebSocket ────┐  │
│  • 3D Visualization                                   │  │
│  • Teach Pendant                                      ▼  │
│  • Serial USB Control    MoveIt 2 (Docker/Linux)         │
│                          • TRAC-IK Solver                │
│                          • OMPL Path Planning            │
│                          • Collision Avoidance            │
│                                   │                      │
│                          Hardware Bridge (ROS Node)       │
│                          • Schema Validation              │
│                          • Protocol Gateway               │
│                                   │                      │
│                          ┌────────┴────────┐             │
│                          │                 │             │
│                     WiFi (TCP)        Gazebo Sim         │
│                          │                               │
│                    ESP32 Controller                       │
│                    • PCA9685 PWM                          │
│                    • Encoder Feedback                     │
│                    • Safety Watchdog                      │
└─────────────────────────────────────────────────────────┘
```

## Directory Structure

| Folder | Purpose |
|--------|---------|
| `RobotSimulator/` | C# WPF desktop app (3D visualization, teach pendant, serial control) |
| `src/` | ROS 2 packages (MoveIt config, hardware bridge, description, Gazebo) |
| `firmware/serial_control/` | ESP32 firmware — USB serial mode (standalone simulator) |
| `firmware/wifi_controller/` | ESP32 firmware — WiFi TCP mode (ROS hardware bridge) |
| `docker/` | Docker Compose configs for SIM and REAL modes |
| `controller/` | `hardware_map.yaml` — joint/pin/encoder mapping |
| `scripts/` | Launch, test, and CI scripts |
| `tools/` | URDF pipeline and SolidWorks export utilities |
| `logic_simulator/` | Python prototype (reference only) |
| `release/` | Prebuilt simulator binaries |

## Quick Start

### Option A: Simulator Only (Windows, no Docker)
```
1. Go to release/ → Run RobotSimulator.exe
2. Use sliders to jog, "Teach" to record points
3. Connect ESP32 via USB → Select COM port → Click "Serial"
```

### Option B: Full ROS 2 Stack (Docker required)
```bash
# Simulation mode (no hardware)
docker compose -f docker/docker-compose.sim.yml up

# Real robot mode (ESP32 on WiFi)
docker compose -f docker/docker-compose.real.yml up
```

### Option C: Simulator + MoveIt (best of both)
```
1. Start Docker stack (Option B)
2. Run RobotSimulator.exe
3. Enter rosbridge URI → Click "ROS"
4. "Go To Point" now plans via MoveIt with collision avoidance
```

## Two Firmware Modes

| Mode | Folder | Communication | Use Case |
|------|--------|---------------|----------|
| **Serial** | `firmware/serial_control/` | USB `<J0:val,...>` | Direct simulator control |
| **WiFi** | `firmware/wifi_controller/` | TCP binary on port 5000 | ROS hardware bridge |

See **[USAGE.md](USAGE.md)** for wiring, firmware flashing, and detailed setup.

---
*Built with ROS 2 Humble, MoveIt 2, Gazebo, .NET 8, PlatformIO*
