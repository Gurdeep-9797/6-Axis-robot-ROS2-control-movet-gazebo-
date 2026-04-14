# RoboForge v8.2 — Setup & Usage Guide

---

## 1. Prerequisites

| Component | Purpose | Version |
|-----------|---------|---------|
| Docker Desktop | Container runtime (ROS2, MoveIt, Gazebo) | 24+ |
| .NET 8 SDK | WPF offline client build | 8.0+ |
| Node.js 20+ | React online IDE (if running outside Docker) | 20+ |
| PlatformIO | ESP32 firmware flashing | Latest |
| Visual Studio 2022 | WPF development (optional) | 2022+ |

**Auto-install VS2022 + .NET 8 + CMake:**
```powershell
.\tools\install_vs_tools.ps1
```

---

## 2. Starting the Full Stack

### Option A: Docker (Recommended)
```powershell
# Start all 5 containers
docker compose up -d

# Wait ~30 seconds for services to initialize
# Then open the online IDE:
start http://localhost:3000
```

**Containers launched:**
| Container | Port | Purpose |
|-----------|------|---------|
| `roboforge_core` | — | robot_state_publisher, TF tree |
| `roboforge_moveit` | — | MoveGroup, IK/FK services |
| `roboforge_pseudo_hw` | — | 250Hz simulated joint states |
| `roboforge_bridge` | 9090 (WS), 8765 (REST) | WebSocket + REST bridge |
| `roboforge_frontend` | 3000 | React Online IDE |

### Option B: Build WPF Offline Client
```powershell
# Build
dotnet build src/RoboForge.Wpf/RoboForge.Wpf.csproj --configuration Debug

# Run
start src/RoboForge.Wpf\bin\x64\Debug\net8.0-windows\RoboForge.Wpf.exe
```

### Option C: Simulation + Gazebo Physics
```powershell
docker compose --profile sim up -d
```

---

## 3. Using the Online IDE

### 3.1 Creating a Program
1. **Program Tree** (left panel) shows default pick-place routine
2. **Click blocks** to select, **right-click** for context menu
3. **Add blocks** from the Block Library panel

### 3.2 Placing Waypoints
1. Click **"Add Waypoint"** in the 3D viewport toolbar
2. **Click anywhere** in the 3D scene to place a waypoint
3. **Drag waypoints** to reposition, or edit coordinates in properties

### 3.3 Running a Program
1. Click **"▶ Run"** in the ribbon toolbar
2. Watch execution in the **Console** (bottom panel)
3. **Pause/Stop** at any time

### 3.4 IK Mode
- **MoveIt 2** (default, green indicator) — Uses ROS2 IK service with collision checking
- **Offline Analytical** (red indicator) — Local JS solver, no collision checking
- **Offline Numerical DLS** (red indicator) — Damped Least Squares, no collision checking

> ⚠️ Switching to offline mode shows a **red warning modal** requiring explicit confirmation.

### 3.5 Motor Speed Control
- Use the **Motor PWM slider** (5-100%) in the ribbon
- Adjusts PID gains and max current for all 6 joints
- Publishes configuration to ROS2 via `/roboforge/motor_config`

---

## 4. Using the Offline WPF Client

### 4.1 Layout
- **Left Panel**: Program Tree with execution controls
- **Center**: 3D Viewport (HelixToolkit)
- **Right Panel**: Scene Outliner + Joint Angle sliders (J1-J6)
- **Bottom**: Console

### 4.2 Running a Program
1. Click **"▶ Run"** — executes blocks sequentially
2. Watch joint angles update in real-time
3. **Pause/Stop** controls available

### 4.3 Jogging Joints
- Use **J1-J6 sliders** in the right panel
- Values in degrees, range ±180° (J6: ±360°)

---

## 5. Physical Hardware Integration

### 5.1 Wiring Diagram

| Component | Signal | ESP32 Pin | PCA9685 Ch |
|-----------|--------|-----------|------------|
| **PCA9685** | SDA | GPIO 21 | — |
| | SCL | GPIO 22 | — |
| | V+ | **External 5V PSU** | — |
| **Joint 1** | Servo | — | Ch 0 |
| | Enc A/B | GPIO 34/35 | — |
| **Joint 2** | Servo | — | Ch 1 |
| | Enc A/B | GPIO 32/33 | — |
| **Joint 3** | Servo | — | Ch 2 |
| | Enc A/B | GPIO 25/26 | — |
| **Joint 4** | Servo | — | Ch 3 |
| | Enc A/B | GPIO 27/14 | — |
| **Joint 5** | Servo | — | Ch 4 |
| | Enc A/B | GPIO 12/13 | — |
| **Joint 6** | Servo | — | Ch 5 |
| | Enc A/B | GPIO 4/16 | — |

> ⚠️ **NEVER** power servos from the ESP32. Use an external 5V 6A PSU on PCA9685 V+.

### 5.2 Flashing Firmware

**WiFi Mode (TCP binary protocol):**
```bash
pio run -e wifi_controller -t upload
```
- Edit `firmware/wifi_controller/config.h` for WiFi SSID/password
- Connects to ROS2 bridge at `CONTROLLER_IP:5000`

**Serial Mode (USB text protocol):**
```bash
pio run -e serial_control -t upload
```
- Protocol: `<J0:90.50,J1:45.00,...,J5:0.00>` at 115200 baud

### 5.3 Real Hardware Mode
```powershell
# Set ESP32 IP address
$env:CONTROLLER_IP = "192.168.1.100"

# Start Docker stack
docker compose up -d
```

---

## 6. Troubleshooting

| Problem | Cause | Fix |
|---------|-------|-----|
| React UI shows white blank area | TopRibbon.tsx layout overflow | Check browser console for errors |
| WPF shows garbled text (Ã¶Ã") | UTF-8 encoding mismatch | Save XAML as UTF-8 with BOM |
| MoveIt IK returns -31 | Pose unreachable for robot model | Try reachable poses: `[0.5, 0, 0.8]` |
| WebSocket connection fails | Bridge not running | `docker ps` — verify `roboforge_bridge` is Up |
| Joint states not updating | pseudo_hardware crashed | `docker logs roboforge_pseudo_hw` |
| Servos jittery | Insufficient power | Use 5V 6A PSU, shared ground |
| COM port unavailable | Port in use by another app | Close serial monitors |

---

## 7. Pipeline Verification

Run the end-to-end test:
```powershell
python tools/test_pipeline.py
```

Expected output:
```
[1] REST API:       ✅ PASS  ({"moveit_ready":true})
[2] IK via MoveIt:  ⚠️ TIMEOUT (pipeline working, pose unreachable)
[3] Joint States:   ✅ PASS  (250Hz from pseudo_hardware_node)
```

---

*Verified against RoboForge v8.2 — 2026-04-09*
