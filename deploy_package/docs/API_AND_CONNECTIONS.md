# RoboForge v8.2 — Complete API & System Connections Reference

> **Authoritative technical reference for all data flows, protocols, and APIs.**
> Verified against live system: 2026-04-08 18:30 IST

---

## 1. System Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                      FRONTEND LAYER                          │
│                                                              │
│  React Online IDE (Vite + Three.js + roslib)                │
│    → ws://localhost:9090 (BackendConnector.ts)              │
│    → http://localhost:8765 (REST health checks)             │
│                                                              │
│  WPF Offline Client (.NET 8 WPF + ClientWebSocket)          │
│    → ws://localhost:9090 (MainViewModel.AutoConnectAsync)   │
│    → /joint_states parsed → J1-J6 properties                │
└────────────────────────┬─────────────────────────────────────┘
                         │
                         ▼
┌──────────────────────────────────────────────────────────────┐
│                   DOCKER (WSL 2)                             │
│  ROS_DOMAIN_ID=42 │ Network: robot_net                       │
│                                                            │
│  ┌─────────────────────┐    ┌────────────────────────────┐  │
│  │ roboforge_bridge    │◄──►│ roboforge_moveit           │  │
│  │  WS 0.0.0.0:9090    │    │  MoveGroup: initialized     │  │
│  │  REST 0.0.0.0:8765  │    │  IK: /compute_ik (responding)│  │
│  │  URDF FK loaded     │    │  KDL kinematics plugin      │  │
│  │  Kinematics: ✅      │    │  SRDF: industrial_6dof      │  │
│  └──────────┬──────────┘    └──────────────┬─────────────┘  │
│             │                               │                 │
│             │ /joint_states (250Hz)         │ /compute_ik     │
│             │ /joint_trajectory_cmd         │ /compute_fk     │
│             ▼                               ▼                 │
│  ┌─────────────────────┐    ┌────────────────────────────┐  │
│  │ pseudo_hardware     │    │ roboforge_core             │  │
│  │  250Hz sim loop     │◄──►│ robot_state_publisher      │  │
│  │  Cubic spline       │    │ /robot_description         │  │
│  │  Home: [0,-0.3,0.2, │    │ /tf, /tf_static            │  │
│  │        0,-0.5,0]    │    │                            │  │
│  └─────────────────────┘    └────────────────────────────┘  │
│                                                              │
│  gazebo (profile=sim): Gazebo Harmonic + gz_ros2_control    │
└────────────────────────┬─────────────────────────────────────┘
                         │
              (Live Mode)│ TCP 192.168.1.100:5000
                         ▼
┌──────────────────────────────────────────────────────────────┐
│                    PHYSICAL HARDWARE                          │
│  ESP32 wifi_controller → PCA9685 PWM → 6 Servos              │
│  ESP32 serial_control → PID DC motors + encoders             │
└──────────────────────────────────────────────────────────────┘
```

---

## 2. WebSocket Protocol (Port 9090)

**Implementation**: `src/roboforge_bridge/roboforge_bridge/bridge_node.py`
**Library**: `websockets` 9.1 (Python asyncio)

### 2.1 Joint States (Bridge → UI)

Pushed at 250Hz from `pseudo_hardware_node`, enriched by bridge with FK + manipulability:

```json
{
  "type": "robot_state",
  "timestamp_ns": 1775652567706190653,
  "joint_pos_rad": [0.0, -0.3, 0.2, 0.0, -0.5, 0.0],
  "manipulability": 0.342,
  "in_singularity": false,
  "source": "gazebo",
  "execution_state": "idle",
  "pc": 0
}
```

### 2.2 Compute IK (UI → Bridge → MoveIt)

**Request**:
```json
{
  "op": "call_service",
  "service": "/compute_ik",
  "id": "ik-001",
  "args": {
    "ik_request": {
      "group_name": "robot_arm",
      "avoid_collisions": true,
      "pose_stamped": {
        "header": { "frame_id": "base_link" },
        "pose": {
          "position": { "x": 0.6, "y": 0.0, "z": 0.8 },
          "orientation": { "x": 0, "y": 0, "z": 0, "w": 1 }
        }
      },
      "timeout": { "sec": 5, "nanosec": 0 }
    }
  }
}
```

**Success**: `error_code.val = 1` with joint positions.
**Error codes**: `1`=SUCCESS, `-1`=PLANNING_FAILED, `-15`=NO_IK_SOLUTION, `-31`=TIMED_OUT

### 2.3 Execute Program (UI → Bridge)

```json
{
  "op": "roboforge/execute_program",
  "program": [{
    "duration_s": 2.0,
    "points": [{
      "positions": [0.0, -0.523, 1.047, 0.0, -0.785, 0.0],
      "velocities": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
      "time_from_start": 2.0
    }]
  }]
}
```

Progress callback: `{"op":"roboforge/execution_progress","pc":3,"total":12,"state":"running"}`

### 2.4 Health Query

**Request**: `{ "op": "roboforge/health" }`

**Response** (verified live):
```json
{
  "op": "roboforge/health_response",
  "status": "ok",
  "mode": "simulate",
  "connected_clients": 0,
  "execution_state": "idle",
  "moveit_ready": true,
  "kinematics_loaded": true
}
```

---

## 3. REST API (Port 8765)

### GET `/health` (verified live)

```json
{
  "status": "ok",
  "mode": "simulate",
  "backend": "ros2_humble",
  "connected_clients": 0,
  "execution_state": "idle",
  "moveit_ready": true,
  "kinematics_loaded": true,
  "robot_loaded": true
}
```

### GET `/api/hardware/ports`

Returns connected USB serial ports via `pyserial`. Returns `[]` in Docker (no USB passthrough).

### GET `/api/logs?n=200`

Returns last N entries from the DataLogger ring buffer (2000 max, JSONL on disk).

---

## 4. ROS 2 Topic & Service Map

### Topics (11 active, verified)

| Topic | Type | Hz | Publisher | Subscriber |
|-------|------|-----|-----------|------------|
| `/joint_states` | `sensor_msgs/JointState` | 250 | pseudo_hw | bridge, moveit |
| `/joint_trajectory_command` | `trajectory_msgs/JointTrajectory` | On-demand | bridge | pseudo_hw |
| `/planned_trajectory` | `trajectory_msgs/JointTrajectory` | On-demand | bridge | bridge |
| `/robot_description` | `std_msgs/String` | Latched | robot_state_pub | moveit |
| `/roboforge/hw_status` | `std_msgs/String` | 10 | pseudo_hw | — |
| `/roboforge/tracking_alert` | `std_msgs/String` | On-demand | gazebo_error_node | bridge |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 1 | bridge | — |
| `/tf` | `tf2_msgs/TFMessage` | Variable | robot_state_pub | all |
| `/tf_static` | `tf2_msgs/TFMessage` | Latched | robot_state_pub | all |

### Services (verified)

| Service | Type | Server | Client |
|---------|------|--------|--------|
| `/compute_ik` | `moveit_msgs/GetPositionIK` | MoveIt move_group | bridge (proxy), UI |
| `/compute_fk` | `moveit_msgs/GetPositionFK` | MoveIt move_group | bridge (proxy) |
| `/roboforge/health_check` | `std_srvs/Trigger` | bridge | UI |

---

## 5. Frontend API Reference

### React Online IDE (`NEW_UI/remix-of-roboflow-studio/`)

**Connection Manager**: `src/services/BackendConnector.ts`

| Method | Route | Description |
|--------|-------|-------------|
| `autoConnect()` | Internal | Priority: localhost:9090 → tunnel URL → offline |
| `computeIK(x, y, z, rx, ry, rz, rw, ikMode, joints)` | WS or local | Routes per ikMode (MoveIt or offline JS) |
| `computeFK(joints)` | WS | Calls bridge FK service |
| `publishTrajectory(joints, durationSec)` | WS | Publishes to `/joint_trajectory_command` |
| `executeProgram(program)` | WS | `roboforge/execute_program` custom op |
| `subscribeJointStates(cb)` | WS | Subscribes to `/joint_states` |
| `requestPOST()` | WS | Calls `/roboforge/health_check` service |
| `publishIO(signalName, value)` | WS | Publishes to `/io/<signal_name>` |

**State Management**: `src/store/AppState.tsx` — React Context (690 lines)
- 40+ state variables, 60+ actions
- Default program: Pick-Place cycle with MoveJ/MoveL/SetDO/Gripper/Wait/Increment
- `runProgram()` flattens tree, executes MoveJ/MoveL/Wait sequentially
- `executeBlock()` animates robot, publishes trajectory, updates timeline

**IK Engine**: `src/engine/IKSolver.ts`
- Analytical: Closed-form geometric (IRB 6700 DH params)
- Numerical: DLS with 8 random seeds, joint regularization
- FK: Standard DH convention, 4×4 homogeneous transforms

### WPF Offline Client (`src/RoboForge.Wpf/`)

**Entry**: `App.xaml.cs` → DI → `MainWindow(MainViewModel)`

| Component | Status |
|-----------|--------|
| `MainViewModel` | ✅ WebSocket connection, message parsing, health check, MoveIt IK, Run command |
| `HardwareConfigViewModel` | ✅ POST test, config save/load (not wired to DI) |
| 9 other ViewModels | ❌ Empty stubs |

**Build**: `dotnet build` → 0 errors, 9 warnings (nullable references, NuGet compatibility)

**Fixed this session**:
- Added `EnumToBoolConverter` for mode toggle radio buttons
- Fixed version string v7.0 → v8.2

---

## 6. Execution Pipeline

```
UI: Run Program
  → Flatten program tree to executable blocks (MoveJ/MoveL/Wait)
  → For each block:
      → computeIK(x, y, z) via BackendConnector
         ├─ MoveIt mode → WS call /compute_ik → joint angles
         └─ Offline mode → IKSolver.ts (analytical or numerical)
      → animateToTarget(joints, duration)
         ├─ Offline: requestAnimationFrame easing → update 3D
         └─ Online: WS publish /joint_trajectory_command → pseudo_hw
      → Wait blocks → setTimeout
  → Program complete → state: IDLE
```

### Trajectory Interpolation (pseudo_hardware_node.py)

```
Input:  JointTrajectory with N waypoints
Method: Cubic Hermite spline: s = elapsed/duration, h = 3s²-2s³
        q(t) = q_start + h * (q_end - q_start)
Output: /joint_states @ 250Hz with positions + velocities
Limits: Enforced per joint (position + velocity), violation → FAULT
```

### Joint Limits (pseudo_hardware_node)

| Joint | Min (°) | Max (°) | Max Vel (°/s) |
|-------|---------|---------|---------------|
| J1 | -170 | +170 | 172 |
| J2 | -65 | +85 | 143 |
| J3 | -180 | +70 | 172 |
| J4 | -300 | +300 | 344 |
| J5 | -130 | +130 | 344 |
| J6 | -360 | +360 | 401 |

---

## 7. Docker Services

| Service | Container | Image | Port | Health |
|---------|-----------|-------|------|--------|
| ros_core | roboforge_core | robotics_base:latest | — | ✅ Healthy |
| moveit | roboforge_moveit | robotics_base:latest | — | ✅ Running |
| pseudo_hardware | roboforge_pseudo_hw | robotics_base:latest | — | ✅ 250Hz |
| bridge | roboforge_bridge | robotics_base:latest | 9090, 8765 | ✅ Active |
| frontend | roboforge_frontend | 6-axis-robot...:latest | 3000 | ✅ Serving |

All containers: `ROS_DOMAIN_ID=42`, network: `robot_net`

---

## 8. Physical Hardware

### ESP32 WiFi Controller (`firmware/wifi_controller/`)

- **Connection**: TCP `192.168.1.100:5000`
- **Protocol**: Binary (MsgHeader type+length, payload)
- **Control loop**: 50Hz (20ms)
- **Hardware**: PCA9685 I2C 0x40, 50Hz PWM, 500-2500μs
- **Encoders**: 6 quadrature (J1-J3: 4096 PPR, J4-J6: 2048 PPR)
- **Message types**: TRAJECTORY(0x01), STOP(0x02), JOINT_STATE(0x10), ACK(0x11), EXEC_STATE(0x12)

### ESP32 Serial Controller (`firmware/serial_control/`)

- **Connection**: USB-Serial 115200 baud
- **Protocol**: Text `<J0:90.5,J1:45.0,...,J5:0.0>`
- **Modes**: SERVO (PWM direct) or DC_ENCODER (PID closed-loop)
- **Fixed this session**: Added missing `#ifdef MOTOR_TYPE_DC_ENCODER` preprocessor guard

---

## 9. File Inventory

### Active (KEEP)
| Path | Status | Notes |
|------|--------|-------|
| `src/roboforge_bridge/` | ✅ Working | WebSocket + REST bridge, kinematics, data logger |
| `src/robot_description/` | ✅ Working | URDF/xacro, joint limits, launch files |
| `src/robot_gazebo/` | ✅ Working | Gazebo Harmonic launch |
| `src/robot_moveit_config/` | ✅ Fixed | MoveIt 2 config (kinematics, OMPL, SRDF, controllers) |
| `src/robot_hardware_bridge/` | ⚠️ Partial | Python nodes working, safety_watchdog.py missing |
| `src/robot_msgs/` | ✅ Working | TrajectoryAck, ExecutionState messages |
| `src/robot_analysis/` | ✅ Working | Accuracy analysis node |
| `src/robot_tests/` | ✅ Working | Motion stack tests |
| `NEW_UI/remix-of-roboflow-studio/` | ✅ Working | React IDE, TypeScript 0 errors |
| `src/RoboForge.Wpf/` | ✅ Fixed | Builds 0 errors, EnumToBoolConverter added |
| `docker-compose.yml` | ✅ Working | 5 services orchestration |
| `firmware/` | ✅ Fixed | Both firmware variants compile |

### Archived
| Path | Notes |
|------|-------|
| `_archive/RoboForge_WPF/` | Legacy WPF with DSL compiler |
| `_archive/TeachPendant_WPF/` | Older teach pendant |

### Orphaned (Safe to Delete)
| Path | Notes |
|------|-------|
| `robot_description/` (root) | Duplicate of `src/robot_description/` |
| `robot_gazebo/` (root) | Duplicate of `src/robot_gazebo/` |
| `robot_moveit_config/` (root) | Duplicate of `src/robot_moveit_config/` |
