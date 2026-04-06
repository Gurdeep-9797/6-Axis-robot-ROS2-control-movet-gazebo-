# RoboForge v8.2 — Complete API & System Connections Reference

> This is the authoritative technical reference for all data flows, protocols, and APIs in the RoboForge system.

---

## 1. System Architecture Overview

```
┌─────────────────────────────┐
│   FRONTEND (Client-Side)    │
│                             │
│  React Online IDE (Vercel)  │──── ws://localhost:9090 ───┐
│  WPF Offline Client (.exe)  │──── ws://localhost:9090 ───┤
└─────────────────────────────┘                            │
                                                           ▼
┌──────────────────────────────────────────────────────────────────────┐
│                    DOCKER CONTAINERS (WSL 2)                        │
│                                                                      │
│  ┌──────────────────┐   ROS 2 Topics    ┌──────────────────────┐    │
│  │ roboforge_bridge  │◄────────────────►│  roboforge_moveit     │    │
│  │  Port 9090 (WS)   │   /compute_ik    │  MoveIt move_group    │    │
│  │  Port 8765 (REST)  │   /compute_fk    │  OMPL path planner   │    │
│  └────────┬─────────┘   /joint_states   └──────────┬───────────┘    │
│           │                                         │                │
│           │ /joint_trajectory_command                │                │
│           ▼                                         ▼                │
│  ┌──────────────────┐                    ┌──────────────────────┐    │
│  │ pseudo_hardware   │◄── /joint_states─►│  roboforge_core       │    │
│  │ 250Hz sim loop    │                    │  robot_state_publisher│    │
│  └──────────────────┘                    └──────────────────────┘    │
│                                                                      │
│  ┌──────────────────┐  (Optional, profile=sim)                      │
│  │ roboforge_gazebo  │  Gazebo Harmonic physics                     │
│  └──────────────────┘                                               │
└──────────────────────────────────────────────────────────────────────┘
                           │
              (Live Mode)  │ USB-Serial @ 1Mbaud
                           ▼
┌──────────────────────────────────────────────────────────────────────┐
│                    PHYSICAL HARDWARE                                 │
│                                                                      │
│  ┌─────────────────┐         ┌─────────────────┐                    │
│  │ encoder_interface │ ←─────│ STM32 / Teensy   │                    │
│  │ node (250Hz pub)  │ Serial│ Encoder board    │                    │
│  └─────────────────┘         └─────────────────┘                    │
│  ┌─────────────────┐         ┌─────────────────┐                    │
│  │ motor_controller  │ ──────│ H-Bridge / FOC   │                    │
│  │ node (250Hz ctrl) │ Serial│ Motor drivers    │                    │
│  └─────────────────┘         └─────────────────┘                    │
└──────────────────────────────────────────────────────────────────────┘
```

All Docker containers share `ROS_DOMAIN_ID=42` on network `robot_net`.

---

## 2. WebSocket Protocol (Port 9090)

The bridge uses a **rosbridge v2-compatible** JSON protocol over WebSocket.

### 2.1 Subscribe to Joint States (UI → Bridge)

```json
{
  "op": "subscribe",
  "topic": "/joint_states",
  "type": "sensor_msgs/JointState"
}
```

**Response** (pushed at ~50Hz, throttled from 250Hz source):
```json
{
  "type": "joint_states",
  "data": {
    "type": "robot_state",
    "timestamp_ns": 1775496144311436032,
    "joint_pos_rad": [0.0, -0.523, 1.047, 0.0, -0.785, 0.0],
    "manipulability": 0.342,
    "in_singularity": false,
    "source": "gazebo",
    "execution_state": "idle",
    "pc": 0
  }
}
```

| Field | Type | Description |
|-------|------|-------------|
| `joint_pos_rad` | `float[6]` | Joint angles in radians, J1–J6 |
| `manipulability` | `float` | Yoshikawa manipulability index (0=singular, >0.05=safe) |
| `in_singularity` | `bool` | True when manipulability < 0.05 |
| `source` | `string` | `"gazebo"` \| `"encoder"` \| `"pseudo_hw"` |
| `execution_state` | `string` | `"idle"` \| `"running"` \| `"paused"` \| `"halted"` |
| `pc` | `int` | Program counter (current segment index during execution) |

### 2.2 Compute IK via MoveIt (UI → Bridge → MoveIt)

**Request:**
```json
{
  "op": "call_service",
  "service": "/compute_ik",
  "id": "ik-req-001",
  "args": {
    "ik_request": {
      "group_name": "manipulator",
      "avoid_collisions": true,
      "robot_state": {
        "joint_state": {
          "name": ["joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"],
          "position": [0.0, -0.5, 1.0, 0.0, -0.8, 0.0]
        }
      },
      "pose_stamped": {
        "header": { "frame_id": "base_link" },
        "pose": {
          "position": { "x": 0.8, "y": 0.0, "z": 1.2 },
          "orientation": { "x": 0, "y": 0, "z": 0, "w": 1 }
        }
      },
      "timeout": { "sec": 5, "nanosec": 0 }
    }
  }
}
```

> **CRITICAL**: Position values are in **metres** (MoveIt standard). The UI stores in mm, so `BackendConnector.ts` divides by 1000 before sending.

**Success Response:**
```json
{
  "op": "service_response",
  "id": "ik-req-001",
  "result": true,
  "values": {
    "error_code": { "val": 1 },
    "solution": {
      "joint_state": {
        "name": ["joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"],
        "position": [0.0, -0.523, 1.047, 0.0, -0.785, 0.0]
      }
    }
  }
}
```

**Error codes** (`error_code.val`): `1` = SUCCESS, `-1` = PLANNING_FAILED, `-31` = NO_IK_SOLUTION

### 2.3 Compute FK (UI → Bridge → Local Kinematics)

**Request:**
```json
{
  "op": "call_service",
  "service": "/compute_fk",
  "id": "fk-req-001",
  "args": {
    "robot_state": {
      "joint_state": {
        "position": [0.0, -0.523, 1.047, 0.0, -0.785, 0.0]
      }
    }
  }
}
```

**Response:**
```json
{
  "op": "service_response",
  "id": "fk-req-001",
  "result": true,
  "values": {
    "pose_stamped": [{
      "pose": {
        "position": { "x": 0.8, "y": 0.0, "z": 1.2 },
        "orientation": { "x": 0, "y": 0, "z": 0, "w": 1 }
      }
    }]
  }
}
```

> FK is computed **locally** in the bridge node (no MoveIt round-trip needed) using the URDF-based kinematics engine.

### 2.4 Execute Compiled Program (UI → Bridge)

```json
{
  "op": "roboforge/execute_program",
  "program": [
    {
      "duration_s": 2.0,
      "points": [
        {
          "positions": [0.0, -0.523, 1.047, 0.0, -0.785, 0.0],
          "velocities": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
          "time_from_start": 2.0
        }
      ]
    }
  ]
}
```

**Progress callback** (per segment):
```json
{
  "op": "roboforge/execution_progress",
  "pc": 3,
  "total": 12,
  "state": "running"
}
```

### 2.5 Publish Raw Trajectory (UI → Bridge → ROS)

```json
{
  "op": "publish",
  "topic": "/joint_trajectory_command",
  "msg": {
    "joint_names": ["joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"],
    "points": [{
      "positions": [0.0, -0.5, 1.0, 0.0, -0.8, 0.0],
      "velocities": [0, 0, 0, 0, 0, 0],
      "time_from_start": { "sec": 1, "nanosec": 0 }
    }]
  }
}
```

### 2.6 Health Query (UI → Bridge)

```json
{ "op": "roboforge/health" }
```

**Response:**
```json
{
  "op": "roboforge/health_response",
  "status": "ok",
  "mode": "simulate",
  "connected_clients": 2,
  "execution_state": "idle",
  "moveit_ready": true,
  "kinematics_loaded": true
}
```

### 2.7 Motor Configuration (UI → Bridge → motor_controller_node)

```json
{
  "op": "roboforge/motor_config",
  "joint_idx": 0,
  "motor_type": "DC",
  "pid": { "kp": 1.2, "ki": 0.05 },
  "gear_ratio": 100,
  "max_current_a": 5.0,
  "signal_type": "PWM"
}
```

### 2.8 Encoder Configuration (UI → Bridge → encoder_interface_node)

```json
{
  "op": "roboforge/encoder_config",
  "joint_idx": 0,
  "encoder_type": "absolute",
  "counts_per_rev": 4096,
  "gear_ratio": 100.0,
  "zero_offset": 2048,
  "direction": 1
}
```

### 2.9 Power-On Self-Test (UI → Bridge)

```json
{ "op": "roboforge/post" }
```

**Response:**
```json
{
  "op": "roboforge/post_result",
  "passed": true,
  "results": [
    { "joint": 1, "encoder_ok": true, "motor_ok": true, "serial_ok": true, "latency_ms": 0.8 },
    { "joint": 2, "encoder_ok": true, "motor_ok": true, "serial_ok": true, "latency_ms": 0.9 }
  ]
}
```

---

## 3. REST API (Port 8765)

### GET `/health`

```json
{
  "status": "ok",
  "mode": "simulate",
  "backend": "ros2_humble",
  "connected_clients": 1,
  "execution_state": "idle",
  "moveit_ready": true,
  "kinematics_loaded": true,
  "robot_loaded": true
}
```

### GET `/api/hardware/ports`

Returns connected USB serial ports (uses `pyserial` port discovery):

```json
{
  "ports": [
    {
      "port": "/dev/ttyUSB0",
      "description": "STM32 Virtual COM Port",
      "vid": "0483",
      "pid": "5740",
      "manufacturer": "STMicroelectronics"
    }
  ]
}
```

### GET `/api/logs`

Returns the last N data log entries (line-delimited JSON):

```json
{
  "entries": [
    {
      "ts": "2026-04-06T22:30:00Z",
      "event": "ik_request",
      "solver": "moveit",
      "target_m": [0.8, 0.0, 1.2],
      "result": "success",
      "latency_ms": 45.2,
      "solution_rad": [0.0, -0.523, 1.047, 0.0, -0.785, 0.0]
    }
  ]
}
```

---

## 4. USB-Serial Hardware Protocols

### 4.1 Encoder Read Protocol (STM32 → PC)

The encoder board sends packets at 1000Hz over USB-Serial at **1 Mbaud**.

```
Packet: [0xAA] [0x55] [J1_hi] [J1_lo] [J2_hi] [J2_lo] ... [J6_hi] [J6_lo] [XOR_CRC]
         ──────────── ──────────────────────────────────────────────── ────────
         Header (2B)  Payload: 6 × uint16 big-endian (12B)            CRC (1B)
         Total: 15 bytes
```

| Field | Bytes | Format | Description |
|-------|-------|--------|-------------|
| Header | 0-1 | `0xAA 0x55` | Sync bytes |
| J1 count | 2-3 | `uint16 BE` | Raw encoder count (0–4095 for 12-bit, 0–65535 for 16-bit) |
| J2 count | 4-5 | `uint16 BE` | Raw encoder count |
| J3 count | 6-7 | `uint16 BE` | Raw encoder count |
| J4 count | 8-9 | `uint16 BE` | Raw encoder count |
| J5 count | 10-11 | `uint16 BE` | Raw encoder count |
| J6 count | 12-13 | `uint16 BE` | Raw encoder count |
| CRC | 14 | `uint8` | XOR of bytes 2-13 |

**Angle conversion**: `angle_rad = ((count - zero_offset) * direction / CPR) * (2π / gear_ratio)`

### 4.2 Motor Command Protocol (PC → Motor Driver)

Sent at 250Hz from `motor_controller_node`.

**DC Motor Command (4 bytes):**
```
[0xCC] [joint_idx] [duty_hi] [duty_lo]
```
- `duty`: uint16, 0=full reverse, 32768=stop, 65535=full forward

**BLDC Motor Command (8 bytes):**
```
[0xBB] [joint_idx] [da_hi] [da_lo] [db_hi] [db_lo] [dc_hi] [dc_lo]
```
- `da/db/dc`: uint16, 3-phase duty cycles 0–65535 (SVPWM output)

### 4.3 Encoder Type Support

| Type | Protocol | Resolution | Output |
|------|----------|-----------|--------|
| Incremental (Quadrature) | A/B/Z channels via STM32 timer | 4× decoding of CPR | Raw count, wraps |
| Absolute (SPI - AS5047P) | SPI @ 10MHz, 14-bit | 16384 CPR | Single-turn absolute |
| Absolute (SSI - BiSS-C) | Clock+Data synchronous serial | Up to 22-bit | Multi-turn absolute |
| Absolute (I2C - AS5600) | I2C @ 400kHz, 12-bit | 4096 CPR | Single-turn absolute |

---

## 5. ROS 2 Topic & Service Map

| Name | Type | Message Type | Hz | Publisher | Subscriber |
|------|------|-------------|-----|-----------|------------|
| `/joint_states` | Topic | `sensor_msgs/JointState` | 250 | pseudo_hw / encoder_node / gazebo | bridge, moveit |
| `/joint_trajectory_command` | Topic | `trajectory_msgs/JointTrajectory` | On-demand | bridge | pseudo_hw, motor_controller |
| `/planned_trajectory` | Topic | `trajectory_msgs/JointTrajectory` | On-demand | bridge (legacy) | bridge |
| `/compute_ik` | Service | `moveit_msgs/GetPositionIK` | On-demand | — | moveit (server), bridge (client) |
| `/compute_fk` | Service | `moveit_msgs/GetPositionFK` | On-demand | — | moveit (server), bridge (client) |
| `/roboforge/health_check` | Service | `std_srvs/Trigger` | On-demand | — | bridge (server) |
| `/roboforge/motor_config` | Topic | `std_msgs/String` (JSON) | On-demand | bridge | motor_controller |
| `/roboforge/encoder_config` | Topic | `std_msgs/String` (JSON) | On-demand | bridge | encoder_interface |
| `/roboforge/hw_status` | Topic | `std_msgs/String` (JSON) | 1 | pseudo_hw | bridge |
| `/roboforge/tracking_alert` | Topic | `std_msgs/String` (JSON) | On-demand | gazebo_error_node | bridge |
| `/encoder/raw` | Topic | `std_msgs/Float64MultiArray` | 250 | encoder_interface | diagnostics UI |
| `/diagnostics` | Topic | `diagnostic_msgs/DiagnosticArray` | 1 | bridge | diagnostics UI |
| `/robot_description` | Topic | `std_msgs/String` | Latched | robot_state_publisher | moveit, gazebo |

---

## 6. IK Solver Routing Policy

The user selects IK mode in **Settings → Motion**. There is **no automatic fallback**.

| Setting | Route | Backend Required? | Solver |
|---------|-------|-------------------|--------|
| **MoveIt 2** (default) | UI → WS → Bridge → `/compute_ik` → MoveIt | ✅ Docker stack must be running | KDL / TRAC-IK |
| **Offline — Analytical** | UI → `IKSolver.ts` (local JS) | ❌ No backend needed | Closed-form geometric |
| **Offline — Numerical DLS** | UI → `IKSolver.ts` (local JS) | ❌ No backend needed | Damped Least Squares, 8 random seeds |

---

## 7. Execution Pipeline (Program Run)

```
User clicks "Run"
       │
       ▼
  AppState.runProgram()
       │
       ├─ For each block in program tree:
       │     │
       │     ├─ MoveJ/MoveL → computeIK(target) via BackendConnector
       │     │     │
       │     │     ├─ (MoveIt mode) → WS call_service /compute_ik → joint angles
       │     │     └─ (Offline mode) → IKSolver.ts → joint angles
       │     │
       │     ├─ animateToTarget(joints, duration) → smooth interpolation → update 3D
       │     │
       │     ├─ publishTrajectory(joints) → WS publish to /joint_trajectory_command
       │     │     │
       │     │     ├─ pseudo_hardware_node → interpolates → publishes /joint_states
       │     │     └─ motor_controller_node → PI/FOC control → serial PWM to motors
       │     │
       │     ├─ SetDO/GetDI → I/O point state update
       │     ├─ Wait → setTimeout
       │     ├─ If/While/For → control flow evaluation
       │     └─ Gripper → gripper command via I/O
       │
       ▼
  Console: "✓ Program execution complete"
```

---

## 8. Docker Network Topology

```
docker-compose.yml services:
  ┌─ ros_core ──────────┐  Base: osrf/ros:humble-desktop-full
  │  robot_state_pub     │  Publishes /robot_description from URDF
  │  ROS_DOMAIN_ID=42    │
  └──────────────────────┘
           ▼ (healthcheck: ros2 node list)
  ┌─ moveit ─────────────┐  Depends: ros_core (healthy)
  │  move_group           │  Provides /compute_ik, /compute_fk
  │  OMPL planner         │  Uses kinematics.yaml, ompl_planning.yaml
  └──────────────────────┘
           ▼
  ┌─ pseudo_hardware ────┐  Depends: ros_core (healthy)
  │  250Hz /joint_states  │  Subscribes /joint_trajectory_command
  │  Cubic interpolation  │  Publishes /roboforge/hw_status
  └──────────────────────┘
           ▼
  ┌─ bridge ─────────────┐  Depends: moveit, pseudo_hardware
  │  WS server :9090      │  REST server :8765
  │  IK/FK proxy          │  Trajectory relay
  │  Health monitoring    │  Data logging
  └──────────────────────┘
           ▼
  ┌─ frontend ───────────┐  Depends: bridge
  │  Vite dev server      │  Port 3000 → 8080
  │  React IDE            │
  └──────────────────────┘

  ┌─ gazebo (OPTIONAL) ──┐  Profile: sim
  │  Gazebo Harmonic      │  Full physics simulation
  └──────────────────────┘
```

---

## 9. Data Logging (bridge_node.py)

All IK/FK requests, execution events, and hardware interactions are logged via the `DataLogger` class.

### Storage
- **In-memory**: Ring buffer of last 2000 entries (deque)
- **On-disk**: Append-only JSONL file at `/ros_ws/logs/roboforge_datalog.jsonl`

### Logged Events

| Event | Fields | When |
|-------|--------|------|
| `ik_request` | solver, target_m, result, latency_ms, solution_rad, error_code | Every MoveIt IK call |
| `program_start` | segments | User clicks Run |
| `program_complete` | segments_executed | All segments done |
| `motor_config` | joint_idx, motor_type, pid, gear_ratio | Hardware config saved |
| `encoder_config` | joint_idx, encoder_type, cpr, gear_ratio | Hardware config saved |
| `post_result` | joint, encoder_ok, motor_ok, serial_ok, latency_ms | POST test complete |

### Example Log Entry
```json
{
  "ts": "2026-04-06T22:30:00.123456+00:00",
  "event": "ik_request",
  "solver": "moveit",
  "target_m": [0.8, 0.0, 1.2],
  "result": "success",
  "latency_ms": 45.21,
  "solution_rad": [0.0, -0.523, 1.047, 0.0, -0.785, 0.0]
}
```

---

## 10. Motor Control Loops (motor_controller_node.py)

The `motor_controller_node` runs at **250Hz** (4ms period). On each tick:

### DC Motor Control (PI Controller)
```
Input:  setpoint_rad (from trajectory), current_rad (from encoder)
Error:  e = setpoint_rad - current_rad
Integral: integral += e * dt
Output: duty_raw = Kp * e + Ki * integral
Clamp:  duty = clamp(duty_raw, -1.0, 1.0)
Scale:  duty_u16 = (duty * 32767) + 32768
Packet: [0xCC] [joint_idx] [duty_hi] [duty_lo] → serial
```

### BLDC Motor Control (FOC — Field-Oriented Control)
```
Input:  setpoint_rad (from trajectory), current_rad (from encoder)

1. Position error → velocity setpoint (outer loop, P control)
2. Electrical angle: θe = (current_rad * gear_ratio * pole_pairs) mod (2π)
3. Phase currents: read Ia, Ib from ADC via serial
4. Clarke transform:  Iα = Ia,  Iβ = (Ia + 2·Ib) / √3
5. Park transform:    Id = Iα·cos(θe) + Iβ·sin(θe)
                      Iq = -Iα·sin(θe) + Iβ·cos(θe)
6. PI control (d-axis):  Vd = Kp_d * (Id_ref - Id) + Ki_d * ∫(Id_ref - Id)
   PI control (q-axis):  Vq = Kp_q * (Iq_ref - Iq) + Ki_q * ∫(Iq_ref - Iq)
   where Id_ref = 0 (field weakening off), Iq_ref ∝ torque command
7. Inverse Park:  Vα = Vd·cos(θe) - Vq·sin(θe)
                  Vβ = Vd·sin(θe) + Vq·cos(θe)
8. SVPWM:  da, db, dc = space_vector_pwm(Vα, Vβ)
9. Packet: [0xBB] [joint_idx] [da_hi] [da_lo] [db_hi] [db_lo] [dc_hi] [dc_lo] → serial
```

### Trajectory Interpolation (pseudo_hardware_node.py)
```
Input:  JointTrajectory message with N waypoints
Method: Cubic spline interpolation between waypoints
Output: /joint_states at 250Hz with interpolated positions
```

---

## 11. Frontend API Reference (BackendConnector.ts)

| Method | Direction | Transport | Description |
|--------|-----------|-----------|-------------|
| `connect(url)` | UI → Bridge | WS | Establish rosbridge connection |
| `computeIK(target, ikMode)` | UI → Bridge | WS | Solve IK (MoveIt or offline) |
| `computeFK(joints)` | UI → Bridge | WS | Compute FK via bridge kinematics |
| `publishTrajectory(trajectory)` | UI → Bridge | WS | Publish to /joint_trajectory_command |
| `executeProgram(program)` | UI → Bridge | WS | Submit compiled program for execution |
| `subscribeJointStates(cb)` | Bridge → UI | WS | Subscribe to /joint_states (50Hz) |
| `publishMotorConfig(idx, config)` | UI → Bridge | WS | Publish motor config to /roboforge/motor_config |
| `publishEncoderConfig(idx, config)` | UI → Bridge | WS | Publish encoder config to /roboforge/encoder_config |
| `onEncoderRaw(cb)` | Bridge → UI | WS | Subscribe to /encoder/raw for live diagnostics |
| `requestPOST()` | UI → Bridge | WS | Call /roboforge/health_check service |
| `onStatus(cb)` | Bridge → UI | WS | Status change callback |
| `onJointState(cb)` | Bridge → UI | WS | Joint state update callback |

### Offline vs Online Behavior

When `mode === 'offline'`:
- `publishMotorConfig`, `publishEncoderConfig`, `requestPOST` → log locally, do not send
- `computeIK` → routed to `IKSolver.ts` (never calls bridge)
- Config is persisted in `localStorage` only

When `mode === 'online'`:
- All methods send data over WebSocket to the bridge
- Config is persisted in `localStorage` AND broadcast to ROS topics
- Live encoder data streams via `/encoder/raw` subscription

---

## 12. WPF ↔ React Parity Map

| Feature | React Online IDE | WPF Offline Client |
|---------|-----------------|-------------------|
| IK Solver | `BackendConnector.computeIK()` → WS | `MainViewModel.ComputeMoveItIKAsync()` → WS |
| Program Execution | `AppState.runProgram()` | `MainViewModel.RunAsync()` |
| Joint State Display | `useAppState().jointAngles` | `MainViewModel.J1-J6` properties |
| Console Logging | `addConsoleEntry()` | `AddConsole()` |
| Hardware Config | `HardwareConfigPage.tsx` | `HardwareConfigViewModel` |
| POST Test | `handleRunPOST()` | `HardwareConfigViewModel.RunPostAsync()` |
| Motor Config Save | `localStorage` + WS publish | `robot_hardware_config.json` file |
| Connection Routing | `BackendConnector` (auto-detect) | `MainViewModel.AutoConnectAsync()` |
| Health Check | `BackendConnector.requestPOST()` | `MainViewModel.RunHealthCheckAsync()` |
| IK Mode Toggle | Settings panel + `IKMode` state | `ToggleIKMode()` command |
