# SYSTEM LOGIC AND FLOW — Industrial Robot Platform

## 1. System Overview

This document defines the complete architecture, data flows, authority hierarchy, and operational logic for a **Plug-and-Play Industrial Robot Platform**. The system is designed for:

- Simulation-to-hardware parity
- Real-time determinism where required
- Strict domain separation
- Docker-first deployment
- Mode switching without code changes

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        INDUSTRIAL ROBOT PLATFORM                            │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                   NON-REAL-TIME DOMAIN (Docker)                     │   │
│  │  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────────────┐    │   │
│  │  │  ROS 2   │  │  MoveIt  │  │  Gazebo  │  │  Optional UI     │    │   │
│  │  │  Core    │  │  Planner │  │  Physics │  │  (Web/Desktop)   │    │   │
│  │  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────────┬─────────┘    │   │
│  │       │             │             │                  │              │   │
│  │       └─────────────┴──────┬──────┴──────────────────┘              │   │
│  │                            │                                        │   │
│  │                   [ ROS 2 Message Bus ]                             │   │
│  │                            │                                        │   │
│  │                   ┌────────┴────────┐                               │   │
│  │                   │ Hardware Bridge │                               │   │
│  │                   │   (ROS Node)    │                               │   │
│  │                   └────────┬────────┘                               │   │
│  └────────────────────────────┼────────────────────────────────────────┘   │
│                               │                                             │
│                    [ COMMUNICATION INTERFACE ]                              │
│                    (Serial / Ethernet / CAN)                                │
│                               │                                             │
│  ┌────────────────────────────┼────────────────────────────────────────┐   │
│  │                   REAL-TIME DOMAIN (Controller)                     │   │
│  │                            │                                        │   │
│  │                   ┌────────┴────────┐                               │   │
│  │                   │  RT Controller  │                               │   │
│  │                   │  (MCU / RT-PC)  │                               │   │
│  │                   └────────┬────────┘                               │   │
│  │                            │                                        │   │
│  │       ┌────────────────────┼────────────────────┐                   │   │
│  │       │                    │                    │                   │   │
│  │  ┌────┴────┐         ┌─────┴─────┐        ┌─────┴─────┐            │   │
│  │  │ Motors  │         │ Encoders  │        │  E-STOP   │            │   │
│  │  │ Drivers │         │ Sensors   │        │  Hardware │            │   │
│  │  └─────────┘         └───────────┘        └───────────┘            │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 2. Domain Separation

The system is strictly divided into two execution domains. **No component may cross domain boundaries.**

### 2.1 NON-REAL-TIME DOMAIN

| Component       | Location        | Purpose                                   |
|-----------------|-----------------|-------------------------------------------|
| ROS 2 Humble    | Docker          | Message routing, node lifecycle           |
| MoveIt 2        | Docker          | Motion planning, collision checking       |
| Gazebo          | Docker          | Physics simulation (SIM mode only)        |
| Hardware Bridge | Docker          | Protocol translation to controller        |
| Web UI          | Docker/Host     | Human interaction (optional)              |
| Analysis Nodes  | Docker          | Logging, accuracy comparison (optional)   |

**Allowed Operations:**
- Trajectory planning
- Collision checking
- Visualization
- Goal selection
- Parameter configuration
- State observation

**Forbidden Operations:**
- Motor control loops
- Safety enforcement
- Encoder interrupt handling
- PWM generation
- Real-time scheduling

### 2.2 REAL-TIME DOMAIN

| Component           | Location     | Purpose                                |
|---------------------|--------------|----------------------------------------|
| RT Controller       | MCU/RT-Linux | Trajectory execution, motor control    |
| Motor Drivers       | Hardware     | Current/voltage application            |
| Encoders            | Hardware     | Position truth generation              |
| Hardware E-STOP     | Hardware     | Immediate power cut                    |
| Safety Watchdog     | MCU          | Timeout and fault detection            |

**Allowed Operations:**
- Fixed-period control loops (1kHz+)
- Trajectory interpolation
- PID/Feedforward control
- Safety limit enforcement
- Encoder reading
- Fault detection and response

**Forbidden Operations:**
- Motion planning
- Collision geometry processing
- Visualization
- UI interactions
- Non-deterministic operations

**Watchdog Configuration:**
- Nominal timeout: 10 ms
- Configurable range: 5–50 ms
- Configuration location: Controller firmware parameters or `config/controller_params.yaml`

---

## 3. Authority Hierarchy

```
                    ┌─────────────────────────┐
                    │     Hardware E-STOP     │  ← ULTIMATE AUTHORITY
                    │   (Physical Switch)     │     (Cuts Power)
                    └───────────┬─────────────┘
                                │
                    ┌───────────▼─────────────┐
                    │    RT Controller        │  ← PRIMARY AUTHORITY
                    │  (Safety + Execution)   │     (All Motor Control)
                    └───────────┬─────────────┘
                                │
                    ┌───────────▼─────────────┐
                    │   Hardware Bridge       │  ← RELAY ONLY
                    │   (Protocol Gateway)    │     (No Authority)
                    └───────────┬─────────────┘
                                │
                    ┌───────────▼─────────────┐
                    │   Motion Planner        │  ← REQUEST ONLY
                    │   (MoveIt / Custom)     │     (Intent Generation)
                    └───────────┬─────────────┘
                                │
                    ┌───────────▼─────────────┐
                    │   User Interface        │  ← OBSERVATION ONLY
                    │   (Web / Gazebo GUI)    │     (Display + Trigger)
                    └─────────────────────────┘
```

### Authority Rules

| Entity             | Can Command Motors | Can Stop Motors | Owns Position Truth | Owns Safety |
|--------------------|-------------------|-----------------|---------------------|-------------|
| Hardware E-STOP    | No (cuts power)   | YES (absolute)  | No                  | YES         |
| RT Controller      | YES               | YES             | YES                 | YES         |
| Hardware Bridge    | No                | Request only    | No                  | No          |
| Motion Planner     | No                | Request only    | No                  | No          |
| User Interface     | No                | Request only    | No                  | No          |
| Gazebo (SIM)       | Emulated (no real authority) | Emulated (no real authority) | Emulated (no real authority) | Emulated (no real authority) |

> **Note on Gazebo Authority:** Gazebo emulates motor control, stopping, position truth, and safety behaviors for testing purposes only. No real authority is granted. SIM mode results do not constitute safety validation.

---

## 4. Data Flow Diagrams

### 4.1 Command Flow (Intent → Execution)

```
┌──────────┐      ┌──────────┐      ┌──────────┐      ┌──────────┐
│   UI     │      │  MoveIt  │      │  Bridge  │      │Controller│
│ (Goal)   │      │(Planner) │      │ (Relay)  │      │  (Exec)  │
└────┬─────┘      └────┬─────┘      └────┬─────┘      └────┬─────┘
     │                 │                 │                 │
     │  1. Goal Pose   │                 │                 │
     │────────────────>│                 │                 │
     │                 │                 │                 │
     │                 │ 2. Plan         │                 │
     │                 │ Trajectory      │                 │
     │                 │────────────────>│                 │
     │                 │                 │                 │
     │                 │                 │ 3. Serialize    │
     │                 │                 │ & Transmit      │
     │                 │                 │────────────────>│
     │                 │                 │                 │
     │                 │                 │                 │ 4. Buffer
     │                 │                 │                 │ & Validate
     │                 │                 │                 │
     │                 │                 │                 │ 5. Execute
     │                 │                 │                 │ (RT Loop)
     │                 │                 │                 │
```

### 4.2 Feedback Flow (State → Observation)

```
┌──────────┐      ┌──────────┐      ┌──────────┐      ┌──────────┐
│Controller│      │  Bridge  │      │   ROS    │      │   UI     │
│ (Truth)  │      │ (Relay)  │      │  (Pub)   │      │(Display) │
└────┬─────┘      └────┬─────┘      └────┬─────┘      └────┬─────┘
     │                 │                 │                 │
     │ 1. Encoder      │                 │                 │
     │ Readings        │                 │                 │
     │────────────────>│                 │                 │
     │                 │                 │                 │
     │                 │ 2. Publish      │                 │
     │                 │ JointState      │                 │
     │                 │────────────────>│                 │
     │                 │                 │                 │
     │                 │                 │ 3. Subscribe    │
     │                 │                 │────────────────>│
     │                 │                 │                 │
     │                 │                 │                 │ 4. Render
     │                 │                 │                 │
```

### 4.3 Mode Comparison: SIM vs REAL

```
SIM MODE:
┌──────────┐     ┌──────────┐     ┌───────────────────────┐
│  MoveIt  │────>│  Bridge  │────>│        Gazebo         │
│(Planner) │     │ (Relay)  │     │ (Non-RT Simulation)   │
└──────────┘     └──────────┘     └───────────────────────┘
                                          │
                      ┌───────────────────┘
                      │
                      ▼
              ┌──────────────┐
              │ Simulated    │
              │ Joint States │
              └──────────────┘

REAL MODE:
┌──────────┐     ┌──────────┐     ┌──────────────┐
│  MoveIt  │────>│  Bridge  │────>│ RT Controller│
│(Planner) │     │ (Relay)  │     │  (Real HW)   │
└──────────┘     └──────────┘     └──────────────┘
                                         │
                      ┌──────────────────┘
                      │
                      ▼
              ┌──────────────┐
              │ Real Encoder │
              │ Joint States │
              └──────────────┘
```

---

## 5. Timing Domains

### 5.1 Timing Budget

| Domain          | Loop Rate     | Jitter Tolerance | Deadline Miss Handling |
|-----------------|---------------|------------------|------------------------|
| RT Controller   | 1000 Hz       | < 100 μs         | Immediate E-STOP       |
| ROS 2 Nodes     | 10-100 Hz     | 10-100 ms        | Logged, degraded mode  |
| Gazebo Physics  | 1000 Hz (sim) | Non-binding      | Frame skip allowed     |
| UI Updates      | 30-60 Hz      | Unbounded        | Frame drop allowed     |

### 5.2 Time Synchronization

```
┌─────────────────────────────────────────────────────────────────┐
│                    TIME DOMAIN BOUNDARIES                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌───────────────────────────────────────────────────────────┐ │
│  │           NON-REAL-TIME (Best Effort)                     │ │
│  │                                                           │ │
│  │    ROS Time ←──── NTP Sync ────→ Host Clock              │ │
│  │         │                                                 │ │
│  │         ▼                                                 │ │
│  │   ┌─────────────┐   ┌─────────────┐   ┌───────────────┐  │ │
│  │   │   MoveIt    │   │   Gazebo    │   │     UI        │  │ │
│  │   │  (ROS Time) │   │ (Sim Time)  │   │  (Wall Time)  │  │ │
│  │   └─────────────┘   └─────────────┘   └───────────────┘  │ │
│  │                                                           │ │
│  └───────────────────────────────────────────────────────────┘ │
│                            │                                    │
│                   ═════════╪═════════ BOUNDARY                  │
│                            │                                    │
│  ┌───────────────────────────────────────────────────────────┐ │
│  │             REAL-TIME (Hard Deadlines)                    │ │
│  │                                                           │ │
│  │    ┌────────────────────────────────────────┐            │ │
│  │    │         RT Controller Clock            │            │ │
│  │    │      (Independent, Deterministic)      │            │ │
│  │    └────────────────────────────────────────┘            │ │
│  │                                                           │ │
│  │    - Does NOT sync to ROS time                           │ │
│  │    - Trajectory timestamps are relative                  │ │
│  │    - Execution timing owned by controller                │ │
│  │                                                           │ │
│  └───────────────────────────────────────────────────────────┘ │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## 6. Module Boundaries

### 6.1 Module Dependency Map

```
┌─────────────────────────────────────────────────────────────────┐
│                     MODULE DEPENDENCY GRAPH                     │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│    ┌─────────────┐                                             │
│    │    UI       │ ─────────────────────────────────┐          │
│    │  (Optional) │                                   │          │
│    └──────┬──────┘                                   │          │
│           │ uses                                     │          │
│           ▼                                          ▼          │
│    ┌─────────────┐      ┌─────────────┐      ┌─────────────┐   │
│    │   Planner   │─────>│   Bridge    │<─────│  Analysis   │   │
│    │  (MoveIt)   │      │   (Node)    │      │  (Optional) │   │
│    └──────┬──────┘      └──────┬──────┘      └─────────────┘   │
│           │                    │                                │
│           │ trajectory         │ serialized                     │
│           ▼                    ▼                                │
│    ┌─────────────────────────────────────────────────────────┐ │
│    │                 ROS 2 MESSAGE LAYER                     │ │
│    │   (sensor_msgs/JointState, trajectory_msgs/*, std_srvs) │ │
│    └─────────────────────────────────────────────────────────┘ │
│           │                    │                                │
│           │ SIM path           │ REAL path                      │
│           ▼                    ▼                                │
│    ┌───────────────────┐ ┌─────────────┐                       │
│    │      Gazebo       │ │ Controller  │                       │
│    │ (Non-RT Simulation)│ │  (Real RT)  │                       │
│    └───────────────────┘ └─────────────┘                       │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### 6.2 Module Responsibility Matrix

| Module            | Plans Motion | Executes Motion | Owns Safety | Owns Position | Optional |
|-------------------|--------------|-----------------|-------------|---------------|----------|
| UI                | No           | No              | No          | No            | YES      |
| MoveIt            | YES          | No              | No          | No            | No       |
| Hardware Bridge   | No           | No              | No          | No            | No       |
| Gazebo            | No           | Emulated (non-RT) | Emulated (non-certified) | Emulated | SIM only |
| RT Controller     | No           | YES             | YES         | YES           | No       |
| Analysis Nodes    | No           | No              | No          | No            | YES      |

> **Note on Gazebo Execution:** Gazebo provides physics emulation for testing and development. It does not provide real-time guarantees, certified safety behavior, or authoritative position data. SIM mode testing does not constitute safety validation. All safety testing must be performed with real hardware in controlled conditions.

### 6.3 Interface Contracts

**Trajectory Command (ROS → Controller):**
```
Message: trajectory_msgs/JointTrajectory
Fields:
  - header.stamp: Reference time (relative, not absolute)
  - joint_names: ["joint_1", "joint_2", ..., "joint_N"]
  - points[]: Array of JointTrajectoryPoint
    - positions[]: Target positions (radians)
    - velocities[]: Target velocities (rad/s)
    - time_from_start: Duration from trajectory start

Constraints:
  - Maximum 100 points per message
  - Points must be temporally ordered
  - Velocities must respect joint limits
  - Controller performs all semantic validation (joint limits, velocity bounds, physics constraints)
  - Controller may reject invalid trajectories
```

**Trajectory Acknowledgment (Controller → ROS):**
```
Message: Custom TrajectoryAck
Fields:
  - trajectory_id: Echoed from command (header.stamp or sequence number)
  - status: ACCEPTED | REJECTED | QUEUED
  - reject_reason: (if REJECTED) Human-readable string description
    - Examples: "JOINT_LIMIT_EXCEEDED", "VELOCITY_TOO_HIGH", "BUFFER_FULL"

Publishing: Immediately upon trajectory receipt and validation
Behavior:
  - ACCEPTED: Trajectory passed validation, will execute
  - REJECTED: Trajectory failed validation, discarded
  - QUEUED: Trajectory accepted but waiting for current execution to complete
```

**Joint State (Controller → ROS):**
```
Message: sensor_msgs/JointState
Fields:
  - header.stamp: Monotonic controller timestamp (microseconds since controller boot)
  - name: ["joint_1", "joint_2", ..., "joint_N"]
  - position: Current positions (radians, from encoders)
  - velocity: Current velocities (rad/s, computed)
  - effort: Current efforts (Nm, if available)

Publishing Rate: 100 Hz minimum
Latency Budget: < 10 ms end-to-end

Note: Timestamp is NOT wall-clock time. It is a monotonic counter from
controller boot, used for sequencing and latency measurement only.
```

**Execution State (Controller → ROS):**
```
Message: Custom ExecutionState
Fields:
  - state: IDLE | EXECUTING | PAUSED | FAULT
  - progress: 0.0 - 1.0 (trajectory completion)
  - fault_code: 0 = OK, non-zero = specific fault
  - fault_message: Human-readable description

Publishing Rate: 10 Hz or on state change

Upstream Behavior on FAULT:
  - When state == FAULT, upstream nodes (MoveIt, UI) MUST suspend
    trajectory planning and command generation
  - Nodes should subscribe to /execution_state and implement fault handling
  - New trajectories will be REJECTED until fault is cleared
```

**Hardware Bridge Validation Scope:**
```
The Hardware Bridge performs ONLY:
  - Schema validation (correct message type, field presence, array lengths)
  - Protocol serialization (ROS → controller wire format)

The Hardware Bridge does NOT perform:
  - Joint limit checking
  - Velocity bound checking
  - Collision detection
  - Physics constraint validation

All semantic validation is the exclusive responsibility of the RT Controller.
```

---

## 7. Failure Scenarios

### 7.1 Failure Response Matrix

| Failure Type                | Detection Point    | Immediate Response           | Recovery Action              |
|-----------------------------|--------------------|------------------------------|------------------------------|
| Hardware E-STOP pressed     | Controller         | Cut motor power              | Manual reset required        |
| Controller watchdog timeout | Controller         | Hold position / coast down   | Restart controller           |
| Communication loss (ROS)    | Controller         | Complete or stop trajectory  | Reconnection allowed         |
| Joint limit approach        | Controller         | Decelerate, reverse if needed| Automatic                    |
| Overcurrent detection       | Controller         | Reduce torque, alert         | Automatic with cooldown      |
| Encoder error               | Controller         | Stop motion, report fault    | Manual inspection required   |
| ROS node crash              | ROS                | Unaffected (controller safe) | Restart node                 |
| Gazebo crash (SIM)          | Gazebo             | Simulation stops             | Restart simulation           |
| UI crash                    | UI                 | No effect on system          | Restart UI                   |

### 7.2 Communication Loss Behavior

```
┌─────────────────────────────────────────────────────────────────┐
│              COMMUNICATION LOSS HANDLING                        │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  Scenario: ROS → Controller link lost during execution          │
│                                                                 │
│  Time ────────────────────────────────────────────────────────> │
│                                                                 │
│  ROS:    ═══════════════╳ (crash/disconnect)                   │
│                          │                                      │
│  Controller:             │                                      │
│    ├── Detect timeout (100ms)                                  │
│    ├── Enter COMM_LOSS state                                   │
│    ├── IF trajectory buffered:                                 │
│    │     └── Complete current trajectory safely                │
│    ├── ELSE:                                                   │
│    │     └── Decelerate to stop                                │
│    └── Enter IDLE, await reconnection                          │
│                                                                 │
│  Motors: SAFE - No jitter, no sudden stops                     │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### 7.3 Fault Propagation

```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│  CRITICAL   │────>│   MAJOR     │────>│   MINOR     │
│   FAULTS    │     │   FAULTS    │     │   FAULTS    │
├─────────────┤     ├─────────────┤     ├─────────────┤
│ - E-STOP    │     │ - Joint lim │     │ - Comm warn │
│ - Overcurr  │     │ - Encoder   │     │ - Temp warn │
│ - Watchdog  │     │ - Overtemp  │     │ - Voltage   │
├─────────────┤     ├─────────────┤     ├─────────────┤
│ Response:   │     │ Response:   │     │ Response:   │
│ Immediate   │     │ Controlled  │     │ Log only    │
│ Stop        │     │ Stop        │     │             │
└─────────────┘     └─────────────┘     └─────────────┘

Upstream Fault Propagation:
  - Controller publishes FAULT state via /execution_state
  - MoveIt: Cancel pending plans, reject new planning requests
  - UI: Display fault indicator, disable execution buttons
  - Analysis: Log fault event with timestamp and code
```

---

## 8. SIM vs REAL Comparison

### 8.1 Component Mapping

| Aspect                | SIM Mode                               | REAL Mode              |
|-----------------------|----------------------------------------|------------------------|
| Physics Engine        | Gazebo (non-certified simulation)      | Real World             |
| Motor Control         | Gazebo controllers (non-RT)            | RT Controller          |
| Encoder Source        | Gazebo joint states (simulated)        | Hardware encoders      |
| Safety Enforcement    | Gazebo collision (non-certified)       | Hardware limits        |
| E-STOP                | Simulated (pause, no power cut)        | Hardware switch        |
| Timing Guarantees     | None (best effort, non-deterministic)  | Hard real-time         |

> **SIM Mode Safety Disclaimer:** SIM mode does not constitute safety validation. Gazebo collision detection and physics are approximations suitable for development and algorithm testing only. Do not deploy to production based solely on SIM testing. All safety-critical validation must be performed with real hardware in controlled conditions with proper safety measures in place.

### 8.2 Identical Elements (Parity Requirements)

| Element                     | SIM | REAL | Notes                          |
|-----------------------------|-----|------|--------------------------------|
| ROS 2 message types         | ✓   | ✓    | Exact same interfaces          |
| URDF robot description      | ✓   | ✓    | Single source of truth         |
| MoveIt configuration        | ✓   | ✓    | Same planning parameters       |
| Launch file structure       | ✓   | ✓    | Mode selected via argument     |
| Hardware bridge node        | ✓   | ✓    | Backend differs, API identical |
| Joint names and ordering    | ✓   | ✓    | Defined in URDF                |

### 8.3 Mode Switching Logic

```
┌─────────────────────────────────────────────────────────────────┐
│                    MODE SELECTION FLOW                          │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   Environment Variable: ROBOT_MODE = "SIM" | "REAL"             │
│                              │                                  │
│                              ▼                                  │
│                    ┌─────────┴─────────┐                       │
│                    │   Launch File     │                       │
│                    │   (Conditional)   │                       │
│                    └─────────┬─────────┘                       │
│                              │                                  │
│           ┌──────────────────┴──────────────────┐              │
│           │                                      │              │
│           ▼                                      ▼              │
│    ┌─────────────┐                        ┌─────────────┐      │
│    │  SIM Mode   │                        │ REAL Mode   │      │
│    │             │                        │             │      │
│    │ - Gazebo    │                        │ - No Gazebo │      │
│    │ - Non-RT    │                        │ - Real HW   │      │
│    │ - Safe test │                        │ - Live exec │      │
│    └─────────────┘                        └─────────────┘      │
│                                                                 │
│   Code Changes Required: NONE                                   │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## 9. Plug-and-Play Swap Explanation

### 9.1 Controller Swap

```
Fake Controller → Real Controller
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

1. Same ROS interface:
   └── trajectory_msgs/JointTrajectory IN
   └── sensor_msgs/JointState OUT
   └── TrajectoryAck OUT
   └── ExecutionState OUT

2. Configuration change only:
   └── CONTROLLER_TYPE="FAKE" → "REAL"
   └── CONTROLLER_IP="127.0.0.1" → "192.168.1.100"

3. No code modifications required
```

### 9.2 UI Swap

```
Web UI → No UI → Gazebo-Only
━━━━━━━━━━━━━━━━━━━━━━━━━━━━

1. UI is completely optional
2. System operates without any UI
3. Gazebo GUI can serve as minimal UI
4. Configuration:
   └── ENABLE_UI="true" | "false"
   └── UI_TYPE="web" | "desktop" | "none"
```

### 9.3 Analysis Swap

```
Analysis ON → Analysis OFF
━━━━━━━━━━━━━━━━━━━━━━━━━━

1. Analysis nodes are optional
2. Zero impact on execution when disabled
3. Configuration:
   └── ENABLE_ANALYSIS="true" | "false"
   └── LOG_LEVEL="DEBUG" | "INFO" | "ERROR"
```

---

## 10. Execution Step-by-Step Flows

### 10.1 Normal Trajectory Execution

```
Step 1: User sets goal via UI or API
        └── Goal pose published to /goal_pose topic

Step 2: MoveIt receives goal
        └── Collision checking performed
        └── Trajectory planned
        └── Trajectory published to /planned_trajectory

Step 3: User confirms execution (optional)
        └── Execution command sent

Step 4: Hardware Bridge receives trajectory
        └── Validates message schema (field types, array lengths)
        └── Serializes for controller protocol
        └── Transmits via configured protocol
        └── NOTE: Bridge does NOT validate joint limits or physics

Step 5: Controller receives trajectory
        └── Performs semantic validation (joint limits, velocities, physics)
        └── Sends TrajectoryAck (ACCEPTED, REJECTED, or QUEUED)
        └── If ACCEPTED: Buffers trajectory points
        └── Begins execution at configured rate (1 kHz)

Step 6: Controller executes
        └── Interpolates between points
        └── Applies PID control
        └── Monitors safety constraints
        └── Publishes joint states (100 Hz)

Step 7: Execution completes
        └── Controller enters IDLE
        └── Final position published
        └── UI updates to show completion
```

### 10.2 Emergency Stop Sequence

```
Step 1: E-STOP triggered
        ├── Hardware E-STOP button pressed (PRIMARY)
        │   └── Electrical circuit opens, power cut immediately
        │   └── Bypasses all software, including controller
        │
        └── Software stop request issued (SECONDARY)
            └── ROS/UI sends stop request message
            └── Controller receives and processes request
            └── Controller decides whether and how to stop
            └── Controller retains authority over stop behavior

Step 2: Controller response (if software stop, or post-hardware-stop)
        └── Control loop interrupted (<1 ms for software stop)
        └── Motor power reduced/cut
        └── Brakes engaged (if available)
        └── State set to FAULT

Step 3: Controller broadcasts fault
        └── Fault code published to /execution_state
        └── Timestamp recorded

Step 4: ROS nodes receive fault (upstream halt behavior)
        └── MoveIt: Cancel pending plans, suspend new planning
        └── UI: Display emergency state, disable controls
        └── Bridge: Queue cleared, new trajectories rejected

Step 5: Recovery
        └── Operator inspects robot
        └── Hardware E-STOP released (if engaged)
        └── Controller reset command issued
        └── Controller clears FAULT, enters IDLE
        └── Upstream nodes resume normal operation
```

> **Software Stop Clarification:** Software stop requests are processed by the Controller, which decides whether and how to stop. Only the hardware E-STOP bypasses controller logic and cuts power directly. Software components (ROS, UI) can REQUEST a stop but cannot COMMAND motors directly.

### 10.3 Mode Switch Sequence

```
Step 1: Operator decides to switch mode
        └── Updates .env file: ROBOT_MODE="SIM" → "REAL"

Step 2: System restart
        └── docker compose down
        └── docker compose up

Step 3: Launch system detects mode
        └── Reads ROBOT_MODE environment variable
        └── Selects appropriate launch profile

Step 4: Components start
        └── SIM: Gazebo (non-RT simulation) + fake controllers
        └── REAL: Hardware bridge + real controller

Step 5: Verification
        └── Joint states received
        └── System enters READY state
```

---

## 11. Observability Architecture

### 11.1 Observability Principles

- **Isolated**: All observability in separate processes/nodes
- **Optional**: System functions without any observability
- **Non-blocking**: Never delays execution path
- **Non-authoritative**: Read-only access to state

### 11.2 Observability Stack

```
┌─────────────────────────────────────────────────────────────────┐
│                    OBSERVABILITY STACK                          │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐            │
│  │   Logging   │  │   Metrics   │  │   Tracing   │            │
│  │   (Files)   │  │ (Prometheus)│  │  (Optional) │            │
│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘            │
│         │                │                │                    │
│         └────────────────┼────────────────┘                    │
│                          │                                      │
│                          ▼                                      │
│               ┌───────────────────┐                            │
│               │ Analysis Node     │                            │
│               │ (ROS 2 Subscriber)│                            │
│               └─────────┬─────────┘                            │
│                         │                                       │
│               Subscribes to (read-only):                        │
│               - /joint_states                                   │
│               - /execution_state                                │
│               - /planned_trajectory                             │
│               - /controller_feedback                            │
│                                                                 │
│  Enable:  ENABLE_ANALYSIS=true                                 │
│  Disable: ENABLE_ANALYSIS=false (default)                      │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### 11.3 Accuracy Comparison Module

```
Purpose: Compare planned vs executed trajectories
Location: Separate ROS 2 node
Authority: NONE (read-only, subscriber-only)

Inputs:
  - Planned trajectory (/planned_trajectory)
  - Actual joint states (/joint_states)

Outputs:
  - Position error over time
  - Velocity error over time
  - Statistical summaries
  - CSV/Log file export

Enabled: ENABLE_ACCURACY_ANALYSIS=true
Disabled: ENABLE_ACCURACY_ANALYSIS=false (default)
```

---

## 12. Validation Checklist

Before deployment, verify:

| Check                                                | Status |
|------------------------------------------------------|--------|
| ROS never sends PWM or direct motor commands         | ☐      |
| UI cannot execute trajectories directly              | ☐      |
| Gazebo never becomes authoritative in REAL mode      | ☐      |
| Controller owns all safety enforcement               | ☐      |
| SIM and REAL use identical ROS interfaces            | ☐      |
| Observability is optional and isolated               | ☐      |
| All config via environment variables                 | ☐      |
| Mode switching requires no code changes              | ☐      |
| E-STOP functions without ROS                         | ☐      |
| Communication loss does not cause unsafe behavior    | ☐      |
| Bridge performs schema validation only               | ☐      |
| Controller performs all semantic validation          | ☐      |
| Trajectory acknowledgment protocol implemented       | ☐      |
| Upstream nodes halt on FAULT state                   | ☐      |
| Watchdog timeout configured appropriately            | ☐      |

---

## 13. Glossary

| Term              | Definition                                                       |
|-------------------|------------------------------------------------------------------|
| Authority         | The right to make final decisions about robot motion/safety      |
| Domain            | An execution context with specific timing/safety guarantees      |
| Intent            | A requested action that requires approval before execution       |
| Truth             | The singular, authoritative source of a particular data point    |
| Parity            | Equivalent behavior/interface between SIM and REAL modes        |
| Real-Time         | Execution with guaranteed timing deadlines                       |
| Non-Real-Time     | Execution without timing guarantees (best effort)                |
| Bridge            | A translation layer between different communication protocols    |
| Watchdog          | A timer that triggers safety actions if not regularly reset      |
| Schema Validation | Checking message structure (types, fields) without semantic meaning |
| Semantic Validation | Checking physical correctness (limits, bounds, physics constraints) |
| Emulated          | Simulated behavior for testing; not authoritative or certified   |

---

*Document Version: 1.1*
*Generated for: Plug-and-Play Industrial Robot Platform*
*Root Directory: D:\robot_system*
