# SYSTEM LOGIC AND FLOW — DEFINITIVE ARCHITECTURE

> **SYSTEM DIRECTIVE OVERRIDE**: This document supersedes all previous architectural definitions.

## 1. Terminology Lock (Non-Negotiable)

| Term | Definition |
|------|------------|
| **USER CONTROLLER** | The Human Interface. (Simulator UI, Web UI, Joystick). |
| **RT CONTROLLER** | The **ESP32** Firmware. Owns Motor Control, PID, Safety, GPIO. |
| **SIMULATOR** | **Gazebo**. Physics & Visualization ONLY. Non-Authoritative in Real Mode. |
| **HARDWARE BRIDGE** | Transport Layer & Schema Validator. NO Control Logic. |
| **ROS DOMAIN** | ROS 2 + MoveIt. Non-Real-Time Intent Generation. |

---

## 2. Operational Modes

### MODE 1: REAL ROBOT + SIMULATOR MIRROR
**Purpose**: Execute on physical hardware; mirror motion in Sim for verification.

**Flow**:
1. **USER CONTROLLER** (Pose Goal) → **ROS 2**
2. **MoveIt** (IK/Planning) → `JointTrajectory` Message
3. **HARDWARE BRIDGE** (Schema Valid) → Serial/TCP
4. **RT CONTROLLER** (ESP32)
   - Interpolation
   - PID Loop (50Hz+)
   - Safety Watchdog
   - GPIO/PWM Generation
5. **REAL ENCODERS** → Position & Velocity
6. **RT CONTROLLER** → `JointState` → **HARDWARE BRIDGE**
7. **ROS /joint_states**
8. **SIMULATOR (Gazebo)** (Visualization Mirror ONLY)

**Constraints**:
- Simulator MUST NOT command motors.
- MoveIt MUST NOT run PID.
- Latency > 100ms is acceptable in ROS (Intent), but NOT in RT Loop.

### MODE 2: PURE SIMULATION
**Purpose**: Algorithm development without hardware.

**Flow**:
1. **USER CONTROLLER** → **MoveIt**
2. **MoveIt** → `JointTrajectory`
3. **SIMULATOR (Gazebo)** (Physics Engine)
   - Simulates Gravity/Inertia
   - Simulates Motor Response
4. **SIMULATED JOINT STATES** → **ROS /joint_states**

---

## 3. Authority Hierarchy

| Level | Component | Authority Scope |
|-------|-----------|-----------------|
| **L0 (Ultimate)** | Hardware E-STOP | Physics Power Cut (Bypasses Software) |
| **L1 (Primary)** | RT CONTROLLER | Motor Commutation, Limits, Safety, Velocity Clamping |
| **L2 (Intent)** | ROS 2 / MoveIt | Trajectory Generation, Collision Avoidance |
| **L3 (Request)** | USER CONTROLLER | High-level Goal Selection |

**Rule**: L(N) cannot override L(N-1). ROS cannot force RT Controller to violate safety.

---

## 4. Hardware Bridge Specification

**Role**: STRICT Domain Separation.
**Functions**:
- Serialization (ROS Msg ↔ Binary Protocol)
- Schema Validation (Array lengths, Data types)
- Transport Management (Reconnect logic)

**Prohibited**:
- PID Control
- Trajectory Smoothing
- Limit Enforcement (Semantic)
- "Smart" Logic

---

## 5. Directory Structure & Ownership

```text
/src
  /robot_description       --> Shared (URDF)
  /robot_moveit_config     --> Shared (MoveIt)
  /robot_hardware_bridge   --> Bridge Logic (Python Node + C++ Stubs)
  /robot_gazebo            --> Simulator Config (Sim Mode execution)
  /robot_analysis          --> Accuracy/Logging Nodes
/firmware
  /esp32_robot_controller  --> RT CONTROLLER Code (C++)
/controller
  hardware_map.yaml        --> Pinout & Electromechanical Config
```

---

## 6. Feedback & Observability

**Accuracy Node**:
- Subscribes: `/planned_trajectory` (Intent)
- Subscribes: `/joint_states` (Actual)
- Logic: Computes RMSE (Root Mean Square Error) between plan and execution.
- Output: Logs pass/fail metrics.

**Visual Mirror**:
- In REAL mode, Gazebo is slaved to `/joint_states`.
- It visualizes what the *Encoders* report, not what MoveIt *planned*.
- Divergence between ghost (plan) and mesh (real) indicates tracking error.
