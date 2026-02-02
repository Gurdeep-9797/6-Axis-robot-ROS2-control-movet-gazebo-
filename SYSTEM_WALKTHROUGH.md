# SYSTEM WALKTHROUGH — Industrial Robot Platform

## Purpose

This document provides a high-level overview of the system architecture and how components interact.

---

## Terminology Lock

| Term | Definition |
|------|------------|
| **USER CONTROLLER** | Human interface (Simulator UI, Web UI, Joystick) |
| **SIMULATOR** | Gazebo. Non-real-time. Visualization + physics ONLY. |
| **RT CONTROLLER** | ESP32 firmware. Owns GPIO, PWM, PID, Safety. |
| **ROS DOMAIN** | ROS 2 + MoveIt. Planning, IK, FK. NEVER real-time. |
| **HARDWARE BRIDGE** | Transport + schema validation ONLY. |

---

## System Goals

- Provide SIM ↔ REAL parity
- Support deterministic RT Controller integration
- Enable optional observability without impacting execution
- Maintain strict authority separation

---

## Phases of Development

| Phase | Description | Status |
|-------|-------------|--------|
| **1** | Logic Simulator | ✅ Complete |
| **2** | ROS 2 Packages | ✅ Complete |
| **3** | Docker Infrastructure | ✅ Complete |
| **4** | SIM Mode Execution | ✅ Working |
| **5** | ESP32 RT Controller Firmware | ✅ Complete |
| **6** | REAL Mode Execution | 🔧 Pending HW |

---

## Data Flow (REAL Mode)

```
USER CONTROLLER (UI)
        │
        ▼
ROS 2 Intent Node
        │
        ▼
MoveIt (IK, FK, Planning)
        │
        ▼
JointTrajectory Message
        │
        ▼
HARDWARE BRIDGE (Schema Validation ONLY)
        │
        ▼
RT CONTROLLER (ESP32)
  ├── Trajectory Interpolation
  ├── PID Control Loop (50Hz+)
  ├── Safety Watchdog
  └── GPIO / PWM Output
        │
        ▼
ENCODERS (Source of Truth)
        │
        ▼
/joint_states → ROS → SIMULATOR (Visual Mirror ONLY)
```

---

## Authority Hierarchy

| Level | Component | Authority |
|-------|-----------|-----------|
| **L0** | Hardware E-STOP | Ultimate (Cuts Power) |
| **L1** | RT CONTROLLER | Full Motor Control |
| **L2** | MoveIt | Intent Generation |
| **L3** | USER CONTROLLER | Goal Selection |

**Rule:** L(N) cannot override L(N-1).

---

## Ownership Table

| Function | Owned By |
|----------|----------|
| IK / FK | MoveIt (ROS Domain) |
| PID Control | RT CONTROLLER (ESP32) |
| Safety | RT CONTROLLER (ESP32) |
| Position Truth | ENCODERS (via RT CONTROLLER) |

---

## Directory Structure

```
/src
  ├── robot_description/        # URDF (Shared)
  ├── robot_moveit_config/      # MoveIt (IK, FK)
  ├── robot_hardware_bridge/    # Bridge (Transport)
  ├── robot_gazebo/             # Simulator config
  └── robot_analysis/           # Observability
/firmware
  └── esp32_robot_controller/   # RT CONTROLLER Code
/controller
  └── hardware_map.yaml         # GPIO Mapping
```

---

## Key Constraints

| Constraint | Rationale |
|------------|-----------|
| No ROS node can move motors | RT CONTROLLER is external to ROS |
| No PID in ROS | PID must be real-time |
| Simulator non-authoritative in REAL mode | Encoders are truth |
| Hardware Bridge has no logic | Separation of concerns |

---

## Next Actions

1. Flash ESP32 with firmware
2. Wire hardware per `docs/ESP32_PCA9685_WIRING_AND_SETUP.md`
3. Run REAL mode test
4. Validate encoder feedback in ROS
