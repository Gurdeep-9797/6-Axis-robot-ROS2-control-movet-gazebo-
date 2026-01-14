# SYSTEM WALKTHROUGH
## Industrial Robot Control Stack — End-to-End Architecture

---

## 1. System Intent

A **plug-and-play industrial robot software stack** designed to:
- Preserve real-time safety boundaries
- Enable SIM ↔ REAL parity
- Enforce strict authority separation
- Support deterministic controller integration
- Enable CI-grade regression testing

---

## 2. Phase Summary

| Phase | Description | Outcome |
|-------|-------------|---------|
| **1** | Logic Simulator | Authority & flow validated in pure Python |
| **2** | Gazebo Integration | Physics added, no authority leakage |
| **3** | ABB IRB 120 URDF | Real industrial kinematics |
| **4** | MoveIt Adapter | Planner isolated from execution |
| **5** | *(Next)* Real Controller | Hardware execution |

---

## 3. Final Execution Flow

```
┌─────────────────────────────────────────────────────────────┐
│                      APPLICATION LAYER                       │
│  ┌─────────────┐                                            │
│  │   MoveIt    │  (Planner, Action Client)                  │
│  │  move_group │  NON-REAL-TIME, REQUEST ONLY               │
│  └──────┬──────┘                                            │
│         │ FollowJointTrajectory Action                      │
│         ▼                                                   │
│  ┌─────────────────┐                                        │
│  │ MoveIt Adapter  │  (Action → Topic Translator)           │
│  │ moveit_adapter  │  NO AUTHORITY                          │
│  └────────┬────────┘                                        │
│           │ /planned_trajectory                             │
├───────────┼─────────────────────────────────────────────────┤
│           │              INTERFACE LAYER                    │
│           ▼                                                 │
│  ┌─────────────────┐                                        │
│  │ Hardware Bridge │  RELAY ONLY, SCHEMA VALIDATION         │
│  │   bridge_node   │  NO SEMANTIC AUTHORITY                 │
│  └────────┬────────┘                                        │
│           │                                                 │
├───────────┼─────────────────────────────────────────────────┤
│           │              EXECUTION LAYER                    │
│           ▼                                                 │
│  ┌─────────────────┐    ┌─────────────────┐                │
│  │  SIM Backend    │ OR │  REAL Backend   │                │
│  │ (Gazebo/FAKE)   │    │ (RT Controller) │                │
│  └────────┬────────┘    └────────┬────────┘                │
│           │                      │                          │
│           ▼                      ▼                          │
│  ┌─────────────────────────────────────────┐               │
│  │            /joint_states                 │               │
│  │     (Bridge Republishes - Single Truth)  │               │
│  └─────────────────────────────────────────┘               │
└─────────────────────────────────────────────────────────────┘
```

---

## 4. Authority Hierarchy

| Component | Authority Level | Can Move Motors? |
|-----------|-----------------|------------------|
| RT Controller | **ABSOLUTE** | ✅ YES (Only) |
| Hardware Bridge | RELAY | ❌ NO |
| Gazebo | EMULATED | ❌ NO (Simulation Only) |
| MoveIt | REQUEST | ❌ NO |
| UI/Analysis | OBSERVE | ❌ NO |

---

## 5. Key Guarantees

| Guarantee | How Enforced |
|-----------|--------------|
| No ROS node can move motors | Controller is external to ROS |
| No simulator can fake truth | Bridge republishes, Gazebo isolated |
| No UI can bypass authority | UI subscribes only, no publishers |
| No planning error can execute | Adapter validates ACK before feedback |
| Faults propagate upstream | ExecutionState.FAULT halts all planning |

---

## 6. File Structure

```
robot_system/
├── docker/                    # Container definitions
├── src/
│   ├── robot_description/     # URDF (ABB IRB 120)
│   ├── robot_moveit_config/   # MoveIt configuration
│   ├── robot_hardware_bridge/ # Bridge + Adapter
│   ├── robot_gazebo/          # Gazebo world + controllers
│   ├── robot_analysis/        # Passive observers
│   └── robot_msgs/            # TrajectoryAck, ExecutionState
├── config/                    # Global parameters
├── scripts/                   # Helper scripts
└── logic_simulator/           # Golden reference (Pure Python)
```

---

## 7. Validation Status

| Test | Status |
|------|--------|
| Logic Simulator Authority | ✅ Validated |
| Gazebo Topic Isolation | ✅ Verified |
| ABB IRB 120 Kinematics | ✅ Implemented |
| MoveIt Adapter Flow | ✅ Implemented |
| End-to-End Execution | ⏳ Blocked (Docker Env) |

---

## 8. Next Steps

1. **Restore Docker Environment**
2. **Run `verify_phase4.ps1`**
3. **Phase 5: Real Controller Integration**
