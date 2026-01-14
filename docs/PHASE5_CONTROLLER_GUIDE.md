# Phase 5: Real Controller Integration Guide

## Controller Requirements

### MUST Implement
- Own clock (independent of ROS)
- Local joint limit enforcement
- Trajectory rejection logic
- ROS disconnect survival
- Authoritative encoder publishing

### MUST NOT
- Depend on ROS timing
- Accept raw motor commands
- Trust upstream safety decisions

---

## Required Interfaces

### Inputs
| Topic | Type | Description |
|-------|------|-------------|
| `/planned_trajectory` | `trajectory_msgs/JointTrajectory` | Trajectory commands |
| Controller-specific | Start/Stop/Reset | Mode control |

### Outputs
| Topic | Type | Description |
|-------|------|-------------|
| `/joint_states` | `sensor_msgs/JointState` | Encoder truth |
| `/execution_state` | `robot_msgs/ExecutionState` | State machine |
| `/trajectory_ack` | `robot_msgs/TrajectoryAck` | Accept/Reject |

---

## Watchdog Requirements

| Watchdog | Timeout |
|----------|---------|
| Control Loop | 1 kHz |
| Communication | 100 ms |
| Internal | 10 ms |

---

## Success Criterion

The real controller must behave **identically** to Gazebo backend, except for:
- Physics (real dynamics)
- Timing (real-time guarantees)

If behavior differs in authority or message flow → **REGRESSION**.

---

## Hardware Acceptance Checklist

### Before Power
- [ ] E-STOP loop verified with multimeter
- [ ] Motors mechanically unloaded
- [ ] Controller watchdog enabled
- [ ] Joint limits flashed into controller
- [ ] ROS NOT running

### First Power
- [ ] Controller boots without ROS
- [ ] Motors remain disabled
- [ ] Encoder values sane (no NaN, within limits)

### ROS Connect
- [ ] Bridge connects successfully
- [ ] `/execution_state == IDLE`
- [ ] No trajectory accepted automatically

### First Motion
- [ ] Single joint jog only
- [ ] Low velocity (< 10% max)
- [ ] Immediate E-STOP test (must halt < 100ms)

---

## Operator Safety SOP

### Golden Rules
1. Never stand inside robot envelope
2. Never bypass E-STOP
3. Never run REAL mode without supervisor
4. Never trust SIM results as safety proof

### Emergency Protocol
1. Hit E-STOP
2. Power down drives
3. Notify supervisor
4. Inspect mechanically
5. Reset controller
6. Resume only after checklist
