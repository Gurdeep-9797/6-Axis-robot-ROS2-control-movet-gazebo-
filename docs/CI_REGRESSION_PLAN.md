# CI/CD Regression Testing Plan

## Overview
Automated testing pipeline for the industrial robot control stack.

---

## CI Stages

### Stage 1: Logic Simulator (No ROS)
```bash
cd logic_simulator
python simulator_main.py --cycles 10 --fault-inject
```
**Validates:**
- Authority checks
- Fault injection & recovery
- Message flow

### Stage 2: Gazebo + ABB URDF
```bash
./scripts/start_sim.ps1
sleep 30
docker exec robot_bridge python3 /ros_ws/scripts/automated_logic_test.py
```
**Validates:**
- Frame consistency
- Joint limits
- Gazebo isolation

### Stage 3: MoveIt Integration
```bash
# After Stage 2 passes
ros2 action send_goal /robot_arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{...}"
```
**Validates:**
- IK solvability
- Planning rejection tests
- Adapter flow

---

## Mandatory Assertions

| Assertion | How Tested |
|-----------|------------|
| Bridge never publishes fake states | Check timestamps monotonic |
| Gazebo never publishes `/joint_states` | Subscribe, verify no messages |
| MoveIt never executes | Action only, no direct topic |
| Controller always ACKs/rejects | Timeout = fail |

---

## Test Matrix

| Test | Logic Sim | Gazebo | MoveIt | Real |
|------|-----------|--------|--------|------|
| Authority | ✅ | ✅ | ✅ | ✅ |
| Schema | ✅ | ✅ | ✅ | ✅ |
| Limits | ✅ | ⚠️ (Warn) | ✅ | ✅ (Reject) |
| Faults | ✅ | ✅ | ✅ | ✅ |
| Cycles | ✅ | ✅ | ✅ | ✅ |

---

## Failure Handling

If ANY test fails:
1. **STOP pipeline**
2. **Log root cause**
3. **Block merge/deploy**
4. **Notify maintainers**

No exceptions.
