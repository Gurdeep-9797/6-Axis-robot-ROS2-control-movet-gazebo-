# Logic Simulator

A lightweight, pure-Python harness to validate architecture logic before ROS 2 integration.

## Purpose
Validates the **Authority Hierarchy** and **Message Flow** of the Antigravity system.
- Proves `HardwareBridge` is relay-only.
- Proves `Controller` enforces limits.
- Proves `PickAndPlace` task waits for execution.

## Components
- `bus.py`: ROS 2 Pub/Sub stub.
- `fake_controller.py`: Emulates RT Controller semantic validation and state machine.
- `hardware_bridge.py`: Mirrors `bridge_node.py` using schema-only validation.

## How to Run
```bash
python simulator_main.py
```

## What This Validates
- [x] Schema validation rejects bad messages at the Bridge.
- [x] Semantic validation rejects limits at the Controller.
- [x] TrajectoryAck protocol propagates back to Task.
- [x] ExecutionState halts Task on FAULT.

## What This Does NOT Simulate
- Physics / Gravity / Collision.
- Real-time OS jitter.
- UDP/TCP network stack (simulated via function calls).
