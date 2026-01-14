"""
Robot Hardware Bridge - Package Init

AUTHORITY: RELAY ONLY - NO CONTROL AUTHORITY

This package:
- Performs SCHEMA validation only (field types, array lengths)
- Relays trajectories to the RT Controller (or Gazebo in SIM mode)
- Publishes joint states from Controller to ROS
- Publishes TrajectoryAck and ExecutionState from Controller

This package does NOT:
- Perform semantic validation (joint limits, velocities, physics)
- Control motors directly
- Make safety decisions
- Own position truth

All semantic validation is the exclusive responsibility of the RT Controller.
"""
