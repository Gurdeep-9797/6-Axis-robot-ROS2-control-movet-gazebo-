"""
Message Definitions (Stub for ROS 2 interfaces)

Simplified Python classes mirroring the structure of:
- trajectory_msgs/JointTrajectory
- sensor_msgs/JointState
- robot_msgs/TrajectoryAck
- robot_msgs/ExecutionState
"""

import time
from dataclasses import dataclass, field
from typing import List

# === Primitives ===

@dataclass
class Header:
    stamp: float = 0.0
    frame_id: str = ""

@dataclass
class Time:
    sec: int = 0
    nanosec: int = 0
    
    @staticmethod
    def from_float(timestamp: float):
        t = Time()
        t.sec = int(timestamp)
        t.nanosec = int((timestamp - int(timestamp)) * 1e9)
        return t

# === Trajectory Messages ===

@dataclass
class JointTrajectoryPoint:
    positions: List[float] = field(default_factory=list)
    velocities: List[float] = field(default_factory=list)
    time_from_start: float = 0.0  # Seconds (simplified from duration)

@dataclass
class JointTrajectory:
    header: Header = field(default_factory=Header)
    joint_names: List[str] = field(default_factory=list)
    points: List[JointTrajectoryPoint] = field(default_factory=list)

# === Sensor Messages ===

@dataclass
class JointState:
    header: Header = field(default_factory=Header)
    name: List[str] = field(default_factory=list)
    position: List[float] = field(default_factory=list)
    velocity: List[float] = field(default_factory=list)
    effort: List[float] = field(default_factory=list)

# === Custom Messages (robot_msgs) ===

@dataclass
class TrajectoryAck:
    # Constants
    STATUS_ACCEPTED = 0
    STATUS_REJECTED = 1
    STATUS_QUEUED = 2
    
    trajectory_id: float = 0.0  # Using timestamp as ID
    status: int = 0
    reject_reason: str = ""

@dataclass
class ExecutionState:
    # Constants
    STATE_IDLE = 0
    STATE_EXECUTING = 1
    STATE_PAUSED = 2
    STATE_FAULT = 3
    STATE_COMM_LOSS = 4
    
    state: int = STATE_IDLE
    progress: float = 0.0
    fault_code: int = 0
    fault_message: str = ""
