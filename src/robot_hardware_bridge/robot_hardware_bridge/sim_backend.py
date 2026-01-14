"""
Simulation Backend for Hardware Bridge

AUTHORITY: EMULATED (No Real Authority)

This backend emulates controller behavior for SIM mode / FAKE controller.
It is NON-REAL-TIME and NON-CERTIFIED.

SIM mode does not constitute safety validation.
All safety-critical validation must be performed with real hardware.

This backend:
- Receives trajectories from the bridge
- Simulates trajectory execution (or forwards to Gazebo)
- Generates simulated joint states
- Generates TrajectoryAck responses
- Generates ExecutionState updates

NOTE: In full Gazebo integration, this would interface with
gazebo_ros2_control. This implementation provides a standalone
simulation for testing without Gazebo.
"""

import time
import threading
import math
from typing import Optional, List

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from robot_msgs.msg import TrajectoryAck, ExecutionState


class SimulationBackend:
    """
    Simulation Backend - Emulated Controller (Non-RT, Non-Certified)
    
    Provides a fake controller for SIM mode testing.
    Does NOT provide real-time guarantees or safety certification.
    """
    
    JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
    
    # Simulated joint limits (for emulated semantic validation)
    JOINT_LIMITS = {
        'joint_1': (-3.14159, 3.14159),
        'joint_2': (-1.5708, 1.5708),
        'joint_3': (-2.3562, 2.3562),
        'joint_4': (-1.5708, 1.5708),
        'joint_5': (-3.14159, 3.14159),
        'joint_6': (-1.5708, 1.5708),
    }
    

    def __init__(self, node, controller_type='FAKE'):
        self.node = node
        self.logger = node.get_logger()
        self.controller_type = controller_type
        
        # Current state
        self.current_positions = [0.0] * 6
        self.current_velocities = [0.0] * 6
        self.execution_state = ExecutionState.STATE_IDLE
        self.execution_progress = 0.0
        self.fault_code = 0
        self.fault_message = ''
        
        # Trajectory execution
        self.trajectory_queue: List[JointTrajectory] = []
        self.current_trajectory: Optional[JointTrajectory] = None
        self.trajectory_start_time = 0.0
        self.current_point_index = 0
        
        # Pending acks
        self.pending_ack: Optional[TrajectoryAck] = None
        
        # Thread control
        self.running = False
        self.execution_thread: Optional[threading.Thread] = None
        self.lock = threading.Lock()
        
        # Timestamp counter (simulated monotonic, microseconds since start)
        self.start_time = time.monotonic()
        
        # GAZEBO MODE: Subscribe to Gazebo joint states and Publish to Controller
        if self.controller_type == 'GAZEBO':
            from rclpy.qos import QoSProfile, ReliabilityPolicy
            # We need to publish to the controller in Gazebo
            self.gazebo_pub = self.node.create_publisher(
                JointTrajectory,
                '/robot_arm_controller/joint_trajectory',
                10
            )
            # We subscribe to Gazebo's output (assumed mapped to /gazebo/joint_states or similar if needed, 
            # but usually it's /joint_states. If we map Gazebo out to /gazebo/joint_states, we listen there.)
            # For now, let's assume we read from /joint_states provided by Gazebo. 
            # BUT: Bridge Node publishes to /joint_states. This creates a loop if we read /joint_states.
            # CRITICAL: We must read from a source independent of our own output.
            # We will assume Gazebo publishes to /gazebo/joint_states
            self.gazebo_sub = self.node.create_subscription(
                JointState,
                '/gazebo/joint_states',
                self._gazebo_state_callback,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            )

    def start(self):
        """Start the simulation backend."""
        self.logger.info(f'Simulation Backend starting ({self.controller_type} Mode)')
        self.running = True
        
        if self.controller_type == 'FAKE':
            self.execution_thread = threading.Thread(target=self._execution_loop, daemon=True)
            self.execution_thread.start()
        # In GAZEBO mode, we rely on callbacks, no internal loop needed for physics
    
    def stop(self):
        """Stop the simulation backend."""
        self.logger.info('Simulation Backend stopping')
        self.running = False
        if self.execution_thread:
            self.execution_thread.join(timeout=1.0)
            
    def _gazebo_state_callback(self, msg):
        """Update internal state from Gazebo."""
        with self.lock:
            # Map names if needed (assuming 1:1 for now)
            try:
                for i, name in enumerate(self.JOINT_NAMES):
                    if name in msg.name:
                        idx = msg.name.index(name)
                        self.current_positions[i] = msg.position[idx]
                        if msg.velocity:
                            self.current_velocities[i] = msg.velocity[idx]
            except Exception as e:
                self.logger.warn(f'Error mapping Gazebo state: {e}')
                
            # Naive IDLE detection: if zero velocity
            if all(abs(v) < 0.001 for v in self.current_velocities):
                if self.execution_state == ExecutionState.STATE_EXECUTING:
                     # Check if we were moving. This is tricky. 
                     # Better to trust the trajectory completion time or feedback from Controller?
                     # Gazebo controller doesn't give "Done" easily without action client.
                     # For now, we remain simple. Logic simulator requires "ExecutionState" logic.
                     # We can guess based on time or velocity.
                     pass 

    def send_trajectory(self, trajectory: JointTrajectory):
        """
        Receive trajectory from bridge.
        """
        self.logger.info(f'SIM Backend received trajectory with {len(trajectory.points)} points')
        
        # Emulated semantic validation (Test Only) since Gazebo might not report errors cleanly
        for point in trajectory.points:
            for i, pos in enumerate(point.positions):
                joint_name = self.JOINT_NAMES[i]
                min_limit, max_limit = self.JOINT_LIMITS[joint_name]
                if pos < min_limit or pos > max_limit:
                    self.logger.warn(f'SIM: Joint {joint_name} position {pos} exceeds limits [{min_limit}, {max_limit}]')
        
        if self.controller_type == 'GAZEBO':
            # Relay to Gazebo
            self.gazebo_pub.publish(trajectory)
            with self.lock:
                 # We assume accepted if we sent it
                 self._set_pending_ack(trajectory, TrajectoryAck.STATUS_ACCEPTED, '')
                 self.execution_state = ExecutionState.STATE_EXECUTING
                 self.trajectory_start_time = time.monotonic()
                 
                 # Estimate duration for IDLE switch (simple logic for now)
                 if trajectory.points:
                     duration = trajectory.points[-1].time_from_start.sec + trajectory.points[-1].time_from_start.nanosec * 1e-9
                     # Start a timer to reset state? Or just use velocity?
                     # Let's use a timer thread for simplicity to reset IDLE
                     threading.Timer(duration + 0.5, self._reset_idle).start()
            return

        with self.lock:
            # FAKE Mode Logic...
            if self.execution_state == ExecutionState.STATE_EXECUTING:
                self.trajectory_queue.append(trajectory)
                self._set_pending_ack(trajectory, TrajectoryAck.STATUS_QUEUED, '')
            else:
                self.current_trajectory = trajectory
                self.trajectory_start_time = time.monotonic()
                self.current_point_index = 0
                self.execution_state = ExecutionState.STATE_EXECUTING
                self.execution_progress = 0.0
                self._set_pending_ack(trajectory, TrajectoryAck.STATUS_ACCEPTED, '')

    def _reset_idle(self):
        with self.lock:
            self.execution_state = ExecutionState.STATE_IDLE
            self.execution_progress = 1.0

    def _set_pending_ack(self, trajectory: JointTrajectory, status: int, reason: str):
        """Set pending ack for next publish cycle."""
        ack = TrajectoryAck()
        ack.trajectory_id = trajectory.header.stamp
        ack.status = status
        ack.reject_reason = reason
        self.pending_ack = ack
    
    def _execution_loop(self):
        """Simulated execution loop (FAKE Mode only)."""
        loop_rate = 0.001  # 1 kHz simulated
        
        while self.running:
            with self.lock:
                if self.current_trajectory is not None:
                    self._step_trajectory()
            
            time.sleep(loop_rate)
    
    def _step_trajectory(self):
        """Advance one step in trajectory execution."""
        if self.current_trajectory is None:
            return
        
        points = self.current_trajectory.points
        if self.current_point_index >= len(points):
            # Trajectory complete
            self.logger.info('SIM: Trajectory execution complete')
            self.execution_state = ExecutionState.STATE_IDLE
            self.execution_progress = 1.0
            self.current_trajectory = None
            
            # Check queue
            if self.trajectory_queue:
                next_traj = self.trajectory_queue.pop(0)
                self.current_trajectory = next_traj
                self.trajectory_start_time = time.monotonic()
                self.current_point_index = 0
                self.execution_state = ExecutionState.STATE_EXECUTING
                self.execution_progress = 0.0
            return
        
        # Get current target point
        target_point = points[self.current_point_index]
        elapsed = time.monotonic() - self.trajectory_start_time
        target_time = target_point.time_from_start.sec + target_point.time_from_start.nanosec * 1e-9
        
        if elapsed >= target_time:
            # Reached point - update positions
            for i in range(6):
                self.current_positions[i] = target_point.positions[i]
                if target_point.velocities:
                    self.current_velocities[i] = target_point.velocities[i] if i < len(target_point.velocities) else 0.0
                else:
                    self.current_velocities[i] = 0.0
            
            self.current_point_index += 1
            self.execution_progress = self.current_point_index / len(points)
        else:
            # Interpolate between previous and target
            if self.current_point_index > 0:
                prev_point = points[self.current_point_index - 1]
                prev_time = prev_point.time_from_start.sec + prev_point.time_from_start.nanosec * 1e-9
                t = (elapsed - prev_time) / (target_time - prev_time) if target_time > prev_time else 1.0
                t = max(0.0, min(1.0, t))
                
                for i in range(6):
                    prev_pos = prev_point.positions[i]
                    target_pos = target_point.positions[i]
                    self.current_positions[i] = prev_pos + t * (target_pos - prev_pos)
    
    def get_joint_state(self) -> JointState:
        """Get current joint state."""
        msg = JointState()
        
        # Monotonic timestamp (microseconds since start)
        elapsed_us = int((time.monotonic() - self.start_time) * 1e6)
        msg.header.stamp.sec = elapsed_us // 1000000
        msg.header.stamp.nanosec = (elapsed_us % 1000000) * 1000
        
        with self.lock:
            msg.name = list(self.JOINT_NAMES)
            msg.position = list(self.current_positions)
            msg.velocity = list(self.current_velocities)
            msg.effort = [0.0] * 6  # Not simulated
        
        return msg
    
    def get_execution_state(self) -> ExecutionState:
        """Get current execution state."""
        msg = ExecutionState()
        
        with self.lock:
            msg.state = self.execution_state
            msg.progress = self.execution_progress
            msg.fault_code = self.fault_code
            msg.fault_message = self.fault_message
        
        return msg
    
    def get_trajectory_ack(self) -> Optional[TrajectoryAck]:
        """Get pending trajectory ack (clears after read)."""
        with self.lock:
            ack = self.pending_ack
            self.pending_ack = None
        return ack

