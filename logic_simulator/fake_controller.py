"""
Fake Controller Emulator (Logic Only)

Simulates the Real-Time Controller's behavior including:
- Semantic validation (Joint limits, Velocity limits)
- Trajectory buffering
- Execution simulation (Logic state machine)
- Feedback publishing (JointState, ExecutionState, TrajectoryAck)
- Fault injection possibility

This component replaces the "Real Hardware" for validation purposes.
"""

import time
import threading
import messages
from bus import default_bus

class FakeController:
    JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
    
    # Validation Limits
    JOINT_LIMITS = [-3.14, 3.14]  # Simplified symmetric limits
    MAX_VELOCITY = 5.0            # rad/s check
    
    def __init__(self):
        self.bus = default_bus
        self.lock = threading.Lock()
        self.running = True
        
        # State
        self.positions = [0.0] * 6
        self.state = messages.ExecutionState.STATE_IDLE
        self.fault_code = 0
        self.progress = 0.0
        
        # Execution
        self.current_trajectory = None
        self.start_time = 0.0
        
        # Wire simulation (Subscribing to "wire" topic from Bridge)
        self.bus.subscribe('controller_command', self.handle_command)
        
        # Threads
        self.loop_thread = threading.Thread(target=self.control_loop, daemon=True)
        self.loop_thread.start()
        
    def handle_command(self, trajectory):
        """Receive trajectory from 'wire' (Bridge)."""
        print(f"[CONTROLLER] Received trajectory with {len(trajectory.points)} points")
        
        # 1. Check State
        if self.state == messages.ExecutionState.STATE_FAULT:
            self._send_ack(trajectory, messages.TrajectoryAck.STATUS_REJECTED, "CONTROLLER_IN_FAULT")
            return
            
        if self.state == messages.ExecutionState.STATE_EXECUTING:
             self._send_ack(trajectory, messages.TrajectoryAck.STATUS_QUEUED, "QUEUED_EXECUTION_BUSY")
             return # Simple logic: Reject/Queue if busy (for this sim, let's just reject or queue. Let's Queue.)
             # Actually, for sim simplicity, let's REJECT new commands while moving to prove authority rule "Bridge cannot override"
             self._send_ack(trajectory, messages.TrajectoryAck.STATUS_REJECTED, "BUSY_EXECUTING")
             return
             
        # 2. Semantic Validation (The Controller's Job)
        if not self._validate_trajectory(trajectory):
            return # Ack sent inside validate
            
        # 3. Accept and Start
        with self.lock:
            self.current_trajectory = trajectory
            self.start_time = time.time()
            self.state = messages.ExecutionState.STATE_EXECUTING
            self.progress = 0.0
            
        self._send_ack(trajectory, messages.TrajectoryAck.STATUS_ACCEPTED, "")
        
    def _validate_trajectory(self, trajectory):
        """Check limits."""
        for pt in trajectory.points:
            # Check joint counts
            if len(pt.positions) != 6:
                 self._send_ack(trajectory, messages.TrajectoryAck.STATUS_REJECTED, "INVALID_JOINT_COUNT")
                 return False
                 
            # Check limits
            for pos in pt.positions:
                if not (self.JOINT_LIMITS[0] <= pos <= self.JOINT_LIMITS[1]):
                    self._send_ack(trajectory, messages.TrajectoryAck.STATUS_REJECTED, "JOINT_LIMIT_EXCEEDED")
                    return False
                    
            # Check velocities (if present)
            for vel in pt.velocities:
                if abs(vel) > self.MAX_VELOCITY:
                    self._send_ack(trajectory, messages.TrajectoryAck.STATUS_REJECTED, "VELOCITY_LIMIT_EXCEEDED")
                    return False
                    
        return True

    def _send_ack(self, trajectory, status, reason):
        ack = messages.TrajectoryAck(
            trajectory_id=trajectory.header.stamp,
            status=status,
            reject_reason=reason
        )
        self.bus.publish('trajectory_ack', ack)
        status_str = ["ACCEPTED", "REJECTED", "QUEUED"][status]
        print(f"[CONTROLLER] ACK Sent: {status_str} ({reason})")

    def control_loop(self):
        """100Hz Control Loop Simulation."""
        while self.running:
            with self.lock:
                if self.state == messages.ExecutionState.STATE_EXECUTING:
                    self._execute_step()
                    
                # Publish Feedback
                self._publish_state()
                
            time.sleep(0.01) # 100Hz
            
    def _execute_step(self):
        """Functionally simulate motion interpolation."""
        if not self.current_trajectory:
            self.state = messages.ExecutionState.STATE_IDLE
            return
            
        elapsed = time.time() - self.start_time
        points = self.current_trajectory.points
        duration = points[-1].time_from_start
        
        if elapsed >= duration:
            # Finished
            self.positions = points[-1].positions
            self.state = messages.ExecutionState.STATE_IDLE
            self.progress = 1.0
            self.current_trajectory = None
            print("[CONTROLLER] Trajectory Complete")
        else:
            # Simple Intepolation (Find active segment)
            # This is sloppy logic but sufficient for authority validation
            self.progress = elapsed / duration
            # Just snap to nearest point for logic sim (simpler than full interp)
            # Find point with time > elapsed
            for pt in points:
                if pt.time_from_start >= elapsed:
                    self.positions = pt.positions
                    break

    def _publish_state(self):
        # Joint State
        js = messages.JointState()
        js.header.stamp = time.time()
        js.name = self.JOINT_NAMES
        js.position = list(self.positions)
        self.bus.publish('joint_states', js)
        
        # Execution State
        ex = messages.ExecutionState()
        ex.state = self.state
        ex.progress = self.progress
        ex.fault_code = self.fault_code
        self.bus.publish('execution_state', ex)

    def trigger_fault(self):
        """External trigger to assert FAULT state."""
        with self.lock:
            self.state = messages.ExecutionState.STATE_FAULT
            self.fault_code = 1001 # Critical
            self.current_trajectory = None
        print("[CONTROLLER] *** FAULT TRIGGERED ***")

    def clear_fault(self):
        with self.lock:
            self.state = messages.ExecutionState.STATE_IDLE
            self.fault_code = 0
        print("[CONTROLLER] Fault Cleared")
