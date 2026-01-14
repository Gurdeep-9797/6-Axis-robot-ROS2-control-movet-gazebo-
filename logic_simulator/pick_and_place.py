"""
Pick and Place Task Logic

Generates trajectories and sequences the robot operation.
Listens to ExecutionState to decide when to send next move.
Retries on rejection.
"""

import time
import threading
import messages
from bus import default_bus

class PickAndPlaceTask:
    def __init__(self):
        self.bus = default_bus
        self.running = True
        
        # Logic State
        self.state = "INIT"
        self.cycle_count = 0
        self.target_cycles = 10
        self.controller_state = messages.ExecutionState.STATE_IDLE
        
        # Poses (Joint Angles)
        self.HOME = [0.0] * 6
        self.PICK = [0.5, 0.5, 0.5, 0.0, 0.0, 0.0]
        self.PLACE = [-0.5, 0.5, 0.5, 0.0, 0.0, 0.0]
        
        # Subscribe to Feedback
        self.bus.subscribe('execution_state', self.on_execution_state)
        self.bus.subscribe('trajectory_ack', self.on_ack)
        
        self.thread = threading.Thread(target=self.task_loop, daemon=True)
        self.thread.start()
        
    def on_execution_state(self, msg):
        self.controller_state = msg.state
        if msg.state == messages.ExecutionState.STATE_FAULT:
            print(f"[TASK] DETECTED FAULT: {msg.fault_code}")
            
    def on_ack(self, msg):
        if msg.status == messages.TrajectoryAck.STATUS_REJECTED:
             print(f"[TASK] TRAJECTORY REJECTED: {msg.reject_reason}")
             
    def send_move(self, target_joints, duration=2.0):
        """Create and publish a trajectory."""
        traj = messages.JointTrajectory()
        traj.header.stamp = time.time()
        traj.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        
        pt = messages.JointTrajectoryPoint()
        pt.positions = list(target_joints)
        pt.time_from_start = duration
        traj.points.append(pt)
        
        print(f"[TASK] Sending Move...")
        self.bus.publish('planned_trajectory', traj)
        
    def wait_for_idle(self):
        """Block until controller is Idle."""
        while self.controller_state != messages.ExecutionState.STATE_IDLE:
            if self.controller_state == messages.ExecutionState.STATE_FAULT:
                return False
            time.sleep(0.1)
        return True

    def task_loop(self):
        """Main Sequence."""
        time.sleep(1) # Let system warmup
        
        print("[TASK] Starting Pick & Place Sequence")
        
        while self.running and self.cycle_count < self.target_cycles:
            print(f"--- Cycle {self.cycle_count + 1} ---")
            
            # 1. Home -> Pick
            self.send_move(self.PICK)
            if not self.wait_for_idle(): break
            time.sleep(0.5) # Simulate gripper action
            
            # 2. Pick -> Place
            self.send_move(self.PLACE)
            if not self.wait_for_idle(): break
            time.sleep(0.5) # Simulate gripper action
            
            # 3. Place -> Home
            self.send_move(self.HOME)
            if not self.wait_for_idle(): break
            
            self.cycle_count += 1
            
        print("[TASK] Sequence Complete or Aborted")
        self.running = False
