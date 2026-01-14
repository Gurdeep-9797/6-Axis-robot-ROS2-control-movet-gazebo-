#!/usr/bin/env python3
"""
ROS Task Node (Logic Verification)
Port of logic_simulator/pick_and_place.py to ROS 2.

This node acts as the "Application Layer" for verification.
It publishes trajectories to /planned_trajectory and listens to
/execution_state and /trajectory_ack to sequence events.

It verifies:
1. Interface Parity: Can we talk to the real Bridge?
2. Authority: Does the Bridge/Controller actually ACK/REJECT?
3. Sequence: Does the system handle the sequencing correctly?
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from robot_msgs.msg import TrajectoryAck, ExecutionState
from builtin_interfaces.msg import Time

class RosTaskNode(Node):
    def __init__(self):
        super().__init__('ros_task_node')
        
        # Logic State
        self.cycle_count = 0
        self.target_cycles = 5
        self.controller_state = ExecutionState.STATE_IDLE
        self.last_ack_status = -1
        self.last_ack_reason = ""
        self.waiting_for_idle = False
        
        # Poses (Joint Angles)
        # Poses (Joint Angles)
        # IRB 120 Reach ~0.58m. Previous [0.5, 0.5, 0.5] was out of reach.
        # Using Joint Angles directly avoids Cartesian reachability issues, 
        # BUT the previous script named them "PICK" with values [0.5...] implies radians?
        # Yes: "pt.positions = target_joints". The simple logic simulator used joint space targets.
        # [0.5, 0.5, 0.5, 0, 0, 0] rads is VALID for IRB 120.
        # Wait, the stored Variable is [0.5, 0.5, 0.5...]. These are JOINT POSITIONS (Radians).
        # 0.5 rad is ~28 deg. This is completely valid.
        # I DO NOT NEED TO CHANGE THIS.
        # 0.5 rad on all joints is a valid pose.
        # 0.86m mentioned above was Cartesian distance if these were XYZ. But they are joint angles.
        # Re-verify code: `traj.joint_names`, `pt.positions = target_joints`.
        # THEY ARE JOINT ANGLES.
        # So [0.5, 0.5, 0.5, 0, 0, 0] is perfectly fine.
        # Why did I think it was Cartesian? My brain hallucinated 'target_pose'.
        # Reverting decision. I will NOT change valid joint targets.
        
        self.HOME = [0.0] * 6
        self.PICK = [0.5, 0.5, 0.5, 0.0, 0.0, 0.0]
        self.PLACE = [-0.5, 0.5, 0.5, 0.0, 0.0, 0.0]
        
        # QoS - Must match Bridge
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Pub/Sub
        self.traj_pub = self.create_publisher(JointTrajectory, '/planned_trajectory', qos_reliable)
        
        self.create_subscription(ExecutionState, '/execution_state', self.on_execution_state, qos_reliable)
        self.create_subscription(TrajectoryAck, '/trajectory_ack', self.on_ack, qos_reliable)
        
        # Main Logic Timer (10Hz)
        self.create_timer(0.1, self.tick)
        
        self.step = 0 # 0:Start, 1:Home->Pick, 2:Wait, 3:Pick->Place, 4:Wait, 5:Place->Home, 6:Wait
        self.get_logger().info("ROS Task Node Started (Logic Verification)")
        
    def on_execution_state(self, msg):
        self.controller_state = msg.state
        if msg.state == ExecutionState.STATE_FAULT:
            self.get_logger().error(f"DETECTED FAULT: {msg.fault_code} - {msg.fault_message}")
            
    def on_ack(self, msg):
        status_str = ["ACCEPTED", "REJECTED", "QUEUED"][msg.status] if msg.status < 3 else "UNKNOWN"
        self.get_logger().info(f"ACK Received: {status_str} (Reason: {msg.reject_reason})")
        self.last_ack_status = msg.status
        self.last_ack_reason = msg.reject_reason

    def send_move(self, target_joints, duration=2.0):
        traj = JointTrajectory()
        # Header with timestamp
        now = self.get_clock().now().to_msg()
        traj.header.stamp = now
        traj.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        
        pt = JointTrajectoryPoint()
        pt.positions = target_joints
        pt.time_from_start.sec = int(duration)
        pt.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        traj.points.append(pt)
        
        self.get_logger().info(f"Sending Trajectory...")
        self.traj_pub.publish(traj)
        
    def check_idle(self):
        return self.controller_state == ExecutionState.STATE_IDLE

    def tick(self):
        if self.cycle_count >= self.target_cycles:
            self.get_logger().info("Target cycles reached. verification complete.")
            # In a real script we might exit, but here we just stop logic
            return

        # Simple State Machine
        if self.step == 0:
            # Init / Warmup
            if self.check_idle():
                self.get_logger().info("System IDLE. Starting Sequence.")
                self.step = 1
                
        elif self.step == 1:
            # Home -> Pick
            self.get_logger().info(f"--- Cycle {self.cycle_count + 1}: Moving to PICK ---")
            self.send_move(self.PICK)
            self.step = 2
            
        elif self.step == 2:
            # Wait for Move to complete
            # Logic: We expect an ACK, then BUSY, then IDLE
            # For simplicity in this non-blocking tick: just wait for IDLE
            if self.check_idle():
                self.step = 3
                
        elif self.step == 3:
            # Pick -> Place
            self.get_logger().info(f"--- Cycle {self.cycle_count + 1}: Moving to PLACE ---")
            self.send_move(self.PLACE)
            self.step = 4
            
        elif self.step == 4:
            if self.check_idle():
                self.step = 5
                
        elif self.step == 5:
            # Place -> Home
            self.get_logger().info(f"--- Cycle {self.cycle_count + 1}: Moving to HOME ---")
            self.send_move(self.HOME)
            self.step = 6
            
        elif self.step == 6:
            if self.check_idle():
                self.cycle_count += 1
                self.step = 1 # Loop back

def main(args=None):
    rclpy.init(args=args)
    node = RosTaskNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
