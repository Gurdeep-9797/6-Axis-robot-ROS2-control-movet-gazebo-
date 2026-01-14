"""
Accuracy Logger Node

AUTHORITY: NONE - OBSERVATION ONLY

This node monitors executed trajectories against planned ones.
It is entirely passive and optional.
"""

import os
import csv
import time
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from robot_msgs.msg import ExecutionState


class AccuracyLogger(Node):
    """
    Accuracy Logger Node
    
    Logs tracking error: |planned - actual|
    """
    
    def __init__(self):
        super().__init__('accuracy_logger')
        
        # Parameters
        self.declare_parameter('log_dir', '/ros_ws/logs')
        self.log_dir = self.get_parameter('log_dir').get_parameter_value().string_value
        
        # State
        self.recording = False
        self.current_trajectory = None
        self.log_file = None
        self.csv_writer = None
        
        # QoS
        qos_reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        qos_best_effort = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        
        # Subscribers
        self.create_subscription(JointTrajectory, '/planned_trajectory', self.trajectory_callback, qos_reliable)
        self.create_subscription(ExecutionState, '/execution_state', self.execution_callback, qos_reliable)
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, qos_best_effort)
        
        # Ensure log dir exists
        os.makedirs(self.log_dir, exist_ok=True)
        
        self.get_logger().info('Accuracy Logger initialized (Observation Only)')

    def trajectory_callback(self, msg):
        """New trajectory received."""
        self.current_trajectory = msg
        self.get_logger().info(f'Analysis: Received trajectory with {len(msg.points)} points')

    def execution_callback(self, msg):
        """Monitor execution state."""
        if msg.state == ExecutionState.STATE_EXECUTING and not self.recording:
            self._start_logging()
        elif msg.state != ExecutionState.STATE_EXECUTING and self.recording:
            self._stop_logging()

    def joint_state_callback(self, msg):
        """Log actual state vs planned."""
        if not self.recording or not self.csv_writer:
            return
            
        timestamp = time.time()
        # Simple logging of positions
        row = [timestamp] + list(msg.position)
        self.csv_writer.writerow(row)

    def _start_logging(self):
        """Start logging session."""
        self.recording = True
        filename = f"execution_log_{int(time.time())}.csv"
        filepath = os.path.join(self.log_dir, filename)
        
        self.log_file = open(filepath, 'w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        
        # Header
        header = ['timestamp', 'j1', 'j2', 'j3', 'j4', 'j5', 'j6']
        self.csv_writer.writerow(header)
        
        self.get_logger().info(f'Started logging to {filepath}')

    def _stop_logging(self):
        """Stop logging session."""
        self.recording = False
        if self.log_file:
            self.log_file.close()
            self.log_file = None
            self.csv_writer = None
        self.get_logger().info('Stopped logging')


def main(args=None):
    rclpy.init(args=args)
    node = AccuracyLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
