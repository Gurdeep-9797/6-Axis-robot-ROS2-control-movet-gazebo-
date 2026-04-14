"""
Accuracy Logger Node

AUTHORITY: NONE - OBSERVATION ONLY

This node monitors executed trajectories against planned ones.
It is entirely passive and optional.
Logs both ACTUAL and PLANNED (interpolated) joint positions so
tracking error can be computed after the fact.
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
    Now includes planned position interpolation for proper error analysis.
    """

    def __init__(self):
        super().__init__('accuracy_logger')

        # Parameters
        self.declare_parameter('log_dir', '/ros_ws/logs')
        self.log_dir = self.get_parameter('log_dir').get_parameter_value().string_value

        # State
        self.recording = False
        self.current_trajectory = None
        self.trajectory_start_time = None
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

        self.get_logger().info('Accuracy Logger initialized (Observation Only — planned+actual tracking)')

    def trajectory_callback(self, msg):
        """New trajectory received — store it for interpolation."""
        self.current_trajectory = msg
        self.trajectory_start_time = time.monotonic()
        self.get_logger().info(f'Analysis: Received trajectory with {len(msg.points)} points')

    def execution_callback(self, msg):
        """Monitor execution state."""
        if msg.state == ExecutionState.STATE_EXECUTING and not self.recording:
            self._start_logging()
        elif msg.state != ExecutionState.STATE_EXECUTING and self.recording:
            self._stop_logging()

    def joint_state_callback(self, msg):
        """Log actual state alongside interpolated planned state."""
        if not self.recording or not self.csv_writer:
            return

        timestamp = time.time()
        actual = list(msg.position)

        # Compute planned (interpolated) position at this moment
        planned = self._interpolate_planned()

        # Compute per-joint error (degrees) if planned is available
        if planned:
            errors = [abs(math.degrees(actual[i] - planned[i])) for i in range(min(len(actual), len(planned)))]
        else:
            errors = [0.0] * len(actual)

        row = [timestamp] + actual + (planned if planned else [0.0]*6) + errors
        self.csv_writer.writerow(row)

    def _interpolate_planned(self):
        """
        Interpolate the expected joint position at the current moment
        from the stored trajectory, using linear interpolation between
        consecutive waypoints.
        """
        if not self.current_trajectory or not self.trajectory_start_time:
            return None

        pts = self.current_trajectory.points
        if not pts:
            return None

        elapsed = time.monotonic() - self.trajectory_start_time

        # Convert each point's time_from_start to float seconds
        def pt_time(pt):
            return pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9

        # Before the first point
        if elapsed <= pt_time(pts[0]):
            return list(pts[0].positions[:6])

        # Past the final point — hold final position
        if elapsed >= pt_time(pts[-1]):
            return list(pts[-1].positions[:6])

        # Find the segment we're in
        for i in range(len(pts) - 1):
            t0 = pt_time(pts[i])
            t1 = pt_time(pts[i + 1])
            if t0 <= elapsed < t1:
                ratio = (elapsed - t0) / max(0.001, t1 - t0)
                q0 = pts[i].positions
                q1 = pts[i + 1].positions
                return [
                    q0[j] + ratio * (q1[j] - q0[j])
                    for j in range(min(6, len(q0), len(q1)))
                ]

        # Fallback
        return list(pts[-1].positions[:6])

    def _start_logging(self):
        """Start logging session."""
        self.recording = True
        filename = f"execution_log_{int(time.time())}.csv"
        filepath = os.path.join(self.log_dir, filename)

        self.log_file = open(filepath, 'w', newline='')
        self.csv_writer = csv.writer(self.log_file)

        # Header: actual + planned + error columns
        header = (
            ['timestamp']
            + [f'actual_j{i+1}' for i in range(6)]
            + [f'planned_j{i+1}' for i in range(6)]
            + [f'error_j{i+1}_deg' for i in range(6)]
        )
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
