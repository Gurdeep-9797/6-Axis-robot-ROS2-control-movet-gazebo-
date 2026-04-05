#!/usr/bin/env python3
"""
gazebo_error_node.py
────────────────────
Subscribes to actual encoder-sourced /joint_states AND Gazebo shadow
/gazebo/joint_states, computes tracking error, and publishes metrics.
Only active in Live mode (checks /roboforge/mode before computing).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float64MultiArray
import numpy as np
import json
import math


class GazeboErrorNode(Node):

    # Threshold: if any joint exceeds this error, log a WARNING
    ERROR_WARN_THRESHOLD_DEG = 2.0    # 2 degrees
    ERROR_CRITICAL_THRESHOLD_DEG = 5.0  # 5 degrees → pause program, alert user

    def __init__(self):
        super().__init__('gazebo_error_node')

        # Latest messages from each source
        self._actual_state:  JointState = None
        self._ideal_state:   JointState = None
        self._current_mode = 'simulate'   # Updated by /roboforge/mode topic

        # ── Subscriptions ─────────────────────────────────────────────────
        self.create_subscription(
            JointState, '/joint_states',
            self._on_actual_state, 10
        )
        self.create_subscription(
            JointState, '/gazebo/joint_states',
            self._on_ideal_state, 10
        )
        self.create_subscription(
            String, '/roboforge/mode',
            lambda msg: setattr(self, '_current_mode', msg.data), 10
        )

        # ── Publishers ─────────────────────────────────────────────────────
        self._error_pub = self.create_publisher(
            String, '/roboforge/tracking_error', 10
        )
        self._alert_pub = self.create_publisher(
            String, '/roboforge/tracking_alert', 10
        )

        # Compute and publish at 50 Hz
        self.create_timer(0.02, self._compute_and_publish)

        self.get_logger().info('GazeboErrorNode started — comparing actual vs ideal')

    def _on_actual_state(self, msg: JointState):
        self._actual_state = msg

    def _on_ideal_state(self, msg: JointState):
        self._ideal_state = msg

    def _compute_and_publish(self):
        # Only compare in live mode when both sources are available
        if self._current_mode != 'live':
            return
        if self._actual_state is None or self._ideal_state is None:
            return

        actual = np.array(self._actual_state.position)
        ideal  = np.array(self._ideal_state.position)

        if len(actual) != len(ideal):
            self.get_logger().warn('Mismatched joint count between actual and ideal states')
            return

        # ── Per-joint error ─────────────────────────────────────────────
        error_rad = actual - ideal
        error_deg = np.degrees(error_rad)
        error_pct = np.abs(error_deg) / 360.0 * 100.0   # % of full rotation

        # ── TCP position error (via FK on both) ─────────────────────────
        # NOTE: This uses the analytical kinematics, not MoveIt service,
        # to avoid latency. The kinematics module is shared from bridge_node.
        # tcp_error_mm = self._kinematics_fk_difference(actual, ideal)
        # Simplified: use joint-space error as proxy for TCP error.
        tcp_error_mm = float(np.linalg.norm(error_rad) * 1000)   # rough approximation in mm

        # ── Max error joint ─────────────────────────────────────────────
        max_error_joint_idx = int(np.argmax(np.abs(error_deg)))
        max_error_deg = float(np.abs(error_deg[max_error_joint_idx]))

        # ── Build and publish error report ──────────────────────────────
        report = {
            'type': 'tracking_error',
            'per_joint_error_deg': error_deg.tolist(),
            'per_joint_error_pct': error_pct.tolist(),
            'tcp_error_mm': tcp_error_mm,
            'max_error_joint': f'joint_{max_error_joint_idx + 1}',
            'max_error_deg': max_error_deg,
            'timestamp_ns': int(
                self._actual_state.header.stamp.sec * 1e9 +
                self._actual_state.header.stamp.nanosec
            )
        }
        msg = String(); msg.data = json.dumps(report)
        self._error_pub.publish(msg)

        # ── Alert checks ─────────────────────────────────────────────────
        if max_error_deg > self.ERROR_CRITICAL_THRESHOLD_DEG:
            alert = {
                'level': 'CRITICAL',
                'message': (
                    f'Joint {max_error_joint_idx+1} tracking error {max_error_deg:.2f}° '
                    f'exceeds {self.ERROR_CRITICAL_THRESHOLD_DEG}° threshold. '
                    f'Halting program execution.'
                ),
                'action': 'halt'   # bridge_node reads this and sends STOP
            }
            a_msg = String(); a_msg.data = json.dumps(alert)
            self._alert_pub.publish(a_msg)
            self.get_logger().error(alert['message'])

        elif max_error_deg > self.ERROR_WARN_THRESHOLD_DEG:
            alert = {
                'level': 'WARNING',
                'message': (
                    f'Joint {max_error_joint_idx+1} tracking error {max_error_deg:.2f}° '
                    f'approaching threshold.'
                ),
                'action': 'warn'
            }
            a_msg = String(); a_msg.data = json.dumps(alert)
            self._alert_pub.publish(a_msg)


def main():
    rclpy.init()
    node = GazeboErrorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
