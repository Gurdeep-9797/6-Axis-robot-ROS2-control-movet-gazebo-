"""
safety_watchdog.py — Real-time safety monitoring for physical robot operation.

PURPOSE:
  Monitors encoder feedback, motor currents, and trajectory tracking errors
  to ensure safe operation in LIVE hardware mode. Triggers E-STOP when
  safety thresholds are exceeded.

SAFETY THRESHOLDS:
  - Tracking error > 2° → WARNING (logged, UI notified)
  - Tracking error > 5° → CRITICAL (E-STOP, trajectory halted)
  - Motor current > max_current_a → OVERCURRENT (E-STOP)
  - Encoder loss > 100ms → COMM_LOSS (E-STOP)
  - Joint position limit violation → LIMIT_EXCEEDED (E-STOP)

PUBLICATION:
  - /roboforge/safety_status (std_msgs/String) — status updates
  - /roboforge/tracking_alert (std_msgs/String) — alert messages for bridge
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import String
import json
import time
import threading
from typing import List, Optional

# Safety thresholds
TRACKING_WARNING_DEG = 2.0
TRACKING_CRITICAL_DEG = 5.0
COMM_LOSS_TIMEOUT_MS = 100
MAX_CURRENT_A = 5.0

JOINT_LIMITS = [
    (-3.14159, 3.14159),   # J1 ±180°
    (-1.5708, 1.5708),     # J2 ±90°
    (-2.3562, 2.3562),     # J3 ±135°
    (-3.14159, 3.14159),   # J4 ±180°
    (-2.0944, 2.0944),     # J5 ±120°
    (-6.2832, 6.2832),     # J6 ±360°
]


class SafetyWatchdog:
    """Per-joint safety monitor."""

    def __init__(self, joint_idx: int):
        self.joint_idx = joint_idx
        self.last_encoder_time = 0.0
        self.last_cmd_time = 0.0
        self.planned_pos = 0.0
        self.actual_pos = 0.0
        self.actual_vel = 0.0
        self.current_a = 0.0
        self.state = "OK"  # OK, WARNING, CRITICAL, E_STOP
        self.fault_code = 0
        self.fault_message = ""

    def update_encoder(self, pos: float, vel: float):
        """Update actual position from encoder feedback."""
        self.actual_pos = pos
        self.actual_vel = vel
        self.last_encoder_time = time.monotonic()
        self._check_limits()

    def update_planned(self, pos: float):
        """Update planned position from trajectory."""
        self.planned_pos = pos
        self.last_cmd_time = time.monotonic()
        self._check_tracking_error()

    def update_current(self, current: float):
        """Update motor current reading."""
        self.current_a = current
        if abs(current) > MAX_CURRENT_A:
            self._trigger_fault(
                code=2001,
                message=f"J{self.joint_idx+1}: Overcurrent {abs(current):.1f}A > {MAX_CURRENT_A}A"
            )

    def _check_limits(self):
        """Check joint position limits."""
        lo, hi = JOINT_LIMITS[self.joint_idx]
        if self.actual_pos < lo - 0.01 or self.actual_pos > hi + 0.01:
            self._trigger_fault(
                code=1001,
                message=f"J{self.joint_idx+1}: Position limit violated ({self.actual_pos:.3f} rad)"
            )

    def _check_tracking_error(self):
        """Check tracking error between planned and actual."""
        error_deg = abs(self.planned_pos - self.actual_pos) * 180 / 3.14159

        if error_deg > TRACKING_CRITICAL_DEG:
            self._trigger_fault(
                code=3001,
                message=f"J{self.joint_idx+1}: Tracking error {error_deg:.1f}° > {TRACKING_CRITICAL_DEG}° (CRITICAL)"
            )
        elif error_deg > TRACKING_WARNING_DEG:
            self.state = "WARNING"
            self.fault_code = 3000
            self.fault_message = f"J{self.joint_idx+1}: Tracking error {error_deg:.1f}° > {TRACKING_WARNING_DEG}°"

    def _trigger_fault(self, code: int, message: str):
        """Trigger fault state."""
        self.state = "E_STOP"
        self.fault_code = code
        self.fault_message = message

    def check_comm_loss(self) -> bool:
        """Check for encoder communication loss."""
        if self.last_encoder_time > 0:
            elapsed_ms = (time.monotonic() - self.last_encoder_time) * 1000
            if elapsed_ms > COMM_LOSS_TIMEOUT_MS:
                self._trigger_fault(
                    code=4001,
                    message=f"J{self.joint_idx+1}: Encoder comm loss ({elapsed_ms:.0f}ms)"
                )
                return True
        return False

    def clear(self):
        """Clear fault state (after manual reset)."""
        if self.fault_code < 2000:  # Don't clear overcurrent automatically
            self.state = "OK"
            self.fault_code = 0
            self.fault_message = ""

    def to_dict(self) -> dict:
        return {
            "joint": self.joint_idx + 1,
            "state": self.state,
            "fault_code": self.fault_code,
            "fault_message": self.fault_message,
            "tracking_error_deg": round(abs(self.planned_pos - self.actual_pos) * 180 / 3.14159, 2),
            "current_a": round(self.current_a, 2),
        }


class SafetyWatchdogNode(Node):
    """ROS 2 node for safety monitoring."""

    def __init__(self):
        super().__init__('safety_watchdog_node')
        self._watchdogs = [SafetyWatchdog(i) for i in range(6)]
        self._is_estopped = False
        self._lock = threading.Lock()

        # Subscriptions
        self._joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self._on_joint_states, 10)
        self._trajectory_sub = self.create_subscription(
            JointTrajectory, '/joint_trajectory_command', self._on_trajectory, 10)

        # Publishers
        self._status_pub = self.create_publisher(String, '/roboforge/safety_status', 10)
        self._alert_pub = self.create_publisher(String, '/roboforge/tracking_alert', 10)

        # Timers
        self._check_timer = self.create_timer(0.01, self._safety_check)  # 100Hz
        self._status_timer = self.create_timer(0.1, self._publish_status)  # 10Hz

        self.get_logger().info('SafetyWatchdog node started — monitoring 6 joints')

    def _on_joint_states(self, msg: JointState):
        """Update actual positions from encoder feedback."""
        positions = list(msg.position)
        velocities = list(msg.velocity) if len(msg.velocity) >= 6 else [0.0] * 6

        with self._lock:
            for i in range(min(6, len(positions))):
                self._watchdogs[i].update_encoder(positions[i], velocities[i])

    def _on_trajectory(self, msg: JointTrajectory):
        """Update planned positions from incoming trajectory."""
        if not msg.points:
            return

        # Use the last point's positions as the current planned target
        last_point = msg.points[-1]
        positions = list(last_point.positions[:6])

        with self._lock:
            for i in range(min(6, len(positions))):
                self._watchdogs[i].update_planned(positions[i])

    def _safety_check(self):
        """Run safety checks at 100Hz."""
        with self._lock:
            any_critical = False

            for wd in self._watchdogs:
                wd.check_comm_loss()
                if wd.state == "E_STOP":
                    any_critical = True

            if any_critical and not self._is_estopped:
                self._is_estopped = True
                self._trigger_estop()
            elif not any_critical and self._is_estopped:
                self._is_estopped = False
                self.get_logger().info('E-STOP cleared — all joints OK')

    def _trigger_estop(self):
        """Trigger emergency stop."""
        faults = [wd.to_dict() for wd in self._watchdogs if wd.state == "E_STOP"]
        alert = {
            "action": "halt",
            "timestamp": time.monotonic(),
            "faults": faults,
            "message": f"E-STOP triggered: {faults[0]['fault_message'] if faults else 'Unknown'}"
        }
        msg = String()
        msg.data = json.dumps(alert)
        self._alert_pub.publish(msg)
        self.get_logger().error(f"E-STOP: {alert['message']}")

    def _publish_status(self):
        """Publish safety status at 10Hz."""
        status = {
            "timestamp": time.monotonic(),
            "estopped": self._is_estopped,
            "joints": [wd.to_dict() for wd in self._watchdogs],
        }
        msg = String()
        msg.data = json.dumps(status)
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyWatchdogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
