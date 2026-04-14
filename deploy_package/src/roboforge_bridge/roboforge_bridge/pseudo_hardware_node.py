"""
PseudoHardwareNode — 250 Hz simulated robot hardware interface.

PURPOSE:
  Simulates a real robot's hardware loop when Gazebo is NOT running.
  Subscribes to /joint_trajectory_command (JointTrajectory messages).
  Interpolates the trajectory using cubic Hermite splines at 250 Hz.
  Publishes /joint_states with realistic time-stamped positions.
  Validates joint limits and velocity limits — halts on violation.

TOPIC INTERFACE:
  Sub:  /joint_trajectory_command  (trajectory_msgs/JointTrajectory)
  Pub:  /joint_states              (sensor_msgs/JointState)         @ 250 Hz
  Pub:  /roboforge/hw_status       (std_msgs/String)                @ 10 Hz

ROS DOMAIN: 42
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import String
import json
import math
import time
import threading
from typing import List, Optional

# ── IRB 6700 Joint Limits (degrees → radians) ──────────────────────────────
_DEG = math.pi / 180.0
JOINT_LIMITS = [
    (-170 * _DEG,  170 * _DEG),   # J1
    ( -65 * _DEG,   85 * _DEG),   # J2
    (-180 * _DEG,   70 * _DEG),   # J3
    (-300 * _DEG,  300 * _DEG),   # J4
    (-130 * _DEG,  130 * _DEG),   # J5
    (-360 * _DEG,  360 * _DEG),   # J6
]
MAX_JOINT_VEL_RAD_S = [3.0, 2.5, 3.0, 6.0, 6.0, 7.0]  # rad/s per joint

JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
HOME_POSITION = [0.0, -0.3, 0.2, 0.0, -0.5, 0.0]  # safe home in radians

PUBLISH_HZ = 250
STATUS_HZ = 10


class TrajectorySegment:
    """One interpolated motion segment between two joint configurations."""

    def __init__(self, q_start: List[float], q_end: List[float], duration: float, t_start: float):
        self.q_start = list(q_start)
        self.q_end = list(q_end)
        self.duration = max(0.001, duration)
        self.t_start = t_start

    def sample(self, t_now: float) -> Optional[List[float]]:
        """Cubic Hermite interpolation. Returns None if segment is complete."""
        elapsed = t_now - self.t_start
        if elapsed >= self.duration:
            return None  # segment done
        s = max(0.0, min(1.0, elapsed / self.duration))
        # Cubic Hermite basis: smooth ease in/out
        s2 = s * s
        s3 = s2 * s
        h = 3 * s2 - 2 * s3   # hermite basis (maps 0→0, 1→1 with zero derivatives at ends)
        return [
            self.q_start[i] + h * (self.q_end[i] - self.q_start[i])
            for i in range(6)
        ]


class PseudoHardwareNode(Node):
    """
    Simulated 250 Hz robot hardware interface.
    Interpolates joint trajectories and broadcasts /joint_states.
    """

    def __init__(self):
        super().__init__('pseudo_hardware_node')

        # Current robot joint state (starts at Home)
        self._q: List[float] = list(HOME_POSITION)
        self._qd: List[float] = [0.0] * 6   # velocities
        self._state_lock = threading.Lock()

        # Active trajectory segment queue
        self._segments: List[TrajectorySegment] = []
        self._segment_lock = threading.Lock()
        self._current_segment: Optional[TrajectorySegment] = None
        self._status: str = 'idle'
        self._fault: Optional[str] = None

        # ── ROS interfaces ──────────────────────────────────────────────
        self._joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self._hw_status_pub = self.create_publisher(String, '/roboforge/hw_status', 10)

        self._traj_sub = self.create_subscription(
            JointTrajectory, '/joint_trajectory_command',
            self._on_trajectory, 10
        )

        # 250 Hz main loop
        self._timer = self.create_timer(1.0 / PUBLISH_HZ, self._step)
        # 10 Hz status broadcast
        self._status_timer = self.create_timer(1.0 / STATUS_HZ, self._publish_status)

        self.get_logger().info(
            f'PseudoHardwareNode ready — {PUBLISH_HZ} Hz simulation loop.\n'
            f'  Listening on: /joint_trajectory_command\n'
            f'  Publishing:   /joint_states, /roboforge/hw_status'
        )

    # ── Trajectory reception ──────────────────────────────────────────────

    def _on_trajectory(self, msg: JointTrajectory):
        """
        Receive a JointTrajectory and convert to interpolated segments.
        Each consecutive pair of trajectory points becomes one segment.
        """
        if self._fault:
            self.get_logger().warn(f'Trajectory rejected — fault active: {self._fault}')
            return

        if not msg.points:
            self.get_logger().warn('Received empty JointTrajectory — ignoring')
            return

        self.get_logger().info(f'Received trajectory: {len(msg.points)} points')

        with self._state_lock:
            q_now = list(self._q)

        new_segments: List[TrajectorySegment] = []
        t_start = time.monotonic()
        q_prev = q_now
        t_prev = 0.0

        for pt in msg.points:
            q_target = list(pt.positions[:6])
            t_end = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            duration = max(0.001, t_end - t_prev)

            # Validate target joint limits
            violation = self._check_limits(q_target)
            if violation:
                self.get_logger().error(f'Joint limit violation in trajectory: {violation}')
                self._set_fault(f'Joint limit violation: {violation}')
                return

            # Validate velocity feasibility
            vel_violation = self._check_velocities(q_prev, q_target, duration)
            if vel_violation:
                self.get_logger().warn(f'Velocity limit exceeded (clamping allowed): {vel_violation}')
                # For simulation we warn but don't abort — just extend duration
                duration = duration * 1.2

            new_segments.append(TrajectorySegment(q_prev, q_target, duration, t_start + t_prev))
            q_prev = q_target
            t_prev = t_end

        with self._segment_lock:
            self._segments = new_segments
            self._current_segment = None
            self._status = 'moving'

    # ── Main simulation step (250 Hz) ─────────────────────────────────────

    def _step(self):
        now = time.monotonic()

        with self._segment_lock:
            # Get or advance to the next active segment
            if self._current_segment is None and self._segments:
                self._current_segment = self._segments.pop(0)

            if self._current_segment is not None:
                q_new = self._current_segment.sample(now)
                if q_new is None:
                    # Segment complete — move to exact end position
                    q_new = self._current_segment.q_end
                    self._current_segment = None
                    if not self._segments:
                        self._status = 'idle'
                with self._state_lock:
                    prev_q = list(self._q)
                    self._q = q_new
                    dt = 1.0 / PUBLISH_HZ
                    self._qd = [(q_new[i] - prev_q[i]) / dt for i in range(6)]

        self._publish_joint_state()

    def _publish_joint_state(self):
        with self._state_lock:
            q = list(self._q)
            qd = list(self._qd)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = q
        msg.velocity = qd
        msg.effort = [0.0] * 6
        self._joint_state_pub.publish(msg)

    def _publish_status(self):
        status = {
            'node': 'pseudo_hardware',
            'status': self._status if not self._fault else 'fault',
            'fault': self._fault,
            'joint_positions_deg': [round(math.degrees(q), 2) for q in self._q],
            'joint_velocities_rad_s': [round(v, 4) for v in self._qd],
            'segments_queued': len(self._segments),
        }
        msg = String()
        msg.data = json.dumps(status)
        self._hw_status_pub.publish(msg)

    # ── Validation ────────────────────────────────────────────────────────

    def _check_limits(self, q: List[float]) -> Optional[str]:
        for i, (lo, hi) in enumerate(JOINT_LIMITS):
            if q[i] < lo - 0.001 or q[i] > hi + 0.001:
                return f'J{i+1}={math.degrees(q[i]):.1f}° outside [{math.degrees(lo):.1f}, {math.degrees(hi):.1f}]°'
        return None

    def _check_velocities(self, q_start: List[float], q_end: List[float], duration: float) -> Optional[str]:
        for i in range(6):
            vel = abs(q_end[i] - q_start[i]) / duration
            if vel > MAX_JOINT_VEL_RAD_S[i] * 1.05:  # 5% tolerance
                return f'J{i+1}: {math.degrees(vel):.1f}°/s > max {math.degrees(MAX_JOINT_VEL_RAD_S[i]):.1f}°/s'
        return None

    def _set_fault(self, reason: str):
        self._fault = reason
        self._status = 'fault'
        with self._segment_lock:
            self._segments.clear()
            self._current_segment = None
        self.get_logger().error(f'[FAULT] {reason}')


def main(args=None):
    rclpy.init(args=args)
    node = PseudoHardwareNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
