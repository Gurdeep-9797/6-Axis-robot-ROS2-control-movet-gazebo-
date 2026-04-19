#!/usr/bin/env python3
"""
motor_controller_node.py
────────────────────────
Subscribes to /joint_trajectory_command (trajectory from MoveIt/compiler).
At each control tick (250 Hz), computes the required torque/current for each
joint and sends PWM commands to the motor driver hardware.

Supports:
  DC motors  — PI current control, output is duty cycle (0–100%)
  BLDC motors — Field-Oriented Control (FOC), output is Id/Iq commands
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
import numpy as np
import serial
import threading
import time
import math
import yaml
import struct
import json

class TrajectorySegment:
    def __init__(self, q_start, q_end, duration, t_start):
        self.q_start = list(q_start)
        self.q_end = list(q_end)
        self.duration = max(0.001, duration)
        self.t_start = t_start

    def sample(self, t_now):
        elapsed = t_now - self.t_start
        if elapsed >= self.duration: return None
        s = max(0.0, min(1.0, elapsed / self.duration))
        h = 3 * (s*s) - 2 * (s*s*s)
        return [self.q_start[i] + h * (self.q_end[i] - self.q_start[i]) for i in range(len(self.q_start))]



class PIController:
    """Generic discrete-time PI controller."""
    def __init__(self, kp: float, ki: float, dt: float,
                 out_min: float = -1.0, out_max: float = 1.0):
        self.kp = kp; self.ki = ki; self.dt = dt
        self.out_min = out_min; self.out_max = out_max
        self._integral = 0.0

    def reset(self):
        self._integral = 0.0

    def update(self, error: float) -> float:
        self._integral = np.clip(
            self._integral + error * self.dt,
            self.out_min / self.ki if self.ki != 0 else -1e9,
            self.out_max / self.ki if self.ki != 0 else  1e9
        )
        return np.clip(self.kp * error + self.ki * self._integral,
                       self.out_min, self.out_max)


class DCJointController:
    """
    DC Motor current control.
    Input:  desired joint position (rad) from trajectory
    Output: duty cycle (−1.0 to +1.0) → sent to H-bridge PWM
    Control law: Position error → velocity setpoint → current setpoint → PWM
    """
    def __init__(self, cfg: dict, dt: float):
        self.name = cfg['joint_name']
        self.max_current = cfg['dc_max_current_a']
        # Outer loop: position → velocity
        self.pos_ctrl = PIController(
            kp=cfg.get('dc_pos_kp', 10.0), ki=0.0, dt=dt,
            out_min=-cfg['dc_max_current_a'], out_max=cfg['dc_max_current_a']
        )
        # Inner loop: current PI
        self.cur_ctrl = PIController(
            kp=cfg['dc_kp'], ki=cfg['dc_ki'], dt=dt,
            out_min=-1.0, out_max=1.0
        )
        self._measured_current = 0.0

    def update(self, pos_setpoint: float, pos_measured: float,
               current_measured: float) -> float:
        """Returns duty cycle -1.0 to +1.0."""
        pos_error = pos_setpoint - pos_measured
        current_setpoint = self.pos_ctrl.update(pos_error)
        current_error    = current_setpoint - current_measured
        return self.cur_ctrl.update(current_error)


class BLDCJointController:
    """
    BLDC motor Field-Oriented Control (FOC).
    FOC decouples torque and flux by transforming phase currents to
    a rotating reference frame (d-q frame) aligned with rotor flux.

    Control structure:
      Position error
        → velocity setpoint (outer PI)
        → Iq setpoint (torque-producing current)
      Id setpoint = 0 (zero d-axis current = maximum torque per ampere)
      Clarke transform: Ia, Ib, Ic → Iα, Iβ
      Park transform:   Iα, Iβ, θ_electrical → Id, Iq
      PI on Id, Iq → Vd, Vq
      Inverse Park:     Vd, Vq, θ → Vα, Vβ
      SVPWM:            Vα, Vβ → 3-phase duty cycles [0..1]
    """
    def __init__(self, cfg: dict, dt: float):
        self.name = cfg['joint_name']
        self.pole_pairs    = cfg['bldc_pole_pairs']
        self.encoder_cpr   = cfg['encoder_cpr']
        self.flux_linkage  = cfg['bldc_flux_linkage_wb']
        self.R_phase       = cfg['bldc_phase_resistance_ohm']

        # Outer position → velocity PI
        self.pos_ctrl = PIController(
            kp=cfg.get('bldc_pos_kp', 8.0), ki=0.0, dt=dt,
            out_min=-20.0, out_max=20.0
        )
        # Velocity → Iq setpoint PI
        self.vel_ctrl = PIController(
            kp=cfg.get('bldc_vel_kp', 0.5), ki=cfg.get('bldc_vel_ki', 5.0),
            dt=dt, out_min=-10.0, out_max=10.0
        )
        # Inner current PI (d-axis — keeps Id = 0)
        self.id_ctrl = PIController(
            kp=cfg['bldc_foc_kp_d'], ki=cfg['bldc_foc_ki_d'], dt=dt,
            out_min=-1.0, out_max=1.0
        )
        # Inner current PI (q-axis — produces torque)
        self.iq_ctrl = PIController(
            kp=cfg['bldc_foc_kp_q'], ki=cfg['bldc_foc_ki_q'], dt=dt,
            out_min=-1.0, out_max=1.0
        )
        self._prev_encoder_count = 0

    def mechanical_to_electrical_angle(self, encoder_count: int) -> float:
        """Convert encoder count to electrical angle θ_e (radians)."""
        mechanical_angle = (encoder_count % self.encoder_cpr) / self.encoder_cpr * 2 * math.pi
        return (mechanical_angle * self.pole_pairs) % (2 * math.pi)

    def update(self, pos_setpoint: float, pos_measured: float,
               vel_measured: float, encoder_count: int,
               ia: float, ib: float, ic: float) -> tuple:
        """
        Returns: (duty_a, duty_b, duty_c) — 3-phase PWM duty cycles [0, 1].
        These are sent directly to the BLDC gate driver.
        """
        # ── Outer loops ─────────────────────────────────────────────────
        vel_setpoint = self.pos_ctrl.update(pos_setpoint - pos_measured)
        iq_setpoint  = self.vel_ctrl.update(vel_setpoint - vel_measured)
        id_setpoint  = 0.0  # Zero d-axis current for maximum torque per ampere (MTPA)

        # ── Electrical angle ─────────────────────────────────────────────
        theta_e = self.mechanical_to_electrical_angle(encoder_count)

        # ── Clarke transform (abc → αβ) ──────────────────────────────────
        # Assumes balanced 3-phase: Ia + Ib + Ic = 0
        i_alpha = ia
        i_beta  = (ia + 2 * ib) / math.sqrt(3)

        # ── Park transform (αβ → dq) ─────────────────────────────────────
        cos_e = math.cos(theta_e)
        sin_e = math.sin(theta_e)
        id_measured = i_alpha * cos_e + i_beta * sin_e
        iq_measured = -i_alpha * sin_e + i_beta * cos_e

        # ── Current PI controllers ───────────────────────────────────────
        vd = self.id_ctrl.update(id_setpoint - id_measured)
        vq = self.iq_ctrl.update(iq_setpoint - iq_measured)

        # ── Decoupling (feed-forward, improves dynamic response) ─────────
        # ω_e = electrical angular velocity
        # vd_ff = -ω_e * Lq * iq,  vq_ff = ω_e * (Ld * id + λ)
        # Simplified here (full decoupling requires motor inductance params):
        # vd += vd_ff; vq += vq_ff

        # ── Inverse Park transform (dq → αβ) ────────────────────────────
        v_alpha = vd * cos_e - vq * sin_e
        v_beta  = vd * sin_e + vq * cos_e

        # ── Space Vector PWM (SVPWM) ─────────────────────────────────────
        duty_a, duty_b, duty_c = self._svpwm(v_alpha, v_beta)
        return duty_a, duty_b, duty_c

    @staticmethod
    def _svpwm(v_alpha: float, v_beta: float) -> tuple:
        """
        Space Vector Pulse Width Modulation.
        Maps αβ voltage vector to 3-phase duty cycles.
        Input: v_alpha, v_beta ∈ [-1, 1] (normalized to bus voltage)
        Output: duty_a, duty_b, duty_c ∈ [0, 1]
        """
        # Sector determination
        v1 = v_beta
        v2 = (math.sqrt(3) * v_alpha - v_beta) / 2
        v3 = (-math.sqrt(3) * v_alpha - v_beta) / 2

        sector = 0
        if v1 > 0: sector |= 1
        if v2 > 0: sector |= 2
        if v3 > 0: sector |= 4

        sector_map = {6:1, 2:2, 3:3, 1:4, 5:5, 4:6}
        sector = sector_map.get(sector, 1)

        # Duty cycles per sector (standard SVPWM lookup)
        # (abbreviated — full 6-sector implementation follows standard textbook)
        magnitude = math.sqrt(v_alpha**2 + v_beta**2)
        angle = math.atan2(v_beta, v_alpha)

        # Clamp magnitude to unit circle
        magnitude = min(magnitude, 1.0 / math.sqrt(3))

        # Simple sinusoidal approximation for now:
        duty_a = 0.5 + magnitude * math.cos(angle)
        duty_b = 0.5 + magnitude * math.cos(angle - 2*math.pi/3)
        duty_c = 0.5 + magnitude * math.cos(angle + 2*math.pi/3)

        # Clamp to [0, 1]
        duty_a = max(0.0, min(1.0, duty_a))
        duty_b = max(0.0, min(1.0, duty_b))
        duty_c = max(0.0, min(1.0, duty_c))
        return duty_a, duty_b, duty_c


class MotorControllerNode(Node):

    def __init__(self):
        super().__init__('motor_controller_node')

        self.declare_parameter('config_file', 'config/robot_config.yaml')
        self.declare_parameter('motor_serial_port', '/dev/ttyUSB1')
        self.declare_parameter('control_rate_hz', 250)

        cfg = self._load_config()
        self._dt = 1.0 / self.get_parameter('control_rate_hz').value

        # ── Build per-joint controllers ──────────────────────────────────
        self._controllers = []
        for joint_cfg in cfg['joints']:
            if joint_cfg.get('motor_type', 'DC') == 'DC':
                self._controllers.append(DCJointController(joint_cfg, self._dt))
            elif joint_cfg.get('motor_type') == 'BLDC':
                self._controllers.append(BLDCJointController(joint_cfg, self._dt))

        # ── State ────────────────────────────────────────────────────────
        self._current_joint_pos = [0.0] * len(self._controllers)
        self._current_joint_vel = [0.0] * len(self._controllers)
        self._setpoint_pos      = [0.0] * len(self._controllers)
        self._encoder_counts    = [0]   * len(self._controllers)
        
        self._segments = []
        self._current_segment = None
        self._state_lock = threading.Lock()

        # ── Subscriptions ────────────────────────────────────────────────
        self.create_subscription(
            JointState, '/joint_states', self._on_joint_states, 10
        )
        self.create_subscription(
            JointTrajectory, '/joint_trajectory_command',
            self._on_trajectory_command, 10
        )
        self.create_subscription(
            String, '/roboforge/motor_config',
            self._on_motor_config, 10
        )
        self._telemetry_pub = self.create_publisher(
            Float64MultiArray, '/roboforge/motor_telemetry', 10
        )

        # ── Control timer ────────────────────────────────────────────────
        self.create_timer(self._dt, self._control_tick)

        # ── Motor hardware serial ────────────────────────────────────────
        port = self.get_parameter('motor_serial_port').value
        try:
            self._motor_serial = serial.Serial(port, 1000000, timeout=0.001)
            self.get_logger().info(f'Motor serial opened: {port}')
        except serial.SerialException as e:
            self._motor_serial = None
            self.get_logger().error(f'Motor serial failed: {e}. PWM commands disabled.')

    def _on_joint_states(self, msg: JointState):
        with self._state_lock:
            # Need to match names, simplified for 6-axis in order
            self._current_joint_pos = list(msg.position)
            self._current_joint_vel = list(msg.velocity) if msg.velocity else [0.0]*6

    def _on_trajectory_command(self, msg: JointTrajectory):
        """Builds a queue of trajectory segments to execute cleanly at 250Hz."""
        if not msg.points: return
        with self._state_lock:
            q_now = list(self._setpoint_pos) # use current setpoint as origin

        new_segments = []
        t_start = time.monotonic()
        q_prev = q_now
        t_prev = 0.0

        for pt in msg.points:
            q_target = list(pt.positions[:len(self._controllers)])
            t_end = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            duration = max(0.001, t_end - t_prev)
            new_segments.append(TrajectorySegment(q_prev, q_target, duration, t_start + t_prev))
            q_prev = q_target
            t_prev = t_end

        with self._state_lock:
            self._segments = new_segments
            self._current_segment = None

    def _control_tick(self):
        """Runs at 250 Hz. Computes motor commands and sends to hardware."""
        now = time.monotonic()
        with self._state_lock:
            if self._current_segment is None and self._segments:
                self._current_segment = self._segments.pop(0)

            if self._current_segment is not None:
                q_new = self._current_segment.sample(now)
                if q_new is None:
                    self._setpoint_pos = list(self._current_segment.q_end)
                    self._current_segment = None
                else:
                    self._setpoint_pos = list(q_new)

            pos = list(self._current_joint_pos)
            vel = list(self._current_joint_vel)
            spt = list(self._setpoint_pos)
            enc = list(self._encoder_counts)

        # If we haven't received initial setpoints, wait
        if len(spt) < len(self._controllers):
            return

        commands = []
        telemetry_data = []
        for i, ctrl in enumerate(self._controllers):
            if i >= len(pos) or i >= len(spt):
                continue
            if isinstance(ctrl, DCJointController):
                duty = ctrl.update(spt[i], pos[i], 0.0)  # 0.0 = no current feedback yet
                commands.append(('DC', duty))
                telemetry_data.extend([duty, 0.0, 0.0]) # Duty, unused, unused
            elif isinstance(ctrl, BLDCJointController):
                # Phase currents: set to 0 if no current sensors available
                ec = enc[i] if i < len(enc) else 0
                da, db, dc = ctrl.update(spt[i], pos[i], vel[i], ec, 0.0, 0.0, 0.0)
                commands.append(('BLDC', da, db, dc))
                telemetry_data.extend([da, db, dc])
        
        t_msg = Float64MultiArray()
        t_msg.data = telemetry_data
        self._telemetry_pub.publish(t_msg)

        self._send_motor_commands(commands)

    def _on_motor_config(self, msg: String):
        """
        Handles real-time configuration updates from the UI.
        JSON format: {"joint_idx": 0, "motor_type": "DC", "pid": {"kp": 1.0, "ki": 0.1}}
        """
        try:
            data = json.loads(msg.data)
            idx = data['joint_idx']
            if idx < 0 or idx >= len(self._controllers): return

            m_type = data.get('motor_type')
            pid = data.get('pid', {})
            
            # Re-initialize controller if motor type changed
            if m_type and m_type != ('DC' if isinstance(self._controllers[idx], DCJointController) else 'BLDC'):
                self.get_logger().info(f"Switching Joint {idx} to {m_type}")
                cfg = {'joint_name': f'joint_{idx+1}', 'motor_type': m_type}
                # Merge existing PID if not provided
                if m_type == 'DC':
                    cfg.update({'dc_max_current_a': 5.0, 'dc_kp': pid.get('kp', 1.0), 'dc_ki': pid.get('ki', 0.1)})
                    self._controllers[idx] = DCJointController(cfg, self._dt)
                else:
                    cfg.update({'bldc_pole_pairs': 7, 'encoder_cpr': 4096, 'bldc_foc_kp_q': pid.get('kp', 0.5), 'bldc_foc_ki_q': pid.get('ki', 5.0)})
                    self._controllers[idx] = BLDCJointController(cfg, self._dt)
            else:
                # Just update PID parameters
                ctrl = self._controllers[idx]
                if isinstance(ctrl, DCJointController):
                    if 'kp' in pid: ctrl.cur_ctrl.kp = pid['kp']
                    if 'ki' in pid: ctrl.cur_ctrl.ki = pid['ki']
                elif isinstance(ctrl, BLDCJointController):
                    if 'kp' in pid: ctrl.iq_ctrl.kp = pid['kp']
                    if 'ki' in pid: ctrl.iq_ctrl.ki = pid['ki']
            
            self.get_logger().info(f"Updated Motor Config for Joint {idx}")
        except Exception as e:
            self.get_logger().warn(f"Failed to update motor config: {e}")

    def _send_motor_commands(self, commands: list):
        """
        Sends motor commands over serial to the motor driver board.
        Protocol per joint:
          DC:   [0xCC] [joint_idx] [duty_hi] [duty_lo]          — 4 bytes
          BLDC: [0xBB] [joint_idx] [da_hi][da_lo][db_hi][db_lo][dc_hi][dc_lo] — 8 bytes
        duty / da/db/dc are uint16: 0=0%, 65535=100%
        """
        if not self._motor_serial:
            return

        buf = bytearray()
        for i, cmd in enumerate(commands):
            if cmd[0] == 'DC':
                duty_raw = int(np.clip((cmd[1] + 1.0) / 2.0, 0, 1) * 65535)
                buf += struct.pack('>BBHH', 0xCC, i, duty_raw, 0)
            elif cmd[0] == 'BLDC':
                da = int(cmd[1] * 65535)
                db = int(cmd[2] * 65535)
                dc = int(cmd[3] * 65535)
                buf += struct.pack('>BBHHH', 0xBB, i, da, db, dc)

        try:
            self._motor_serial.write(buf)
        except serial.SerialException as e:
            self.get_logger().warn(f'Motor serial write failed: {e}')

    def _load_config(self) -> dict:
        import os
        from ament_index_python.packages import get_package_share_directory
        # Provide sensible defaults if file missing to avoid crash
        default = {
            'joints': [
                {'joint_name': f'joint_{i}', 'motor_type': 'DC', 'dc_max_current_a': 5.0, 'dc_kp': 1.0, 'dc_ki': 0.1}
                for i in range(1, 7)
            ]
        }
        try:
            path = self.get_parameter('config_file').value
            with open(path) as f:
                data = yaml.safe_load(f)
                return data if data else default
        except Exception as e:
            self.get_logger().warn(f"Failed to load motor config, using defaults: {e}")
            return default


def main():
    rclpy.init()
    node = MotorControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
