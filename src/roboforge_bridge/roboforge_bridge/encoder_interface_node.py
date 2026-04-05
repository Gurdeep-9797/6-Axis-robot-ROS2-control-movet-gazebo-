#!/usr/bin/env python3
"""
encoder_interface_node.py
─────────────────────────
Reads encoder values from hardware and publishes /joint_states at 250 Hz.
Supports three hardware backends: USB-Serial (STM32/Teensy), SPI (AS5047P),
and simulated (for testing without hardware).

The published /joint_states message is IDENTICAL in format to what Gazebo
publishes, so the rest of the pipeline is hardware-agnostic.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
import serial
import serial.tools.list_ports
import threading
import time
import math
import yaml
import struct
from pathlib import Path
from dataclasses import dataclass, field
from typing import List, Optional


@dataclass
class JointEncoderConfig:
    """Per-joint encoder configuration loaded from encoder_config.yaml."""
    joint_name: str              # e.g. "joint_1"
    counts_per_rev: int          # e.g. 4096 for 12-bit absolute, 8192 for quadrature × 2
    gear_ratio: float            # e.g. 100.0 for 100:1 gearbox
    zero_offset_counts: int      # count value corresponding to zero radians (set by calibration)
    direction: int               # +1 or -1 (reverses encoder polarity)
    joint_limit_min_rad: float   # e.g. -math.pi
    joint_limit_max_rad: float   # e.g.  math.pi

    def counts_to_radians(self, raw_count: int) -> float:
        """Convert raw encoder count to joint angle in radians."""
        adjusted = (raw_count - self.zero_offset_counts) * self.direction
        # Wrap to [-CPR/2, CPR/2] for absolute encoders
        adjusted = adjusted % self.counts_per_rev
        if adjusted > self.counts_per_rev / 2:
            adjusted -= self.counts_per_rev
        # Scale: angle = (counts / CPR) * (2π / gear_ratio)
        angle_rad = (adjusted / self.counts_per_rev) * (2.0 * math.pi / self.gear_ratio)
        # Clamp to joint limits
        return max(self.joint_limit_min_rad, min(self.joint_limit_max_rad, angle_rad))


class EncoderInterfaceNode(Node):

    def __init__(self):
        super().__init__('encoder_interface_node')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('hardware_backend', 'serial')
        # Options: 'serial' | 'spi' | 'ethercat' | 'simulated'

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        # On Windows: 'COM3'. On Linux: '/dev/ttyUSB0'

        self.declare_parameter('serial_baud', 1000000)
        # Use 1Mbaud for fast encoder read. STM32 / Teensy must match.

        self.declare_parameter('publish_rate_hz', 250)
        # Output rate to /joint_states. Do NOT exceed hardware read rate.

        self.declare_parameter('config_file', 'config/encoder_config.yaml')

        # ── Load encoder config ──────────────────────────────────────────────
        config_path = Path(self.get_parameter('config_file').value)
        self._joint_configs: List[JointEncoderConfig] = self._load_config(config_path)
        self._num_joints = len(self._joint_configs)

        # ── State (written by reader thread, read by publish timer) ──────────
        self._state_lock = threading.Lock()
        self._raw_counts:       List[int]   = [0] * self._num_joints
        self._joint_angles_rad: List[float] = [0.0] * self._num_joints
        self._joint_velocities: List[float] = [0.0] * self._num_joints  # rad/s, computed
        self._prev_angles:      List[float] = [0.0] * self._num_joints
        self._prev_time:        float       = time.monotonic()
        self._hardware_ok:      bool        = False

        # ── Publishers ──────────────────────────────────────────────────────
        self._joint_state_pub = self.create_publisher(
            JointState, '/joint_states', 10
        )
        self._raw_encoder_pub = self.create_publisher(
            Float64MultiArray, '/encoder/raw', 10
        )
        self._health_pub = self.create_publisher(
            String, '/roboforge/encoder_health', 10
        )

        # ── Hardware reader thread ───────────────────────────────────────────
        backend = self.get_parameter('hardware_backend').value
        if backend == 'serial':
            self._reader_thread = threading.Thread(
                target=self._serial_reader_loop, daemon=True
            )
        elif backend == 'simulated':
            self._reader_thread = threading.Thread(
                target=self._simulated_reader_loop, daemon=True
            )
        else:
            raise ValueError(f"Unknown backend: {backend}. Use 'serial' or 'simulated'.")
        self._reader_thread.start()

        # ── Publish timer ────────────────────────────────────────────────────
        rate_hz = self.get_parameter('publish_rate_hz').value
        self.create_timer(1.0 / rate_hz, self._publish_joint_states)

        self.get_logger().info(
            f'EncoderInterfaceNode started | backend={backend} | '
            f'joints={self._num_joints} | rate={rate_hz}Hz'
        )

    def _load_config(self, path: Path) -> List[JointEncoderConfig]:
        """Load encoder_config.yaml. Creates a default if missing."""
        if not path.exists():
            self.get_logger().warn(
                f'encoder_config.yaml not found at {path}. '
                f'Using defaults (6-joint, 4096 CPR, 100:1 ratio). '
                f'Run calibration to set zero offsets.'
            )
            return [
                JointEncoderConfig(
                    joint_name=f'joint_{i+1}',
                    counts_per_rev=4096,
                    gear_ratio=100.0,
                    zero_offset_counts=2048,   # mid-range default
                    direction=1,
                    joint_limit_min_rad=-math.pi,
                    joint_limit_max_rad=math.pi
                )
                for i in range(6)
            ]
        with open(path) as f:
            data = yaml.safe_load(f)
        return [JointEncoderConfig(**j) for j in data['joints']]

    # ── SERIAL READER (STM32 / Teensy / Arduino) ───────────────────────────
    # Protocol: STM32 sends a 26-byte packet at 1000 Hz:
    #   [0xAA] [0x55] [j1_hi] [j1_lo] ... [j6_hi] [j6_lo] [checksum]
    # Each joint value is a 16-bit unsigned int (0–65535 for 16-bit, 0–4095 for 12-bit).
    # Checksum = XOR of all data bytes.
    PACKET_HEADER = bytes([0xAA, 0x55])
    PACKET_SIZE   = 2 + 6 * 2 + 1  # header + 6 × uint16 + checksum = 15 bytes

    def _serial_reader_loop(self):
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('serial_baud').value

        while rclpy.ok():
            try:
                with serial.Serial(port, baud, timeout=0.05) as ser:
                    self._hardware_ok = True
                    self.get_logger().info(f'Encoder serial port opened: {port} @ {baud}')
                    self._publish_health('connected', port)
                    buf = bytearray()

                    while rclpy.ok():
                        chunk = ser.read(64)
                        if not chunk:
                            continue
                        buf.extend(chunk)

                        # Scan for valid packet header
                        while len(buf) >= self.PACKET_SIZE:
                            idx = buf.find(self.PACKET_HEADER)
                            if idx == -1:
                                buf.clear()
                                break
                            if idx > 0:
                                del buf[:idx]   # discard garbage bytes before header
                            if len(buf) < self.PACKET_SIZE:
                                break

                            packet = bytes(buf[:self.PACKET_SIZE])
                            del buf[:self.PACKET_SIZE]

                            # Validate checksum
                            data_bytes = packet[2:-1]
                            expected_crc = 0
                            for b in data_bytes:
                                expected_crc ^= b
                            if expected_crc != packet[-1]:
                                self.get_logger().warn('Encoder packet CRC mismatch — dropped')
                                continue

                            # Unpack 6 × uint16 joint counts
                            counts = struct.unpack('>6H', data_bytes)
                            self._update_state(list(counts))

            except serial.SerialException as e:
                self._hardware_ok = False
                self._publish_health('disconnected', str(e))
                self.get_logger().error(f'Encoder serial error: {e}. Retrying in 2s...')
                time.sleep(2.0)

    # ── SIMULATED READER (for testing without hardware) ────────────────────
    def _simulated_reader_loop(self):
        """Generates sinusoidal encoder motion for integration testing."""
        self._hardware_ok = True
        t = 0.0
        while rclpy.ok():
            counts = [
                int(2048 + 800 * math.sin(t + i * math.pi / 3))
                for i in range(self._num_joints)
            ]
            self._update_state(counts)
            t += 0.001
            time.sleep(0.001)   # 1000 Hz read rate

    # ── STATE UPDATE (called from reader thread) ───────────────────────────
    def _update_state(self, counts: List[int]):
        now = time.monotonic()
        angles = [
            cfg.counts_to_radians(counts[i])
            for i, cfg in enumerate(self._joint_configs)
        ]

        with self._state_lock:
            dt = now - self._prev_time
            if dt > 0.0:
                self._joint_velocities = [
                    (angles[i] - self._prev_angles[i]) / dt
                    for i in range(self._num_joints)
                ]
            self._raw_counts       = list(counts)
            self._joint_angles_rad = angles
            self._prev_angles      = angles
            self._prev_time        = now

    # ── PUBLISHER (called from ROS 2 timer at 250 Hz) ─────────────────────
    def _publish_joint_states(self):
        with self._state_lock:
            angles     = list(self._joint_angles_rad)
            velocities = list(self._joint_velocities)
            raw_counts = list(self._raw_counts)

        # /joint_states — IDENTICAL FORMAT to Gazebo output
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.name     = [cfg.joint_name for cfg in self._joint_configs]
        msg.position = angles
        msg.velocity = velocities
        msg.effort   = [0.0] * self._num_joints   # effort not available from encoder alone
        self._joint_state_pub.publish(msg)

        # /encoder/raw — raw counts for diagnostics panel
        raw_msg = Float64MultiArray()
        raw_msg.data = [float(c) for c in raw_counts]
        self._raw_encoder_pub.publish(raw_msg)

    def _publish_health(self, status: str, detail: str):
        msg = String()
        msg.data = f'{{"status":"{status}","detail":"{detail}"}}'
        self._health_pub.publish(msg)


def main():
    rclpy.init()
    node = EncoderInterfaceNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
