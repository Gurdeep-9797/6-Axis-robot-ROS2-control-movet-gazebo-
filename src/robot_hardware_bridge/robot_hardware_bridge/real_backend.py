"""
Real Controller Backend for Hardware Bridge

AUTHORITY: RELAY ONLY - NO CONTROL AUTHORITY

This backend communicates with the actual RT Controller hardware.
It is a protocol gateway only - the RT Controller is the authority.

The RT Controller:
- Performs ALL semantic validation (joint limits, velocities, physics)
- Owns motor control
- Owns safety enforcement
- Owns position truth (encoder data)

This backend:
- Serializes ROS messages for controller protocol
- Deserializes controller responses to ROS messages
- Handles connection management and reconnection
- Does NOT make control decisions
"""

import socket
import struct
import threading
import time
from typing import Optional

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from robot_msgs.msg import TrajectoryAck, ExecutionState


class RealControllerBackend:
    """
    Real Controller Backend - Protocol Gateway to RT Controller
    
    This class handles communication with the real RT Controller.
    It has NO authority - the Controller makes all decisions.
    """
    
    JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
    
    # Protocol constants
    MSG_TRAJECTORY = 0x01
    MSG_STOP = 0x02
    MSG_JOINT_STATE = 0x10
    MSG_TRAJECTORY_ACK = 0x11
    MSG_EXECUTION_STATE = 0x12
    
    def __init__(self, node, ip: str, port: int, protocol: str):
        self.node = node
        self.logger = node.get_logger()
        self.ip = ip
        self.port = port
        self.protocol = protocol
        
        # Connection state
        self.socket: Optional[socket.socket] = None
        self.connected = False
        self.reconnect_interval = 1.0  # seconds
        
        # Current state (from controller)
        self.current_joint_state: Optional[JointState] = None
        self.current_execution_state: Optional[ExecutionState] = None
        self.pending_ack: Optional[TrajectoryAck] = None
        
        # Thread control
        self.running = False
        self.receive_thread: Optional[threading.Thread] = None
        self.lock = threading.Lock()
    
    def start(self):
        """Start the real controller backend."""
        self.logger.info(f'Real Controller Backend connecting to {self.ip}:{self.port}')
        self.running = True
        self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.receive_thread.start()
    
    def stop(self):
        """Stop the real controller backend."""
        self.logger.info('Real Controller Backend stopping')
        self.running = False
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
        if self.receive_thread:
            self.receive_thread.join(timeout=1.0)
    
    def _connect(self) -> bool:
        """Attempt to connect to controller."""
        try:
            if self.protocol == 'TCP':
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.settimeout(5.0)
                self.socket.connect((self.ip, self.port))
            elif self.protocol == 'UDP':
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.socket.settimeout(0.1)
            else:
                self.logger.error(f'Unknown protocol: {self.protocol}')
                return False
            
            self.connected = True
            self.logger.info(f'Connected to controller at {self.ip}:{self.port}')
            return True
            
        except socket.error as e:
            self.logger.warn(f'Connection failed: {e}')
            self.connected = False
            return False
    
    def _receive_loop(self):
        """Receive data from controller."""
        while self.running:
            if not self.connected:
                if not self._connect():
                    time.sleep(self.reconnect_interval)
                    continue
            
            try:
                if self.protocol == 'TCP':
                    # Read message header (type + length)
                    header = self._receive_exact(4)
                    if header is None:
                        continue
                    
                    msg_type, msg_len = struct.unpack('<BH', header[:3])
                    
                    # Read message body
                    body = self._receive_exact(msg_len)
                    if body is None:
                        continue
                    
                    self._process_message(msg_type, body)
                    
                elif self.protocol == 'UDP':
                    try:
                        data, _ = self.socket.recvfrom(1024)
                        if len(data) >= 3:
                            msg_type = data[0]
                            self._process_message(msg_type, data[3:])
                    except socket.timeout:
                        pass
                        
            except socket.error as e:
                self.logger.warn(f'Connection error: {e}')
                self.connected = False
                time.sleep(self.reconnect_interval)
    
    def _receive_exact(self, num_bytes: int) -> Optional[bytes]:
        """Receive exactly num_bytes from TCP socket."""
        data = b''
        while len(data) < num_bytes:
            try:
                chunk = self.socket.recv(num_bytes - len(data))
                if not chunk:
                    self.connected = False
                    return None
                data += chunk
            except socket.timeout:
                return None
            except socket.error:
                self.connected = False
                return None
        return data
    
    def _process_message(self, msg_type: int, data: bytes):
        """Process message from controller."""
        if msg_type == self.MSG_JOINT_STATE:
            self._process_joint_state(data)
        elif msg_type == self.MSG_TRAJECTORY_ACK:
            self._process_trajectory_ack(data)
        elif msg_type == self.MSG_EXECUTION_STATE:
            self._process_execution_state(data)
    
    def _process_joint_state(self, data: bytes):
        """Process joint state from controller."""
        # Format: timestamp_us(8) + positions(6*8) + velocities(6*8) + efforts(6*8)
        if len(data) < 8 + 6*8 + 6*8 + 6*8:
            return
        
        offset = 0
        timestamp_us = struct.unpack('<Q', data[offset:offset+8])[0]
        offset += 8
        
        positions = list(struct.unpack('<6d', data[offset:offset+48]))
        offset += 48
        
        velocities = list(struct.unpack('<6d', data[offset:offset+48]))
        offset += 48
        
        efforts = list(struct.unpack('<6d', data[offset:offset+48]))
        
        msg = JointState()
        msg.header.stamp.sec = timestamp_us // 1000000
        msg.header.stamp.nanosec = (timestamp_us % 1000000) * 1000
        msg.name = list(self.JOINT_NAMES)
        msg.position = positions
        msg.velocity = velocities
        msg.effort = efforts
        
        with self.lock:
            self.current_joint_state = msg
    
    def _process_trajectory_ack(self, data: bytes):
        """Process trajectory acknowledgment from controller."""
        # Format: timestamp(8) + status(1) + reason_len(2) + reason(var)
        if len(data) < 11:
            return
        
        timestamp_us = struct.unpack('<Q', data[0:8])[0]
        status = data[8]
        reason_len = struct.unpack('<H', data[9:11])[0]
        reason = data[11:11+reason_len].decode('utf-8') if reason_len > 0 else ''
        
        msg = TrajectoryAck()
        msg.trajectory_id.sec = timestamp_us // 1000000
        msg.trajectory_id.nanosec = (timestamp_us % 1000000) * 1000
        msg.status = status
        msg.reject_reason = reason
        
        with self.lock:
            self.pending_ack = msg
    
    def _process_execution_state(self, data: bytes):
        """Process execution state from controller."""
        # Format: state(1) + progress(4) + fault_code(4) + fault_msg_len(2) + fault_msg(var)
        if len(data) < 11:
            return
        
        state = data[0]
        progress = struct.unpack('<f', data[1:5])[0]
        fault_code = struct.unpack('<I', data[5:9])[0]
        msg_len = struct.unpack('<H', data[9:11])[0]
        fault_msg = data[11:11+msg_len].decode('utf-8') if msg_len > 0 else ''
        
        msg = ExecutionState()
        msg.state = state
        msg.progress = progress
        msg.fault_code = fault_code
        msg.fault_message = fault_msg
        
        with self.lock:
            self.current_execution_state = msg
    
    def send_trajectory(self, trajectory: JointTrajectory):
        """Send trajectory to controller."""
        if not self.connected:
            self.logger.warn('Cannot send trajectory: not connected')
            return
        
        # Serialize trajectory
        # Format: timestamp(8) + num_points(2) + [point data...]
        # Point: time_from_start(8) + positions(6*8) + velocities(6*8)
        
        timestamp_us = trajectory.header.stamp.sec * 1000000 + trajectory.header.stamp.nanosec // 1000
        
        data = struct.pack('<Q', timestamp_us)
        data += struct.pack('<H', len(trajectory.points))
        
        for point in trajectory.points:
            time_ns = point.time_from_start.sec * 1000000000 + point.time_from_start.nanosec
            data += struct.pack('<Q', time_ns)
            
            # Positions
            for i in range(6):
                pos = point.positions[i] if i < len(point.positions) else 0.0
                data += struct.pack('<d', pos)
            
            # Velocities
            for i in range(6):
                vel = point.velocities[i] if point.velocities and i < len(point.velocities) else 0.0
                data += struct.pack('<d', vel)
        
        # Send with header
        header = struct.pack('<BH', self.MSG_TRAJECTORY, len(data))
        
        try:
            if self.protocol == 'TCP':
                self.socket.sendall(header + data)
            elif self.protocol == 'UDP':
                self.socket.sendto(header + data, (self.ip, self.port))
            
            self.logger.debug(f'Sent trajectory with {len(trajectory.points)} points')
            
        except socket.error as e:
            self.logger.error(f'Failed to send trajectory: {e}')
            self.connected = False
    
    def get_joint_state(self) -> Optional[JointState]:
        """Get current joint state from controller."""
        with self.lock:
            return self.current_joint_state
    
    def get_execution_state(self) -> Optional[ExecutionState]:
        """Get current execution state from controller."""
        with self.lock:
            if self.current_execution_state:
                return self.current_execution_state
            
            # Default state if not received yet
            msg = ExecutionState()
            msg.state = ExecutionState.STATE_IDLE if self.connected else ExecutionState.STATE_COMM_LOSS
            msg.progress = 0.0
            msg.fault_code = 0
            msg.fault_message = '' if self.connected else 'No connection to controller'
            return msg
    
    def get_trajectory_ack(self) -> Optional[TrajectoryAck]:
        """Get pending trajectory ack (clears after read)."""
        with self.lock:
            ack = self.pending_ack
            self.pending_ack = None
        return ack
