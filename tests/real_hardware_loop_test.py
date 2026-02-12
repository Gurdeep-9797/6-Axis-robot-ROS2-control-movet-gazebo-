#!/usr/bin/env python3
import socket
import struct
import time
import sys

# Configuration
CONTROLLER_IP = '192.168.1.100' # Default
CONTROLLER_PORT = 5000

# Protocol Constants
MSG_TRAJECTORY = 0x01
MSG_JOINT_STATE = 0x10
MSG_TRAJECTORY_ACK = 0x11
MSG_EXECUTION_STATE = 0x12

JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

def log(tag, msg):
    print(f"[{time.strftime('%H:%M:%S')}] [{tag}] {msg}")

class HardwareTest:
    def __init__(self, ip=CONTROLLER_IP, port=CONTROLLER_PORT):
        self.ip = ip
        self.port = port
        self.sock = None
        
    def connect(self):
        log("CONN", f"Connecting to {self.ip}:{self.port}...")
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(2.0)
            self.sock.connect((self.ip, self.port))
            log("CONN", "Connected successfully")
            return True
        except Exception as e:
            log("CONN", f"Connection Failed: {e}")
            return False

    def send_trajectory(self):
        # Create a simple shimmy trajectory
        # Move J1 to 0.1 rad then back to 0.0
        log("TEST", "Generating Test Trajectory (Shimmy)")
        
        # Binary Packing (Little Endian)
        # Header: Type(1) + Len(2)
        # Body: Timestamp(8) + NumPoints(2) + Points...
        
        timestamp_us = int(time.time() * 1e6)
        num_points = 1
        
        # Point 1: 1.0 second, J1=0.2 rad
        p1_time_ns = 1 * 1000000000
        p1_pos = [0.2, 0.0, 0.0, 0.0, 0.0, 0.0]
        p1_vel = [0.0] * 6
        
        # Serialize Payload
        payload = struct.pack('<Q', timestamp_us)
        payload += struct.pack('<H', num_points)
        
        # Point 1
        payload += struct.pack('<Q', p1_time_ns)
        for val in p1_pos: payload += struct.pack('<d', val)
        for val in p1_vel: payload += struct.pack('<d', val)
        
        # Header
        header = struct.pack('<BH', MSG_TRAJECTORY, len(payload))
        
        self.sock.sendall(header + payload)
        log("TEST", "Trajectory Sent")

    def listen_feedback(self, duration=5.0):
        log("FEEDBACK", "Listening for Joint States...")
        start = time.time()
        while time.time() - start < duration:
            try:
                # Read Header
                header = self.sock.recv(3)
                if not header: break
                
                msg_type, msg_len = struct.unpack('<BH', header)
                body = b''
                while len(body) < msg_len:
                    body += self.sock.recv(msg_len - len(body))
                
                if msg_type == MSG_JOINT_STATE:
                    # timestamp(8) + pos(48) + vel(48) + eff(48)
                    offset = 8
                    positions = struct.unpack('<6d', body[offset:offset+48])
                    log("STATE", f"Pos: {[round(x,3) for x in positions]}")
                elif msg_type == MSG_TRAJECTORY_ACK:
                    status = body[8]
                    log("ACK", f"Trajectory Status: {status}")
                    
            except socket.timeout:
                continue
            except Exception as e:
                log("ERR", str(e))
                break

    def run(self):
        if not self.connect():
            log("FAIL", "Cannot proceed without hardware connection.")
            return
            
        self.send_trajectory()
        self.listen_feedback()
        self.sock.close()

if __name__ == '__main__':
    test = HardwareTest()
    test.run()
