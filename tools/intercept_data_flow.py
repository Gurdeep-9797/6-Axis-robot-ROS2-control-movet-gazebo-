#!/usr/bin/env python3
"""
RoboForge Data Flow Interceptor
───────────────────────────────
Real-time interception of all data flowing between:
- Gazebo Simulator → /joint_states
- ROS2 Topics → Bridge
- Bridge → React IDE / WPF Client
- MoveIt → IK/FK Services
- User Commands → Trajectory Execution

This script connects to the ROS2 bridge WebSocket and logs ALL messages
in real-time, providing a complete view of the data flow.

Usage: python intercept_data_flow.py
"""

import asyncio
import json
import websockets
import time
from datetime import datetime
from collections import deque

# Configuration
BRIDGE_URL = "ws://localhost:9090"
LOG_FILE = "data_flow_log.jsonl"
MAX_BUFFER = 1000  # Keep last 1000 messages in memory

class DataFlowInterceptor:
    def __init__(self):
        self.message_buffer = deque(maxlen=MAX_BUFFER)
        self.stats = {
            "joint_states_received": 0,
            "ik_requests": 0,
            "ik_responses": 0,
            "trajectory_commands": 0,
            "io_updates": 0,
            "errors": 0,
            "start_time": None,
        }
        
    def log_message(self, direction, source, message_type, data):
        """Log a message with timestamp and metadata"""
        entry = {
            "timestamp": datetime.now().isoformat(),
            "direction": direction,  # IN/OUT
            "source": source,
            "type": message_type,
            "data": data,
        }
        self.message_buffer.append(entry)
        
        # Update stats
        if message_type == "joint_states":
            self.stats["joint_states_received"] += 1
        elif "ik" in message_type.lower():
            if direction == "OUT":
                self.stats["ik_requests"] += 1
            else:
                self.stats["ik_responses"] += 1
        elif "trajectory" in message_type.lower():
            self.stats["trajectory_commands"] += 1
        elif "io" in message_type.lower():
            self.stats["io_updates"] += 1
            
    def print_stats(self):
        """Print current statistics"""
        elapsed = time.time() - self.stats["start_time"] if self.stats["start_time"] else 0
        print("\n" + "=" * 60)
        print("📊 DATA FLOW STATISTICS")
        print("=" * 60)
        print(f"  Uptime:               {elapsed:.1f}s")
        print(f"  Joint States:         {self.stats['joint_states_received']}")
        print(f"  IK Requests:          {self.stats['ik_requests']}")
        print(f"  IK Responses:         {self.stats['ik_responses']}")
        print(f"  Trajectory Commands:  {self.stats['trajectory_commands']}")
        print(f"  IO Updates:           {self.stats['io_updates']}")
        print(f"  Errors:               {self.stats['errors']}")
        print(f"  Messages Buffered:    {len(self.message_buffer)}")
        print("=" * 60)

    async def intercept(self):
        """Main interception loop"""
        print("🔌 RoboForge Data Flow Interceptor")
        print("=" * 60)
        print(f"Connecting to {BRIDGE_URL}...")
        
        self.stats["start_time"] = time.time()
        
        try:
            async with websockets.connect(BRIDGE_URL) as ws:
                print("✅ Connected to ROS2 Bridge")
                print("📡 Intercepting all data flow...")
                print("   (Press Ctrl+C to stop)")
                print("=" * 60)
                
                # Subscribe to joint states
                subscribe_msg = json.dumps({
                    "op": "subscribe",
                    "topic": "/joint_states",
                    "type": "sensor_msgs/msg/JointState"
                })
                await ws.send(subscribe_msg)
                
                # Subscribe to planned trajectory
                await ws.send(json.dumps({
                    "op": "subscribe",
                    "topic": "/planned_trajectory",
                    "type": "trajectory_msgs/msg/JointTrajectory"
                }))
                
                print("\n📡 Subscribed to:")
                print("   /joint_states")
                print("   /planned_trajectory")
                print()
                
                # Main receive loop
                async for message in ws:
                    try:
                        data = json.loads(message)
                        await self.process_message(data)
                    except json.JSONDecodeError:
                        self.stats["errors"] += 1
                    except Exception as e:
                        self.stats["errors"] += 1
                        print(f"❌ Error: {e}")
                        
        except websockets.exceptions.ConnectionClosed:
            print("\n❌ Connection closed by server")
        except ConnectionRefusedError:
            print(f"\n❌ Connection refused - is the bridge running at {BRIDGE_URL}?")
        except Exception as e:
            print(f"\n❌ Error: {e}")
        finally:
            self.print_stats()
            
    async def process_message(self, data):
        """Process intercepted message"""
        op = data.get("op", "")
        
        if op == "publish":
            topic = data.get("topic", "")
            msg = data.get("msg", {})
            
            if topic == "/joint_states":
                self.handle_joint_states(msg)
            elif topic == "/planned_trajectory":
                self.handle_trajectory(msg)
                
        elif op == "service_response":
            self.handle_service_response(data)
            
        elif op == "status":
            self.handle_status(data)

    def handle_joint_states(self, msg):
        """Handle joint state messages from Gazebo"""
        positions = msg.get("position", [])
        velocities = msg.get("velocity", [])
        names = msg.get("name", [])
        
        self.log_message("IN", "Gazebo", "joint_states", {
            "names": names[:6],
            "positions_rad": positions[:6],
            "positions_deg": [round(p * 180 / 3.14159, 2) for p in positions[:6]],
            "velocities": velocities[:6],
        })
        
        # Print live joint states
        if positions:
            print(f"🦾 GAZEBO → Joint States ({len(positions)} joints)")
            for i, (name, pos) in enumerate(zip(names[:6], positions[:6])):
                deg = pos * 180 / 3.14159
                print(f"   {name}: {deg:+7.2f}° ({pos:+.4f} rad)")
            print()

    def handle_trajectory(self, msg):
        """Handle planned trajectory from MoveIt"""
        self.log_message("IN", "MoveIt", "planned_trajectory", {
            "points": len(msg.get("points", [])),
        })
        
        points = msg.get("points", [])
        print(f"🎯 MOVEIT → Planned Trajectory ({len(points)} waypoints)")
        for i, point in enumerate(points[:3]):  # Show first 3
            positions = point.get("positions", [])
            time_from_start = point.get("time_from_start", {})
            secs = time_from_start.get("sec", 0)
            print(f"   Point {i+1}: {positions[:3]}... (t={secs}s)")
        print()

    def handle_service_response(self, data):
        """Handle service responses (IK/FK)"""
        self.log_message("OUT", "Client", "service_response", {
            "id": data.get("id", ""),
        })
        
        service = data.get("service", "unknown")
        values = data.get("values", {})
        
        print(f"⚙️  SERVICE RESPONSE → {service}")
        if "error_code" in values:
            ec = values["error_code"]
            code = ec.get("val", -1)
            status = "✅ SUCCESS" if code == 1 else f"❌ ERROR ({code})"
            print(f"   Error Code: {status}")
        print()

    def handle_status(self, data):
        """Handle status messages"""
        level = data.get("level", 0)
        msg = data.get("msg", "")
        
        if level > 0:
            self.stats["errors"] += 1
            
        print(f"📢 STATUS: {msg}")
        print()

    def save_log(self):
        """Save message buffer to JSONL file"""
        with open(LOG_FILE, "w") as f:
            for entry in self.message_buffer:
                f.write(json.dumps(entry) + "\n")
        print(f"💾 Log saved to {LOG_FILE} ({len(self.message_buffer)} entries)")


async def main():
    interceptor = DataFlowInterceptor()
    
    try:
        await interceptor.intercept()
    except KeyboardInterrupt:
        print("\n\n⏹️  Intercepting stopped")
        interceptor.save_log()


if __name__ == "__main__":
    asyncio.run(main())
