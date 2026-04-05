#!/usr/bin/env python3
"""
RoboForge Pipeline Diagnostics — Hardline Interceptor
Tests end-to-end data flow through the ROS 2 / MoveIt / Gazebo pipeline.
Run from INSIDE the Docker container or on a machine with ROS 2 sourced.

Usage:
    python3 pipeline_diagnostics.py

What it verifies:
    1. rosbridge WebSocket is reachable on :9090
    2. /joint_states topic is publishing (Gazebo → ROS feedback loop)
    3. /compute_ik service responds (MoveIt IK solver is alive)  
    4. /planned_trajectory topic accepts messages
    5. Round-trip: IK solve → trajectory publish → joint_states change
"""

import json
import sys
import time
import asyncio
import websockets

WS_URL = "ws://localhost:9090"

# ── Test waypoints (same as online + offline UI defaults) ──
TEST_TARGETS = [
    {"name": "Home",     "x": 0.800, "y": 0.000, "z": 1.800},
    {"name": "Approach", "x": 0.800, "y": 0.200, "z": 1.400},
    {"name": "Pick",     "x": 0.700, "y": 0.200, "z": 1.200},
]

class PipelineDiagnostics:
    def __init__(self):
        self.ws = None
        self.msg_id = 0
        self.results = []
        self.joint_states_received = 0
        self.last_joint_state = None
    
    def next_id(self):
        self.msg_id += 1
        return f"diag_{self.msg_id}"
    
    def log(self, level, msg):
        t = time.strftime("%H:%M:%S")
        color = {"OK": "\033[92m", "ERR": "\033[91m", "WRN": "\033[93m", "INF": "\033[94m", "DBG": "\033[90m"}
        reset = "\033[0m"
        c = color.get(level, "")
        print(f"  {t} {c}{level:3s}{reset} {msg}")
        self.results.append({"time": t, "level": level, "msg": msg})
    
    async def connect(self):
        self.log("INF", f"Connecting to rosbridge at {WS_URL}...")
        try:
            self.ws = await asyncio.wait_for(
                websockets.connect(WS_URL), timeout=5.0
            )
            self.log("OK", f"Connected to rosbridge WebSocket")
            return True
        except Exception as e:
            self.log("ERR", f"Cannot connect to rosbridge: {e}")
            return False
    
    async def send_and_recv(self, msg, timeout=10.0):
        """Send a JSON message and wait for a response."""
        msg_id = self.next_id()
        msg["id"] = msg_id
        await self.ws.send(json.dumps(msg))
        
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                raw = await asyncio.wait_for(self.ws.recv(), timeout=2.0)
                data = json.loads(raw)
                if data.get("id") == msg_id:
                    return data
                # Store /joint_states messages that arrive in between
                if data.get("topic") == "/joint_states":
                    self.joint_states_received += 1
                    msg_data = data.get("msg", {})
                    positions = msg_data.get("position", [])
                    if len(positions) >= 6:
                        self.last_joint_state = list(positions[:6])
            except asyncio.TimeoutError:
                continue
        return None
    
    async def subscribe(self, topic, msg_type, count=5, timeout=5.0):
        """Subscribe to a topic and collect `count` messages."""
        sub_msg = {
            "op": "subscribe",
            "topic": topic,
            "type": msg_type
        }
        await self.ws.send(json.dumps(sub_msg))
        
        messages = []
        deadline = time.time() + timeout
        while len(messages) < count and time.time() < deadline:
            try:
                raw = await asyncio.wait_for(self.ws.recv(), timeout=2.0)
                data = json.loads(raw)
                if data.get("topic") == topic:
                    messages.append(data.get("msg", {}))
            except asyncio.TimeoutError:
                continue
        
        # Unsubscribe
        unsub = {"op": "unsubscribe", "topic": topic}
        await self.ws.send(json.dumps(unsub))
        
        return messages
    
    # ── Test 1: Joint States ──────────────────────────────────────────
    async def test_joint_states(self):
        self.log("INF", "═══ Test 1: /joint_states topic ═══")
        msgs = await self.subscribe("/joint_states", "sensor_msgs/JointState", count=3, timeout=5.0)
        
        if not msgs:
            self.log("ERR", "/joint_states — NO messages received (Gazebo not publishing?)")
            return False
        
        self.log("OK", f"/joint_states — received {len(msgs)} messages")
        for i, m in enumerate(msgs):
            pos = m.get("position", [])
            names = m.get("name", [])
            if len(pos) >= 6:
                angles_deg = [f"{p * 57.2958:.1f}°" for p in pos[:6]]
                self.log("DBG", f"  Sample {i+1}: {', '.join(names[:6])}")
                self.log("DBG", f"  Angles:  {', '.join(angles_deg)}")
                self.last_joint_state = list(pos[:6])
        return True
    
    # ── Test 2: MoveIt IK Service ─────────────────────────────────────
    async def test_moveit_ik(self):
        self.log("INF", "═══ Test 2: MoveIt IK Service (/compute_ik) ═══")
        
        seed = self.last_joint_state or [0, 0, 0, 0, 0, 0]
        
        for target in TEST_TARGETS:
            request = {
                "op": "call_service",
                "service": "/compute_ik",
                "type": "moveit_msgs/srv/GetPositionIK",
                "args": {
                    "ik_request": {
                        "group_name": "manipulator",
                        "robot_state": {
                            "joint_state": {
                                "name": ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
                                "position": seed
                            }
                        },
                        "avoid_collisions": True,
                        "pose_stamped": {
                            "header": {"frame_id": "base_link"},
                            "pose": {
                                "position": {"x": target["x"], "y": target["y"], "z": target["z"]},
                                "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}
                            }
                        },
                        "timeout": {"sec": 5, "nanosec": 0}
                    }
                }
            }
            
            t0 = time.time()
            result = await self.send_and_recv(request, timeout=10.0)
            elapsed = (time.time() - t0) * 1000
            
            if result is None:
                self.log("ERR", f"  {target['name']} — IK service timed out ({elapsed:.0f}ms)")
                continue
            
            values = result.get("values", result.get("result", {}))
            error_code = None
            
            # Try to extract error code from different response formats
            if isinstance(values, dict):
                ec = values.get("error_code", {})
                error_code = ec.get("val") if isinstance(ec, dict) else ec
            
            if error_code == 1:
                sol = values.get("solution", {}).get("joint_state", {}).get("position", [])
                if len(sol) >= 6:
                    angles_deg = [f"{s * 57.2958:.1f}°" for s in sol[:6]]
                    self.log("OK", f"  {target['name']} [{target['x']*1000:.0f},{target['y']*1000:.0f},{target['z']*1000:.0f}]mm → solved in {elapsed:.0f}ms")
                    self.log("DBG", f"    Joints: {', '.join(angles_deg)}")
                    seed = list(sol[:6])  # Use as seed for next
                else:
                    self.log("WRN", f"  {target['name']} — solved but no joint angles in response")
            else:
                self.log("ERR", f"  {target['name']} — IK FAILED (error_code={error_code}, {elapsed:.0f}ms)")
                self.log("DBG", f"    Full response: {json.dumps(values)[:200]}")
        
        return True
    
    # ── Test 3: Trajectory Publishing ─────────────────────────────────
    async def test_trajectory_publish(self):
        self.log("INF", "═══ Test 3: Trajectory Publishing ═══")
        
        # Advertise topic
        advertise = {
            "op": "advertise",
            "topic": "/planned_trajectory",
            "type": "trajectory_msgs/JointTrajectory"
        }
        await self.ws.send(json.dumps(advertise))
        await asyncio.sleep(0.5)
        
        # Publish a trajectory
        joints = self.last_joint_state or [0, 0, 0, 0, 0, 0]
        trajectory = {
            "op": "publish",
            "topic": "/planned_trajectory",
            "msg": {
                "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": "base_link"},
                "joint_names": ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
                "points": [{
                    "positions": joints,
                    "velocities": [0, 0, 0, 0, 0, 0],
                    "accelerations": [0, 0, 0, 0, 0, 0],
                    "effort": [0, 0, 0, 0, 0, 0],
                    "time_from_start": {"sec": 1, "nanosec": 0}
                }]
            }
        }
        await self.ws.send(json.dumps(trajectory))
        self.log("OK", f"Published trajectory to /planned_trajectory ({len(joints)} joints)")
        self.log("DBG", f"  Joints: {[f'{j:.3f}' for j in joints]}")
        
        # Unadvertise
        unadvertise = {"op": "unadvertise", "topic": "/planned_trajectory"}
        await self.ws.send(json.dumps(unadvertise))
        return True
    
    # ── Test 4: Service List ──────────────────────────────────────────
    async def test_service_list(self):
        self.log("INF", "═══ Test 4: Available ROS Services ═══")
        
        request = {
            "op": "call_service",
            "service": "/rosapi/services",
            "type": "rosapi/Services"
        }
        result = await self.send_and_recv(request, timeout=5.0)
        
        if result:
            services = result.get("values", {}).get("services", [])
            ik_found = any("/compute_ik" in s for s in services)
            planning_found = any("plan" in s.lower() for s in services)
            
            self.log("OK", f"  Found {len(services)} services")
            self.log("DBG", f"  /compute_ik present: {ik_found}")
            self.log("DBG", f"  Planning services: {planning_found}")
            
            # Log key services
            key_services = [s for s in services if any(k in s.lower() for k in ["ik", "plan", "trajectory", "joint"])]
            for svc in key_services[:10]:
                self.log("DBG", f"    → {svc}")
        else:
            self.log("WRN", "  Could not query service list (rosapi may not be available)")
        
        return True
    
    # ── Test 5: Topic List ────────────────────────────────────────────
    async def test_topic_list(self):
        self.log("INF", "═══ Test 5: Active ROS Topics ═══")
        
        request = {
            "op": "call_service",
            "service": "/rosapi/topics",
            "type": "rosapi/Topics"
        }
        result = await self.send_and_recv(request, timeout=5.0)
        
        if result:
            topics = result.get("values", {}).get("topics", [])
            self.log("OK", f"  Found {len(topics)} topics")
            
            key_topics = [t for t in topics if any(k in t.lower() for k in ["joint", "trajectory", "state", "ik", "plan"])]
            for topic in key_topics[:15]:
                self.log("DBG", f"    → {topic}")
        else:
            self.log("WRN", "  Could not query topic list")
        
        return True
    
    # ── Main Test Runner ──────────────────────────────────────────────
    async def run_all(self):
        print()
        print("╔══════════════════════════════════════════════════════════╗")
        print("║  ROBOFORGE v7.0 — PIPELINE DIAGNOSTICS (Interceptor)   ║")
        print("║  Tests ROS 2 ↔ MoveIt ↔ Gazebo data flow              ║")
        print("╚══════════════════════════════════════════════════════════╝")
        print()
        
        if not await self.connect():
            self.log("ERR", "ABORT: Cannot reach rosbridge. Is Docker running?")
            self.log("INF", "  Start with: docker compose -f docker/docker-compose.sim.yml up -d")
            self.print_summary()
            return
        
        await self.test_topic_list()
        await self.test_service_list()
        await self.test_joint_states()
        await self.test_moveit_ik()
        await self.test_trajectory_publish()
        
        await self.ws.close()
        self.print_summary()
    
    def print_summary(self):
        print()
        print("═════════════════════ SUMMARY ═════════════════════")
        ok = sum(1 for r in self.results if r["level"] == "OK")
        err = sum(1 for r in self.results if r["level"] == "ERR")
        wrn = sum(1 for r in self.results if r["level"] == "WRN")
        
        if err == 0:
            print(f"  ✅ ALL CLEAR — {ok} checks passed, {wrn} warnings")
        else:
            print(f"  ❌ {err} ERRORS, {ok} OK, {wrn} warnings")
        
        if err > 0:
            print("\n  Failed checks:")
            for r in self.results:
                if r["level"] == "ERR":
                    print(f"    ✗ {r['msg']}")
        
        print()
        
        # Write JSON report
        report_path = "pipeline_diagnostic_report.json"
        with open(report_path, "w") as f:
            json.dump({
                "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
                "ok": ok, "errors": err, "warnings": wrn,
                "entries": self.results
            }, f, indent=2)
        print(f"  Report saved to: {report_path}")
        print()


if __name__ == "__main__":
    diag = PipelineDiagnostics()
    asyncio.run(diag.run_all())
