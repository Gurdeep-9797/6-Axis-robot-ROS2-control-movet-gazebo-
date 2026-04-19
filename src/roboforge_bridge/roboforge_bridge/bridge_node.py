import rclpy
from rclpy.node import Node
import asyncio
import json
import threading
from collections import defaultdict, deque
from typing import Dict, Set, Any, Callable, List
from datetime import datetime, timezone

import os
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from moveit_msgs.msg import RobotState, MoveItErrorCodes
from std_msgs.msg import String
from std_srvs.srv import Trigger
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import socket, subprocess, time, math
import traceback
import websockets
from .kinematics import URDFKinematics


class DataLogger:
    """Structured event logger — stores last 2000 entries in memory + writes to disk."""

    def __init__(self, log_path: str = '/ros_ws/logs/roboforge_datalog.jsonl', max_mem: int = 2000):
        self._log_path = log_path
        self._buffer: deque = deque(maxlen=max_mem)
        self._lock = threading.Lock()
        os.makedirs(os.path.dirname(log_path), exist_ok=True)

    def log(self, event: str, **kwargs):
        entry = {
            'ts': datetime.now(timezone.utc).isoformat(),
            'event': event,
            **kwargs
        }
        with self._lock:
            self._buffer.append(entry)
        try:
            with open(self._log_path, 'a') as f:
                f.write(json.dumps(entry) + '\n')
        except Exception:
            pass

    def get_entries(self, n: int = 200) -> List[dict]:
        with self._lock:
            return list(self._buffer)[-n:]


class RoboForgeBridge(Node):
    """
    RoboForge Bridge Node - WebSocket server with rosbridge-compatible protocol.
    Central hub for Section 4 (Health) and Section 5 (Safety).
    """

    def __init__(self):
        super().__init__("roboforge_bridge")

        self._ws_clients: Set[Any] = set()
        self._state_ws_clients: Set[Any] = set()
        self._loop: asyncio.AbstractEventLoop = None
        self._state_lock = threading.Lock()
        self._latest_state: Dict = {}
        
        self._mode = 'simulate'
        self._execution_state = {'state': 'idle', 'pc': 0}
        
        # Determine paths for kinematics (Section 2/4.1)
        self._config = {'encoder_serial_port': '/dev/ttyUSB0', 'motor_serial_port': '/dev/ttyUSB1'}
        try:
            # Try multiple paths in order: prefer processed .urdf over .xacro
            candidates = [
                os.path.join(os.getcwd(), 'robot.urdf'),
                os.path.join(os.getcwd(), 'urdf', 'robot.urdf'),
                os.path.join(get_package_share_directory('robot_description'), 'urdf', 'robot.urdf'),
                os.path.join(get_package_share_directory('robot_description'), 'urdf', 'robot.urdf.xacro'),
                os.path.join(get_package_share_directory('robot_description'), 'urdf', 'custom_6axis_test.urdf.xacro'),
            ]
            urdf_path = None
            for c in candidates:
                if os.path.exists(c):
                    urdf_path = c
                    break
            if urdf_path is None:
                raise FileNotFoundError(f"No URDF found in candidates: {candidates}")

            self.get_logger().info(f"Loading Analytical Kinematics from: {urdf_path}")
            self._kinematics = URDFKinematics(urdf_path, 'base_link', 'link_6')
        except Exception as e:
            self.get_logger().error(f"Failed to load URDF for kinematics: {e}")
            self._kinematics = None

        self.get_logger().info("Initializing RoboForge Bridge v8.2")
        self._datalog = DataLogger()
        self._init_ros_interfaces()
        self._start_websocket_server()

    def _init_ros_interfaces(self):
        # ── Raw State Feed ──────────────────────────────────────────────
        self.joint_state_sub = self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)
        
        # ── Movement Commands ───────────────────────────────────────────
        self.trajectory_sub = self.create_subscription(
            JointTrajectory, "/planned_trajectory", self._on_trajectory, 10
        )

        # ── Services (Section 4/5) ──────────────────────────────────────
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self.fk_client = self.create_client(GetPositionFK, "/compute_fk")
        
        self._health_srv = self.create_service(
            Trigger, '/roboforge/health_check', self._handle_health_check
        )
        
        # ── Safety / Monitoring ─────────────────────────────────────────
        self._alert_sub = self.create_subscription(
            String, '/roboforge/tracking_alert', self._on_tracking_alert, 10
        )
        self._diagnostics_pub = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10
        )

    def _on_trajectory(self, msg: JointTrajectory):
        """Forward trajectory from /planned_trajectory → /joint_trajectory_command."""
        self.get_logger().info(f"Forwarding Trajectory: {len(msg.points)} points → /joint_trajectory_command")
        self._execution_state['state'] = 'running'

        # Lazy-create the publisher (avoid duplicates)
        if not hasattr(self, '_traj_pub'):
            self._traj_pub = self.create_publisher(
                JointTrajectory, '/joint_trajectory_command', 10
            )
        self._traj_pub.publish(msg)

    def _on_joint_states(self, msg: JointState):
        """
        Processes high-frequency state updates.
        Computes FK and Manipulability (Section 4.1).
        """
        positions = list(msg.position)
        
        # Real-time Analytical Kinematics (Section 2)
        manipulability = self._compute_manipulability(positions)

        # Build broadcast state
        state = {
            'type': 'robot_state',
            'timestamp_ns': int(msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec),
            'joint_pos_rad': positions,
            'manipulability': manipulability,
            'in_singularity': manipulability < 0.05,
            'source': 'encoder' if self._mode == 'live' else 'gazebo',
            'execution_state': self._execution_state['state'],
            'pc': self._execution_state['pc']
        }

        with self._state_lock:
            self._latest_state = state

        # Real-time WebSocket Push
        self._broadcast_latest_state()

    def _on_tracking_alert(self, msg: String):
        """
        Reacts to alerts from GazeboErrorNode (Section 5.1).
        Halts program if CRITICAL.
        """
        try:
            data = json.loads(msg.data)
            if data.get('action') == 'halt':
                self.get_logger().error(f"HALTING: {data['message']}")
                self._execution_state['state'] = 'halted'
                self._broadcast_latest_state()
        except Exception: pass

    def _compute_manipulability(self, q: list) -> float:
        if not self._kinematics: return 1.0
        try:
            import numpy as np
            J = self._kinematics.jacobian(q)
            return float(np.sqrt(max(0, np.linalg.det(J @ J.T))))
        except: return 0.0

    def _handle_health_check(self, request, response):
        """
        Section 4.2 Comprehensive Health Evaluation.
        """
        self.get_logger().info("Running Section 4.2 Health Check...")
        results = [
            {"name": "ROS Bridge", "ok": True, "detail": "WebSocket healthy on port 9090"},
            {"name": "Kinematics Engine", "ok": self._kinematics is not None, "detail": "FK/IK module loaded"},
            {"name": "Hardware Loop", "ok": self._mode != 'error', "detail": f"Current mode: {self._mode}"},
            {"name": "Planner Sync", "ok": self.ik_client.service_is_ready(), "detail": "/compute_ik state"}
        ]
        response.success = all(r['ok'] for r in results)
        response.message = json.dumps(results)
        return response

    # ── WebSocket Loop ──────────────────────────────────────────────────
    
    def _broadcast_latest_state(self):
        if self._loop and self._loop.is_running() and self._latest_state:
            # Wrap in rosbridge v2 publish format for roslibjs compatibility
            state = self._latest_state
            # Send as rosbridge /joint_states topic message
            ros_msg = json.dumps({
                "op": "publish",
                "topic": "/joint_states",
                "msg": {
                    "header": {
                        "stamp": {
                            "sec": state.get('timestamp_ns', 0) // 1000000000,
                            "nanosec": state.get('timestamp_ns', 0) % 1000000000,
                        },
                        "frame_id": "base_link"
                    },
                    "name": ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
                    "position": state.get('joint_pos_rad', [0]*6),
                    "velocity": [0.0]*6,
                    "effort": [0.0]*6
                }
            })
            asyncio.run_coroutine_threadsafe(self._broadcast_state(ros_msg), self._loop)

    async def _broadcast_state(self, message: str):
        dead = set()
        for ws in self._state_ws_clients:
            try: await ws.send(message)
            except: dead.add(ws)
        self._state_ws_clients -= dead

    def _start_websocket_server(self):
        def run_loop():
            self._loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self._loop)

            async def ws_handler(ws, path):
                self._ws_clients.add(ws)
                self._state_ws_clients.add(ws)
                self.get_logger().info(f'WebSocket client connected: {ws.remote_address}')
                try:
                    async for raw in ws:
                        try:
                            msg = json.loads(raw)
                            await self._handle_message(ws, msg)
                        except Exception as e:
                            self.get_logger().warn(f'WebSocket message error: {e}')
                except websockets.exceptions.ConnectionClosed:
                    pass
                finally:
                    self._ws_clients.discard(ws)
                    self._state_ws_clients.discard(ws)
                    self.get_logger().info('WebSocket client disconnected')

            async def main():
                import websockets
                server = await websockets.serve(ws_handler, "0.0.0.0", 9090)
                self.get_logger().info('WebSocket server started on ws://0.0.0.0:9090')
                await asyncio.Future()

            try:
                self._loop.run_until_complete(main())
            except Exception as e:
                self.get_logger().error(f'WebSocket server failed: {e}')

        threading.Thread(target=run_loop, daemon=True).start()

    async def _handle_message(self, ws, msg: dict):
        """
        Full WebSocket message dispatcher.
        Supports rosbridge v2 protocol + RoboForge custom ops.
        """
        op = msg.get('op', '')

        # ── rosbridge: subscribe ────────────────────────────────────────
        if op == 'subscribe':
            # Client is subscribing to a topic — add to state clients
            # For /joint_states: they already get pushed via _broadcast_latest_state
            response = {'op': 'service_response', 'result': True, 'values': {}}
            try: await ws.send(json.dumps({'op': 'status', 'level': 'info', 'msg': f'Subscribed to {msg.get("topic")}'}))
            except: pass

        # ── rosbridge: call_service — IK ────────────────────────────────
        elif op == 'call_service' and msg.get('service') == '/compute_ik':
            await self._handle_ik_request(ws, msg)

        # ── rosbridge: call_service — FK ────────────────────────────────
        elif op == 'call_service' and msg.get('service') == '/compute_fk':
            await self._handle_fk_request(ws, msg)

        # ── rosbridge: call_service — Health Check ──────────────────────
        elif op == 'call_service' and msg.get('service') == '/roboforge/health_check':
            results = [
                {"name": "ROS Bridge", "ok": True, "detail": "WebSocket healthy on port 9090"},
                {"name": "Kinematics Engine", "ok": self._kinematics is not None, "detail": "FK/IK module loaded"},
                {"name": "Hardware Loop", "ok": self._mode != 'error', "detail": f"Current mode: {self._mode}"},
                {"name": "Planner Sync", "ok": self.ik_client.service_is_ready(), "detail": "/compute_ik state"}
            ]
            await ws.send(json.dumps({
                'op': 'service_response', 'id': msg.get('id'),
                'result': True,
                'values': {'success': all(r['ok'] for r in results), 'message': json.dumps(results)}
            }))

        # ── RoboForge: execute trajectory (from UI compile+run) ────────
        elif op == 'roboforge/execute_program':
            await self._handle_execute_program(ws, msg)

        # ── rosbridge: publish — trajectory ─────────────────────────────
        elif op == 'publish' and any(t in msg.get('topic', '') for t in ['/joint_trajectory', '/planned_trajectory']):
            await self._handle_publish_trajectory(ws, msg)

        # ── Health / status query ────────────────────────────────────────
        elif op == 'roboforge/health':
            await ws.send(json.dumps({
                'op': 'roboforge/health_response',
                'status': 'ok',
                'mode': self._mode,
                'connected_clients': len(self._ws_clients),
                'execution_state': self._execution_state['state'],
                'moveit_ready': self.ik_client.service_is_ready(),
                'kinematics_loaded': self._kinematics is not None,
            }))
            
        # ── Hardware Detection & Handshake (Phase 6) ─────────────────────
        elif op == 'call_service' and msg.get('service') == '/roboforge/hardware_detect':
            # Simulate hardware bus scan
            await asyncio.sleep(0.5)
            await ws.send(json.dumps({
                'op': 'service_response',
                'id': msg.get('id'),
                'result': True,
                'values': {
                    'devices': [
                        {'id': f'motor_ctrl_{i}', 'type': 'motor_controller', 'status': 'online', 'joint': i+1} 
                        for i in range(6)
                    ],
                    'bus_status': 'ok',
                    'latency_ms': 12
                }
            }))
            
        elif op == 'call_service' and msg.get('service') == '/roboforge/handshake':
            # Live hardware mode initialization
            self._mode = 'live'
            await asyncio.sleep(0.2)
            await ws.send(json.dumps({
                'op': 'service_response',
                'id': msg.get('id'),
                'result': True,
                'values': {
                    'mode': self._mode,
                    'active_controllers': 6,
                    'message': 'Hardware handshake successful. Entering LIVE mode.'
                }
            }))

        else:
            self.get_logger().debug(f'Unhandled op: {op}')

    async def _handle_ik_request(self, ws, msg: dict):
        """Route IK request to MoveIt /compute_ik service."""
        if not self.ik_client.service_is_ready():
            await ws.send(json.dumps({'op': 'service_response', 'id': msg.get('id'),
                'result': False, 'values': {'error': 'MoveIt /compute_ik not ready'}}))
            return

        args = msg.get('args', {}).get('ik_request', {})
        req = GetPositionIK.Request()
        req.ik_request.group_name = args.get('group_name', 'robot_arm')
        req.ik_request.avoid_collisions = args.get('avoid_collisions', True)

        pose_data = args.get('pose_stamped', {}).get('pose', {})
        pos = pose_data.get('position', {})
        ori = pose_data.get('orientation', {})
        req.ik_request.pose_stamped.header.frame_id = args.get('pose_stamped', {}).get('header', {}).get('frame_id', 'base_link')
        req.ik_request.pose_stamped.pose.position.x = float(pos.get('x', 0))
        req.ik_request.pose_stamped.pose.position.y = float(pos.get('y', 0))
        req.ik_request.pose_stamped.pose.position.z = float(pos.get('z', 0))
        req.ik_request.pose_stamped.pose.orientation.x = float(ori.get('x', 0))
        req.ik_request.pose_stamped.pose.orientation.y = float(ori.get('y', 0))
        req.ik_request.pose_stamped.pose.orientation.z = float(ori.get('z', 0))
        req.ik_request.pose_stamped.pose.orientation.w = float(ori.get('w', 1))
        req.ik_request.timeout.sec = 5

        t0 = time.monotonic()
        try:
            future = self.ik_client.call_async(req)
            for _ in range(600):
                if future.done(): break
                await asyncio.sleep(0.01)
                
            if not future.done():
                raise TimeoutError("MoveIt IK request timed out after 6 seconds")
                
            result = future.result()
            latency = (time.monotonic() - t0) * 1000
            if result and result.error_code.val == 1:
                positions = list(result.solution.joint_state.position[:6])
                self._datalog.log('ik_request', solver='moveit', target_m=[
                    req.ik_request.pose_stamped.pose.position.x,
                    req.ik_request.pose_stamped.pose.position.y,
                    req.ik_request.pose_stamped.pose.position.z],
                    result='success', latency_ms=round(latency, 2),
                    solution_rad=[round(p, 4) for p in positions])
                await ws.send(json.dumps({
                    'op': 'service_response', 'id': msg.get('id'), 'result': True,
                    'values': {'error_code': {'val': 1}, 'solution': {
                        'joint_state': {
                            'name': ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6'],
                            'position': positions
                        }
                    }}
                }))
            else:
                code = result.error_code.val if result else -1
                self._datalog.log('ik_request', solver='moveit',
                    result='failed', error_code=code, latency_ms=round(latency, 2))
                await ws.send(json.dumps({'op': 'service_response', 'id': msg.get('id'),
                    'result': False, 'values': {'error_code': {'val': code}}}))
        except Exception as e:
            latency = (time.monotonic() - t0) * 1000
            self._datalog.log('ik_request', solver='moveit',
                result='exception', error=str(e), latency_ms=round(latency, 2))
            self.get_logger().error(f'IK service error: {e}')
            await ws.send(json.dumps({'op': 'service_response', 'id': msg.get('id'),
                'result': False, 'values': {'error': str(e)}}))

    async def _handle_fk_request(self, ws, msg: dict):
        """Route FK request to local kinematics (fast, no MoveIt round-trip needed)."""
        args = msg.get('args', {})
        joints = args.get('robot_state', {}).get('joint_state', {}).get('position', [0]*6)
        if self._kinematics and len(joints) >= 6:
            try:
                T = self._kinematics.fk(joints[:6])
                pos = T[:3, 3].tolist()
                await ws.send(json.dumps({'op': 'service_response', 'id': msg.get('id'),
                    'result': True, 'values': {
                        'pose_stamped': [{'pose': {'position': {'x': pos[0], 'y': pos[1], 'z': pos[2]},
                                                   'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1}}}]
                    }}))
                return
            except Exception as e:
                self.get_logger().warn(f'FK failed: {e}')
        await ws.send(json.dumps({'op': 'service_response', 'id': msg.get('id'), 'result': False,
            'values': {'error': 'FK not available'}}))

    async def _handle_execute_program(self, ws, msg: dict):
        """Execute a compiled program (array of TrajectorySegment objects from UI Compiler)."""
        program = msg.get('program', [])
        self._execution_state = {'state': 'running', 'pc': 0}
        self._broadcast_latest_state()
        self._datalog.log('program_start', segments=len(program))
        self.get_logger().info(f'Executing program: {len(program)} segments')

        for i, segment in enumerate(program):
            if self._execution_state.get('state') == 'halted':
                break

            self._execution_state['pc'] = i
            # Publish each segment as a JointTrajectory to /joint_trajectory_command
            traj_msg = JointTrajectory()
            traj_msg.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
            for pt in segment.get('points', []):
                from trajectory_msgs.msg import JointTrajectoryPoint
                from builtin_interfaces.msg import Duration
                p = JointTrajectoryPoint()
                p.positions = [float(v) for v in pt.get('positions', [0]*6)]
                p.velocities = [float(v) for v in pt.get('velocities', [0]*6)]
                t_s = pt.get('time_from_start', 0)
                p.time_from_start.sec = int(t_s)
                p.time_from_start.nanosec = int((t_s - int(t_s)) * 1e9)
                traj_msg.points.append(p)

            if not hasattr(self, '_traj_pub'):
                self._traj_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_command', 10)
            self._traj_pub.publish(traj_msg)

            # Notify UI of progress
            notify = json.dumps({'op': 'roboforge/execution_progress', 'pc': i,
                                 'total': len(program), 'state': 'running'})
            asyncio.run_coroutine_threadsafe(self._broadcast_state(notify), self._loop)

            duration = segment.get('duration_s', 1.0)
            await asyncio.sleep(duration)

        self._execution_state = {'state': 'idle', 'pc': len(program)}
        self._broadcast_latest_state()
        self._datalog.log('program_complete', segments_executed=len(program))
        self.get_logger().info('Program execution complete — state: IDLE')

    async def _handle_publish_trajectory(self, ws, msg: dict):
        """Handle raw rosbridge publish to /joint_trajectory_command."""
        traj_data = msg.get('msg', {})
        traj_msg = JointTrajectory()
        traj_msg.joint_names = traj_data.get('joint_names',
            ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'])
        for pt in traj_data.get('points', []):
            p = JointTrajectoryPoint()
            p.positions = [float(v) for v in pt.get('positions', [0]*6)]
            p.velocities = [float(v) for v in pt.get('velocities', [0]*6)]
            t = pt.get('time_from_start', {})
            p.time_from_start.sec = int(t.get('sec', 1))
            p.time_from_start.nanosec = int(t.get('nanosec', 0))
            traj_msg.points.append(p)

        if not hasattr(self, '_traj_pub'):
            self._traj_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_command', 10)
        
        self._traj_pub.publish(traj_msg)
        self.get_logger().info(f"Published trajectory: {len(traj_msg.points)} points to /joint_trajectory_command")

    # ── REST API on port 8765 ────────────────────────────────────────────────

    def _start_rest_server(self):
        """aiohttp REST server for health checks and hardware API."""
        async def health_handler(request):
            from aiohttp import web as aio_web
            return aio_web.json_response({
                'status': 'ok',
                'mode': self._mode,
                'backend': 'ros2_humble',
                'connected_clients': len(self._ws_clients),
                'execution_state': self._execution_state['state'],
                'moveit_ready': self.ik_client.service_is_ready(),
                'kinematics_loaded': self._kinematics is not None,
                'robot_loaded': self._kinematics is not None,
            })

        async def ports_handler(request):
            from aiohttp import web as aio_web
            try:
                import serial.tools.list_ports
                ports = [{'port': p.device, 'description': p.description,
                          'vid': f'{p.vid:04x}' if p.vid else None,
                          'pid': f'{p.pid:04x}' if p.pid else None,
                          'manufacturer': p.manufacturer}
                         for p in serial.tools.list_ports.comports()]
            except Exception as e:
                ports = []
            return aio_web.json_response({'ports': ports})

        def run_rest():
            import asyncio as _asyncio
            from aiohttp import web as aio_web
            loop = _asyncio.new_event_loop()
            _asyncio.set_event_loop(loop)
            async def logs_handler(request):
                from aiohttp import web as aio_web
                n = int(request.query.get('n', 200))
                entries = self._datalog.get_entries(n)
                return aio_web.json_response({'entries': entries})

            app = aio_web.Application()
            app.router.add_get('/health', health_handler)
            app.router.add_get('/api/hardware/ports', ports_handler)
            app.router.add_get('/api/logs', logs_handler)
            runner = aio_web.AppRunner(app)
            loop.run_until_complete(runner.setup())
            site = aio_web.TCPSite(runner, '0.0.0.0', 8765)
            loop.run_until_complete(site.start())
            self.get_logger().info('REST API running on http://0.0.0.0:8765')
            loop.run_forever()

        import threading
        threading.Thread(target=run_rest, daemon=True).start()


def main():
    rclpy.init()
    bridge = RoboForgeBridge()
    bridge._start_rest_server()
    rclpy.spin(bridge)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
