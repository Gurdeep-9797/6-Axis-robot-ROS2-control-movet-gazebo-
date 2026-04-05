# ROBOFORGE v7.0 — UNIFIED ARCHITECTURE BIBLE
## Online ↔ Offline Mirror, Shared ROS2/MoveIt/Gazebo Backend, Full 3D Viewport, and Real Pipeline

---

> **AI IMPLEMENTATION NOTE:** This document is the **single source of truth** for RoboForge v7.0. Every section has a corresponding artifact in code. Nothing here is aspirational — everything has a concrete implementation path. When you implement any feature, trace its lineage through this document from Section 1 (Pipeline) → Section 2 (Backend) → Section 3 (Online UI) → Section 4 (Offline UI) → Section 5 (3D Viewport) → Section 6 (Sync Engine). Do not skip sections. Do not simplify the pipeline. Follow every step as written.

---

## SECTION 0 — ROOT CAUSE ANALYSIS & FIX

### 0.1 Why `ws://localhost:9090` fails on Vercel

When the browser runs on **any machine other than the user's local PC** (Vercel CI, a remote subagent's cloud VM, another user's browser), `localhost` resolves to **that machine**, not the user's Docker stack. The WebSocket handshake immediately closes. This is not a bug — it is correct browser security behavior.

**Three-layer fix required:**

```
Layer A: Teach the app to detect its own environment
Layer B: Route traffic correctly per environment
Layer C: Expose ROS 2 safely over the network when needed
```

### 0.2 Why IK returns `NaNmm`

The default boilerplate waypoints `{X:450, Y:200, Z:300}` in millimeters are **not** expressed in the robot's base frame correctly. The offline JS IK solver expects the target in meters in ROS convention `(X forward, Y left, Z up)`. The boilerplate passes raw UI millimeter values directly without the `÷1000` conversion, causing floating-point division that produces `NaN` when the solver's `acos` argument exceeds `[-1, 1]` due to unnormalized input.

**Exact fix (JS IK Solver):**

```typescript
// BEFORE (broken):
const target = { x: block.x, y: block.y, z: block.z }; // raw mm

// AFTER (correct):
const target = {
  x: block.x / 1000.0,   // mm → meters
  y: block.y / 1000.0,
  z: block.z / 1000.0,
};

// THEN validate before passing to solver:
const reach = Math.sqrt(target.x**2 + target.y**2 + target.z**2);
const MAX_REACH_M = 2.55; // ABB IRB 6700 max reach
const MIN_REACH_M = 0.35;
if (reach > MAX_REACH_M || reach < MIN_REACH_M) {
  console.error(`IK target unreachable: reach=${(reach*1000).toFixed(1)}mm, must be ${MIN_REACH_M*1000}–${MAX_REACH_M*1000}mm`);
  return null;
}
```

**Safe default boilerplate values** (inside reachable workspace):

```typescript
const DEFAULT_PROGRAM = [
  { type: 'MoveJ', label: 'Home',     x: 0,   y: 0,   z: 1500, rx: 0, ry: 0, rz: 0, v: 100, z_blend: 0 },
  { type: 'MoveL', label: 'Approach', x: 500,  y: 0,   z: 1200, rx: 0, ry: 180, rz: 0, v: 50, z_blend: 5 },
  { type: 'MoveL', label: 'Pick',     x: 500,  y: 0,   z: 900,  rx: 0, ry: 180, rz: 0, v: 20, z_blend: 0 },
];
// All values in mm, converted ÷1000 before IK call
```

---

## SECTION 1 — FULL PIPELINE: EVERY VALUE, EVERY WIRE

### 1.1 Pipeline Overview

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    ROBOFORGE PROGRAMMING PIPELINE                        │
│                                                                          │
│  ┌───────────┐   ①    ┌────────────┐   ②    ┌───────────────────────┐  │
│  │  UI Layer │──────▶│  Compiler  │──────▶ │  Motion Intent Graph  │  │
│  │ (Block/   │ Block  │  (Parser + │  IR    │  (DAG of motion ops)  │  │
│  │  Code)    │ JSON   │  IR Gen)   │ Ops    │                       │  │
│  └───────────┘        └────────────┘        └──────────┬────────────┘  │
│                                                         │ ③             │
│  ┌───────────────────────────────────────────────────── ▼ ──────────┐  │
│  │              CONSTRAINT RESOLVER                                   │  │
│  │  • Joint limits   • Velocity caps   • Workspace bounds           │  │
│  │  • Collision clearance (AABB pre-check)                          │  │
│  └───────────────────────────────────────────────────── ┬ ──────────┘  │
│                                                          │ ④             │
│          ┌───────────────────────────────────────────── ▼ ──────────┐  │
│          │                   IK SOLVER CHAIN                         │  │
│          │                                                            │  │
│          │  ┌─────────────────┐    fail    ┌──────────────────────┐  │  │
│          │  │  MoveIt 2 IK    │──────────▶│  Offline Numerical IK│  │  │
│          │  │  (via rosbridge │            │  (DLS, JS/C# impl.)  │  │  │
│          │  │   WebSocket)    │            └──────────────────────┘  │  │
│          │  └────────┬────────┘                                       │  │
│          │           │ success                                         │  │
│          └───────────┼─────────────────────────────────────────────── ┘  │
│                      │ ⑤ Joint angles [q1..q6] + timestamps              │
│                      ▼                                                    │
│          ┌────────────────────────────────────────────┐                  │
│          │         TRAJECTORY GENERATOR               │                  │
│          │  Quintic polynomial (MoveJ)                │                  │
│          │  Cartesian linear + SLERP (MoveL)          │                  │
│          │  Circular arc + SLERP (MoveC)              │                  │
│          │  Zone blending (quadratic Bézier)          │                  │
│          └─────────────────────┬──────────────────────┘                  │
│                                │ ⑥ JointTrajectory message              │
│                                ▼                                          │
│          ┌────────────────────────────────────────────┐                  │
│          │         ROS 2 BRIDGE                        │                  │
│          │  Topic: /joint_trajectory_command           │                  │
│          │  Type: trajectory_msgs/JointTrajectory      │                  │
│          └─────────────────────┬──────────────────────┘                  │
│                                │ ⑦ DDS (FastDDS)                        │
│                      ┌─────────▼──────────┐                             │
│                      │   MoveIt 2          │                             │
│                      │  move_group node   │◀── planning_scene           │
│                      └─────────┬──────────┘                             │
│                                │ ⑧ Planned trajectory                   │
│                      ┌─────────▼──────────┐                             │
│                      │    Gazebo Ignition  │                             │
│                      │  Physics Engine    │                             │
│                      │  (250 Hz sim step) │                             │
│                      └─────────┬──────────┘                             │
│                                │ ⑨ /joint_states (50 Hz feedback)       │
│                      ┌─────────▼──────────┐                             │
│                      │  State Aggregator   │                             │
│                      │  FK → TCP pose     │                             │
│                      │  Singularity check │                             │
│                      └─────────┬──────────┘                             │
│                                │ ⑩ RobotStateBuffer → UI render        │
│                                ▼                                          │
│          ┌────────────────────────────────────────────┐                  │
│          │     3D VIEWPORT (Three.js / DirectX 12)    │                  │
│          │  Animated joint positions, path ghost,     │                  │
│          │  TCP trace, diagnostic overlays            │                  │
│          └────────────────────────────────────────────┘                  │
└─────────────────────────────────────────────────────────────────────────┘
```

### 1.2 Exact Data Contracts Between Every Pipeline Stage

#### Stage ① → ② : Block JSON → Compiler Input

```typescript
// Every block in the program tree serializes to this:
interface BlockNode {
  id: string;                    // UUID v4
  type: 'MoveJ'|'MoveL'|'MoveC'|'MoveAbsJ'|'SetDO'|'GetDI'|
        'If'|'While'|'For'|'Call'|'Wait'|'Comment'|'SyncBarrier';
  label: string;
  params: MotionParams | IOParams | LogicParams | SyncParams;
  children?: BlockNode[];        // for If/While/For bodies
  enabled: boolean;
  breakpoint: boolean;
}

interface MotionParams {
  // Target specification (UI stores mm, degrees)
  targetType: 'cartesian' | 'joint';
  x_mm: number; y_mm: number; z_mm: number;
  rx_deg: number; ry_deg: number; rz_deg: number;
  // OR joint target:
  j1_deg: number; j2_deg: number; j3_deg: number;
  j4_deg: number; j5_deg: number; j6_deg: number;
  // Motion parameters
  speed_pct: number;             // 1-100, % of max joint speed
  accel_pct: number;             // 1-100
  zone_mm: number;               // blend zone radius in mm (0 = fine)
  tool_id: string;               // reference to tool frame
  wobj_id: string;               // reference to work object frame
  conf_str: string;              // configuration string "cfx 0 0 0 0"
}
```

#### Stage ② → ③ : IR Instructions

```typescript
enum IROpcode {
  MOVE_J = 'MOVE_J',           // joint-space motion
  MOVE_L = 'MOVE_L',           // linear Cartesian motion
  MOVE_C = 'MOVE_C',           // circular arc
  SET_DO  = 'SET_DO',
  GET_DI  = 'GET_DI',
  WAIT    = 'WAIT',
  JUMP    = 'JUMP',            // unconditional (for loops)
  BRANCH  = 'BRANCH',          // conditional
  CALL    = 'CALL',
  RETURN  = 'RETURN',
  SYNC    = 'SYNC',            // multi-robot barrier
  NOP     = 'NOP',
}

interface IRInstruction {
  addr: number;                  // program counter address
  op: IROpcode;
  // For motion ops:
  target_m?: [number,number,number];     // meters, ROS convention
  target_quat?: [number,number,number,number]; // xyzw quaternion
  joint_rad?: number[];          // joint angles in radians (if joint target)
  speed_frac: number;            // 0.0–1.0
  accel_frac: number;
  zone_m: number;                // blend zone in meters
  tool_frame?: SE3;
  wobj_frame?: SE3;
  // For logic ops:
  condition?: string;            // evaluated expression
  jump_addr?: number;
  // Metadata:
  source_block_id: string;       // back-reference for highlighting
}

type SE3 = { pos: [number,number,number]; quat: [number,number,number,number] };
```

#### Stage ③ → ④ : Constraint Resolver Output

```typescript
interface ConstraintResolverResult {
  feasible: boolean;
  violations: ConstraintViolation[];
  clamped_instructions: IRInstruction[];  // speed/accel clamped to limits
  warnings: string[];
}

interface ConstraintViolation {
  type: 'JOINT_LIMIT'|'VELOCITY'|'ACCELERATION'|'WORKSPACE'|'COLLISION';
  instruction_addr: number;
  detail: string;
  severity: 'error'|'warning';
}
```

#### Stage ④ → ⑤ : IK Result (per waypoint)

```typescript
interface IKResult {
  success: boolean;
  solver_used: 'moveit'|'offline_analytical'|'offline_numerical';
  joint_angles_rad: [number,number,number,number,number,number];
  error_mm: number;              // Cartesian position error of solution
  manipulability: number;        // Yoshikawa index (>0.05 = healthy)
  config: RobotConfig;
  solve_time_ms: number;
  failure_reason?: 'UNREACHABLE'|'SINGULARITY'|'JOINT_LIMIT'|'TIMEOUT';
}

interface RobotConfig {
  shoulder: 'front' | 'back';
  elbow: 'up' | 'down';
  wrist: 'no_flip' | 'flip';
}
```

#### Stage ⑤ → ⑥ : Trajectory (per motion segment)

```typescript
interface TrajectorySegment {
  block_id: string;
  motion_type: 'MoveJ'|'MoveL'|'MoveC';
  duration_s: number;
  dt_s: number;                  // control period (0.004 = 250Hz)
  points: TrajectoryPoint[];
  zone_entry_idx: number;        // index where zone blending begins
  zone_exit_idx: number;
}

interface TrajectoryPoint {
  t: number;                     // time from segment start (seconds)
  q: [number,number,number,number,number,number];    // joint angles (rad)
  qd: [number,number,number,number,number,number];   // joint velocities (rad/s)
  qdd: [number,number,number,number,number,number];  // joint accels (rad/s²)
  tcp_pos: [number,number,number];                   // meters
  tcp_quat: [number,number,number,number];           // xyzw
}
```

#### Stage ⑥ : ROS 2 Message Construction

```typescript
// Built from TrajectorySegment[] before publishing:
function buildJointTrajectoryMsg(segments: TrajectorySegment[]): RosMessage {
  const JOINT_NAMES = [
    'joint_1','joint_2','joint_3','joint_4','joint_5','joint_6'
  ];
  
  // Flatten all segments, offset timestamps
  let t_offset = 0;
  const points: RosTrajectoryPoint[] = [];
  
  for (const seg of segments) {
    for (const pt of seg.points) {
      points.push({
        positions:     Array.from(pt.q),
        velocities:    Array.from(pt.qd),
        accelerations: Array.from(pt.qdd),
        time_from_start: {
          sec:  Math.floor(t_offset + pt.t),
          nanosec: Math.round(((t_offset + pt.t) % 1) * 1e9)
        }
      });
    }
    t_offset += seg.duration_s;
  }

  return {
    header: { stamp: { sec: 0, nanosec: 0 }, frame_id: 'base_link' },
    joint_names: JOINT_NAMES,
    points
  };
}
```

#### Stage ⑧ → ⑨ : Gazebo → ROS 2 Feedback

```
Topic:    /joint_states
Type:     sensor_msgs/JointState
Rate:     50 Hz (configurable up to 1000 Hz in Gazebo plugin)
Fields:
  header.stamp     — simulation time
  name[]           — ['joint_1',...,'joint_6']
  position[]       — current joint angles (rad)
  velocity[]       — current joint velocities (rad/s)
  effort[]         — joint torques (Nm) from Gazebo physics
```

#### Stage ⑨ → ⑩ : State Aggregator → Render

```typescript
interface RobotStateBuffer {
  // Written by ROS subscriber thread, read by render thread
  // Lock-free double buffer (ping-pong)
  timestamp_ns: bigint;
  joint_pos_rad: Float64Array;   // length 6
  joint_vel_rads: Float64Array;  // length 6
  joint_torque_nm: Float64Array; // length 6
  tcp_pos_m: [number,number,number];
  tcp_quat: [number,number,number,number];
  tcp_vel_ms: [number,number,number];   // linear velocity m/s
  manipulability: number;
  in_singularity: boolean;
  singularity_type?: 'shoulder'|'elbow'|'wrist';
  program_counter: number;       // currently executing IR address
  execution_state: 'idle'|'running'|'paused'|'error';
  active_errors: DiagnosticEntry[];
}
```

### 1.3 MoveIt 2 IK Service Call (Online → rosbridge)

```typescript
// Called via rosbridge WebSocket (roslibjs)
async function callMoveItIK(
  target_pos: [number,number,number],
  target_quat: [number,number,number,number],
  seed_angles: number[]
): Promise<IKResult> {

  const request = {
    ik_request: {
      group_name: 'manipulator',
      robot_state: {
        joint_state: {
          name: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6'],
          position: seed_angles
        }
      },
      pose_stamped: {
        header: { frame_id: 'base_link' },
        pose: {
          position:    { x: target_pos[0], y: target_pos[1], z: target_pos[2] },
          orientation: { x: target_quat[0], y: target_quat[1],
                         z: target_quat[2], w: target_quat[3] }
        }
      },
      timeout: { sec: 5, nanosec: 0 },
      avoid_collisions: true
    }
  };

  return new Promise((resolve, reject) => {
    const svc = new ROSLIB.Service({
      ros: rosConnection,
      name: '/compute_ik',
      serviceType: 'moveit_msgs/GetPositionIK'
    });
    
    svc.callService(new ROSLIB.ServiceRequest(request), (result) => {
      if (result.error_code.val !== 1) {  // 1 = SUCCESS
        reject({ code: result.error_code.val });
        return;
      }
      resolve({
        success: true,
        solver_used: 'moveit',
        joint_angles_rad: result.solution.joint_state.position,
        error_mm: 0,
        manipulability: 1.0,
        config: parseConfig(result.solution),
        solve_time_ms: 0
      });
    }, reject);
  });
}
```

### 1.4 Gazebo Plugin Configuration (physics ↔ ROS 2)

```xml
<!-- In robot URDF/SDF — attach to each joint -->
<gazebo>
  <plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">
    <parameters>$(find roboforge_description)/config/ros2_control.yaml</parameters>
  </plugin>
</gazebo>

<!-- ros2_control.yaml -->
controller_manager:
  ros__parameters:
    update_rate: 250  # Hz — physics timestep

joint_state_broadcaster:
  ros__parameters:
    joints:
      - joint_1
      - joint_2
      - joint_3
      - joint_4
      - joint_5
      - joint_6

joint_trajectory_controller:
  ros__parameters:
    joints:
      - joint_1
      - joint_2
      - joint_3
      - joint_4
      - joint_5
      - joint_6
    command_interfaces: [position]
    state_interfaces:   [position, velocity, effort]
    interpolation_method: splines   # cubic spline between trajectory points
    allow_partial_joints_goal: false
    open_loop_control: false        # MUST be false: uses feedback
    allow_integration_in_goal_trajectories: true
    constraints:
      stopped_velocity_tolerance: 0.01
      goal_time: 0.5
      joint_1: { trajectory: 0.05, goal: 0.01 }
      joint_2: { trajectory: 0.05, goal: 0.01 }
      joint_3: { trajectory: 0.05, goal: 0.01 }
      joint_4: { trajectory: 0.05, goal: 0.01 }
      joint_5: { trajectory: 0.05, goal: 0.01 }
      joint_6: { trajectory: 0.05, goal: 0.01 }
```

---

## SECTION 2 — SHARED BACKEND ARCHITECTURE

### 2.1 The Single Backend Principle

```
┌──────────────────────────────────────────────────────────────────────┐
│                     ROBOFORGE BACKEND STACK                           │
│                   (IDENTICAL for Online & Offline)                    │
│                                                                        │
│  ┌───────────────────────────────────────────────────────────────┐   │
│  │                  USER'S LOCAL MACHINE                          │   │
│  │                                                                │   │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐  │   │
│  │  │  ROS 2      │  │  MoveIt 2   │  │  Gazebo Ignition     │  │   │
│  │  │  Humble     │  │  move_group │  │  Fortress            │  │   │
│  │  │  (Docker)   │  │  (Docker)   │  │  (Docker)            │  │   │
│  │  └──────┬──────┘  └──────┬──────┘  └──────────┬──────────┘  │   │
│  │         │ DDS/FastDDS    │                     │             │   │
│  │  ┌──────▼──────────────────────────────────────▼──────────┐  │   │
│  │  │              roboforge_bridge node                       │  │   │
│  │  │  • rosbridge_server (WebSocket :9090)                    │  │   │
│  │  │  • REST API server   (:8765)                             │  │   │
│  │  │  • gRPC server       (:50051)                            │  │   │
│  │  │  • State broadcaster (WebSocket :9091) — push-only       │  │   │
│  │  └──────┬────────────────────────────────────────┬─────────┘  │   │
│  │         │                                        │             │   │
│  │  ┌──────▼──────┐                      ┌─────────▼─────────┐  │   │
│  │  │  ONLINE UI  │                      │   OFFLINE UI       │  │   │
│  │  │  (Browser)  │                      │   (.NET/WPF)       │  │   │
│  │  │  Vite+React │                      │   WinUI 3          │  │   │
│  │  └─────────────┘                      └───────────────────┘  │   │
│  └───────────────────────────────────────────────────────────────┘   │
│                                                                        │
│  OPTIONAL: Remote Access via Secure Tunnel                            │
│  ┌─────────────────────────────────────────────────────────────────┐ │
│  │  cloudflared tunnel → exposes :9090, :8765, :9091               │ │
│  │  → User can access from any machine / Vercel-deployed frontend   │ │
│  └─────────────────────────────────────────────────────────────────┘ │
└──────────────────────────────────────────────────────────────────────┘
```

### 2.2 The `roboforge_bridge` Node (Critical Component)

This is a single ROS 2 node that acts as the **universal adapter** between the backend and both frontends.

```python
# roboforge_bridge/roboforge_bridge/bridge_node.py

import rclpy
from rclpy.node import Node
import asyncio
import websockets
import json
import grpc
from concurrent import futures
import threading
from aiohttp import web

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from moveit_msgs.msg import RobotState
from std_msgs.msg import String

class RoboForgeBridge(Node):
    def __init__(self):
        super().__init__('roboforge_bridge')
        
        # ── ROS 2 Publishers ──────────────────────────────────────────
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_command',
            10
        )
        self.prog_state_pub = self.create_publisher(
            String, '/roboforge/program_state', 10
        )
        
        # ── ROS 2 Subscribers ─────────────────────────────────────────
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states',
            self._on_joint_states, 10
        )
        
        # ── Service Clients ───────────────────────────────────────────
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
        
        # ── State buffer (thread-safe) ────────────────────────────────
        self._state_lock = threading.Lock()
        self._latest_state: dict = {}
        
        # ── Connected WebSocket clients ───────────────────────────────
        self._ws_clients: set = set()
        
        self.get_logger().info('RoboForge Bridge initialized')
    
    def _on_joint_states(self, msg: JointState):
        """Receives Gazebo feedback at 50Hz, broadcasts to all UI clients."""
        state = {
            'type': 'joint_states',
            'timestamp_ns': int(msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec),
            'positions': list(msg.position),
            'velocities': list(msg.velocity),
            'efforts': list(msg.effort),
        }
        with self._state_lock:
            self._latest_state = state
        
        # Push to all connected WebSocket clients (both online and offline)
        asyncio.run_coroutine_threadsafe(
            self._broadcast(json.dumps(state)),
            self._loop
        )
    
    async def _broadcast(self, message: str):
        """Push state to ALL connected frontend clients simultaneously."""
        if not self._ws_clients:
            return
        dead = set()
        for ws in self._ws_clients:
            try:
                await ws.send(message)
            except websockets.exceptions.ConnectionClosed:
                dead.add(ws)
        self._ws_clients -= dead
    
    async def _ws_handler(self, websocket, path):
        """Handle both rosbridge protocol AND roboforge native protocol."""
        self._ws_clients.add(websocket)
        try:
            async for raw in websocket:
                msg = json.loads(raw)
                op = msg.get('op')
                
                if op == 'publish':
                    await self._handle_publish(msg)
                elif op == 'call_service':
                    result = await self._handle_service(msg)
                    await websocket.send(json.dumps(result))
                elif op == 'subscribe':
                    # Send current state immediately
                    with self._state_lock:
                        if self._latest_state:
                            await websocket.send(json.dumps(self._latest_state))
                elif op == 'roboforge/execute_program':
                    await self._execute_program(msg['program'], websocket)
        finally:
            self._ws_clients.discard(websocket)
    
    async def _execute_program(self, trajectory_segments: list, ws):
        """Receives compiled trajectory from UI, publishes to ROS 2."""
        for seg in trajectory_segments:
            traj_msg = self._build_trajectory_msg(seg)
            self.traj_pub.publish(traj_msg)
            await ws.send(json.dumps({
                'type': 'execution_status',
                'status': 'segment_sent',
                'block_id': seg['block_id']
            }))
    
    def _build_trajectory_msg(self, seg: dict) -> JointTrajectory:
        msg = JointTrajectory()
        msg.joint_names = ['joint_1','joint_2','joint_3',
                           'joint_4','joint_5','joint_6']
        for pt in seg['points']:
            p = JointTrajectoryPoint()
            p.positions     = pt['q']
            p.velocities    = pt['qd']
            p.accelerations = pt['qdd']
            sec = int(pt['t'])
            nsec = int((pt['t'] % 1) * 1e9)
            p.time_from_start.sec     = sec
            p.time_from_start.nanosec = nsec
            msg.points.append(p)
        return msg

# Entry point
def main():
    rclpy.init()
    node = RoboForgeBridge()
    
    loop = asyncio.new_event_loop()
    node._loop = loop
    
    def run_ros():
        rclpy.spin(node)
    
    threading.Thread(target=run_ros, daemon=True).start()
    
    # WebSocket server on :9090 (rosbridge-compatible) and :9091 (state push)
    start_server = websockets.serve(node._ws_handler, '0.0.0.0', 9090)
    loop.run_until_complete(start_server)
    loop.run_forever()
```

### 2.3 Docker Compose (Full Stack)

```yaml
# docker-compose.sim.yml
version: '3.8'

services:
  
  # ── ROS 2 Core ──────────────────────────────────────────────────
  ros_core:
    image: roboforge/ros2-humble:latest
    build:
      context: ./docker/ros2
      dockerfile: Dockerfile
    environment:
      - ROS_DOMAIN_ID=42
      - FASTRTPS_DEFAULT_PROFILES_FILE=/config/fastdds.xml
    volumes:
      - ./ros_ws:/ros_ws
      - ./config:/config
    networks:
      - roboforge_net
    command: >
      bash -c "
        source /opt/ros/humble/setup.bash &&
        cd /ros_ws &&
        colcon build --symlink-install &&
        source install/setup.bash &&
        ros2 launch roboforge_bringup roboforge.launch.py
      "
    healthcheck:
      test: ["CMD", "ros2", "node", "list"]
      interval: 5s
      timeout: 3s
      retries: 10

  # ── MoveIt 2 ─────────────────────────────────────────────────────
  moveit:
    image: roboforge/moveit2:latest
    depends_on:
      ros_core:
        condition: service_healthy
    environment:
      - ROS_DOMAIN_ID=42
    volumes:
      - ./ros_ws:/ros_ws
      - ./config:/config
    networks:
      - roboforge_net
    command: >
      bash -c "
        source /opt/ros/humble/setup.bash &&
        source /ros_ws/install/setup.bash &&
        ros2 launch roboforge_moveit move_group.launch.py
      "

  # ── Gazebo Ignition Fortress ──────────────────────────────────────
  gazebo:
    image: roboforge/gazebo-ignition:fortress
    depends_on:
      - moveit
    environment:
      - ROS_DOMAIN_ID=42
      - DISPLAY=${DISPLAY:-:99}   # headless in docker, :0 locally
      - IGN_GAZEBO_PHYSICS_ENGINE=ignition-physics-dartsim-plugin
    volumes:
      - ./ros_ws:/ros_ws
      - ./models:/models
      - /tmp/.X11-unix:/tmp/.X11-unix  # for GUI if needed
    networks:
      - roboforge_net
    command: >
      bash -c "
        source /opt/ros/humble/setup.bash &&
        source /ros_ws/install/setup.bash &&
        ros2 launch roboforge_simulation gazebo.launch.py
      "

  # ── RoboForge Bridge ──────────────────────────────────────────────
  bridge:
    build:
      context: ./docker/bridge
      dockerfile: Dockerfile
    depends_on:
      - gazebo
    ports:
      - "9090:9090"    # WebSocket (rosbridge-compatible)
      - "9091:9091"    # WebSocket (state push, read-only)
      - "8765:8765"    # REST API
      - "50051:50051"  # gRPC (for .NET offline client)
    environment:
      - ROS_DOMAIN_ID=42
    networks:
      - roboforge_net
    restart: unless-stopped

networks:
  roboforge_net:
    driver: bridge
```

### 2.4 Remote Access via Cloudflare Tunnel

```bash
# On user's machine — run ONCE to set up permanent tunnel
cloudflared tunnel create roboforge-tunnel
cloudflared tunnel route dns roboforge-tunnel ws.roboforge.local.YOURDOMAIN.com

# config.yml for cloudflared
tunnel: <TUNNEL_ID>
credentials-file: ~/.cloudflared/<TUNNEL_ID>.json

ingress:
  - hostname: ws.roboforge.local.YOURDOMAIN.com
    service: ws://localhost:9090
  - hostname: api.roboforge.local.YOURDOMAIN.com
    service: http://localhost:8765
  - hostname: state.roboforge.local.YOURDOMAIN.com
    service: ws://localhost:9091
  - service: http_status:404

# Result: Online UI can now connect to YOUR machine's backend
# from anywhere, with TLS, without VPN.
```

### 2.5 Environment Detection & Connection Routing (Frontend)

```typescript
// src/services/BackendConnector.ts (shared by both online and offline builds)

export class BackendConnector {
  private wsRos: WebSocket | null = null;
  private wsState: WebSocket | null = null;
  private mode: 'local' | 'tunnel' | 'offline' = 'offline';

  async connect(): Promise<void> {
    // Priority 1: Try local backend
    if (await this.tryConnect('ws://localhost:9090')) {
      this.mode = 'local';
      console.log('[Bridge] Connected to local Docker backend');
      return;
    }
    
    // Priority 2: Try user-configured tunnel URL
    const tunnelUrl = localStorage.getItem('roboforge_tunnel_ws');
    if (tunnelUrl && await this.tryConnect(tunnelUrl)) {
      this.mode = 'tunnel';
      console.log(`[Bridge] Connected via tunnel: ${tunnelUrl}`);
      return;
    }
    
    // Priority 3: Offline mode — use JS IK/trajectory solvers
    this.mode = 'offline';
    console.warn('[Bridge] No backend available — offline simulation mode');
    this.initOfflineFallback();
  }

  async computeIK(target: CartesianTarget, seed: number[]): Promise<IKResult> {
    if (this.mode !== 'offline') {
      try {
        return await this.callMoveItIK(target, seed);
      } catch (e) {
        console.warn('[Bridge] MoveIt IK failed, falling back to offline IK');
      }
    }
    return offlineIKSolver.solve(target, seed);
  }
  
  // ... rest of methods
}
```

---

## SECTION 3 — ONLINE UI (React/Vite) — COMPLETE SPECIFICATION

### 3.1 Project Structure

```
roboforge-online/
├── src/
│   ├── main.tsx
│   ├── App.tsx                   — root layout, panel manager
│   ├── store/
│   │   ├── AppState.ts           — Zustand global store
│   │   ├── ProgramStore.ts       — block tree + compile state
│   │   ├── RobotState.ts         — live robot state (from bridge)
│   │   └── UIState.ts            — panel visibility, selection
│   ├── services/
│   │   ├── BackendConnector.ts   — connection routing
│   │   ├── Compiler.ts           — Block→IR→Trajectory pipeline
│   │   ├── IKSolver.ts           — offline JS IK (fallback)
│   │   ├── TrajectoryGenerator.ts
│   │   └── StateAggregator.ts    — FK + manipulability
│   ├── components/
│   │   ├── layout/
│   │   │   ├── AppShell.tsx      — overall layout grid
│   │   │   ├── TopRibbon.tsx
│   │   │   └── StatusBar.tsx
│   │   ├── panels/
│   │   │   ├── BlockEditor/
│   │   │   │   ├── BlockEditorPanel.tsx
│   │   │   │   ├── BlockCard.tsx
│   │   │   │   ├── BlockLibrary.tsx
│   │   │   │   └── BlockConfigPopup.tsx
│   │   │   ├── CodeEditor/
│   │   │   │   └── MonacoEditor.tsx
│   │   │   ├── PropertiesPanel/
│   │   │   │   ├── PropertiesPanel.tsx
│   │   │   │   └── JogControls.tsx
│   │   │   ├── DiagnosticsPanel.tsx
│   │   │   ├── ConsolePanel.tsx
│   │   │   └── ROS2Panel.tsx
│   │   └── viewport/
│   │       ├── Viewport3D.tsx    — Three.js canvas wrapper
│   │       ├── RobotMesh.tsx     — articulated robot 3D model
│   │       ├── PathVisualization.tsx
│   │       ├── GizmoManipulator.tsx
│   │       ├── SelectionManager.tsx   — raycasting, marquee select
│   │       ├── ViewportToolbar.tsx
│   │       └── WorkcellGrid.tsx
│   ├── engine/
│   │   ├── IKSolver.ts           — DLS numerical IK
│   │   ├── TrajectoryPlanner.ts
│   │   ├── MotionTypes.ts
│   │   └── Kinematics.ts         — DH forward kinematics
│   └── styles/
│       ├── glass.css             — glassmorphism token system
│       └── tokens.css
├── public/
│   ├── models/                   — robot GLTF files
│   └── env/                      — HDR environment maps
└── vite.config.ts
```

### 3.2 App Shell Layout (CSS Grid)

```typescript
// AppShell.tsx
const SHELL_GRID = `
  "ribbon   ribbon   ribbon  " 48px
  "left     viewport right   " 1fr
  "left     console  right   " 240px
  "status   status   status  " 24px
  / 320px   1fr      360px
`;

// CSS:
// display: grid;
// grid-template: ${SHELL_GRID};
// height: 100vh;
// background: var(--color-bg-base);
```

### 3.3 Top Ribbon — Complete Button Specification

Every button has exactly 6 states: `idle | hover | active | disabled | focused | loading`.

```typescript
interface RibbonButton {
  id: string;
  label: string;
  icon: string;           // Lucide icon name
  shortcut: string;       // keyboard shortcut
  group: string;
  states: ButtonStates;
  action: () => void | Promise<void>;
  loading_triggers: string[];  // store events that set loading state
}

const RIBBON_BUTTONS: RibbonButton[] = [
  // ── FILE GROUP ──────────────────────────────────────────────────
  {
    id: 'file_new',     label: 'New',     icon: 'FilePlus2', shortcut: 'Ctrl+N',
    group: 'File',
    action: () => showNewProjectWizard(),
    // Wizard steps: 1) Robot model select, 2) Workcell config,
    //               3) Tool definitions, 4) Safety zones, 5) Confirm
  },
  {
    id: 'file_open',    label: 'Open',    icon: 'FolderOpen', shortcut: 'Ctrl+O',
    group: 'File',
    action: () => openFilePicker({ accept: '.rfp,.json,.rapid,.urscript' }),
  },
  {
    id: 'file_save',    label: 'Save',    icon: 'Save', shortcut: 'Ctrl+S',
    group: 'File',
    action: async () => saveCurrentProgram(),
    loading_triggers: ['save_in_progress'],
  },
  {
    id: 'file_export',  label: 'Export',  icon: 'Download', shortcut: 'Ctrl+E',
    group: 'File',
    action: () => showExportDialog({
      formats: ['ABB RAPID', 'URScript', 'KUKA KRL', 'Fanuc TP', 'JSON', 'ROS 2 YAML']
    }),
  },
  
  // ── COMPILE & EXECUTE GROUP ─────────────────────────────────────
  {
    id: 'compile',      label: 'Compile', icon: 'Cpu', shortcut: 'F7',
    group: 'Execute',
    action: async () => {
      setButtonLoading('compile');
      const result = await compiler.compile(programStore.blocks);
      if (result.errors.length > 0) {
        showErrorsInConsole(result.errors);
        highlightErrorBlocks(result.errors);
      } else {
        programStore.setCompiledTrajectory(result.trajectory);
        showConsoleMessage('success', `Compiled: ${result.stats.block_count} blocks → ${result.stats.trajectory_points} points`);
      }
      clearButtonLoading('compile');
    },
    loading_triggers: ['compile_in_progress'],
  },
  {
    id: 'run',          label: 'Run',     icon: 'Play', shortcut: 'F5',
    group: 'Execute',
    // Disabled until compiled. Shows green when running.
    action: async () => {
      if (!programStore.compiledTrajectory) {
        await buttons['compile'].action();
      }
      await backendConnector.executeProgram(programStore.compiledTrajectory);
    },
  },
  {
    id: 'pause',        label: 'Pause',   icon: 'Pause', shortcut: 'F6',
    group: 'Execute',
    // Only enabled during execution
    action: () => backendConnector.pauseExecution(),
  },
  {
    id: 'stop',         label: 'Stop',    icon: 'Square', shortcut: 'Esc',
    group: 'Execute',
    // Always enabled during execution. Red color.
    action: () => backendConnector.stopExecution(),
  },
  {
    id: 'step',         label: 'Step',    icon: 'StepForward', shortcut: 'F10',
    group: 'Execute',
    // Only in paused state
    action: () => backendConnector.stepOneBlock(),
  },
  
  // ── MODE TOGGLE ─────────────────────────────────────────────────
  {
    id: 'mode_edit',    label: 'Edit',    icon: 'Edit3',  shortcut: 'Alt+1',
    group: 'Mode',
    // Theme: --color-accent = blue
  },
  {
    id: 'mode_simulate',label: 'Simulate',icon: 'Monitor',shortcut: 'Alt+2',
    group: 'Mode',
    // Theme: --color-accent = purple. Gazebo physics active.
  },
  {
    id: 'mode_live',    label: 'Live',    icon: 'Radio',  shortcut: 'Alt+3',
    group: 'Mode',
    // Theme: --color-accent = red. REAL ROBOT. Safety confirmation dialog required.
    action: () => showLiveModeConfirmation(),
  },
];
```

---

## SECTION 4 — OFFLINE UI (.NET / WPF + WinUI 3)

### 4.1 Why Carbon Copy Is Achievable Despite Different Frameworks

The online (Vite/React) and offline (.NET/WPF) UIs appear **identical** to the user because:

1. **Design tokens are framework-agnostic** — defined in a JSON file, consumed by both
2. **Component behavior is specified in a shared contract** — a `components.spec.json`
3. **Business logic is in the shared backend** — the UI is purely presentation
4. **A pixel-diff CI test** validates visual parity after every commit

```
┌─────────────────────────────────────────────────────────────────────┐
│                   DESIGN TOKEN SYNC PIPELINE                         │
│                                                                       │
│  tokens/design-tokens.json   ←── single source of truth             │
│          │                                                            │
│          ├──▶ scripts/gen-css-tokens.ts  ──▶ src/styles/tokens.css  │
│          │    (for Vite/React online)                                 │
│          │                                                            │
│          └──▶ scripts/gen-xaml-tokens.ts ──▶ RoboForge.Desktop/     │
│               (for .NET offline)               Themes/Tokens.xaml    │
└─────────────────────────────────────────────────────────────────────┘
```

### 4.2 Design Token Schema

```json
// tokens/design-tokens.json
{
  "color": {
    "bg": {
      "base":     { "value": "#0a0e14" },
      "surface":  { "value": "#111622" },
      "elevated": { "value": "#161d2e" }
    },
    "glass": {
      "l0": { "value": "rgba(255,255,255,0.02)", "blur": "0px"   },
      "l1": { "value": "rgba(255,255,255,0.04)", "blur": "8px"   },
      "l2": { "value": "rgba(255,255,255,0.07)", "blur": "16px"  },
      "l3": { "value": "rgba(255,255,255,0.10)", "blur": "24px"  },
      "l4": { "value": "rgba(255,255,255,0.14)", "blur": "40px"  }
    },
    "accent": {
      "primary":  { "value": "#3b82f6" },
      "success":  { "value": "#22c55e" },
      "warning":  { "value": "#f59e0b" },
      "error":    { "value": "#ef4444" },
      "simulate": { "value": "#a855f7" },
      "live":     { "value": "#ef4444" }
    },
    "text": {
      "primary":  { "value": "#f1f5f9" },
      "secondary":{ "value": "#94a3b8" },
      "muted":    { "value": "#475569" },
      "accent":   { "value": "#60a5fa" }
    },
    "border": {
      "subtle":   { "value": "rgba(255,255,255,0.06)" },
      "default":  { "value": "rgba(255,255,255,0.10)" },
      "strong":   { "value": "rgba(255,255,255,0.18)" }
    }
  },
  "spacing": {
    "xs": { "value": "4px"  },
    "sm": { "value": "8px"  },
    "md": { "value": "12px" },
    "lg": { "value": "16px" },
    "xl": { "value": "24px" }
  },
  "radius": {
    "sm":  { "value": "4px"  },
    "md":  { "value": "8px"  },
    "lg":  { "value": "12px" },
    "xl":  { "value": "16px" },
    "full":{ "value": "9999px"}
  },
  "font": {
    "mono":    { "value": "'JetBrains Mono', 'Cascadia Code', monospace" },
    "sans":    { "value": "'Geist', 'DM Sans', system-ui, sans-serif" },
    "display": { "value": "'Clash Display', 'Sora', sans-serif" }
  },
  "size": {
    "ribbon_height":  { "value": "48px"  },
    "sidebar_width":  { "value": "320px" },
    "props_width":    { "value": "360px" },
    "console_height": { "value": "240px" },
    "status_height":  { "value": "24px"  }
  }
}
```

### 4.3 Token Generation Scripts

```typescript
// scripts/gen-css-tokens.ts — generates tokens.css for online
import tokens from '../tokens/design-tokens.json';

function flatten(obj: any, prefix = '--rf'): Record<string, string> {
  const result: Record<string, string> = {};
  for (const [k, v] of Object.entries(obj)) {
    const key = `${prefix}-${k}`;
    if (typeof v === 'object' && 'value' in v) {
      result[key] = (v as any).value;
    } else if (typeof v === 'object') {
      Object.assign(result, flatten(v, key));
    }
  }
  return result;
}

const vars = flatten(tokens);
const css = `:root {\n${Object.entries(vars).map(([k,v]) => `  ${k}: ${v};`).join('\n')}\n}`;
fs.writeFileSync('src/styles/tokens.css', css);
```

```csharp
// scripts/GenXamlTokens/Program.cs — generates Tokens.xaml for offline
// Reads design-tokens.json, writes WPF ResourceDictionary

var tokens = JsonSerializer.Deserialize<JsonElement>(
    File.ReadAllText("tokens/design-tokens.json")
);

var sb = new StringBuilder();
sb.AppendLine("<ResourceDictionary xmlns=\"http://schemas.microsoft.com/winfx/2006/xaml/presentation\"");
sb.AppendLine("                    xmlns:x=\"http://schemas.microsoft.com/winfx/2006/xaml\">");

void Flatten(JsonElement el, string prefix) {
    foreach (var prop in el.EnumerateObject()) {
        var key = $"{prefix}.{prop.Name}";
        if (prop.Value.TryGetProperty("value", out var val)) {
            var v = val.GetString()!;
            // Convert CSS color to WPF color
            if (v.StartsWith("#")) {
                sb.AppendLine($"  <Color x:Key=\"{key}\">{CssToWpfColor(v)}</Color>");
                sb.AppendLine($"  <SolidColorBrush x:Key=\"{key}.Brush\" Color=\"{{StaticResource {key}}}\"/>");
            } else if (v.StartsWith("rgba")) {
                sb.AppendLine($"  <Color x:Key=\"{key}\">{RgbaToWpfColor(v)}</Color>");
                sb.AppendLine($"  <SolidColorBrush x:Key=\"{key}.Brush\" Color=\"{{StaticResource {key}}}\"/>");
            } else {
                sb.AppendLine($"  <sys:String x:Key=\"{key}\">{v}</sys:String>");
            }
        } else {
            Flatten(prop.Value, key);
        }
    }
}

Flatten(tokens, "RF");
sb.AppendLine("</ResourceDictionary>");
File.WriteAllText("RoboForge.Desktop/Themes/Tokens.xaml", sb.ToString());
```

### 4.4 Component Spec (Shared Contract)

```json
// tokens/components.spec.json
{
  "BlockCard": {
    "dimensions": {
      "min_height": "64px",
      "padding": "var(--rf-spacing-md)",
      "border_radius": "var(--rf-radius-lg)",
      "border_width": "1px"
    },
    "states": {
      "idle":      { "background": "var(--rf-color-glass-l1)", "border": "var(--rf-color-border-subtle)" },
      "selected":  { "background": "var(--rf-color-glass-l2)", "border": "var(--rf-color-accent-primary)", "glow": "0 0 8px rgba(59,130,246,0.4)" },
      "executing": { "background": "rgba(34,197,94,0.08)",     "border": "var(--rf-color-accent-success)", "animation": "pulse 1s ease-in-out infinite" },
      "error":     { "background": "rgba(239,68,68,0.08)",     "border": "var(--rf-color-accent-error)" },
      "disabled":  { "background": "var(--rf-color-glass-l0)", "opacity": "0.4" }
    },
    "indent_per_level": "24px",
    "drag_handle": { "width": "4px", "color": "var(--rf-color-border-subtle)" }
  },
  "RibbonButton": {
    "height": "32px",
    "padding": "0 var(--rf-spacing-md)",
    "border_radius": "var(--rf-radius-md)",
    "icon_size": "16px",
    "font_size": "12px",
    "states": {
      "idle":     { "background": "transparent",               "color": "var(--rf-color-text-secondary)" },
      "hover":    { "background": "var(--rf-color-glass-l2)",  "color": "var(--rf-color-text-primary)" },
      "active":   { "background": "var(--rf-color-glass-l3)",  "color": "var(--rf-color-accent-primary)" },
      "disabled": { "opacity": "0.35",                         "cursor": "not-allowed" },
      "focused":  { "outline": "2px solid var(--rf-color-accent-primary)", "outline_offset": "2px" },
      "loading":  { "cursor": "wait", "icon": "spinner" }
    }
  }
}
```

### 4.5 .NET WPF Implementation of Glass Panel

```xml
<!-- RoboForge.Desktop/Controls/GlassPanel.xaml -->
<UserControl x:Class="RoboForge.Desktop.Controls.GlassPanel"
             xmlns="http://schemas.microsoft.com/winfx/2006/xaml/presentation"
             xmlns:x="http://schemas.microsoft.com/winfx/2006/xaml">
  
  <Border x:Name="PART_Border"
          Background="{StaticResource RF.color.glass.l1.Brush}"
          BorderBrush="{StaticResource RF.color.border.subtle.Brush}"
          BorderThickness="1"
          CornerRadius="12">
    
    <!-- Acrylic blur effect via Windows 11 compositor -->
    <Border.Effect>
      <BlurEffect Radius="0"/>
      <!-- Real blur: applied via DwmSetWindowAttribute in code-behind -->
    </Border.Effect>
    
    <ContentPresenter/>
  </Border>
</UserControl>
```

```csharp
// GlassPanel.xaml.cs — applies real Windows blur
using System.Runtime.InteropServices;

public partial class GlassPanel : UserControl {
    
    [DllImport("dwmapi.dll")]
    static extern int DwmSetWindowAttribute(IntPtr hwnd, int attr, ref int attrValue, int attrSize);
    
    [DllImport("user32.dll")]
    static extern IntPtr GetParent(IntPtr hwnd);
    
    // Windows 11: use Mica/Acrylic backdrop
    protected override void OnSourceInitialized(EventArgs e) {
        base.OnSourceInitialized(e);
        var hwnd = new WindowInteropHelper(Window.GetWindow(this)).Handle;
        
        // Apply Acrylic backdrop
        int backdropType = 3; // DWMSBT_MAINWINDOW for Mica, 4 for Acrylic
        DwmSetWindowAttribute(hwnd, 38, ref backdropType, sizeof(int));
        
        // Dark mode
        int darkMode = 1;
        DwmSetWindowAttribute(hwnd, 20, ref darkMode, sizeof(int));
    }
}
```

### 4.6 Offline UI — Block Editor (WPF, pixel-copy of React version)

```xml
<!-- BlockEditorPanel.xaml -->
<Grid>
  <!-- Left: Block Library (320px) -->
  <Border Width="320" HorizontalAlignment="Left">
    <ListView x:Name="BlockLibraryList"
              ItemsSource="{Binding BlockCategories}"
              Background="Transparent"
              BorderThickness="0">
      <ListView.ItemTemplate>
        <DataTemplate>
          <Expander Header="{Binding CategoryName}" IsExpanded="True">
            <ItemsControl ItemsSource="{Binding Blocks}">
              <ItemsControl.ItemTemplate>
                <DataTemplate>
                  <!-- Each block type draggable into editor -->
                  <Border Height="40" Margin="4"
                          Background="{StaticResource RF.color.glass.l1.Brush}"
                          CornerRadius="8"
                          AllowDrop="True"
                          MouseMove="BlockLibraryItem_MouseMove">
                    <StackPanel Orientation="Horizontal" Margin="8,0">
                      <TextBlock Text="{Binding Icon}" FontFamily="Segoe MDL2 Assets" Width="20"/>
                      <TextBlock Text="{Binding Name}" Margin="8,0,0,0"
                                 Foreground="{StaticResource RF.color.text.primary.Brush}"/>
                    </StackPanel>
                  </Border>
                </DataTemplate>
              </ItemsControl.ItemTemplate>
            </ItemsControl>
          </Expander>
        </DataTemplate>
      </ListView.ItemTemplate>
    </ListView>
  </Border>
  
  <!-- Right: Program Tree (block cards, drag-drop, nesting) -->
  <ScrollViewer Margin="320,0,0,0">
    <ItemsControl x:Name="ProgramTree"
                  ItemsSource="{Binding ProgramBlocks}"
                  AllowDrop="True"
                  Drop="ProgramTree_Drop">
      <!-- Each block rendered as BlockCard UserControl -->
    </ItemsControl>
  </ScrollViewer>
</Grid>
```

### 4.7 Offline ↔ Online gRPC Bridge

The offline .NET app communicates with the shared backend via **gRPC** (faster than WebSocket for high-frequency state updates).

```protobuf
// proto/roboforge.proto
syntax = "proto3";
package roboforge;

service RoboForgeBackend {
  rpc ExecuteProgram (ProgramRequest)    returns (stream ExecutionUpdate);
  rpc ComputeIK      (IKRequest)         returns (IKResponse);
  rpc ComputeFK      (FKRequest)         returns (FKResponse);
  rpc GetRobotState  (Empty)             returns (stream RobotStateMsg);
  rpc PauseExecution (Empty)             returns (StatusResponse);
  rpc StopExecution  (Empty)             returns (StatusResponse);
  rpc JogJoint       (JogJointRequest)   returns (StatusResponse);
  rpc JogCartesian   (JogCartRequest)    returns (StatusResponse);
}

message ProgramRequest {
  repeated TrajectorySegment segments = 1;
}

message TrajectorySegment {
  string block_id = 1;
  string motion_type = 2;  // "MoveJ", "MoveL", "MoveC"
  repeated TrajectoryPoint points = 3;
}

message TrajectoryPoint {
  double t = 1;
  repeated double q   = 2;  // joint angles (rad)
  repeated double qd  = 3;  // velocities (rad/s)
  repeated double qdd = 4;  // accelerations (rad/s²)
}

message RobotStateMsg {
  int64  timestamp_ns    = 1;
  repeated double joint_pos_rad     = 2;
  repeated double joint_vel_rads    = 3;
  repeated double joint_torque_nm   = 4;
  repeated double tcp_pos_m         = 5;  // xyz
  repeated double tcp_quat          = 6;  // xyzw
  double  manipulability            = 7;
  bool    in_singularity            = 8;
  string  singularity_type          = 9;
  int32   program_counter           = 10;
  string  execution_state           = 11;
}
```

```csharp
// C# gRPC client in .NET offline app
public class GrpcBackendClient : IBackendClient {
    private readonly RoboForgeBackend.RoboForgeBackendClient _client;
    
    public GrpcBackendClient() {
        var channel = GrpcChannel.ForAddress("http://localhost:50051");
        _client = new RoboForgeBackend.RoboForgeBackendClient(channel);
    }
    
    public async IAsyncEnumerable<RobotState> StreamRobotState(
        CancellationToken ct) 
    {
        var call = _client.GetRobotState(new Empty(), cancellationToken: ct);
        await foreach (var msg in call.ResponseStream.ReadAllAsync(ct)) {
            yield return MapToRobotState(msg);
        }
    }
    
    public async Task<IKResult> ComputeIK(CartesianTarget target, double[] seed) {
        var req = new IKRequest {
            TargetX = target.X, TargetY = target.Y, TargetZ = target.Z,
            QuatX = target.QuatX, QuatY = target.QuatY,
            QuatZ = target.QuatZ, QuatW = target.QuatW,
        };
        req.SeedAngles.AddRange(seed);
        var resp = await _client.ComputeIKAsync(req);
        return new IKResult {
            Success = resp.Success,
            JointAnglesRad = resp.JointAnglesRad.ToArray(),
            SolverUsed = resp.SolverUsed,
        };
    }
}
```

---

## SECTION 5 — SYNCHRONIZATION ENGINE (MIRROR ARCHITECTURE)

### 5.1 The Mirror Contract

Both UIs must **always reflect the same state**. State lives in the backend (ROS 2 + bridge), not in either UI.

```
┌─────────────────────────────────────────────────────────────────────┐
│                        STATE OWNERSHIP                               │
│                                                                       │
│  OWNED BY BACKEND (roboforge_bridge):                                │
│    • Robot state (joint positions, velocities, torques)              │
│    • Execution state (running/paused/stopped)                        │
│    • Program counter                                                  │
│    • Diagnostic messages                                             │
│    • Planning scene (obstacles, tool frames, work objects)           │
│                                                                       │
│  OWNED BY USER'S SESSION (local to each UI):                         │
│    • Program blocks (saved to .rfp file)                             │
│    • UI layout preferences                                           │
│    • Connection settings                                             │
│    • Selected objects in viewport                                    │
│                                                                       │
│  SHARED via FILE (co-editing not supported — one UI at a time):     │
│    • .rfp program file (JSON, watched by both UIs via file watcher)  │
└─────────────────────────────────────────────────────────────────────┘
```

### 5.2 Mutual Exclusion — Preventing Both UIs Running Simultaneously

```python
# In roboforge_bridge:
import fcntl, os

LOCK_FILE = '/tmp/roboforge_session.lock'

def acquire_session_lock(client_id: str, client_type: str) -> bool:
    """Only one UI can be in EXECUTE mode at a time."""
    lock_data = { 'client_id': client_id, 'type': client_type, 'pid': os.getpid() }
    try:
        with open(LOCK_FILE, 'x') as f:  # exclusive create
            json.dump(lock_data, f)
        return True
    except FileExistsError:
        with open(LOCK_FILE) as f:
            existing = json.load(f)
        # Allow READ but not EXECUTE if another client holds lock
        return False

def release_session_lock():
    try:
        os.remove(LOCK_FILE)
    except FileNotFoundError:
        pass
```

### 5.3 CI Pixel-Diff Test

```typescript
// tests/visual-parity.spec.ts (Playwright)
import { chromium } from 'playwright';
import { PNG } from 'pngjs';
import pixelmatch from 'pixelmatch';

test('Online and offline UIs are visually identical', async () => {
  // Screenshot online UI
  const browser = await chromium.launch();
  const onlinePage = await browser.newPage({ viewport: { width: 1920, height: 1080 } });
  await onlinePage.goto('http://localhost:5173');
  await onlinePage.waitForLoadState('networkidle');
  const onlineShot = await onlinePage.screenshot({ fullPage: false });
  
  // Screenshot offline UI via WPF automation → screenshot
  // (Use FlaUI or Windows UI Automation to capture offline app screenshot)
  const offlineShot = await captureWpfScreenshot();
  
  const img1 = PNG.sync.read(onlineShot);
  const img2 = PNG.sync.read(offlineShot);
  const diff = new PNG({ width: 1920, height: 1080 });
  
  const mismatch = pixelmatch(img1.data, img2.data, diff.data, 1920, 1080, {
    threshold: 0.1,        // 10% color tolerance for font rendering diffs
    alpha: 0.5,
    includeAA: false       // ignore anti-aliasing differences
  });
  
  // Allow up to 2% pixel difference (font rendering, DPI differences)
  expect(mismatch / (1920 * 1080)).toBeLessThan(0.02);
});
```

---

## SECTION 6 — 3D VIEWPORT: COMPLETE ALGORITHMS

### 6.1 Scene Graph Structure

```
Scene Root
├── Environment
│   ├── HemisphereLight (sky: #1e3a5f, ground: #0a0a0a, intensity: 0.4)
│   ├── DirectionalLight (pos: [5,8,5], intensity: 1.2, castShadow)
│   ├── PointLight (pos: [-3,2,-3], color: #3b82f6, intensity: 0.3)
│   ├── WorkcellGrid (infinite grid, XZ plane, minor: 100mm, major: 1000mm)
│   └── EnvironmentMap (HDR, for PBR reflections)
│
├── WorkCell
│   ├── FloorMesh (ShadowReceiver)
│   ├── Obstacles[] (static collision geometry)
│   ├── SafetyZones[] (transparent volume meshes)
│   └── WorkObjects[] (coordinate frame indicators)
│
├── RobotGroup
│   ├── BaseLink (fixed to world)
│   ├── Link_1 (rotates about joint_1)
│   │   └── Link_2 (rotates about joint_2)
│   │       └── Link_3 (rotates about joint_3)
│   │           └── Link_4 (rotates about joint_4)
│   │               └── Link_5 (rotates about joint_5)
│   │                   └── Link_6 (rotates about joint_6)
│   │                       └── ToolFrame
│   │                           └── TCPMarker (axis indicator)
│   └── GhostRobot (semi-transparent copy at target pose)
│
├── PathVisualization
│   ├── WaypointMarkers[] (per-block spheres + coordinate frames)
│   ├── PathSpline (CatmullRom through TCP positions)
│   ├── BlendZones[] (transparent spheres at zone radii)
│   └── TCPTrace (real-time tail of executed TCP path)
│
├── GizmoManipulator (attached to selected waypoint)
│   ├── TranslateGizmo (X/Y/Z handles + XY/YZ/XZ planes)
│   └── RotateGizmo (X/Y/Z arcs)
│
└── SelectionManager (raycasting, marquee select)
```

### 6.2 Selection Algorithms

#### 6.2.1 Single-Click Ray Casting

```typescript
// SelectionManager.ts

export class SelectionManager {
  private raycaster = new THREE.Raycaster();
  private mouse = new THREE.Vector2();
  
  // Selectables registry — all objects that can be selected
  private selectables: Map<string, SelectableObject> = new Map();
  
  onPointerDown(event: PointerEvent, camera: THREE.Camera): SelectionResult {
    // 1. Compute NDC coordinates
    const rect = this.canvas.getBoundingClientRect();
    this.mouse.x = ((event.clientX - rect.left) / rect.width)  * 2 - 1;
    this.mouse.y = -((event.clientY - rect.top)  / rect.height) * 2 + 1;
    
    // 2. Cast ray
    this.raycaster.setFromCamera(this.mouse, camera);
    this.raycaster.params.Line.threshold = 0.01;  // 10mm line selection
    this.raycaster.params.Points.threshold = 0.02;
    
    // 3. Test against selectables (sorted front-to-back)
    const selectableObjects = Array.from(this.selectables.values())
      .map(s => s.mesh)
      .flat();
    
    const hits = this.raycaster.intersectObjects(selectableObjects, true);
    
    if (hits.length === 0) {
      // Click on empty space — deselect all (unless Shift held)
      if (!event.shiftKey) this.clearSelection();
      return { type: 'empty' };
    }
    
    // 4. Resolve hit object back to selectable entity
    const hitObject = hits[0].object;
    const entity = this.findEntityForMesh(hitObject);
    
    if (!entity) return { type: 'empty' };
    
    // 5. Handle selection modifiers
    if (event.shiftKey) {
      this.toggleSelection(entity.id);
    } else if (event.ctrlKey) {
      this.addToSelection(entity.id);
    } else {
      this.setSelection([entity.id]);
    }
    
    return { type: 'entity', entity, hitPoint: hits[0].point };
  }
}
```

#### 6.2.2 Marquee (Rubber-Band) Selection

```typescript
// Box selection by frustum culling

onMarqueeEnd(startNDC: THREE.Vector2, endNDC: THREE.Vector2, camera: THREE.Camera) {
  // Build selection frustum from 2D box
  const frustum = this.buildSelectionFrustum(startNDC, endNDC, camera);
  
  const selected: string[] = [];
  for (const [id, entity] of this.selectables) {
    const worldPos = new THREE.Vector3();
    entity.mesh[0].getWorldPosition(worldPos);
    
    if (frustum.containsPoint(worldPos)) {
      selected.push(id);
    }
  }
  
  this.setSelection(selected);
}

private buildSelectionFrustum(
  start: THREE.Vector2,
  end:   THREE.Vector2,
  camera: THREE.Camera
): THREE.Frustum {
  // Construct a tight frustum from the 2D screen rectangle
  const projMatrix = new THREE.Matrix4();
  
  // Map NDC box to clip space, build custom projection
  const minX = Math.min(start.x, end.x);
  const maxX = Math.max(start.x, end.x);
  const minY = Math.min(start.y, end.y);
  const maxY = Math.max(start.y, end.y);
  
  // Scale the camera's projection matrix to the sub-region
  const scaleX = 2 / (maxX - minX);
  const scaleY = 2 / (maxY - minY);
  const offsetX = -(maxX + minX) / (maxX - minX);
  const offsetY = -(maxY + minY) / (maxY - minY);
  
  const offsetMatrix = new THREE.Matrix4().set(
    scaleX, 0,      0, offsetX,
    0,      scaleY, 0, offsetY,
    0,      0,      1, 0,
    0,      0,      0, 1
  );
  
  projMatrix.multiplyMatrices(offsetMatrix, camera.projectionMatrix);
  const vpMatrix = new THREE.Matrix4().multiplyMatrices(projMatrix, camera.matrixWorldInverse);
  
  const frustum = new THREE.Frustum();
  frustum.setFromProjectionMatrix(vpMatrix);
  return frustum;
}
```

#### 6.2.3 Background / Floor Detection

```typescript
// WorkcellGrid.ts — detect click on infinite floor grid

class WorkcellGrid {
  private gridPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0); // Y=0
  
  // Returns world position of floor intersection, or null
  intersectFloor(ray: THREE.Ray): THREE.Vector3 | null {
    const target = new THREE.Vector3();
    const hit = ray.intersectPlane(this.gridPlane, target);
    if (!hit) return null;
    
    // Snap to grid if Shift held
    if (this.snapEnabled) {
      target.x = Math.round(target.x / this.snapSize) * this.snapSize;
      target.z = Math.round(target.z / this.snapSize) * this.snapSize;
    }
    return target;
  }
  
  // Check if ray hit the background (sky/environment)
  isBackgroundHit(hits: THREE.Intersection[]): boolean {
    return hits.length === 0 || 
      hits.every(h => h.object.userData.isBackground === true);
  }
}
```

#### 6.2.4 Gizmo Manipulation

```typescript
// GizmoManipulator.ts

type GizmoAxis = 'X' | 'Y' | 'Z' | 'XY' | 'YZ' | 'XZ';
type GizmoMode = 'translate' | 'rotate' | 'scale';

class GizmoManipulator {
  private dragAxis: GizmoAxis | null = null;
  private dragStartPoint = new THREE.Vector3();
  private dragStartWorldPos = new THREE.Vector3();
  private dragConstraintPlane = new THREE.Plane();
  
  startDrag(hitAxis: GizmoAxis, ray: THREE.Ray) {
    this.dragAxis = hitAxis;
    this.target.getWorldPosition(this.dragStartWorldPos);
    
    // Determine constraint plane normal
    const planeNormal = this.getConstraintPlaneNormal(hitAxis, ray);
    this.dragConstraintPlane.setFromNormalAndCoplanarPoint(
      planeNormal, 
      this.dragStartWorldPos
    );
    
    ray.intersectPlane(this.dragConstraintPlane, this.dragStartPoint);
  }
  
  updateDrag(ray: THREE.Ray): THREE.Vector3 | null {
    if (!this.dragAxis) return null;
    
    const currentPoint = new THREE.Vector3();
    if (!ray.intersectPlane(this.dragConstraintPlane, currentPoint)) {
      return null;
    }
    
    const delta = currentPoint.clone().sub(this.dragStartPoint);
    
    // Project delta onto allowed axes
    const constrainedDelta = this.constrainDelta(delta, this.dragAxis);
    
    // Apply to target
    const newPos = this.dragStartWorldPos.clone().add(constrainedDelta);
    this.target.position.copy(newPos);
    
    return newPos;
  }
  
  private getConstraintPlaneNormal(axis: GizmoAxis, ray: THREE.Ray): THREE.Vector3 {
    // For single-axis moves, use the plane most perpendicular to camera view
    const cameraDir = ray.direction.clone().negate();
    
    switch (axis) {
      case 'X': {
        // Constrain to XY or XZ — pick the one more aligned to camera
        const xyNormal = new THREE.Vector3(0, 0, 1);
        const xzNormal = new THREE.Vector3(0, 1, 0);
        return Math.abs(cameraDir.dot(xyNormal)) > Math.abs(cameraDir.dot(xzNormal))
          ? xyNormal : xzNormal;
      }
      case 'Y': return new THREE.Vector3(0, 0, 1); // YZ plane
      case 'Z': return new THREE.Vector3(0, 1, 0); // XZ plane
      case 'XY': return new THREE.Vector3(0, 0, 1);
      case 'YZ': return new THREE.Vector3(1, 0, 0);
      case 'XZ': return new THREE.Vector3(0, 1, 0);
    }
  }
  
  private constrainDelta(delta: THREE.Vector3, axis: GizmoAxis): THREE.Vector3 {
    switch (axis) {
      case 'X':  return new THREE.Vector3(delta.x, 0, 0);
      case 'Y':  return new THREE.Vector3(0, delta.y, 0);
      case 'Z':  return new THREE.Vector3(0, 0, delta.z);
      case 'XY': return new THREE.Vector3(delta.x, delta.y, 0);
      case 'YZ': return new THREE.Vector3(0, delta.y, delta.z);
      case 'XZ': return new THREE.Vector3(delta.x, 0, delta.z);
    }
  }
}
```

### 6.3 Camera Controls

```typescript
// CameraController.ts — full orbit/pan/zoom + named views

class CameraController {
  private camera: THREE.PerspectiveCamera;
  private target = new THREE.Vector3(0, 0.8, 0); // orbit center
  private spherical = new THREE.Spherical(3.5, Math.PI/4, Math.PI/4);
  
  // Orbit (left drag)
  onOrbit(dx: number, dy: number) {
    this.spherical.theta -= dx * 0.005;  // azimuth
    this.spherical.phi   -= dy * 0.005;  // elevation
    this.spherical.phi = THREE.MathUtils.clamp(
      this.spherical.phi, 0.01, Math.PI - 0.01
    );
    this.updateCameraPosition();
  }
  
  // Pan (middle drag)
  onPan(dx: number, dy: number) {
    const panSpeed = this.spherical.radius * 0.001;
    const right = new THREE.Vector3().crossVectors(
      this.camera.getWorldDirection(new THREE.Vector3()),
      this.camera.up
    ).normalize();
    const up = this.camera.up.clone();
    
    this.target.addScaledVector(right, -dx * panSpeed);
    this.target.addScaledVector(up,    dy * panSpeed);
    this.updateCameraPosition();
  }
  
  // Zoom (scroll wheel)
  onZoom(delta: number) {
    this.spherical.radius *= (1 + delta * 0.001);
    this.spherical.radius = THREE.MathUtils.clamp(
      this.spherical.radius, 0.5, 20.0
    );
    this.updateCameraPosition();
  }
  
  // Named views (Numpad 1/2/3/4/5/6)
  setNamedView(view: 'front'|'back'|'left'|'right'|'top'|'home') {
    const views = {
      front: { theta: 0,        phi: Math.PI/2, radius: 3.5, target: [0,0.8,0] },
      back:  { theta: Math.PI,  phi: Math.PI/2, radius: 3.5, target: [0,0.8,0] },
      left:  { theta: -Math.PI/2, phi: Math.PI/2, radius: 3.5, target: [0,0.8,0] },
      right: { theta:  Math.PI/2, phi: Math.PI/2, radius: 3.5, target: [0,0.8,0] },
      top:   { theta: 0,        phi: 0.01,      radius: 5.0, target: [0,0,0] },
      home:  { theta: Math.PI/4, phi: Math.PI/3, radius: 3.5, target: [0,0.8,0] },
    };
    
    const v = views[view];
    // Animate transition (0.4s ease-out)
    this.animateTo({
      spherical: new THREE.Spherical(v.radius, v.phi, v.theta),
      target: new THREE.Vector3(...v.target)
    }, 400);
  }
  
  // Frame selected (F key)
  frameSelection(objects: THREE.Object3D[]) {
    const box = new THREE.Box3();
    for (const obj of objects) box.expandByObject(obj);
    
    const center = box.getCenter(new THREE.Vector3());
    const size = box.getSize(new THREE.Vector3());
    const maxDim = Math.max(size.x, size.y, size.z);
    
    this.animateTo({
      spherical: new THREE.Spherical(maxDim * 2.0, this.spherical.phi, this.spherical.theta),
      target: center
    }, 300);
  }
}
```

### 6.4 Viewport Toolbar — All 9 Buttons

```typescript
const VIEWPORT_TOOLS = [
  {
    id: 'select',         icon: 'MousePointer2', shortcut: 'Q',
    tooltip: 'Select (click or drag for marquee)',
    action: () => setTool('select'),
  },
  {
    id: 'translate',      icon: 'Move',           shortcut: 'W',
    tooltip: 'Move waypoint (XYZ gizmo)',
    action: () => setTool('translate'),
  },
  {
    id: 'rotate',         icon: 'RotateCcw',      shortcut: 'E',
    tooltip: 'Rotate waypoint orientation',
    action: () => setTool('rotate'),
  },
  {
    id: 'addWaypoint',    icon: 'MapPin',          shortcut: 'A',
    tooltip: 'Click floor/surface to add waypoint at position',
    action: () => setTool('addWaypoint'),
  },
  {
    id: 'frameAll',       icon: 'Maximize2',       shortcut: 'Numpad0',
    tooltip: 'Frame all objects in view',
    action: () => camera.frameAll(),
  },
  {
    id: 'frameSelected',  icon: 'Focus',           shortcut: 'F',
    tooltip: 'Frame selected waypoint',
    action: () => camera.frameSelection(selectedObjects),
  },
  {
    id: 'gridSnap',       icon: 'Grid3x3',         shortcut: 'G',
    tooltip: 'Toggle grid snap (10mm)',
    action: () => toggleGridSnap(),
    isToggle: true,
  },
  {
    id: 'showCollision',  icon: 'AlertTriangle',   shortcut: 'C',
    tooltip: 'Show/hide collision geometry',
    action: () => toggleCollisionMeshes(),
    isToggle: true,
  },
  {
    id: 'perspective',    icon: 'View',            shortcut: 'Numpad5',
    tooltip: 'Toggle perspective/orthographic',
    action: () => camera.toggleProjection(),
    isToggle: true,
  },
];
```

### 6.5 PBR Robot Shader (Three.js)

```typescript
// RobotMesh.tsx — using MeshStandardMaterial (PBR)

const LINK_MATERIAL = new THREE.MeshStandardMaterial({
  color: new THREE.Color(0x1a2a3a),
  metalness: 0.85,
  roughness: 0.15,
  envMapIntensity: 1.2,
  // Slight emissive glow on moving links during execution:
  emissive: new THREE.Color(0x001133),
  emissiveIntensity: 0.0,  // animated to 0.3 during motion
});

// Per-link execution highlight:
function setLinkExecuting(linkIdx: number, isMoving: boolean) {
  LINK_MATERIAL.emissiveIntensity = isMoving ? 0.3 : 0.0;
  LINK_MATERIAL.needsUpdate = true;
}

// Ghost (target) robot — semi-transparent:
const GHOST_MATERIAL = new THREE.MeshStandardMaterial({
  color: new THREE.Color(0x3b82f6),
  metalness: 0.3,
  roughness: 0.7,
  transparent: true,
  opacity: 0.25,
  depthWrite: false,  // CRITICAL: prevents Z-fighting with real robot
  side: THREE.FrontSide,
});

// Singularity warning: override color to red
const SINGULARITY_MATERIAL = LINK_MATERIAL.clone();
SINGULARITY_MATERIAL.color.set(0x7f1d1d);
SINGULARITY_MATERIAL.emissive.set(0x450a0a);
SINGULARITY_MATERIAL.emissiveIntensity = 0.5;
```

### 6.6 Path Visualization

```typescript
// PathVisualization.tsx

class PathVisualization {
  private pathLine: THREE.Line;
  private waypointMarkers: THREE.Mesh[] = [];
  private blendZones: THREE.Mesh[] = [];
  private tcpTrace: THREE.BufferGeometry;  // real-time tail
  private tracePoints: THREE.Vector3[] = [];
  
  updateFromTrajectory(trajectory: TrajectorySegment[]) {
    // Collect all TCP positions across segments
    const tcpPositions: THREE.Vector3[] = [];
    for (const seg of trajectory) {
      for (const pt of seg.points) {
        tcpPositions.push(new THREE.Vector3(...pt.tcp_pos));
      }
    }
    
    // CatmullRom spline through waypoints for smooth path line
    const curve = new THREE.CatmullRomCurve3(
      trajectory.map(seg => new THREE.Vector3(...seg.points[0].tcp_pos)),
      false, 'catmullrom', 0.5
    );
    const splinePoints = curve.getPoints(200);
    
    this.pathLine.geometry.setFromPoints(splinePoints);
    
    // Update waypoint spheres
    this.clearWaypoints();
    for (let i = 0; i < trajectory.length; i++) {
      const seg = trajectory[i];
      const pos = new THREE.Vector3(...seg.points[0].tcp_pos);
      this.addWaypointMarker(i, pos, seg.motion_type, seg.block_id);
      
      if (seg.zone_m > 0) {
        this.addBlendZone(pos, seg.zone_m);
      }
    }
  }
  
  // Called every frame during execution with real joint_states
  pushTCPTrace(tcpPos: THREE.Vector3) {
    this.tracePoints.push(tcpPos.clone());
    if (this.tracePoints.length > 2000) this.tracePoints.shift();
    
    const positions = new Float32Array(this.tracePoints.length * 3);
    for (let i = 0; i < this.tracePoints.length; i++) {
      positions[i*3]   = this.tracePoints[i].x;
      positions[i*3+1] = this.tracePoints[i].y;
      positions[i*3+2] = this.tracePoints[i].z;
    }
    this.tcpTrace.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    this.tcpTrace.computeBoundingSphere();
  }
}
```

---

## SECTION 7 — IMPLEMENTATION STEPS (ORDERED, NO SHORTCUTS)

### Phase 1: Fix the Existing Online App (Days 1–3)

**Step 1.1** — Fix IK unit conversion bug:
```
File: src/engine/IKSolver.ts
Change: All target inputs divided by 1000.0 before entering solver
Add: reach validation (MIN=350mm, MAX=2550mm for ABB IRB 6700)
Add: pre-IK check: if target inside singularity zone (reach<400mm), warn
```

**Step 1.2** — Fix default program blocks:
```
File: src/store/AppState.ts
Change: DEFAULT_PROGRAM waypoints to valid in-workspace positions
Values: Home(0,0,1500), Approach(500,0,1200,rx=180°), Pick(500,0,900,rx=180°)
```

**Step 1.3** — Implement BackendConnector with environment detection:
```
Create: src/services/BackendConnector.ts
Logic: Try localhost:9090 → try tunnel URL → fall back to offline mode
Add: Settings panel for tunnel URL configuration
```

**Step 1.4** — Add Settings panel tunnel configuration:
```
UI element: Settings → Connection tab
Field: "ROS2 Backend WebSocket URL"
Default: ws://localhost:9090
Placeholder: wss://ws.yourdomain.com
Save: localStorage key "roboforge_tunnel_ws"
```

### Phase 2: Build the Shared Backend Docker Stack (Days 4–10)

**Step 2.1** — Set up ROS 2 workspace:
```bash
mkdir -p ros_ws/src
cd ros_ws/src
git clone https://github.com/ros-planning/moveit2.git
# Create roboforge_description package (URDF/SDF for ABB IRB 6700)
# Create roboforge_bringup package (launch files)
# Create roboforge_moveit package (MoveIt config)
# Create roboforge_simulation package (Gazebo world)
# Create roboforge_bridge package (Python bridge node)
```

**Step 2.2** — Create URDF for ABB IRB 6700:
```
robot_description/urdf/irb6700.urdf.xacro
- All 6 joints with correct DH parameters
- Visual meshes (.dae from ABB's ROS package)
- Collision meshes (simplified convex hulls)
- Inertial properties for Gazebo dynamics
- ros2_control tags with joint limits
```

**Step 2.3** — MoveIt 2 configuration:
```
config/kinematics.yaml:
  manipulator:
    kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
    kinematics_solver_attempts: 3
    kinematics_solver_timeout: 0.05

config/joint_limits.yaml: [all 6 joints with pos/vel/accel limits]
config/ompl_planning.yaml: [RRTConnect planner config]
config/moveit_controllers.yaml: [joint_trajectory_controller]
```

**Step 2.4** — Build the roboforge_bridge node (Section 2.2):
```
Implements: WebSocket server (:9090 rosbridge protocol)
Implements: WebSocket server (:9091 state push)
Implements: gRPC server (:50051 for .NET offline client)
Implements: REST API (:8765 for health/config)
```

**Step 2.5** — Write docker-compose.sim.yml (Section 2.3):
```
Services: ros_core, moveit, gazebo, bridge
Networks: roboforge_net (internal bridge network)
Healthchecks: each service waits for previous
Volumes: ros_ws, config, models
```

**Step 2.6** — Test the full backend pipeline:
```bash
docker compose -f docker-compose.sim.yml up -d
# Wait for all services healthy (~3-5 minutes for colcon build)
ros2 topic echo /joint_states           # should show 0.0 all joints
ros2 service call /compute_ik ...       # should return IK solution
ign gazebo --version                    # should start Gazebo
```

### Phase 3: Connect Online UI to Backend (Days 11–14)

**Step 3.1** — Replace inline IK call in Compiler with BackendConnector.computeIK()
**Step 3.2** — Replace direct trajectory publish with BackendConnector.executeProgram()
**Step 3.3** — Subscribe to BackendConnector state stream → update RobotStateBuffer
**Step 3.4** — Feed RobotStateBuffer into Three.js robot joint angles each frame
**Step 3.5** — Feed RobotStateBuffer TCP trace into PathVisualization.pushTCPTrace()
**Step 3.6** — Test full loop: block → compile → IK (MoveIt) → trajectory → Gazebo → feedback → 3D view

### Phase 4: Set Up Cloudflare Tunnel (Day 15)

```bash
# Install cloudflared on user's machine
winget install Cloudflare.cloudflared  # or brew install cloudflared

# Authenticate
cloudflared tunnel login

# Create tunnel
cloudflared tunnel create roboforge

# Add DNS route (requires domain on Cloudflare)
cloudflared tunnel route dns roboforge ws.YOUR-DOMAIN.com

# Run tunnel (or install as Windows Service)
cloudflared tunnel run roboforge

# User sets ws://ws.YOUR-DOMAIN.com in Settings → Connection
```

### Phase 5: Build Offline .NET App (Days 16–35)

**Step 5.1** — Project setup:
```
dotnet new wpf -n RoboForge.Desktop
dotnet add package Helix.Toolkit.WPF
dotnet add package Google.Protobuf
dotnet add package Grpc.Net.Client
dotnet add package Grpc.Tools
dotnet add package MaterialDesignThemes (for MDI panel layout)
```

**Step 5.2** — Token generation pipeline (run in CI):
```bash
node scripts/gen-xaml-tokens.ts
# Output: RoboForge.Desktop/Themes/Tokens.xaml
# Include Tokens.xaml in App.xaml ResourceDictionary
```

**Step 5.3** — Glass effect implementation:
```
GlassPanel UserControl (Section 4.5)
Windows 11 DWM Acrylic via SetWindowCompositionAttribute
Dark mode via DwmSetWindowAttribute(20, 1)
```

**Step 5.4** — App shell XAML layout:
```
Grid with same proportions as React:
  Row 0: TopRibbon (48px)
  Row 1: Main content area (*)
  Row 2: Console (240px)
  Row 3: StatusBar (24px)
Column 0: Left sidebar (320px)
Column 1: 3D Viewport (*)
Column 2: Properties (360px)
```

**Step 5.5** — HelixViewport3D robot rendering:
```
Use Helix Toolkit's HelixViewport3D control
Load COLLADA (.dae) robot meshes at startup
Update joint transforms each frame from gRPC RobotStateMsg stream
```

**Step 5.6** — All panels (BlockEditor, Properties, Diagnostics, Console, ROS2) ported to WPF:
```
Each panel's behavior defined in components.spec.json
Implement each as UserControl
Use same token values for all colors/sizes
```

**Step 5.7** — gRPC client (Section 4.7):
```
GrpcBackendClient implements IBackendClient
Try localhost:50051 first → else show "Backend not running" status
Background thread streams RobotStateMsg at 50Hz
Dispatches to UI thread via Application.Current.Dispatcher
```

**Step 5.8** — Selection algorithms in HelixViewport3D:
```
Helix: MouseDown → Visual3D hit testing (built-in)
Marquee: track drag, compute frustum, test all waypoint positions
Gizmo: custom adorner overlay using System.Windows.Media.Media3D
```

### Phase 6: Mirror Validation (Days 36–40)

**Step 6.1** — Run pixel-diff CI test (Section 5.3)
**Step 6.2** — Run functional parity tests:
```
Both UIs: compile same program → send to same backend → compare trajectory JSON
Expected: bit-identical trajectory (both use same backend compiler)
```
**Step 6.3** — Performance test:
```
Online UI: 50Hz state update → Three.js render → no dropped frames
Offline UI: 50Hz state update → HelixViewport3D → no dropped frames
```
**Step 6.4** — Edge case: both UIs open simultaneously:
```
Both should receive state updates (bridge broadcasts to all WS clients)
Only one can EXECUTE at a time (session lock in bridge, Section 5.2)
If second UI tries to execute: bridge returns LOCKED error, UI shows toast
```

---

## SECTION 8 — EDGE CASES

### 8.1 Backend not running when UI starts
```
Online: BackendConnector falls to offline mode. Console shows:
  [WARN] No ROS 2 backend found. Using offline simulation.
  [INFO] To enable full simulation, run: docker compose -f docker-compose.sim.yml up -d
  [INFO] Then configure tunnel URL in Settings → Connection.
Offline: Same message. gRPC reconnect attempt every 5 seconds (exponential backoff).
```

### 8.2 Backend disconnects during program execution
```
Behavior: Emergency stop sent to last known bridge state.
If bridge reconnects within 3s: offer resume from last completed block.
If not: mark program as aborted. Show reconnect dialog.
Do NOT continue animation — freeze robot at last known joint state.
```

### 8.3 MoveIt planning fails for a block
```
Behavior: Fall back to offline numerical IK for that block.
Log: [WARN] MoveIt planning failed for block 'Approach' (code -31: GOAL_IN_COLLISION).
     Using offline IK — trajectory may not be collision-free.
Show: Yellow warning badge on the block in program tree.
```

### 8.4 Singularity during trajectory
```
Detection: manipulability < 0.05 (Yoshikawa index)
Behavior: Constraint Resolver clamps speed to 10% near singularity.
Visualization: Robot links glow red. Status bar shows "⚠ Near singularity: wrist".
User action: Dialog suggests adjusting wrist orientation by ±5°.
```

### 8.5 IK target changes during live mode
```
Prevent: Gizmo drag is disabled in Live mode.
Manual jog: Only via JogControls (incremental, speed-limited).
```

### 8.6 Cloudflare tunnel latency
```
IK service call: max 200ms latency tolerated (5s timeout set).
State update: rosbridge push at 50Hz — with 50ms latency, UI sees 40Hz effective.
Path trace: buffer 100 frames locally, replay at correct rate.
```

### 8.7 .NET token generation fails in CI
```
CI step: node scripts/gen-xaml-tokens.ts
On fail: build fails with error "XAML token generation failed — check design-tokens.json"
Prevents: deploying offline app with stale visual tokens.
```

### 8.8 User opens same .rfp file in both UIs simultaneously
```
Both UIs watch the file with a FileSystemWatcher.
When one UI saves → triggers reload prompt in the other.
A version vector in the .rfp file detects merge conflicts.
Conflict resolution: show diff dialog, user picks which version to keep.
```

---

## APPENDIX A — File Naming Convention

```
roboforge/
├── tokens/                       ← Design tokens (single source of truth)
│   ├── design-tokens.json
│   └── components.spec.json
├── scripts/                      ← Token generators
│   ├── gen-css-tokens.ts
│   └── gen-xaml-tokens.ts
├── roboforge-online/             ← React/Vite online frontend
├── ros_ws/                       ← ROS 2 workspace (shared backend)
│   └── src/
│       ├── roboforge_description/
│       ├── roboforge_bringup/
│       ├── roboforge_moveit/
│       ├── roboforge_simulation/
│       └── roboforge_bridge/
├── docker/                       ← Dockerfiles
│   ├── ros2/Dockerfile
│   ├── bridge/Dockerfile
│   └── gazebo/Dockerfile
├── docker-compose.sim.yml
├── proto/                        ← gRPC proto files
│   └── roboforge.proto
├── RoboForge.Desktop/            ← .NET WPF offline frontend
│   ├── Themes/
│   │   └── Tokens.xaml           ← generated by gen-xaml-tokens.ts
│   ├── Controls/
│   ├── Panels/
│   └── Viewport/
└── tests/
    ├── visual-parity.spec.ts     ← pixel-diff test
    ├── ik-roundtrip.spec.ts
    └── trajectory-parity.spec.ts
```

---

*End of RoboForge Unified Architecture Bible — v7.0*
*AI: Implement in order. Do not skip phases. All pipeline stages are mandatory.*
