## Audit Results — 2026-04-18

### Core Stack
- **Frontend Framework**: React 18.3.1 (Vite + TypeScript)
- **Styling**: TailwindCSS 3.4.17 + Shadcn UI (Radix)
- **3D Engine**: Three.js 0.170.0 (@react-three/fiber + @react-three/drei)
- **State Management**: Zustand 4.5.2
- **Backend/ROS 2 Bridge**: 
    - **Primary**: Python `rclpy` node (`roboforge_bridge`) with `websockets` + `aiohttp`.
    - **Mock**: Node.js WebSocket bridge (`server.js`) on port 9090.
- **ROS 2 Distribution**: Humble
- **Communication Protocol**: rosbridge v2 compatible JSON over WebSockets (Port 9090)
- **MoveIt version**: MoveIt 2 (humble)

### Robotics Components
- **Robot Model**: IRB 6700 (referenced in multiple files)
- **IK Solver**: MoveIt `/compute_ik` service + Analytical fallback in `bridge_node.py` and `IKSolver.ts`.
- **Trajectory Planning**: 
    - **Frontend**: Trapezoidal velocity profiles + Quadratic Bézier blending (`TrajectoryPlanner.ts`).
    - **Backend**: Cubic Hermite Spline interpolation (`pseudo_hardware_node.py`).
- **Hardware Interface**: `pseudo_hardware_node` (250Hz simulation). **Real Hardware Interface missing**.
- **Gazebo Integration**: `robot_gazebo` package exists. Physics simulation referenced in `OnlineUpdate.md`.

### UI Layout (Status)
- **Requested**: ROS 2 Bridge and Debug panels moved to LEFT detail panel.
- **Current**: Need verification via browser or CSS audit. React component tree suggests a standard sidebar + main area.

### Missing Features (based on OnlineUpdate.md)
- [ ] Phase 1: Sidebar-centric layout corrections.
- [ ] Phase 6/7: Real-time hardware handshake and PID control loop (non-simulated).
- [ ] Phase 8: Comprehensive Robot Configuration Tab (UI + Backend Persistence).
- [ ] Phase 9: URDF/STEP Import Pipeline (FreeCAD-based server-side conversion).
- [ ] Phase 10: Performance metrics and Gazebo headless optimization.
- [ ] Phase 11: Premium Glassy UI theme pass.

### Kinematics Params (IRB 6700)
- **Joints**: 6 (revolute)
- **IK Geometry**: Spherical wrist (J4-J5-J6), standard industrial topology.
- **Limits**: Configured in `pseudo_hardware_node.py`.
