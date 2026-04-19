# Audit Results — 2026-04-15

## System Overview

| Item | Value |
|---|---|
| **Frontend** | React 18.3.1 + Vite 5.4 + TypeScript 5.8 |
| **UI Framework** | TailwindCSS 3.4 + Radix UI + shadcn/ui + Lucide icons |
| **State Management** | React Context (AppState.tsx) + Zustand (RosConnection.ts) |
| **Backend** | No standalone backend server. ROS 2 bridge node (Python) acts as backend via WebSocket |
| **ROS 2 Distro** | ROS 2 Humble (confirmed in docker-compose.yml: `source /opt/ros/humble/setup.bash`) |
| **ROS 2 Bridge Method** | roslib.js (npm `roslib ^2.1.0`) → WebSocket to rosbridge_suite on port 9090 |
| **MoveIt Version** | MoveIt 2 (for ROS 2 Humble) |
| **Gazebo Version** | Gazebo Ignition/Harmonic (`ros_gz_sim`, `ros_gz_bridge`, `gz_ros2_control` in package.xml) |
| **IK Solver** | TRAC-IK (`trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin` in kinematics.yaml) — ✅ already configured |
| **Path Planner** | OMPL (RRTConnect, RRTstar, PRM) — PILZ is **NOT** configured |
| **3D Renderer** | Three.js 0.170 via @react-three/fiber 8.18 + @react-three/drei 9.122 |
| **ros2_control** | Yes — `gz_ros2_control` in URDF xacro files, `FollowJointTrajectory` controller configured |

## File Inventory

### Frontend Components (src/components/robot/)

| Component | File | Status |
|---|---|---|
| Layout (Main) | `pages/Index.tsx` | ✅ Exists — uses `react-resizable-panels` |
| Left Icon Sidebar | `LeftSidebar.tsx` | ✅ Exists — 11 icons (Program, Blocks, Scene, 3D View, Motion, IO, Debug, Config, ROS2, Settings, Project) |
| Left Detail Panel | `pages/Index.tsx` lines 71-79 | ✅ Exists — renders ProgramTreePanel, BlockLibraryPanel, SceneOutlinerPanel, ProjectManagerPanel based on activePanel. **BUT motion/io/diagnostics/config/ros2 panels render a placeholder instead of their actual panels** |
| Right Inspector | `InspectorPanel.tsx` | ⚠️ **PROBLEM**: Contains Properties, I/O, Debug, AND ROS 2 tabs — ROS 2 should NOT be here |
| Top Toolbar | `TopRibbon.tsx` | ✅ Exists |
| Program Tree | `ProgramTreePanel.tsx` | ✅ Exists |
| Block Library | `BlockLibraryPanel.tsx` | ✅ Exists |
| 3D Viewport | `Viewport3D.tsx` | ✅ Exists — Three.js with react-three-fiber |
| Robot Model | `IndustrialRobot.tsx` | ✅ Exists — FK-based joint rendering |
| Properties Panel | `PropertiesPanel.tsx` | ✅ Exists — block properties + waypoint properties |
| Code Editor | `CodeEditor.tsx` | ✅ Exists |
| Console Panel | `ConsolePanel.tsx` | ✅ Exists |
| Diagnostics Panel | `DiagnosticsPanel.tsx` | ✅ Exists |
| I/O Panel | `IOPanel.tsx` | ✅ Exists |
| ROS 2 Panel | `ROS2Panel.tsx` | ✅ Exists |
| Robot Config | `RobotConfigPanel.tsx` | ✅ Exists — Overview, DH, Joints, Links, TCP, Hardware tabs |
| Hardware Config (full page) | `HardwareConfigPage.tsx` | ✅ Exists |
| Settings Page | `SettingsPage.tsx` | ✅ Exists |
| Settings Panel | `SettingsPanel.tsx` | ✅ Exists |
| Jog Panel | `JogPanel.tsx` | ✅ Exists |
| Motion Planning | `MotionPlanningPanel.tsx` | ✅ Exists |
| Scene Outliner | `SceneOutlinerPanel.tsx` | ✅ Exists |
| Project Manager | `ProjectManagerPanel.tsx` | ✅ Exists |
| Gizmo Manipulator | `GizmoManipulator.tsx` | ✅ Exists |
| Path Visualization | `PathVisualization.tsx` | ✅ Exists |
| Timeline | `TimelinePanel.tsx` | ✅ Exists |
| Block Context Menu | `BlockContextMenu.tsx` | ✅ Exists |
| Block Config Popup | `BlockConfigPopup.tsx` | ✅ Exists |
| Health Check Modal | `HealthCheckModal.tsx` | ✅ Exists |
| Live Telemetry | `LiveTelemetryWidget.tsx` | ✅ Exists |
| Tracking Error | `TrackingErrorWidget.tsx` | ✅ Exists |

### Engine Layer (src/engine/)

| File | Purpose | Status |
|---|---|---|
| `IKSolver.ts` | Local analytical + numerical IK solver | ✅ Exists |
| `MotionTypes.ts` | IRB6700 joint limits, zone types, motion types | ✅ Exists |
| `TrajectoryPlanner.ts` | Blended path generation | ✅ Exists |

### Services (src/services/)

| File | Purpose | Status |
|---|---|---|
| `BackendConnector.ts` | Routes IK to local solver or MoveIt via rosbridge | ✅ Exists |

### State (src/store/)

| File | Purpose | Status |
|---|---|---|
| `AppState.tsx` | Main application state context (waypoints, blocks, mode, etc.) | ✅ Exists |
| `RosConnection.ts` | ROSLIB WebSocket connection + topic/service management | ✅ Exists |

### ROS 2 Packages (src/)

| Package | Purpose | Status |
|---|---|---|
| `roboforge_bridge` | Main bridge node (WebSocket + trajectories + IK + motor control) | ✅ Exists |
| `robot_description` | URDF/Xacro files + joint_limits.yaml | ✅ Exists |
| `robot_moveit_config` | MoveIt 2 config: kinematics, OMPL, controllers, SRDF, trajectory_execution | ✅ Exists |
| `robot_gazebo` | Gazebo world + launch files | ✅ Exists |
| `robot_hardware_bridge` | Real/Sim backend (bridge_node, sim_backend, real_backend, moveit_adapter) | ✅ Exists |
| `robot_msgs` | Custom messages | ✅ Exists |
| `robot_analysis` | Analysis tools | ✅ Exists |
| `robot_tests` | Test scripts | ✅ Exists |

### ROS 2 Bridge Node Files (src/roboforge_bridge/roboforge_bridge/)

| File | Purpose |
|---|---|
| `bridge_node.py` | Main WebSocket bridge + trajectory execution |
| `pseudo_hardware_node.py` | 250Hz pseudo-hardware simulator |
| `motor_controller_node.py` | Motor PWM control |
| `encoder_interface_node.py` | Encoder data handling |
| `kinematics.py` | FK/IK solver (Python backend) |
| `gazebo_error_node.py` | Gazebo error handling |

### Docker Services (docker-compose.yml)

| Service | Container | Port | Status |
|---|---|---|---|
| ros_core | roboforge_core | — | ✅ Launches robot_description display.launch.py |
| moveit | roboforge_moveit | — | ✅ Launches move_group.launch.py |
| pseudo_hardware | roboforge_pseudo_hw | — | ✅ Runs pseudo_hardware_node |
| gazebo | roboforge_gazebo | — | ✅ Optional profile=sim |
| bridge | roboforge_bridge | 9090 (WebSocket), 8765 (REST) | ✅ Runs roboforge_bridge |
| frontend | roboforge_frontend | 3000→8080 | ✅ Serves built React app |

### Tools

| Tool | Path | Status |
|---|---|---|
| sw2urdf (SolidWorks exporter) | `tools/SolidWorksExporter/` | ✅ Exists — C# SolidWorks Add-In |

---

## Phase 0 Audit Checklist — Results

```
[x] Identify frontend framework: React 18 + Vite + TypeScript + TailwindCSS
[x] Identify backend: No standalone server — ROS 2 bridge Python node acts as backend
[x] Identify ROS 2 distro: Humble
[x] Identify ROS 2 bridge method: roslib.js via WebSocket to rosbridge on port 9090
[x] Identify MoveIt version: MoveIt 2
[x] Identify Gazebo version: Gazebo Ignition/Harmonic (ros_gz_sim)
[x] Identify ros2_control: Yes — gz_ros2_control + FollowJointTrajectory controller
[x] Read Program Tree / Block Editor: ProgramTreePanel.tsx, BlockLibraryPanel.tsx, BlockContextMenu.tsx, BlockConfigPopup.tsx
[x] Read 3D Viewport: Viewport3D.tsx — Three.js via @react-three/fiber
[x] Read ROS 2 Bridge panel: ROS2Panel.tsx — CURRENTLY IN RIGHT INSPECTOR (wrong position)
[x] Check URDF loader/parser: EXISTS — IndustrialRobot.tsx uses FK chain, URDF xacros on ROS side
[x] Check config tab: EXISTS — RobotConfigPanel.tsx + HardwareConfigPage.tsx
[x] Check motor/encoder config: EXISTS (basic) — JointMotorConfig in RobotConfigPanel (DC/BLDC toggle, PID gains)
[x] List active ROS 2 nodes: /move_group, /roboforge_bridge, /pseudo_hardware_node, /robot_state_publisher
[x] List published topics: /joint_states, /tcp_pose, /trajectory
[x] List subscribed topics: /cmd_joint, /cmd_pose, /external_trigger
[x] Check IK solver: TRAC-IK configured in kinematics.yaml ✅
[x] Check path planner: OMPL (RRTConnect) configured. PILZ NOT configured ⚠️
[x] Identify Gazebo launch: Docker container via docker-compose (profile=sim)
[x] Check IO subsystem: IOPanel.tsx exists with digital/analog in/out
[x] Identify file storage: Local filesystem, Docker volumes mount src/
[x] Check sw2urdf: EXISTS at tools/SolidWorksExporter/
```

---

## Key Issues Found (To Fix in Subsequent Phases)

### Phase 1 Issues (UI Layout)
1. **ROS 2 Panel in wrong location**: `InspectorPanel.tsx` renders ROS2Panel as a tab in the RIGHT panel. According to spec, it must be in the LEFT detail panel.
2. **Left panel missing content for motion/io/diagnostics/config/ros2**: `Index.tsx` line 76-78 shows a placeholder div instead of rendering the actual panels (MotionPlanningPanel, IOPanel, DiagnosticsPanel, RobotConfigPanel, ROS2Panel).

### Phase 3 Issues (Pipeline)
1. **PILZ Industrial Motion Planner NOT configured**: Only OMPL RRTConnect is available. Need to add PILZ for PTP/LIN/CIRC motion types.
2. **joint_limits.yaml exists** ✅ with velocity + acceleration limits for all 6 joints.
3. **Trajectory time parameterization**: `AddTimeOptimalParameterization` is configured as a response adapter ✅.

### Phase 8 Issues (Config Tab)
1. **Motor type options incomplete**: Only DC/BLDC toggle exists. Spec requires: BLDC-Sinusoidal, BLDC-Trapezoidal, DC Brushed, Stepper-Full, Stepper-Micro, Integrated Servo.
2. **Encoder config missing**: No encoder type/resolution/interface fields in current UI.
3. **Communication section missing**: No Serial/EtherCAT/UDP config.
4. **No /api/config endpoints**: No REST API for config CRUD.

### Phase 9 Issues (URDF Import)
1. **No server-side STEP-to-URDF converter**: Only the SolidWorks Add-In exists.
2. **No URDF upload/import UI**: No drag-and-drop URDF upload feature.
3. **No post-import detection popup**.
