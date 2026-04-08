# ROBOFORGE — GLOBAL SYSTEM STATE v8.2
### Canonical Reference | Updated: 2026-04-09

> **Purpose**: Single source of truth for the entire RoboForge project.  
> Every feature, file, status, and responsibility is tracked here.  
> Both AI instances MUST update this file as work progresses.

---

## 1. REPOSITORY MAP (Complete)

```
d:\Projects\ROBOT\6-Axis-robot-ROS2-control-movet-gazebo-\
│
├── NEW_UI/remix-of-roboflow-studio/          ── ONLINE WEB APP
│   └── src/
│       ├── pages/Index.tsx                   Layout shell (LayoutSlot system)
│       ├── store/
│       │   ├── AppState.tsx                  Central React state (RESTORED ✅)
│       │   └── RosConnection.ts             Zustand store + legacy ROS class
│       ├── engine/
│       │   ├── IKSolver.ts                  JS IK: analytical + numerical DLS ✅
│       │   ├── TrajectoryPlanner.ts         Path blending + quintic interpolation
│       │   └── MotionTypes.ts              IRB6700 DH params + joint limits
│       ├── services/
│       │   └── BackendConnector.ts         Auto-connect: local→tunnel→offline ✅
│       └── components/robot/
│           ├── layout/LayoutSlot.tsx        Dynamic panel slot system ✅
│           ├── Viewport3D.tsx              Three.js scene + gizmos ✅
│           ├── IndustrialRobot.tsx         IRB 6700 3D mesh
│           ├── TopRibbon.tsx               Toolbar + mode switcher ✅
│           ├── LeftSidebar.tsx             VS Code-style activity bar ✅
│           ├── CodeEditor.tsx              Block + Script editor
│           ├── IOPanel.tsx                 I/O signals (ROS-connected) ✅
│           ├── PropertiesPanel.tsx         Waypoint/object inspector
│           ├── ConsolePanel.tsx            Dev console output
│           ├── SceneOutlinerPanel.tsx      Scene tree (inline rename ✅)
│           ├── DiagnosticsPanel.tsx        Diagnostics tab
│           ├── ROS2Panel.tsx               ROS bridge status
│           ├── HardwareConfigPage.tsx      Motor/encoder hardware config ✅
│           ├── JogPanel.tsx                Manual joint jog
│           ├── SettingsPage.tsx            App settings full page
│           └── BlockLibraryPanel.tsx       Drag-drop block library
│
├── src/
│   ├── RoboForge.Wpf/                        ── OFFLINE NATIVE APP (WPF)
│   │   ├── MainWindow.xaml                  Native WPF layout (4-column grid)
│   │   ├── ViewModels.cs                    MVVM ViewModel (WebSocket, IK, IO) ✅
│   │   ├── Converters.cs                    WPF value converters
│   │   ├── Themes/Styles.xaml              Dark premium theme ✅
│   │   └── RoboForge.Wpf.csproj            .NET 8, WPF, CommunityToolkit.Mvvm
│   │
│   ├── RoboForge.ROS2Bridge/                 ── C# ROS2 BRIDGE SERVICE
│   │   ├── IRos2BridgeService.cs            Interface: IK, MoveGroup, IO ✅
│   │   └── Ros2BridgeService.cs            Implementation (mock skeleton)
│   │
│   ├── RoboForge.Api/                        REST API server
│   ├── RoboForge.Application/               Application layer (CQRS)
│   ├── RoboForge.Domain/                    Domain models (Pose, IKResult, etc.)
│   └── RoboForge.Execution/                 Trajectory execution engine
│
├── src/roboforge_bridge/                     ── PYTHON ROS2 BRIDGE
│   └── bridge_node.py                       WebSocket :9090, REST :8765
│
├── bridge-server/                            Node.js bridge alternative
├── docker-compose.yml                        ROS2 + MoveIt + Gazebo stack
├── version-lock.yaml                         All pinned versions ✅
├── ROBOFORGE_UNIFIED_ARCHITECTURE.md        Full architecture bible
├── ROBOFORGE_GLOBAL_SYSTEM.md              ← THIS FILE
└── AI_WORKFLOW.md                           Combined AI task coordination
```

---

## 2. VERSION LOCK (from version-lock.yaml)

| Component | Version | Notes |
|-----------|---------|-------|
| .NET | 8.0 (net8.0-windows) | WPF target framework |
| ROS 2 | Humble | Docker, rmw_cyclonedds |
| Gazebo | Harmonic | Docker |
| rosbridge port | :9090 | WebSocket |
| ROS Domain ID | 42 | FastDDS |
| Robot model | IRB 6700-235/2.65 | Max reach 2.65m |
| MSVC | 19.50.35724 | VS 2026 Community |
| CMake | 4.1.2 | |
| ImGui | 1.91.8 | vcpkg |

---

## 3. BUILD STATUS

| Component | Build | Runtime | Notes |
|-----------|-------|---------|-------|
| **Online UI (React/Vite)** | ✅ CLEAN | ✅ RUNNING :8080 | 2762 modules, 0 errors |
| **WPF Offline App** | ✅ CLEAN | ✅ READY | Program Tree + Jog sliders implemented |
| **C# ROS2Bridge** | ✅ COMPILES | 🔲 MOCK | SolveIKAsync = stub |
| **Python roboforge_bridge** | ✅ EXISTS | 🔴 NEEDS DOCKER | :9090 WebSocket |
| **ROS 2 Backend** | 🔴 NOT RUNNING | 🔴 OFFLINE | Docker stack needed |

---

## 4. ONLINE UI — EXACT FEATURE STATUS

### ✅ WORKING
- 3D Viewport (Three.js, IRB 6700, gizmos, orbit controls, right-click add menu)
- Block editor with drag/drop and context menu
- IK Solver: offline-numerical (DLS with joint regularization)
- IK Solver: offline-analytical (full arm kinematics)
- BackendConnector: auto-detect local→tunnel→offline
- I/O Panel: toggle DO, read DI, analog sliders, ROS publish, offline sim
- Scene Outliner: object list, inline rename, lock/hide
- Top Ribbon: toolbar, mode switcher, IK mode toggle, motor PWM, bridge status
- LayoutSlot system: 3 switchable slots (explorer, bottom, inspector)
- GlobalStartPoint: "Home" button in toolbar sends robot to home config
- Settings page: full-screen overlay
- Health check modal: pre-flight check before Live mode
- Hardware config page: motor + encoder per-joint settings

### ⚠️ NEEDS WORK
- PropertiesPanel: not shown by default in inspector slot (now wired to 'config')
- GizmoManipulator: rotation not fully persisted for waypoints
- Waypoint orientation (quat) not visualized as TCP frame in 3D

### 🔲 TODO
- Program save/load: localStorage working, need file I/O (browser download)
- Multi-robot: 2nd robot joint angles separate from first

---

## 5. OFFLINE WPF APP — EXACT FEATURE STATUS

### ✅ WORKING
- Dark premium theme (Styles.xaml)
- WebSocket auto-connect: local→tunnel→offline (ViewModels.cs)
- Joint state parsing from ROS /joint_states topic
- Tracking error display
- Health check service call
- MoveIt IK call via rosbridge WebSocket
- Motor speed (PWM) slider
- Console log entries
- Mode toggles: Edit / Simulate / Live
- Program blocks list (ObservableCollection)
- **Program Tree panel with status indicators** (NEW ✅)
- **Joint Jog sliders J1-J6 with mode selection** (NEW ✅)
- **3D Viewport with HelixToolkit placeholder** (NEW ✅)
- **Tabbed right panel: Properties/Jog/Diagnostics** (NEW ✅)

### ⚠️ NEEDS WORK
- 3D robot viewport needs actual URDF mesh loading
- No I/O toggle UI yet
- No block execution visualization

### 🔲 TODO (Iterative Replacement Plan)
1. ~~Replace left panel placeholder → Program Tree~~ DONE ✅
2. ~~Add joint jog sliders in right panel~~ DONE ✅
3. Add I/O toggle grid below console
4. Wire HelixToolkit viewport to JointAngles for live robot display (needs URDF loader)
5. Connect C# IKSolver (offline) to block execution

---

## 6. SHARED BACKEND (ROS 2)

### Connection Priority (BOTH apps follow same order)
```
1. ws://localhost:9090   → local Docker rosbridge  (3s timeout)
2. TunnelUrl (config)   → cloudflare tunnel        (3s timeout)  
3. Offline mode         → JS/C# local solver only
```

### ROS Topics Used
| Topic | Type | Direction | Purpose |
|-------|------|-----------|---------|
| /joint_states | sensor_msgs/JointState | SUB | Robot feedback 50Hz |
| /joint_trajectory_command | trajectory_msgs/JointTrajectory | PUB | Motion commands |
| /roboforge/tracking_error | std_msgs/String (JSON) | SUB | Error diagnostics |
| /roboforge/health_status | std_msgs/String (JSON) | SUB | Health check results |
| /io/{name} | std_msgs/Bool or Float64 | PUB | I/O signal control |
| /roboforge/scene_update | std_msgs/String (JSON) | PUB | Collision objects |

### ROS Services Used
| Service | Type | Purpose |
|---------|------|---------|
| /compute_ik | moveit_msgs/GetPositionIK | IK solving via MoveIt |
| /roboforge/health_check | std_srvs/Trigger | Pre-flight check |

---

## 7. KNOWN BUGS & ISSUES

| # | Severity | Component | Issue | Status |
|---|----------|-----------|-------|--------|
| 1 | ~~HIGH~~ | Online | Block execution loses IK seed between blocks | FIXED ✅ |
| 2 | ~~HIGH~~ | WPF | Program Tree panel shows placeholder | FIXED ✅ |
| 3 | ~~MED~~ | Online | LayoutSlot used static JSX (stale nodes) | FIXED ✅ |
| 4 | ~~MED~~ | Online | setActivePanel doesn't update LayoutSlot | FIXED ✅ |
| 5 | ~~MED~~ | WPF | No Jog panel | FIXED ✅ |
| 6 | ~~MED~~ | Online | ConsolePanel not in slot dropdown | FIXED ✅ |
| 7 | LOW | Online | Waypoint rotation not visualized | TODO |
| 8 | LOW | WPF | C# IKSolver is a stub | TODO |
| 9 | MED | WPF | No I/O toggle UI | TODO |
| 10 | MED | WPF | 3D robot needs URDF mesh loading | TODO |

---

## 8. IK POLICY (ENFORCED STRICTLY)

```
Online:  BackendConnector.computeIK() is the ONLY entry point
  └─ ikMode=moveit         → /compute_ik ROS service (backend required)
  └─ ikMode=offline-*      → IKSolver.solveIK() (local JS, no backend)
  └─ Default when offline  → offline-numerical (auto-detected)

WPF:    MainViewModel.ComputeMoveItIKAsync() for MoveIt
  └─ Fallback              → C# IKSolverService (offline analytical+numerical)
```

---

## 9. AI TASK DIVISION

### ANTIGRAVITY — Online Web App (React/Vite)
**Files owned**: `NEW_UI/remix-of-roboflow-studio/src/**`

| Task | Priority | Status |
|------|----------|--------|
| Fix LayoutSlot stale JSX | HIGH | DONE ✅ |
| Wire ConsolePanel to dropdown | MED | DONE ✅ |
| Fix setActivePanel→slot mapping | MED | DONE ✅ |
| Add GlobalStartPoint set-via-UI | MED | DONE ✅ |
| Fix IK seed persistence in block exec | HIGH | DONE ✅ |
| Improve error messages for IK failure | MED | TODO |
| PropertiesPanel in inspector slot | LOW | DONE ✅ |

### QWEN — Offline WPF App
**Files owned**: `src/RoboForge.Wpf/**`, `src/RoboForge.ROS2Bridge/**`

| Task | Priority | Status |
|------|----------|--------|
| Replace left panel placeholder with ProgramTree | HIGH | DONE ✅ |
| Add joint jog sliders (right panel) | HIGH | DONE ✅ |
| Add I/O toggle grid | MED | TODO |
| Wire HelixToolkit to joint angles | HIGH | TODO |
| Implement C# offline IK solver | MED | TODO |
| Connect C# IK to block execution | MED | TODO |
| Build verification | HIGH | DONE ✅ |

### BOTH — Shared
| Task | Priority |
|------|----------|
| Python bridge_node: add gRPC endpoint for WPF | HIGH |
| Docker-compose: test healthcheck end-to-end | MED |
| URDF: verify joint names match code (joint_1..joint_6) | HIGH |
