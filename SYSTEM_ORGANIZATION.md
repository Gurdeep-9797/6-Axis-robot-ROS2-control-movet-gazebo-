# RoboForge v8.2 — System Organization Map

## 📁 Directory Structure

```
Project Root/
│
├── 🌐 ONLINE SYSTEM (React + ROS2 Backend)
│   ├── NEW_UI/remix-of-roboflow-studio/     # React Online IDE
│   │   ├── src/components/robot/            # 32 robot-specific components
│   │   ├── src/store/AppState.tsx           # State management (773 lines)
│   │   ├── src/services/BackendConnector.ts # ROS2 WebSocket bridge
│   │   └── Dockerfile                       # Containerized deployment
│   │
│   ├── src/                                  # ROS2 Backend Source
│   │   ├── robot_description/               # URDF robot model (xacro)
│   │   ├── robot_moveit_config/             # MoveIt 2 motion planning
│   │   ├── robot_gazebo/                    # Gazebo simulation
│   │   ├── roboforge_bridge/                # WebSocket + REST bridge
│   │   ├── robot_msgs/                      # Custom ROS2 messages
│   │   ├── robot_hardware_bridge/           # Hardware integration
│   │   └── robot_analysis/                  # Accuracy analysis
│   │
│   ├── docker-compose.yml                   # Online system orchestration
│   ├── docker/Dockerfile.gazebo_vnc         # Gazebo VNC container
│   └── docker/entrypoint.sh                 # Gazebo VNC startup
│
├── 🖥️ OFFLINE SYSTEM (WPF Desktop Client)
│   └── src/RoboForge.Wpf/                   # .NET 8 WPF Application
│       ├── Models/                          # Robot structure models
│       │   └── RobotModel.cs                # Link, Joint, Sensor, etc.
│       ├── AST/                             # Program AST system
│       │   └── AstNodes.cs                  # All block types (MoveJ, IO, etc.)
│       ├── Core/                            # Execution engine
│       │   ├── StateBus.cs                  # Rx.NET state distribution
│       │   ├── Compiler.cs                  # AST → InstructionList
│       │   └── ExecutionEngines.cs          # Ghost + Real engines
│       ├── MainWindow.xaml                  # 3-column CAD workspace UI
│       └── MainWindow.xaml.cs               # UI logic + subscriptions
│
├── 📦 DEPLOYMENT
│   ├── deploy_package/                      # Portable offline package
│   ├── LAUNCH.ps1                           # One-click launcher
│   └── CREATE_DEPLOY_PACKAGE.ps1            # Package creator
│
├── 📚 DOCUMENTATION
│   ├── README.md                            # Master system overview
│   ├── ONLINE_SYSTEM.md                     # Online system guide (this file)
│   ├── OFFLINE_SYSTEM.md                    # Offline system guide
│   ├── QUICK_ACCESS.md                      # Quick reference
│   ├── TEST_SIMULATION_REPORT.md            # Test results
│   ├── VERIFICATION_REPORT.md               # Build verification
│   └── API_AND_CONNECTIONS.md               # Technical API reference
│
└── 🛠️ UTILITIES
    ├── monitor_pipeline.ps1                 # Real-time monitoring
    ├── test_simulation.ps1                  # Simulation test suite
    ├── live_simulation_monitor.ps1          # Live data monitor
    ├── verify_pipeline.ps1                  # Pipeline verification
    └── tools/
        └── parse_joints.py                  # Joint state parser
```

---

## ✅ Current Status

### Online System
- **React Online IDE**: ✅ HTTP 200 at http://localhost:3000
- **ROS2 Bridge**: ✅ WebSocket 9090 + REST 8765 healthy
- **MoveIt 2**: ✅ IK/FK services operational
- **Gazebo VNC**: ✅ 3D simulation at http://localhost:6080
- **Pseudo Hardware**: ✅ 250Hz joint state publishing
- **ROS Core**: ✅ Healthy (robot_state_publisher active)

### Offline System
- **WPF Build**: ✅ 0 errors, compiles successfully
- **Scene Graph**: ✅ TRS hierarchy with 6 articulated links
- **AST System**: ✅ 18 block types defined (MoveJ, MoveL, IO, Flow, etc.)
- **State Bus**: ✅ Rx.NET reactive state distribution
- **Compiler**: ✅ AST → InstructionList with control flow
- **Ghost Engine**: ✅ 60Hz simulation with interpolation
- **UI Layout**: ✅ 3-column CAD workspace with scene tree + 3D viewport

---

## 🔄 How to Rollback Changes

All changes are tracked via Git. To rollback:

```powershell
# View recent commits
git log --oneline -10

# Revert to specific commit
git checkout <commit-hash>

# Revert specific file
git checkout <commit-hash> -- path/to/file

# Create backup branch before major changes
git branch backup-$(Get-Date -Format "yyyy-MM-dd")
```

---

## 📊 Component Count

| Component | Count | Status |
|-----------|-------|--------|
| ROS2 Nodes | 8 | ✅ Running |
| ROS2 Topics | 20 | ✅ Active |
| ROS2 Services | 45+ | ✅ Active |
| React Components | 32 | ✅ Working |
| WPF Models | 6 | ✅ Defined |
| AST Node Types | 18 | ✅ Defined |
| Execution Engines | 2 | ✅ Working |
| Docker Containers | 6 | ✅ Running |

---

*Last Updated: 2026-04-14*
*System Version: v8.2*
*Build Status: All Green*
