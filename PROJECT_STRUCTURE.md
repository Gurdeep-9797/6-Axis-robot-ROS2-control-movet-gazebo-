# RoboForge — Repository Structure

```
┌─────────────────────────────────────────────────────────────────┐
│                     RoboForge Project Root                        │
├─────────────────────────────────────────────────────────────────┤
│                                                                   │
│  📂 src/                      ← Source Code                      │
│  ├── 📂 RoboForge.Wpf/        ← Offline Desktop Client (.NET 8)  │
│  │   ├── 📂 AST/              ← Program AST nodes                │
│  │   ├── 📂 Bridge/           ← ROS2 WebSocket client            │
│  │   ├── 📂 Core/             ← Execution engines, state bus     │
│  │   ├── 📂 Docking/          ← Panel docking system             │
│  │   ├── 📂 FileSystem/       ← .rfproj save/load                │
│  │   ├── 📂 Homing/           ← Robot homing sequence            │
│  │   ├── 📂 IO/               ← Device detection, handshake      │
│  │   ├── 📂 Models/           ← Robot structure models           │
│  │   ├── 📂 Themes/           ← XAML styles & colors             │
│  │   ├── MainWindow.xaml      ← Main UI layout                   │
│  │   └── RoboForge.Wpf.csproj ← Project file                    │
│  │                                                                 │
│  ├── 📂 robot_description/    ← URDF robot model (xacro)         │
│  ├── 📂 robot_moveit_config/  ← MoveIt 2 motion planning         │
│  ├── 📂 robot_gazebo/         ← Gazebo simulation launch         │
│  ├── 📂 roboforge_bridge/     ← WebSocket + REST bridge          │
│  ├── 📂 robot_msgs/           ← Custom ROS2 messages             │
│  └── 📂 robot_hardware_bridge/← Hardware integration              │
│                                                                   │
│  📂 NEW_UI/                   ← Online System                    │
│  └── 📂 remix-of-roboflow-studio/ ← React Online IDE             │
│      ├── 📂 src/              ← React components                 │
│      ├── 📂 public/           ← Static assets                    │
│      ├── Dockerfile           ← Container definition             │
│      └── package.json       ← Dependencies                      │
│                                                                   │
│  📂 docker/                   ← Docker configurations            │
│  ├── Dockerfile.gazebo_vnc    ← Gazebo VNC container             │
│  └── entrypoint.sh          ← VNC startup script                 │
│                                                                   │
│  📂 tools/                    ← Utility scripts                  │
│  └── parse_joints.py        ← Joint state parser                │
│                                                                   │
│  📂 deploy_package/           ← Portable deployment              │
│                                                                   │
│  📂 .github/                  ← GitHub CI/CD                     │
│  └── 📂 workflows/          ← Build automation                   │
│                                                                   │
│  📄 docker-compose.yml        ← Service orchestration            │
│  📄 LAUNCH.ps1               ← One-click launcher                │
│  📄 README.md                ← Main documentation                │
│  📄 ONLINE_SYSTEM.md         ← Online system guide               │
│  📄 OFFLINE_SYSTEM.md        ← Offline system guide              │
│  📄 CODE_REVIEW.md           ← Complete code review              │
│  └── 📄 VERIFICATION_REPORT.md ← Build verification              │
└─────────────────────────────────────────────────────────────────┘
```
