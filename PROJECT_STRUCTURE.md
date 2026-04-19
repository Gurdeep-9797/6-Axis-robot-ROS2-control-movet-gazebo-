# RoboForge — Repository Structure

```
┌─────────────────────────────────────────────────────────────────┐
│                     RoboForge Project Root                        │
├─────────────────────────────────────────────────────────────────┤
│                                                                   │
│  📂 src/                      ← ROS 2 Workspace Source           │
│  ├── 📂 robot_description/    ← URDF robot model                 │
│  ├── 📂 robot_moveit_config/  ← MoveIt 2 motion planning         │
│  ├── 📂 robot_gazebo/         ← Gazebo simulation launch         │
│  ├── 📂 roboforge_bridge/     ← WebSocket + ROS 2 bridge (Python)│
│  ├── 📂 robot_msgs/           ← Custom ROS 2 messages            │
│  └── 📂 robot_hardware_bridge/← Hardware integration             │
│                                                                   │
│  📂 NEW_UI/                   ← Frontend Application             │
│  └── 📂 remix-of-roboflow-studio/ ← React/Vite IDE               │
│      ├── 📂 src/              ← React components & engine        │
│      ├── 📂 public/           ← Static assets                    │
│      ├── Dockerfile           ← Container definition             │
│      └── package.json         ← Dependencies                     │
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
