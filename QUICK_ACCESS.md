# RoboForge v8.2 — Quick Access Guide

## 🌐 Web Interfaces (All Running Now!)

| Interface | URL | Purpose |
|-----------|-----|---------|
| **React Online IDE** | http://localhost:3000 | Full robot programming IDE with 3D viewport |
| **Gazebo GUI (VNC)** | http://localhost:6080/vnc.html | 3D simulation with physics, visual robot control |
| **Bridge REST API** | http://localhost:8765/health | System health check endpoint |
| **Pipeline Monitor** | Run: `.\monitor_pipeline.ps1` | Real-time terminal dashboard |

---

## 📺 Accessing Gazebo GUI

1. **Open**: http://localhost:6080/vnc.html in your browser
2. **Click**: "Connect" button (no password required for view-only)
3. **Password** (if prompted): `roboforge`
4. **You'll see**: Gazebo 3D simulation window with robot model

### Gazebo GUI Features:
- 3D viewport with robot model
- Scene hierarchy (left panel)
- Component inspector (right panel)
- World settings and physics
- Real-time simulation controls (play/pause/step)

---

## 📊 Data Flow Verification

### Complete Pipeline:
```
┌─────────────────────────────────────────────────────────────┐
│                    GAZEBO SIMULATION                         │
│  - 3D Physics Engine (Ignition Gazebo Harmonic)             │
│  - Robot Model with 6 DOF joints                            │
│  - Real-time joint state publishing @ 250Hz                 │
└───────────────────────┬─────────────────────────────────────┘
                        │
                        ├─ /joint_states (positions, velocities)
                        ├─ /tf (transform frames)
                        ├─ /robot_description (URDF)
                        │
                        ▼
┌─────────────────────────────────────────────────────────────┐
│                    ROS2 MIDDLEWARE                           │
│  Topics:                                                     │
│    /joint_states              ← Robot joint positions        │
│    /joint_trajectory_command  ← Movement commands            │
│    /planned_trajectory        ← MoveIt planned paths         │
│    /tf                        ← Coordinate transforms        │
│                                                                        │
│  Services:                                                   │
│    /compute_ik                ← Inverse kinematics           │
│    /compute_fk                ← Forward kinematics           │
│    /roboforge/health_check    ← System health                │
└───────────────────────┬─────────────────────────────────────┘
                        │
                        ▼
┌─────────────────────────────────────────────────────────────┐
│                    BRIDGE LAYER                              │
│  WebSocket (port 9090):                                      │
│    - Proxies ROS2 topics to web clients                     │
│    - IK/FK service calls to MoveIt                          │
│    - Trajectory execution feedback                           │
│                                                                        │
│  REST API (port 8765):                                       │
│    - /health            → System status                     │
│    - /api/hardware/ports → Hardware config                  │
│    - /api/logs          → Execution logs                    │
└───────────────────────┬─────────────────────────────────────┘
                        │
                        ▼
┌─────────────────────────────────────────────────────────────┐
│                    FRONTEND APPLICATIONS                     │
│                                                                        │
│  React Online IDE (port 3000):                               │
│    - 3D viewport with Three.js                              │
│    - Program tree editor                                    │
│    - Waypoint management                                  │
│    - Real-time joint state display                          │
│    - IK computation & visualization                         │
│    - Execution console                                     │
│                                                                        │
│  Gazebo VNC GUI (port 6080):                                 │
│    - Native Gazebo 3D interface                             │
│    - Physics simulation controls                            │
│    - Scene outliner & property editor                       │
│    - Direct robot visualization                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 🔍 Comparing Values Between Interfaces

### Joint Positions:
- **Gazebo GUI**: Left panel → "joint_state_publisher" → expand to see positions
- **React IDE**: Scene Outliner → Joint States panel
- **Terminal**: `docker exec roboforge_core bash -c "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && ros2 topic echo /joint_states --once"`

### Robot Pose:
- **Gazebo GUI**: Visual 3D model shows exact pose
- **React IDE**: 3D viewport renders robot mesh (may differ if using offline IK)
- **Verification**: Both should match when in Live mode

### IK Solutions:
- **React IDE**: Click in 3D viewport → IK computed → joint angles updated
- **Gazebo GUI**: Observe robot moving to target position
- **Verification**: Compare final joint positions in both interfaces

---

## 🛠️ Useful Commands

### Check System Status:
```powershell
.\verify_pipeline.ps1
```

### Real-Time Monitoring:
```powershell
.\monitor_pipeline.ps1
```

### View Gazebo Logs:
```powershell
docker logs roboforge_gazebo --tail 50 -f
```

### View Bridge Logs:
```powershell
docker logs 9f5db7628868_roboforge_bridge --tail 50 -f
```

### Restart Gazebo:
```powershell
docker restart roboforge_gazebo
```

### Stop All Services:
```powershell
docker compose down
```

---

## 📋 Pipeline Reality Flow Checklist

Use this to verify complete data flow:

- [ ] **Gazebo GUI loads** at http://localhost:6080/vnc.html
- [ ] **Robot model visible** in Gazebo 3D viewport
- [ ] **Joint states updating** in Gazebo (left panel)
- [ ] **React IDE loads** at http://localhost:3000
- [ ] **Connection status shows "Live"** in React IDE (top-right indicator)
- [ ] **3D viewport shows robot** in React IDE
- [ ] **Joint states match** between Gazebo and React IDE
- [ ] **IK service works**: Click in React 3D viewport → robot moves
- [ ] **Movement visible in Gazebo**: Robot moves when commanded from React IDE
- [ ] **REST API healthy**: http://localhost:8765/health returns `"status": "ok"`

---

## 🎯 Next Steps

1. **Open Gazebo GUI**: Visit http://localhost:6080/vnc.html and click Connect
2. **Open React IDE**: Visit http://localhost:3000
3. **Arrange windows side-by-side** to compare values in real-time
4. **Run a test program** in React IDE and observe in Gazebo
5. **Use pipeline monitor**: `.\monitor_pipeline.ps1` for live metrics

---

*System Version: v8.2 | All services operational*
