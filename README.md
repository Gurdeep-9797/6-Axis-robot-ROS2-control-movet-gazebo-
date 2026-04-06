# RoboForge v8.0 

[![Build & Release](https://github.com/Gurdeep-9797/6-Axis-robot-ROS2-control-movet-gazebo-/actions/workflows/build.yml/badge.svg)](https://github.com/Gurdeep-9797/6-Axis-robot-ROS2-control-movet-gazebo-/actions/workflows/build.yml)

A professional full-stack advanced industrial robot control system featuring a **React Online Web-IDE**, a **C# WPF Offline Client**, backed by **ROS 2 / MoveIt Planning** and an **ESP32 Real-Time Hardware Bridge**.

## v8.0 Architecture & Features

RoboForge v8.0 acts as a modern, Blender-style 3D CAD environment and visual node-based programmer integrated directly into the physical robotic workspace.

### Key Upgrades
- **Blender-Style 3D Workspace**: A fully interactive 3D viewport featuring `@react-three/drei` Cartesian Transform Controls. Click and drag geometric shapes and waypoints directly in 3D space with professional gizmos.
- **Bi-Directional Scene Outliner**: Real-time synchronization between the Worktree and the 3D viewport. Drop primitive solids (box, sphere, cylinder) directly into the virtual workspace via the floating context menu.
- **Real-Time Hardware Telemetry Pipeline**: The frontend accurately reflects 250Hz hardware encoder joint definitions utilizing the ROS 2 WebSocket bridge. 
- **Universal Parity**: Seamless telemetry and interface parity across the Web Browser Simulator and the offline `.NET 8 WPF` executable.

## Architecture

```text
┌─────────────────────────────────────────────────────────────┐
│  Frontends (React & WPF)    ──── rosbridge WebSocket ────┐  │
│  • Full 3D Editor (Gizmos)                               │  │
│  • Visual Scene Outliner                                 ▼  │
│  • Diagnostics & Jog                                MoveIt 2│
│                                                          │  │
│                          Hardware Bridge (ROS Node)      │  │
│                          • Real/Simulated Fallbacks      │  │
│                                                          │  │
│                          ┌────────┴────────┐             │  │
│                     WiFi (TCP)        Gazebo Harmonic    │  │
│                          │                               │  │
│                    ESP32 Controller                      │  │
│                    • 250Hz Encoders                      │  │
│                    • PID FOC Support                     │  │
└─────────────────────────────────────────────────────────────┘
```

## Setup & Deployment

### 1. Zero-Friction Pre-Requisites (Windows)
To automatically install Visual Studio 2022 Desktop Workloads, .NET 8.0 SDK, and CMake:
```powershell
.\tools\install_vs_tools.ps1
```

### 2. Launch the Entire Stack
The automated bootstrap script pulls up the ROS 2 container ecosystem (MoveIt, Gazebo, Bridge) and serves the React Web IDE seamlessly.
```powershell
.\run_roboforge.ps1
```

### 3. Build the Offline Client (Optional)
If a web browser is unavailable in your cleanroom environment:
```powershell
.\build_wpf.ps1
```

## Physical Hardware Handshake (Encoder Calibration)

When attaching RoboForge to physical cobots:
1. Validate connectivity against the Bridge Node using `tests/real_hardware_loop_test.py`.
2. Hardware Mode will automatically reject motion controls without first passing the *Pre-Flight 11-Point Health Check*. 
3. If hardware connects, the `TrackingErrorWidget` will actively gauge encoder derivation against commanded velocities.

---
*Built with ROS 2 Humble, React Three Fiber, MoveIt 2, .NET 8, and Gazebo Harmonic.*
