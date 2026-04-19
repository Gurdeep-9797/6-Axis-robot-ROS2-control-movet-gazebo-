# ANTIGRAVITY — RoboForge Full Stack Update Specification
**Version:** 1.0 | **Prepared for:** Antigravity AI Development Agent  
**Project:** RoboForge Online Robot Programming Platform  
**Reference Screenshots:** PickPlace_Main.mod (Blocks + 3D Viewport), Motion/IO Block Panel, Full IDE with ROS 2 Bridge  

---

> **AGENT DIRECTIVE — READ FIRST:**  
> You are updating an existing, live, online robot programming platform called **RoboForge**. Before making **any** change:  
> 1. **Audit first** — Read every existing file, component, service, and API route relevant to the section you are working on.  
> 2. **Compare** — Match what exists against what is required in this document.  
> 3. **Do not duplicate** — If a function, route, or component already exists and satisfies the requirement, reuse it. Only create new code when nothing existing covers the need.  
> 4. **Verify after every phase** — At the end of each phase, run a checklist (defined at the bottom of each phase) to confirm the implementation is working correctly before moving to the next phase.  
> 5. **This document is sequential** — Do not skip phases. Each phase depends on the prior one being verified.  
> 6. **Ask before assuming** — If you are unsure about an existing implementation, search the codebase for references before writing new code.

---

## Table of Contents

1. [Phase 0 — Full Codebase Audit & Baseline](#phase-0)
2. [Phase 1 — UI Layout Corrections](#phase-1)
3. [Phase 2 — Programming Block System & Program Logic Architecture](#phase-2)
4. [Phase 3 — Simulator Mode: Full Data Pipeline (Block → ROS 2 → MoveIt → Gazebo → Viewer)](#phase-3)
5. [Phase 4 — Single Cell vs Full Program Execution Logic](#phase-4)
6. [Phase 5 — Path Smoothing & Trajectory Generation in MoveIt](#phase-5)
7. [Phase 6 — Live Mode: Hardware Detection, Handshake & Homing](#phase-6)
8. [Phase 7 — Live Mode: Real-Time Control Pipeline (Encoders → ROS → MoveIt → Motor Commands)](#phase-7)
9. [Phase 8 — Robot Configuration Tab (Frontend + Backend)](#phase-8)
10. [Phase 9 — URDF Import Pipeline (SolidWorks / STEP / Other CAD Formats)](#phase-9)
11. [Phase 10 — Performance Optimization (Gazebo + ROS CPU Load)](#phase-10)
12. [Phase 11 — Modern Glassy UI Pass](#phase-11)
13. [Master Verification Checklist](#master-verification-checklist)

---

<a name="phase-0"></a>
## Phase 0 — Full Codebase Audit & Baseline

### 0.1 What You Must Do First

Before touching a single line of code, perform the following audit and document your findings:

```
AUDIT CHECKLIST — Complete all items and record results:

[ ] Identify the frontend framework (React, Vue, Angular, Svelte, etc.)
[ ] Identify the backend framework and language (Node/Express, FastAPI, Django, etc.)
[ ] Identify the ROS 2 distribution in use (Humble, Iron, Jazzy, etc.)
[ ] Identify how the frontend communicates with ROS 2 (rosbridge_suite websocket, custom bridge, rclnodejs, etc.)
[ ] Identify the current MoveIt version (MoveIt 2 for ROS 2, or MoveIt 1 for ROS 1)
[ ] Identify if Gazebo Classic or Gazebo Ignition/Harmonic is in use
[ ] Identify if ros2_control is set up — list all hardware interfaces and controllers
[ ] Read the existing Program Tree / Block Editor component — locate all files
[ ] Read the existing 3D Viewport component — identify the rendering library (Three.js, Babylon.js, etc.)
[ ] Read the existing ROS 2 Bridge panel component — note its current position in the layout
[ ] Check if a URDF loader/parser already exists
[ ] Check if there is an existing config tab or config schema for robot parameters
[ ] Check if any motor/encoder configuration already exists
[ ] List all currently active ROS 2 nodes (cross-reference with screenshot: /move_group, /roboforge_bridge, /pseudo_hardware_node, /robot_state_publisher)
[ ] List all currently published topics (cross-reference: /joint_states, /tcp_pose, /trajectory)
[ ] List all currently subscribed topics (cross-reference: /cmd_joint, /cmd_pose, /external_trigger)
[ ] Check if any IK solver is already configured (KDL, TRAC-IK, BioIK, etc.)
[ ] Check if any path planning pipeline is configured (OMPL, STOMP, PILZ, etc.)
[ ] Identify how Gazebo is currently launched — roslaunch file, subprocess, Docker container
[ ] Check the IO subsystem — how digital outputs (SetDO), digital inputs (GetDI), analog I/O are currently handled
[ ] Identify the file storage system for .mod program files and URDF files
[ ] Check if any sw2urdf or STEP-to-URDF converter is already present in the repo
```

### 0.2 Screenshot Baseline — What Currently Exists (Confirmed from Visual Inspection)

The following are confirmed from the provided screenshots:

**Program Tree (Left Panel — Image 3):**
- `Main Program > Initialize` (function block)
- `Main Program > PickAndPlace` (function block inside `While TRUE` loop)
- Waypoints visible in tree: `WP_Approach`, `WP_Pick`, `WP_Place`
- I/O blocks: `SetDO gripper ON`, `SetDO gripper OFF`

**Block Library (Image 2 — Left Panel):**
- **MOTION:** MoveJ, MoveL, MoveC, MoveAbsJ, SearchL, Honing Cycle
- **I/O:** SetDO, PulseDO, GetDI, WaitDI, SetAO, GetAI
- **FLOW:** Wait (confirmed), others likely exist (If, While, For, etc. — verify)

**3D Viewport (Image 1 & 3):**
- Robot model visible (IRB 6700 referenced in top right)
- Workspace bounding box visible (orange frame)
- Coordinate axes gizmo visible (bottom right of viewport)
- Speed slider (100%) at bottom

**ROS 2 Integration (Image 3 — Right Panel):**
- ROS 2 Bridge panel currently on the **right side** — **THIS IS WRONG, must move to left**
- Connected: Namespace `/robot_1`, Publish Rate 50Hz
- Published: `/joint_states`, `/tcp_pose`, `/trajectory`
- Subscribed: `/cmd_joint`, `/cmd_pose`, `/external_trigger`
- Active Nodes: `/move_group`, `/roboforge_bridge`, `/pseudo_hardware_node`, `/robot_state_publisher`

**Debug Panel (Image 3 — Bottom):**
- Tabs: Diagnostics, ROS2 Monitor, Console, Node Graph
- Active Nodes list showing "alive" status

**Top Toolbar (Image 3):**
- New, Open, Save, Export, Home, Undo, Redo, Compile, Run, Pause, Stop, Step
- Operation Mode: Edit / Simulate / Live
- IK Solver toggle
- Injector PWM slider (50%)
- Bridge: OFFLINE / Online toggle

### 0.3 Record Your Findings

After completing the audit, create a file called `AUDIT_RESULTS.md` at the root of the project. This file will be referenced in every subsequent phase. Format it as:

```markdown
## Audit Results — [Date]
- Frontend: [framework name + version]
- Backend: [framework + language]
- ROS 2 Distro: [distro]
- ROS 2 Bridge Method: [method]
- MoveIt Version: [version]
- Gazebo Version: [version]
- IK Solver: [current solver]
- URDF Parser: [exists / does not exist]
- Config Tab: [exists / does not exist]
- Motor Config Schema: [exists / does not exist]
- sw2urdf: [found at path X / not found]
... (complete all items)
```

---

<a name="phase-1"></a>
## Phase 1 — UI Layout Corrections

> **Priority: HIGH — Fix this before all other UI work. All subsequent panels must be placed correctly.**

### 1.1 Current Problem (Confirmed from Screenshots)

The **ROS 2 Bridge** panel and **Debug/Diagnostics** panels are currently rendered on the **right side** of the screen (Image 3, right column). Their correct position, based on the product design, is:

- **Left sidebar** — Program Tree, Block Library, Scene, 3D View, Motion, I/O, Debug, Config, ROS 2, Settings, Project (these are the left icon navigation tabs visible in Image 3, left edge)
- **Right panel** — Properties, I/O quick view, Debug output, ROS 2 Bridge detail — these appear as a **detail/inspector panel** that opens when a left nav icon is clicked
- **Bottom panel** — Debug & Diagnostics output area (Diagnostics, ROS2 Monitor, Console, Node Graph tabs) — this is correct and must stay at the bottom

### 1.2 Required Layout Structure

```
┌─────────────────────────────────────────────────────────────────────┐
│  TOP TOOLBAR (New, Open, Save, ..., Run, Pause, Stop, Mode, Bridge) │
├──────┬─────────────────────────────────────────┬────────────────────┤
│ ICON │  LEFT DETAIL PANEL (context-sensitive)  │  MAIN WORKSPACE    │
│ NAV  │  Opens when icon is clicked:            │  (Code Editor      │
│      │  - Program Tree                         │   or               │
│  ▪   │  - Block Library                        │   3D Viewport)     │
│  ▪   │  - Scene Objects                        │                    │
│  ▪   │  - 3D View Settings                     │                    │
│  ▪   │  - Motion Settings                      │                    │
│  ▪   │  - I/O Panel                            │                    │
│  ▪   │  - Debug Panel                          │                    │
│  ▪   │  - Config Tab                           │                    │
│  ▪   │  - ROS 2 Bridge (MOVED HERE from right) │                    │
│  ▪   │  - Settings                             │                    │
│  ▪   │  - Project                              │                    │
├──────┴─────────────────────────────────────────┴────────────────────┤
│  BOTTOM PANEL: Debug & Diagnostics                                  │
│  [Diagnostics] [ROS2 Monitor] [Console] [Node Graph]               │
└─────────────────────────────────────────────────────────────────────┘
```

### 1.3 Steps to Fix

1. **Audit** the layout component file (likely `AppLayout.tsx` or `Layout.vue` or similar). Find where `ROS2BridgePanel` is currently rendered.
2. **Move** the `ROS2BridgePanel` component to be rendered inside the left detail panel slot, mapped to the ROS 2 icon in the left nav.
3. **Inspect** the Properties / Inspector panel on the right — verify if it is currently used for something else (e.g., showing selected block properties). If it is, keep it on the right ONLY for block/waypoint property inspection. The ROS 2 Bridge panel must NOT be on the right.
4. **Test** that clicking the ROS 2 icon on the left nav shows the ROS 2 Bridge detail panel in the left detail area.
5. **Ensure** the bottom Debug & Diagnostics panel remains at the bottom and contains: Diagnostics, ROS2 Monitor, Console, Node Graph.

### Phase 1 Verification Checklist
```
[ ] ROS 2 Bridge panel renders in LEFT detail panel when ROS 2 icon is clicked
[ ] Right panel is ONLY used for Properties/Inspector of selected block or waypoint
[ ] Bottom panel contains Diagnostics, ROS2 Monitor, Console, Node Graph tabs
[ ] Left icon nav icons are clickable and each opens the correct panel
[ ] Layout is responsive and panels can be resized or collapsed
[ ] No duplicate panels rendering in two places simultaneously
```

---

<a name="phase-2"></a>
## Phase 2 — Programming Block System & Program Logic Architecture

### 2.1 Architecture Overview

The block-based programming system works on a **hierarchical tree model**. Each node in the tree is a typed instruction block. The program is serialized to an intermediate representation (IR) which is then compiled into a ROS 2 executable plan.

```
Program Tree Structure:
─────────────────────────────────────────
Main Program (root)
  ├── fn: Initialize          ← Function block (container)
  │     ├── MoveJ: Home
  │     ├── SetDO: Gripper OFF
  │     └── Set: counter = 0
  └── While: TRUE             ← Loop block (container)
        └── fn: PickAndPlace  ← Nested function block
              ├── MoveJ: Approach
              ├── MoveL: Pick
              ├── SetDO: Gripper Close
              ├── Wait: 0.2s
              ├── MoveL: Lift
              ├── MoveJ: Place
              ├── SetDO: Gripper Open
              └── counter++
```

### 2.2 Block Types — Full Specification

**Verify which of these already exist before adding new ones.**

#### MOTION Blocks
| Block | Parameters | Notes |
|---|---|---|
| `MoveJ` | Target waypoint or joint angles `[J1..J6]`, Speed %, Zone/Blend radius | Joint-space interpolation. Uses IK to get joints from waypoint. |
| `MoveL` | Target pose `[X,Y,Z,Rx,Ry,Rz]`, Speed mm/s, Zone | Linear TCP path in Cartesian space. IK solved at each interpolation step. |
| `MoveC` | Via-point pose, Target pose, Speed mm/s, Zone | Circular arc. IK solved along arc. User must define via-point and end-point. |
| `MoveAbsJ` | Absolute joint values `[J1..J6]` in degrees | Bypasses IK. Directly commands joint angles. |
| `SearchL` | Direction vector, Speed, Digital Input trigger | Move linearly until a DI goes HIGH/LOW. Then stop and record position. |
| `Honing Cycle` | Axis, Stroke, RPM, Feed | Helical/spiral path for honing operations. |

#### MoveC Block Special Requirement (Circular Path Definition):
The `MoveC` block must support **two methods** for defining the circular path:
1. **3D Workspace Selection** — User switches to 3D Viewport, enters a "Pick Point" mode, clicks two points on the workspace (via-point and end-point). These are stored as waypoints and auto-populated into the block.
2. **Parameter Input** — User directly types the via-point `[X,Y,Z,Rx,Ry,Rz]` and end-point `[X,Y,Z,Rx,Ry,Rz]` as numeric fields inside the block panel.

Both methods must update the same underlying waypoint data store so they stay in sync.

#### I/O Blocks
| Block | Parameters |
|---|---|
| `SetDO` | Output channel ID, Value (0 or 1) |
| `PulseDO` | Output channel ID, Pulse width ms |
| `GetDI` | Input channel ID → returns 0/1 (stored to variable) |
| `WaitDI` | Input channel ID, Expected value (0/1), Timeout ms |
| `SetAO` | Analog output channel ID, Value (float 0.0–10.0V or 0–20mA) |
| `GetAI` | Analog input channel ID → returns float (stored to variable) |

#### FLOW Blocks
| Block | Parameters |
|---|---|
| `Wait` | Duration in seconds |
| `While` | Condition expression (e.g., `TRUE`, `counter < 10`, `DI[1] == 1`) |
| `If / ElseIf / Else` | Condition expression |
| `For` | Variable, Start, End, Step |
| `Function (fn)` | Name, optional parameters |
| `Set Variable` | Variable name, Value expression |
| `Increment` | Variable name (shorthand `var++`) |
| `Return` | Optional return value |

**Verify which of the FLOW blocks above exist. Add only the missing ones.**

### 2.3 Intermediate Representation (IR) Schema

Each block serializes to a JSON IR node. This IR is the contract between the frontend and the backend execution engine.

```json
{
  "id": "uuid-v4",
  "type": "MoveL",
  "label": "MoveL Pick",
  "params": {
    "target": {
      "type": "waypoint",
      "name": "WP_Pick",
      "pose": { "x": 500, "y": 0, "z": 900, "rx": 0, "ry": 0, "rz": 0 },
      "frame": "base_link"
    },
    "speed": 200,
    "speed_unit": "mm/s",
    "zone": 5,
    "blend_radius": 5
  },
  "children": [],
  "enabled": true
}
```

Container blocks (fn, While, If, For) have a `children` array containing nested IR nodes. The full program is one root IR tree.

### 2.4 Waypoint Store

All waypoints must be stored in a centralized **Waypoint Store** (frontend state + persisted to backend file). Each waypoint has:

```json
{
  "name": "WP_Pick",
  "pose": { "x": 500, "y": 0, "z": 900, "rx": 0, "ry": 0, "rz": 0 },
  "joint_angles": [J1, J2, J3, J4, J5, J6],
  "frame": "base_link",
  "recorded_in_mode": "simulation | live",
  "timestamp": "ISO8601"
}
```

The `joint_angles` are calculated by MoveIt IK at the time of waypoint recording and cached. If the URDF changes, the cache must be invalidated.

### Phase 2 Verification Checklist
```
[ ] All listed block types are present in the Block Library panel
[ ] Each block, when added to the tree, serializes correctly to IR JSON
[ ] MoveC block supports both 3D workspace click mode and manual parameter entry
[ ] Waypoint store persists across sessions (saved to backend)
[ ] Program tree renders the hierarchy correctly (nested fn inside While, etc.)
[ ] Blocks can be drag-reordered within the tree
[ ] Blocks can be enabled/disabled (toggle without deletion)
[ ] IR JSON can be fully reconstructed from the program tree and vice versa
```

---

<a name="phase-3"></a>
## Phase 3 — Simulator Mode: Full Data Pipeline

> This phase defines the complete flow from a block being executed to the 3D Viewer updating — **without any real robot hardware**.

### 3.1 Full Pipeline Diagram

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                        SIMULATOR MODE PIPELINE                               │
│                                                                              │
│  [Frontend - Browser]                                                        │
│  ┌─────────────────┐                                                         │
│  │ Block / Program │  User clicks "Run" on a block or the full program       │
│  │ (IR JSON Tree)  │                                                         │
│  └────────┬────────┘                                                         │
│           │ HTTP POST /api/program/execute  (or WebSocket message)           │
│           ▼                                                                  │
│  [Backend - RoboForge Server]                                                │
│  ┌──────────────────────────┐                                                │
│  │  Program Compiler        │  Validates IR, resolves waypoints,             │
│  │  (IR → ROS 2 Goal)       │  resolves variables, flattens control flow     │
│  └────────────┬─────────────┘                                                │
│               │ ROS 2 Action / Service Call                                  │
│               ▼                                                              │
│  [ROS 2 Layer]                                                               │
│  ┌──────────────────────────┐                                                │
│  │  /roboforge_bridge node  │  Receives compiled goals, dispatches to        │
│  │  (existing node)         │  appropriate ROS 2 actions                     │
│  └────────────┬─────────────┘                                                │
│               │                                                              │
│     ┌─────────┴──────────┐                                                   │
│     │                    │                                                   │
│     ▼                    ▼                                                   │
│  MoveGroup Action     SetDO/GetDI Services                                   │
│  (for motion)         (for I/O — map to Gazebo plugins)                      │
│     │                                                                        │
│     ▼                                                                        │
│  [MoveIt 2 — /move_group node]                                               │
│  ┌──────────────────────────────────────────────┐                            │
│  │  1. Receives MoveGroup goal (target pose or  │                            │
│  │     joint target)                            │                            │
│  │  2. Runs IK Solver (TRAC-IK recommended —    │                            │
│  │     check if already configured)             │                            │
│  │     Input:  Target Cartesian pose            │                            │
│  │     Model:  URDF/SRDF of robot               │                            │
│  │     Output: Joint angle solution [J1..Jn]    │                            │
│  │  3. Runs Path Planner (OMPL — RRTConnect     │                            │
│  │     or PILZ PTP/LIN/CIRC)                    │                            │
│  │     Output: JointTrajectory message           │                           │
│  │  4. Publishes trajectory to /trajectory      │                            │
│  └──────────────────────────────────────────────┘                            │
│               │                                                              │
│               │ JointTrajectory → ros2_control controller                    │
│               ▼                                                              │
│  [ros2_control — Joint Trajectory Controller]                                │
│  ┌────────────────────────────────────┐                                      │
│  │  Receives JointTrajectory          │                                      │
│  │  Interpolates setpoints at control │                                      │
│  │  rate (default 1kHz)              │                                      │
│  │  Publishes /joint_commands         │                                      │
│  └──────────────┬─────────────────────┘                                     │
│                 │                                                            │
│                 ▼                                                            │
│  [Gazebo (Physics Simulation)]                                               │
│  ┌─────────────────────────────────────────────────────────┐                │
│  │  GazeboROS2ControlPlugin / gazebo_ros2_control          │                │
│  │  Receives joint commands                                │                │
│  │  Runs physics step (ODE/Bullet/DART engine)             │                │
│  │  Calculates: joint positions, velocities, efforts       │                │
│  │  Publishes: /joint_states at sim rate                   │                │
│  └──────────────────────────────┬──────────────────────────┘                │
│                                 │                                            │
│                                 │ /joint_states topic                        │
│                                 ▼                                            │
│  [/robot_state_publisher node]                                               │
│  ┌────────────────────────────────────┐                                      │
│  │  Subscribes to /joint_states       │                                      │
│  │  Publishes TF transforms           │                                      │
│  │  (base_link → link1 → ... → tcp)  │                                      │
│  └──────────────────────────────┬─────┘                                     │
│                                 │ TF2 transforms                             │
│                                 │                                            │
│  [/roboforge_bridge]            │                                            │
│  ┌────────────────────────────────────┐                                      │
│  │  Subscribes to /joint_states       │                                      │
│  │  Subscribes to /tcp_pose           │                                      │
│  │  Forwards state data to frontend   │                                      │
│  │  via WebSocket at 50Hz             │                                      │
│  └──────────────────────────────┬─────┘                                     │
│                                 │ WebSocket push (50Hz)                      │
│                                 ▼                                            │
│  [Frontend — 3D Viewport]                                                    │
│  ┌────────────────────────────────────┐                                      │
│  │  Receives joint state updates      │                                      │
│  │  Updates 3D robot model via        │                                      │
│  │  Three.js / Babylon.js skeleton    │                                      │
│  │  joints driven by joint angles     │                                      │
│  │  User sees real-time animation     │                                      │
│  └────────────────────────────────────┘                                      │
└──────────────────────────────────────────────────────────────────────────────┘
```

### 3.2 IK Solver Selection

**Check first:** Open the MoveIt configuration package for the robot. Look at `kinematics.yaml`. It will show which solver is currently configured.

**Recommendation if not set:**
- Use **TRAC-IK** (`trac_ik_kinematics_plugin`). It is faster and more reliable than the default KDL solver for 6-DOF arms, handles near-singular configurations better, and is available as a ROS 2 package.
- If TRAC-IK is not installable in the current environment, fall back to **KDL** (default, already in MoveIt).
- **Do NOT use BioIK** unless the robot has >7 DOF — it is overkill for standard 6-DOF arms.

```yaml
# kinematics.yaml — example
robot_group:
  kinematics_solver: trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin
  kinematics_solver_attempts: 3
  kinematics_solver_timeout: 0.05
  kinematics_solver_search_resolution: 0.005
```

### 3.3 Path Planning Pipeline

**Check first:** Verify which planners are configured in `ompl_planning.yaml` or `pilz_cartesian_limits.yaml`.

**Required planners by block type:**

| Block Type | Recommended Planner | Why |
|---|---|---|
| `MoveJ` | PILZ `PTP` | Point-to-point in joint space, predictable, fast |
| `MoveL` | PILZ `LIN` | Guaranteed linear TCP path in Cartesian space |
| `MoveC` | PILZ `CIRC` | Guaranteed circular TCP path |
| `MoveAbsJ` | PILZ `PTP` | Direct joint space, no IK needed |
| General | OMPL `RRTConnect` | Fallback for complex environments with obstacles |

**Why PILZ Industrial Motion Planner:**
PILZ produces deterministic, smooth trajectories suitable for industrial robotics. It is available in MoveIt 2 as `pilz_industrial_motion_planner`. Check if it is already loaded in `move_group.launch.py`.

### 3.4 Bridge Node — WebSocket API Contract

The `/roboforge_bridge` node must implement this WebSocket message contract between ROS 2 and the frontend:

**Backend → Frontend (state updates, pushed at 50Hz):**
```json
{
  "type": "robot_state",
  "timestamp": 1713200000.123,
  "joint_states": {
    "names": ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
    "positions": [0.0, -0.785, 1.571, 0.0, 0.785, 0.0],
    "velocities": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "efforts": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  },
  "tcp_pose": {
    "x": 500.0, "y": 0.0, "z": 1200.0,
    "rx": 0.0, "ry": 0.0, "rz": 0.0,
    "frame": "base_link"
  },
  "execution_status": "running | idle | paused | error",
  "current_block_id": "uuid-of-block-being-executed"
}
```

**Frontend → Backend (commands):**
```json
{
  "type": "execute_program",
  "mode": "simulation",
  "program_ir": { ... },
  "start_from_block_id": "uuid | null"
}
```
```json
{
  "type": "execute_block",
  "mode": "simulation",
  "block": { ... }
}
```
```json
{
  "type": "control",
  "command": "pause | stop | step | resume"
}
```

**Important:** Verify that the existing `/roboforge_bridge` node already publishes joint states to the frontend. If it does, do not duplicate this logic. Only add missing message types.

### Phase 3 Verification Checklist
```
[ ] Clicking "Run" on a single MoveJ block sends a goal to /move_group via the bridge
[ ] MoveIt IK solver returns valid joint angles (check kinematics.yaml is correctly configured)
[ ] PILZ planners (PTP, LIN, CIRC) are loaded and selectable per block type
[ ] Joint trajectory is forwarded to Gazebo via ros2_control
[ ] Gazebo simulates the motion and publishes /joint_states
[ ] 3D Viewport updates the robot model in real-time as joints move (at minimum 20Hz visual refresh)
[ ] Current executing block is highlighted in the program tree
[ ] SetDO/GetDI blocks trigger corresponding Gazebo I/O plugin responses
[ ] Execution status (running/idle/paused/error) is shown in the UI
```

---

<a name="phase-4"></a>
## Phase 4 — Single Cell vs Full Program Execution Logic

### 4.1 Single Block / Cell Execution

When the user right-clicks a single block and selects "Run from here" or clicks a play button on that block:

1. Extract the single block's IR JSON.
2. Resolve any referenced waypoints from the Waypoint Store.
3. Send as a **single-block execution request** to the backend.
4. The backend compiles only that block into a ROS 2 goal.
5. The goal is sent to `/roboforge_bridge` → MoveIt → Gazebo.
6. Result (success/failure) is returned and shown on the block.

**Constraint:** Single block execution does NOT carry state from previous blocks (no variable values, no counter states). It is stateless and used for testing individual moves.

### 4.2 Full Program Execution

When the user clicks the main "Run" button in the toolbar:

1. The entire Program Tree is serialized to a full IR JSON tree.
2. A **Program Compiler** on the backend processes the IR:

```
Compiler Steps:
  a. Validate schema — all blocks have required params, all waypoints exist
  b. Resolve variables — assign initial values (counter = 0, etc.)
  c. Flatten control flow — unroll the execution sequence:
     - While loops: mark as iterative execution nodes (not unrolled statically)
     - If/Else: evaluated at runtime
     - Function blocks: expanded inline (like macro expansion)
  d. Resolve waypoints — attach pose data to each MoveJ/MoveL/MoveC block
  e. Assign planner per block — MoveJ→PTP, MoveL→LIN, MoveC→CIRC
  f. Generate Execution Plan: ordered list of typed goals
```

3. The compiled **Execution Plan** is sent to the backend executor as a single request.
4. The **backend executor** processes the plan sequentially:
   - For each step in the plan:
     - If it is a motion step: send to MoveIt, await completion, then proceed to next
     - If it is an I/O step: send SetDO/GetDI service call, await ack, proceed
     - If it is a Wait step: sleep for duration, proceed
     - If it is a conditional (If/While): evaluate condition against current variable state, branch accordingly
     - If it is a function call: push a stack frame, process the function body, pop and continue
5. As each block starts executing, the backend sends a WebSocket message to the frontend with the `current_block_id`, which the frontend uses to highlight the active block in the tree.
6. On completion, success or error state is sent back.

### 4.3 Nested Function Logic (Function within Function)

Just as a programming language supports nested function calls, the execution engine must support nested function blocks:

```
Main Program
  └── While TRUE                    ← Loop — evaluates condition each iteration
        └── fn: PickAndPlace        ← Function call — pushes stack frame
              ├── MoveJ: Approach   ← Motion step
              ├── MoveL: Pick       ← Motion step (awaits completion)
              ├── fn: GripperAction ← Nested function call — pushes new stack frame
              │     ├── SetDO: Close gripper
              │     └── Wait: 0.2s
              └── MoveL: Lift       ← Resumes after GripperAction returns
```

**Execution Stack Model:**
```
Stack Frame: {
  function_id: "uuid",
  current_step_index: int,
  local_variables: { key: value },
  parent_frame: StackFrame | null
}
```

The executor maintains a call stack. When a function is entered, a new frame is pushed. When the function's last step completes, the frame is popped and execution resumes at the parent frame's next step.

### 4.4 Pause / Step / Stop Controls

| Control | Behavior |
|---|---|
| **Pause** | After current motion step completes, suspend. Robot holds position. Resume resumes from next step. |
| **Step** | Execute one block/step, then pause. Wait for next "Step" click. |
| **Stop** | Send stop command to MoveIt (cancel all goals). Robot decelerates to halt. Execution state is cleared. |
| **Resume** | Continue from paused state. |

**Stop command to MoveIt:** Use the `/move_group/cancel` action or call `MoveGroupInterface::stop()` in the bridge node.

### Phase 4 Verification Checklist
```
[ ] Single block run works without executing surrounding context
[ ] Full program run executes all blocks in correct sequence
[ ] While loop iterates correctly (counter increments, loop exits on condition)
[ ] Nested function calls work (function inside function — verified with 2 levels deep)
[ ] Current block is highlighted in tree during execution
[ ] Pause stops after current motion step completes (robot does not jerk)
[ ] Stop sends cancel goal to MoveIt and robot decelerates
[ ] Step mode executes exactly one block per click
[ ] Variable state is maintained across blocks during a full program run
```

---

<a name="phase-5"></a>
## Phase 5 — Path Smoothing & Trajectory Generation in MoveIt

### 5.1 Why Path Smoothing Matters

Without smoothing, a sequence of waypoints produces trajectories with:
- Sudden velocity changes at waypoints (jerks)
- Excessive inertia loads on joints
- Vibration in the end effector
- Potential motor current spikes

### 5.2 MoveIt Trajectory Execution with Time Parameterization

After MoveIt's path planner generates a geometric path (list of joint configurations), it must apply **time parameterization** to assign velocities and accelerations to each point. This is what produces smooth motion.

**Check first:** Verify if `trajectory_processing` is configured in the MoveIt config.

**Required configuration in `moveit_config/config/trajectory_execution.launch.xml` or equivalent:**

Use the **Iterative Spline Parameterization (ISP)** algorithm, which is MoveIt 2's recommended smoother:

```yaml
# In move_group configuration
trajectory_execution:
  allowed_execution_duration_scaling: 1.2
  allowed_goal_duration_margin: 0.5
  allowed_start_tolerance: 0.01

planning_pipeline:
  planning_plugin: pilz_industrial_motion_planner/CommandPlanner
  
# Time parameterization (applied after planning)
trajectory_processing: iterative_spline_parameterization
```

**For PILZ planners specifically:** PILZ LIN and PTP already produce time-parameterized trajectories respecting velocity and acceleration limits defined in `joint_limits.yaml`. Verify this file exists and has correct limits per joint.

```yaml
# joint_limits.yaml — example
joint_limits:
  joint_1:
    has_velocity_limits: true
    max_velocity: 2.094    # rad/s (~120 deg/s)
    has_acceleration_limits: true
    max_acceleration: 3.14  # rad/s²
  joint_2:
    # ... same pattern for all joints
```

### 5.3 Blend Radius (Zone) for Multi-Waypoint Programs

When executing a sequence of moves (e.g., Approach → Pick → Lift), the robot should blend between waypoints rather than stopping at each one. This is controlled by the **Zone** or **Blend Radius** parameter on each block.

- Zone = 0: Robot comes to a complete stop at the waypoint before starting the next move
- Zone > 0: Robot starts transitioning to the next move when it is within `zone` mm of the waypoint

**Implementation:** PILZ supports blend radius natively. When compiling the execution plan, detect consecutive motion blocks with non-zero zone and issue a **SequenceRequest** (PILZ's `moveit_msgs/BlendedTrajectory`) rather than individual goals. This allows PILZ to compute smooth blends between motions.

```
Verify: Is PILZ SequenceRequest support implemented in /roboforge_bridge?
If not, implement: collect consecutive motion blocks with zone>0, pack into a 
SequenceRequest, send as one request to /pilz_industrial_motion_planner
```

### 5.4 Singularity Avoidance

When MoveIt's IK solver encounters a configuration near a singularity, it must:
1. Detect the near-singular configuration (determinant of Jacobian approaching zero)
2. Apply a small perturbation or use TRAC-IK's redundancy resolution to find an alternative
3. If unavoidable, report a warning to the frontend (not silently fail)

**Frontend must display:** A warning banner in the Debug panel when a singularity warning is received from MoveIt.

### Phase 5 Verification Checklist
```
[ ] joint_limits.yaml exists and has velocity + acceleration limits for all joints
[ ] ISP or PILZ time parameterization is active (verify by checking trajectory timestamps — they should be spaced non-uniformly based on dynamics)
[ ] Multi-waypoint program executes with smooth blending when zone > 0
[ ] Multi-waypoint program stops at each waypoint when zone = 0
[ ] No sudden velocity discontinuities visible in the 3D viewport animation
[ ] Singularity warnings from MoveIt appear in the Debug panel
[ ] PILZ SequenceRequest is used for consecutive blended motion blocks
```

---

<a name="phase-6"></a>
## Phase 6 — Live Mode: Hardware Detection, Handshake & Homing

### 6.1 Hardware Detection — Automatic Controller Discovery

When the RoboForge software starts (or when the user opens the I/O tab), it must **automatically scan available hardware interfaces** to detect if a robot controller is connected.

**Detection Methods (try in order):**

1. **USB/Serial Scan:** Enumerate `/dev/ttyUSB*`, `/dev/ttyACM*`, `/dev/serial/by-id/*`. For each port, attempt to query a device identity string using a defined query packet (see handshake below).

2. **EtherCAT Scan:** If an EtherCAT master library is configured (e.g., `soem_ros2`), scan the EtherCAT bus for slaves. Match found slaves against a known device database (motor drive ESI files).

3. **Ethernet/UDP Scan:** Send a UDP broadcast on the local subnet to a discovery port (e.g., UDP 11511). Devices that respond with a valid device descriptor are listed.

4. **USB HID:** Enumerate connected USB HID devices, filter by known VID/PID pairs from the supported controller list.

**The software must NOT enable "Live Mode" until at least one valid controller is detected and verified.**

### 6.2 Handshake Protocol — Controller Contract

When a potential controller is found on any interface, the software initiates a **structured handshake**. This handshake establishes a **Device Contract** — a machine-readable description of the controller's capabilities.

**Handshake Sequence:**

```
Step 1: IDENTITY REQUEST
  Software sends: { "cmd": "WHO_ARE_YOU", "protocol_version": "1.0" }
  
Step 2: IDENTITY RESPONSE (controller replies)
  Controller sends:
  {
    "device_type": "motor_controller",
    "manufacturer": "...",
    "model": "...",
    "firmware_version": "1.x.x",
    "num_axes": 6,
    "protocol_version": "1.0",
    "capabilities": ["position_control", "velocity_control", "torque_control"],
    "encoder_types": ["absolute", "incremental"],   ← per axis if mixed
    "communication_rate_hz": 1000,
    "supported_motor_types": ["BLDC_sinusoidal", "BLDC_trapezoidal", "stepper", "DC_brushed"]
  }

Step 3: VERIFICATION
  Software checks:
  - protocol_version is compatible
  - num_axes matches configured robot DOF
  - capabilities include at least "position_control"
  
Step 4: CONFIG PUSH
  Software sends the Robot Config Contract (see Phase 7 for full schema):
  {
    "cmd": "CONFIGURE",
    "axes": [
      {
        "axis_id": 1,
        "motor_type": "BLDC_sinusoidal",
        "encoder_type": "absolute",
        "encoder_resolution": 4096,
        "poles": 14,
        "rated_current_A": 10.0,
        "max_velocity_rpm": 3000,
        "gear_ratio": 100.0
      },
      ... (one entry per axis)
    ]
  }

Step 5: CONFIG ACK
  Controller sends: { "cmd": "CONFIGURE_ACK", "status": "ok" | "error", "error_details": "..." }

Step 6: ENABLE LIVE MODE
  If all steps succeed: software enables the "Live" toggle in Operation Mode selector.
  If any step fails: software keeps Live Mode disabled and displays an error in the Debug panel 
  with the specific failure reason.
```

**Ongoing Health Monitoring:** After successful handshake, the controller must send a **heartbeat** at 10Hz:
```json
{ "cmd": "HEARTBEAT", "timestamp": 1713200000.0, "status": "ok" | "fault", "fault_code": 0 }
```
If 3 consecutive heartbeats are missed, the software:
1. Immediately pauses execution if running
2. Displays a "Controller Connection Lost" alert
3. Disables Live Mode toggle
4. Requires re-handshake to re-enable

### 6.3 Homing Sequence

Homing is performed **automatically after a successful handshake** when:
- The software detects the controller has just powered on (check via a `power_on_flag` in the heartbeat), OR
- The user manually triggers homing via the Config tab

**Homing sequence differs by encoder type:**

| Encoder Type | Homing Method |
|---|---|
| **Absolute** | No physical homing needed. Read encoder position immediately. Send to MoveIt as current state. |
| **Incremental** | Must perform physical homing: move to a hardware limit switch or index pulse. Record as zero position. |

**Homing Procedure for Incremental Encoders:**

```
For each axis (in order J1 → J6, or as defined in homing_config):
  1. Move at slow speed (5% of max) toward home direction
  2. Detect: either limit switch trigger (DI goes LOW/HIGH) or encoder index pulse
  3. Record this position as joint home (0° or as defined in URDF)
  4. Back off by the configured offset amount
  5. Mark axis as "homed" 
When all axes are homed:
  6. Send current joint positions to /roboforge_bridge → publishes to /joint_states
  7. robot_state_publisher reads /joint_states → publishes TF
  8. 3D Viewport updates to show the robot's actual current position
  9. Display: "All axes homed — Ready for Live operation"
```

**Display during homing:** Show a progress UI in the Debug panel / status bar:
```
Homing: ◉ J1 ● J2 ○ J3 ○ J4 ○ J5 ○ J6   [Cancel]
Legend: ◉ = complete, ● = in progress, ○ = pending
```

### Phase 6 Verification Checklist
```
[ ] Software scans for hardware on startup (USB, EtherCAT, or configured interface)
[ ] If no controller found: Live Mode toggle remains greyed out
[ ] If controller found: Handshake completes successfully (test with actual controller or a software mock)
[ ] Config is pushed to controller and acknowledged
[ ] Live Mode toggle becomes active after successful handshake
[ ] Heartbeat monitoring is active — test by disconnecting controller: software detects within 300ms
[ ] Homing sequence runs automatically on first connection
[ ] Absolute encoder axes skip physical homing, read position directly
[ ] Incremental encoder axes perform physical home move
[ ] 3D Viewport shows robot in its actual homed position after homing completes
[ ] Homing progress is shown in the UI
```

---

<a name="phase-7"></a>
## Phase 7 — Live Mode: Real-Time Control Pipeline

### 7.1 What is Different in Live Mode vs Simulator Mode

In **Simulator Mode:** `joint_commands → Gazebo (software physics) → joint_states`

In **Live Mode:** `joint_commands → Real Motor Controller → Real Encoders → joint_states`

Gazebo is **not used** in live mode. The physics are real. The encoder feedback replaces Gazebo's state publishing.

### 7.2 Where Encoder Values Are Used

This is a critical distinction — encoders are used at **two different levels**:

```
Level 1: State Estimation (what is the robot's current pose?)
  - Encoder positions → joint_states message → robot_state_publisher → TF tree
  - MoveIt reads current joint state from /joint_states before planning
  - This ensures MoveIt knows the true start configuration for each new motion

Level 2: Servo / Closed-Loop Control (is the motor tracking the commanded trajectory?)
  - Joint trajectory controller receives JointTrajectory from MoveIt
  - At each control cycle (e.g., 1kHz), it reads encoder position
  - It computes error: error = commanded_position - actual_position
  - It runs a PID controller to compute motor current/voltage command
  - This happens in the hardware interface (on the motor controller or in ros2_control)
```

### 7.3 ros2_control Hardware Interface for Live Mode

The existing `/pseudo_hardware_node` (confirmed in screenshots) is the **simulation hardware interface**. For live mode, a **real hardware interface** must be implemented (or verified if it exists).

**Check first:** Does a `real_hardware_interface` node or package exist in the repository? Look for files named `*_hardware_interface*`, `*_ros2_control*`, or similar.

**If it does not exist, implement:**

```cpp
// RealHardwareInterface : public hardware_interface::SystemInterface
// Key methods:

on_activate():
  // Open communication to controller (Serial/EtherCAT/UDP)
  // Perform handshake (Phase 6 logic)
  // Start encoder read loop at communication_rate_hz

read(time, period):
  // Called at control rate (e.g., 1kHz)
  // Read encoder positions from controller
  // Convert encoder counts to radians:
  //   For incremental: position_rad = (counts / encoder_resolution) * 2π / gear_ratio
  //   For absolute: position_rad = (counts / (2^bits)) * 2π / gear_ratio
  // Store in state_interfaces_ (position, velocity, effort)
  // Publish to /joint_states via state_broadcaster

write(time, period):
  // Called at control rate
  // Read commanded positions from command_interfaces_ (set by JointTrajectoryController PID)
  // Convert to motor commands (counts, PWM, current, etc. based on motor type)
  // Send to motor controller over the communication interface
```

### 7.4 Control Loop Architecture — PID vs Iterative IK

**Question answered:** Use **cascaded PID** at the joint control level, NOT iterative IK. Here is the reasoning:

```
Level 1 (Trajectory Level — MoveIt):
  Iterative IK / Path Planning
  Input: Target Cartesian pose
  Output: JointTrajectory (sequence of joint positions + timestamps)
  Rate: Per-motion (not real-time)
  
Level 2 (Joint Trajectory Tracking — ros2_control JointTrajectoryController):
  PID control
  Input: Desired joint position at time t (from trajectory interpolation)
  Output: Motor command (position setpoint or current command)
  Rate: 1kHz (control rate)
  
Level 3 (Motor Current Control — in motor controller firmware):
  Inner PID (current/torque loop)
  Input: Desired current from Level 2
  Output: PWM duty cycle to motor phases
  Rate: 10-100kHz (depends on motor controller)
```

**Rationale:** Iterative IK operates on Cartesian space and is used ONCE per motion to find the joint solution. Once the joint trajectory is computed, you do NOT need IK in the real-time loop. The real-time loop only needs to track joint position references using PID. This is the industry-standard approach (used by ABB, KUKA, Fanuc, etc.).

### 7.5 PID Tuning Parameters — Exposed in Config Tab

Each axis's joint trajectory controller PID must be tunable from the Config tab UI (Phase 8). The parameters are:

```yaml
# Per-joint PID (in ros2_control JointTrajectoryController params)
joint_1:
  p: 100.0
  i: 0.01
  d: 10.0
  i_clamp: 10.0
  antiwindup: true
```

These must be dynamically reconfigurable via ROS 2 parameter server without restarting the controller (use `rclcpp::Parameter` and `on_set_parameters_callback`).

### 7.6 Motor Command Output — Dynamic Based on Motor and Encoder Type

The Device Contract established during handshake (Phase 6) defines what command type to send:

| Motor Type | Control Mode | Command Format |
|---|---|---|
| BLDC Sinusoidal | FOC Current Control | Desired q-axis current (A) |
| BLDC Trapezoidal | Commutation + PWM | 6-step PWM pattern + magnitude |
| DC Brushed | Voltage / PWM | PWM duty cycle (0–100%) |
| Stepper | Step + Direction | Step pulses + direction bit |
| Servo (integrated) | Position | Target position in counts or radians |

The hardware interface's `write()` method must format the command based on the `motor_type` field from the Device Contract. This mapping is done once at startup, not on every control cycle.

### 7.7 ROS 2 Contract Document — Sent on Successful Handshake

After homing and POST checks, the software formalizes a **ROS 2 Contract** published to a `/robot_contract` topic (latched):

```json
{
  "timestamp": "ISO8601",
  "robot_name": "MyRobot",
  "dof": 6,
  "axes": [
    {
      "axis_id": 1,
      "joint_name": "joint_1",
      "motor_type": "BLDC_sinusoidal",
      "encoder_type": "absolute",
      "encoder_bits": 17,
      "encoder_resolution": 131072,
      "poles": 14,
      "gear_ratio": 100.0,
      "max_velocity_rpm": 3000,
      "max_current_A": 10.0,
      "control_mode": "current"
    }
  ],
  "communication_interface": "EtherCAT | Serial | UDP",
  "communication_rate_hz": 1000,
  "homing_complete": true,
  "all_axes_ready": true
}
```

This contract is consumed by:
- The hardware interface (to configure the `write()` command format)
- MoveIt's `joint_limits.yaml` validator
- The frontend's Config tab (to display current configuration)
- The ROS 2 Monitor tab in the Debug panel (to show live status)

### Phase 7 Verification Checklist
```
[ ] Real hardware interface reads encoder positions at configured rate
[ ] /joint_states updates from real encoder data when in Live Mode
[ ] 3D Viewport shows real robot position based on encoder feedback
[ ] JointTrajectoryController PID tracks commanded trajectory (check error < threshold)
[ ] Motor command format matches motor type (verify by checking actual motor response)
[ ] PID gains are tunable from UI without restarting the controller
[ ] Robot Contract is published to /robot_contract topic on successful handshake
[ ] Heartbeat monitoring continues during live execution
[ ] Stopping execution in live mode sends a deceleration command (not an abrupt stop)
[ ] Encoder overflow/rollover is handled correctly for incremental encoders
```

---

<a name="phase-8"></a>
## Phase 8 — Robot Configuration Tab (Frontend + Backend)

### 8.1 Config Tab Structure

The Config tab (accessible via the left nav icon) must have the following sections:

```
Config Tab
├── Robot Identity
├── URDF / Model
├── Axes & Motors        ← One sub-section per axis (J1 to Jn)
├── I/O Configuration
├── Communication
├── PID Tuning           ← Live tuning (only enabled in Live Mode)
└── Safety Limits
```

### 8.2 Robot Identity Section

| Field | Type | Notes |
|---|---|---|
| Robot Name | Text input | Used in URDF name and ROS namespace |
| Robot Type | Dropdown | 6-DOF Serial, SCARA, Delta, Cartesian XYZ, 7-DOF Serial |
| Manufacturer | Text input | Optional, for documentation |
| DOF | Number (read-only after initial setup) | Drives how many Axis cards are shown |
| Description | Textarea | Free text notes |

### 8.3 URDF / Model Section

| Field | Type | Notes |
|---|---|---|
| Current URDF File | File display (read-only) + "Edit" button + "Re-import" button | Shows filename and last modified date |
| Edit URDF | Opens a code editor (Monaco/CodeMirror) with the raw URDF XML | Syntax highlighting for URDF/XML |
| Robot Dimensions | Per-link length, mass, inertia inputs | These feed into the URDF template regeneration |
| Validate URDF | Button → runs `check_urdf` / `urdf_to_graphiz` | Shows validation result + joint tree diagram |
| Preview | Button → opens the robot model in a mini 3D viewer within the Config tab | |

**URDF Field Mapping — User-editable robot dimensions:**

For each link in the URDF:
```
Link: [link_name]
  ├── Length (mm): [input]
  ├── Mass (kg): [input]
  ├── Inertia Ixx/Iyy/Izz (kg·m²): [inputs]
  └── Visual mesh: [file picker or primitive shape selector]

Joint: [joint_name]
  ├── Type: fixed | revolute | prismatic | continuous
  ├── Axis: X | Y | Z (or custom vector)
  ├── Limit min (deg or mm): [input]
  ├── Limit max (deg or mm): [input]
  ├── Velocity limit (deg/s or mm/s): [input]
  └── Effort limit (N·m or N): [input]
```

**On any change to robot dimensions or joint limits:**
1. Regenerate the URDF file from a template
2. Save the new URDF to backend storage
3. Re-upload to the ROS 2 parameter server (`robot_description` parameter)
4. Send a signal to `/robot_state_publisher` to reload
5. Invalidate the waypoint joint-angle cache
6. Show a notification: "URDF updated — waypoint IK cache invalidated. Re-verify waypoints."

### 8.4 Axes & Motors Section

For each axis (J1 to Jn), show a card with:

**Motor Configuration:**
```
Axis [N] — [Joint Name]
┌──────────────────────────────────────────────┐
│ Motor Type:     [Dropdown]                   │
│   Options:                                   │
│   • BLDC — Sinusoidal (FOC)                  │
│   • BLDC — Trapezoidal (6-step)              │
│   • DC Brushed                               │
│   • Stepper — Full Step                      │
│   • Stepper — Microstepping                  │
│   • Integrated Servo (position command)      │
│                                              │
│ Number of Phases: [2 / 3]                    │
│ Number of Poles: [number input]              │
│ Rated Current (A): [number input]            │
│ Max Velocity (RPM): [number input]           │
│ Gear Ratio: [number input] : 1               │
│                                              │
│ For BLDC:                                    │
│   Control Mode: [Sinusoidal FOC / Trapezoidal]│
│   Phase Order: [ABC / ACB]                   │
│                                              │
│ For Stepper:                                 │
│   Microstepping: [1 / 2 / 4 / 8 / 16 / 32]  │
│   Steps/Rev: [auto-calculated]              │
└──────────────────────────────────────────────┘

Encoder Configuration:
┌──────────────────────────────────────────────┐
│ Encoder Category: [Dropdown]                 │
│   • Magnetic                                 │
│   • Optical                                  │
│                                              │
│ Encoder Mode: [Dropdown]                     │
│   • Absolute (single-turn)                   │
│   • Absolute (multi-turn)                    │
│   • Incremental (quadrature)                 │
│                                              │
│ Resolution / Bits: [number input]            │
│   (for absolute: bits, e.g., 17-bit = 131072 │
│    for incremental: PPR, e.g., 2048)         │
│                                              │
│ Interface: [SSI / SPI / BiSS-C / ABI / CAN] │
└──────────────────────────────────────────────┘
```

### 8.5 Communication Section

```
Interface Type: [Dropdown]
  • Serial (USB/RS232/RS485)
  • EtherCAT
  • UDP/Ethernet
  • CAN Bus

Serial Settings (if Serial selected):
  Port: [auto-detected dropdown / manual entry]
  Baud Rate: [9600 / 57600 / 115200 / 230400 / 1000000]
  Data bits: 8, Stop bits: 1, Parity: None

EtherCAT Settings:
  Network Adapter: [dropdown of available adapters]
  
Communication Rate: [number input] Hz (max based on interface)
```

### 8.6 Backend — Config Storage and Application

**Config Schema (stored as JSON, e.g., `robot_config.json`):**
```json
{
  "robot_id": "uuid",
  "robot_name": "MyRobot",
  "robot_type": "6DOF_serial",
  "dof": 6,
  "urdf_path": "/configs/robots/myrobot.urdf",
  "axes": [ { ...per-axis motor and encoder config... } ],
  "communication": { ...interface config... },
  "pid_gains": { ...per-joint PID... },
  "safety": { "max_tcp_velocity_mm_s": 500, "workspace_box": {...} }
}
```

**API Endpoints (check if exist, add if missing):**
```
GET  /api/config          → Returns current robot_config.json
PUT  /api/config          → Updates and saves robot_config.json
POST /api/config/apply    → Applies config: regenerates URDF, pushes to ROS 2 params, 
                            rebuilds hardware interface config
POST /api/config/validate → Runs urdf validation, returns warnings/errors
```

### Phase 8 Verification Checklist
```
[ ] Config tab is accessible from left nav
[ ] All sections (Robot Identity, URDF, Axes & Motors, I/O, Communication, PID, Safety) are present
[ ] Changing robot name updates the ROS namespace
[ ] Changing joint limits regenerates URDF and reloads robot_state_publisher
[ ] Motor type dropdown shows all 6 motor types
[ ] Encoder type dropdown shows all 4 encoder categories + modes
[ ] Configuration is persisted (survives browser refresh)
[ ] /api/config GET returns current config
[ ] /api/config PUT saves and /api/config/apply pushes to ROS 2
[ ] PID gains are editable and applied to JointTrajectoryController live (Live Mode only)
[ ] URDF editor shows syntax-highlighted URDF XML
[ ] URDF validate button shows pass/fail with details
```

---

<a name="phase-9"></a>
## Phase 9 — URDF Import Pipeline (SolidWorks / STEP / Other CAD Formats)

### 9.1 Overview

This phase enables users to import a 3D CAD model of their robot and have it automatically converted to a URDF file that RoboForge and ROS 2 can use.

### 9.2 Currently Available Tool — sw2urdf (SolidWorks to URDF)

**You mentioned a GitHub repository for SolidWorks to URDF conversion.** The most widely used tool is:

**`ros/solidworks_urdf_exporter`** (on GitHub at `github.com/ros/solidworks_urdf_exporter`)

This is a SolidWorks Add-In that runs inside SolidWorks and exports directly to URDF. It is **not a server-side converter** — it requires SolidWorks to be installed on the user's machine.

**What this means for RoboForge:**
- You cannot run `sw2urdf` on the server because it requires SolidWorks
- The user must run `sw2urdf` on their own machine first, which produces a URDF + mesh files
- The user then uploads the resulting URDF package to RoboForge

**However, for STEP files** (which are SolidWorks-independent), server-side conversion IS possible.

### 9.3 Supported Import Formats

| Format | Conversion Method | Where it Runs |
|---|---|---|
| `.URDF` (direct) | No conversion needed | Server: parse and load |
| `.STEP / .STP` | `step_to_urdf` via OpenCASCADE + onshape-api or `freecad` CLI | Server-side |
| `.STL` (mesh only) | User provides mesh, UI assists with joint definition | Server: use mesh as visual, user defines joints |
| `.SolidWorks` export (from sw2urdf) | User runs sw2urdf plugin, uploads resulting URDF package | Client-side conversion, server receives URDF |
| Fusion 360 | Export to STEP from Fusion 360, then use STEP pipeline | |
| Blender | Export to Collada (.dae) or STL, then assemble URDF | |

### 9.4 STEP to URDF Server-Side Conversion Pipeline

**Verify first:** Check if FreeCAD or OpenCASCADE is installed on the server. Run `freecad --version` or check Docker image.

**If available (FreeCAD is recommended — it is open source and has a Python API):**

```python
# Server-side script: step_to_urdf.py
# 1. Accept uploaded .STEP file
# 2. Open with FreeCAD Python API
import FreeCAD, Import, Part

doc = FreeCAD.newDocument()
Import.insert(step_file_path, doc.Name)

# 3. Extract all bodies (links)
links = [obj for obj in doc.Objects if obj.TypeId in ["Part::Feature", "PartDesign::Body"]]

# 4. For each link: extract bounding box, volume, center of mass
for link in links:
    shape = link.Shape
    bbox = shape.BoundBox
    volume = shape.Volume
    com = shape.CenterOfMass

# 5. Detect joints (this is the hard part — see below)
# 6. Export each link mesh as .STL or .DAE
# 7. Generate URDF XML from extracted data
```

**Joint Detection (Critical Step):**

Fully automatic joint detection from a STEP file is **not reliably possible** without additional metadata. The approach is:

1. **Auto-detect candidates:** Find pairs of bodies that share a face or are within a configurable distance threshold. These are candidate joint locations.
2. **Show the user a detection popup** (see 9.5 below) — do NOT silently assign joint types.
3. **User verifies and corrects** all joint definitions before the URDF is finalized.

### 9.5 Post-Import Detection Popup

After the STEP/URDF file is processed, display a **modal popup** with:

```
┌─────────────────────────────────────────────────────────────────┐
│  URDF Import Detection Results                                  │
│                                                                 │
│  [3D preview image of detected robot structure]                 │
│                                                                 │
│  Detected Links: 7                                             │
│  Detected Joints: 6                                            │
│                                                                 │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │ Joint 1: base_link → link_1                             │   │
│  │   Type:     [revolute ▼]  ← user can change            │   │
│  │   Axis:     [Z ▼]                                       │   │
│  │   Min:      [-180°] Max: [180°]                         │   │
│  │   Origin:   x:[0] y:[0] z:[120mm] (auto-detected)       │   │
│  │   Override: [Edit manually]                             │   │
│  ├─────────────────────────────────────────────────────────┤   │
│  │ Joint 2: link_1 → link_2           [same layout]        │   │
│  │ ...                                                     │   │
│  └─────────────────────────────────────────────────────────┘   │
│                                                                 │
│  ⚠ WARNING: Joint origin positions are estimated. Verify all   │
│  values against your CAD drawing before confirming.            │
│                                                                 │
│  [Cancel]                    [Confirm & Generate URDF]         │
└─────────────────────────────────────────────────────────────────┘
```

### 9.6 URDF Generation and Delivery

After the user confirms the joint definitions:

1. Generate the URDF XML file on the server
2. Save it to the configured URDF storage path (e.g., `/storage/robots/{robot_name}/robot.urdf`)
3. Save associated mesh files to `/storage/robots/{robot_name}/meshes/`
4. Update `robot_config.json` with the new URDF path
5. **Push to ROS 2:**
   - Load the URDF content
   - Set the `robot_description` ROS 2 parameter on the parameter server
   - Restart or signal `/robot_state_publisher` to reload
   - Restart `/move_group` with the new URDF (required for MoveIt to update its planning scene)
6. **Notify the frontend:** Show success notification with link to open the Config tab and verify

### 9.7 File Management — Online Platform Considerations

Since this is an online platform (not a local desktop application), file management must account for:

- **Storage:** Use a server-side file storage (local filesystem, S3-compatible, or similar). Each robot config is a folder: `/storage/robots/{robot_id}/`
- **Upload:** Use multipart form upload for STEP/URDF files. Show upload progress.
- **Security:** Validate uploaded file types (MIME + magic bytes). Maximum file size limit (e.g., 50MB).
- **Mesh serving:** Serve mesh files (STL/DAE) via a static file server or CDN so the 3D Viewport can load them in the browser.
- **ROS 2 integration:** The ROS 2 stack runs on the server (or in a container). The URDF is stored server-side and loaded by server-side ROS 2 nodes. The browser only needs to load mesh files for visualization.

### Phase 9 Verification Checklist
```
[ ] URDF direct upload works (drag-and-drop or file picker)
[ ] STEP file upload triggers server-side FreeCAD conversion
[ ] Post-import popup shows 3D preview and detected joints
[ ] User can change joint type, axis, limits, and origin in the popup
[ ] Confirming generates a valid URDF (passes check_urdf validation)
[ ] URDF is saved to server storage and pushed to ROS 2 robot_description param
[ ] /robot_state_publisher reloads with new URDF
[ ] 3D Viewport updates to show the new robot model
[ ] Mesh files are accessible to the browser (served correctly)
[ ] Waypoint IK cache is invalidated after URDF change
[ ] sw2urdf-exported URDF packages (folder with URDF + meshes) can be uploaded as a zip
```

---

<a name="phase-10"></a>
## Phase 10 — Performance Optimization (Gazebo + ROS CPU Load)

### 10.1 Identify the Problem First

Before optimizing, measure current performance:
```bash
# In a terminal on the server:
top -b -n 1 | grep -E "gzserver|gzclient|rviz|move_group|robot_state_publisher"
# Or:
htop  # visual CPU per-process

# For ROS 2 message frequency:
ros2 topic hz /joint_states
ros2 topic hz /clock
```

Record: CPU% per process, RAM usage, message frequencies. This is your baseline.

### 10.2 Gazebo-Specific Optimizations

**Check which Gazebo version is in use first.**

**For Gazebo Classic (gazebo 11):**

```xml
<!-- In the .world file: -->
<physics type="ode">
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>  <!-- Reduce if CPU limited -->
  <max_step_size>0.001</max_step_size>
  <ode>
    <solver>
      <type>quick</type>        <!-- 'quick' is faster than 'world' -->
      <iters>50</iters>         <!-- Reduce to 20-30 if acceptable -->
      <sor>1.3</sor>
    </solver>
  </ode>
</physics>
```

**Reduce visual complexity:**
- Use simplified collision meshes (not visual meshes) for physics. Collision meshes should be primitive shapes (cylinders, boxes) not full STL.
- Disable shadows in Gazebo world file: `<shadows>false</shadows>`
- If using headless Gazebo (no GUI): use `gzserver` only, not `gzclient`

**For Gazebo Ignition/Harmonic:**
- Similar physics parameters via SDF
- Use `gz sim --headless-rendering` if no visual needed server-side

### 10.3 Consider Replacing Gazebo with a Lighter Alternative

**This is a critical decision — check if either of these is a better fit:**

| Option | CPU Load | Accuracy | ROS 2 Support | Notes |
|---|---|---|---|---|
| **Gazebo Classic** | High | Good | Via gazebo_ros_pkgs | Mature but heavy |
| **Gazebo Ignition/Harmonic** | Medium-High | Good | Native | Recommended by ROS 2 |
| **Isaac Sim** | High (GPU) | Excellent | Yes | Requires NVIDIA GPU |
| **Webots** | Low-Medium | Good | Native ROS 2 | Much lighter than Gazebo |
| **PyBullet** | Low | Moderate | Manual bridge | Not native ROS |
| **MuJoCo** | Low | Excellent | Via mujoco_ros2 | Best accuracy/speed ratio |
| **Custom Three.js forward kinematics** | Very Low | No physics | N/A | Only for visual, not dynamics |

**Recommendation for RoboForge (online platform):**
- If physics accuracy is essential: **Webots** is the best balance (native ROS 2, lower CPU than Gazebo, web renderer available)
- If you only need visual motion (no collision/dynamics): Replace Gazebo entirely with **forward kinematics computed client-side in Three.js**. MoveIt computes joint angles → send to browser → Three.js moves the robot model. No simulator needed. **This reduces CPU load drastically.**

**The FK-only approach is viable because:**
- For most robot programming use cases (pick and place, welding paths), users need to see if the motion reaches the target and avoids singularities — not realistic physics
- Collision checking can be done by MoveIt (which uses FCL for collision detection) without Gazebo
- If you need dynamics (force simulation, gravity), keep Gazebo but use it sparingly

**Action: Evaluate with the team which level of simulation fidelity is required. If FK-only is sufficient, replace Gazebo with Three.js FK visualization. If dynamics are required, switch from Gazebo Classic to Webots or MuJoCo.**

### 10.4 ROS 2 Node Optimization

```
Check each node's CPU usage:
/robot_state_publisher: Should be <1% CPU. If higher, check TF publish rate.
  → Reduce TF publish rate if >50Hz: set publish_frequency to 30Hz in launch args.

/move_group: Spikes during planning. Idle should be <2% CPU.
  → This is expected behavior.

/roboforge_bridge: Should be <2% CPU.
  → If higher, check if it is doing unnecessary computation or republishing at too high a rate.
  → Cap WebSocket push rate to frontend at 50Hz maximum (already set per screenshots).
```

### 10.5 Frontend 3D Viewport Optimization

- Use **instanced meshes** in Three.js for repeated geometries
- Use **LOD (Level of Detail)** — simplified mesh for far camera distances
- **Cap animation frame rate** to 60fps (requestAnimationFrame already does this, but ensure no forced re-renders on every WebSocket message — batch updates)
- Only update the 3D model when joint values actually change (delta threshold)
- Use **WebGL2** if available

### Phase 10 Verification Checklist
```
[ ] Baseline CPU usage is measured and documented
[ ] Gazebo physics update rate is configured appropriately (not running at 1000Hz if not needed)
[ ] Gazebo uses simplified collision meshes (primitive shapes) not full visual meshes
[ ] Decision made on Gazebo vs lighter alternative — documented in AUDIT_RESULTS.md
[ ] /robot_state_publisher TF publish rate is ≤ 50Hz
[ ] WebSocket push to frontend is capped at 50Hz
[ ] 3D Viewport renders at consistent 60fps without frame drops during motion
[ ] Total server CPU during simulation is within acceptable limits (define target: <70% on reference hardware)
```

---

<a name="phase-11"></a>
## Phase 11 — Modern Glassy UI Pass

> This phase applies after all functional phases are verified. Do NOT start this phase until Phase 1–10 are complete and verified.

### 11.1 Design System — Glassy / Modern Dark Theme

**Core visual principles:**
- **Dark base** with transparency layers (glassmorphism)
- **High contrast** for interactive elements (accessibility)
- **Real-time data** should feel alive — smooth transitions, not hard cuts
- **Consistency** — same spacing, radius, shadow system everywhere

**CSS Custom Properties (define globally):**
```css
:root {
  /* Glass panels */
  --glass-bg: rgba(20, 24, 36, 0.75);
  --glass-blur: blur(16px);
  --glass-border: 1px solid rgba(255, 255, 255, 0.08);
  --glass-shadow: 0 8px 32px rgba(0, 0, 0, 0.4);
  
  /* Primary accent */
  --accent-primary: #3B82F6;       /* Blue — action buttons, highlights */
  --accent-success: #10B981;       /* Green — running, connected, alive */
  --accent-warning: #F59E0B;       /* Amber — warnings, pending */
  --accent-danger: #EF4444;        /* Red — errors, stopped, offline */
  --accent-info: #6366F1;          /* Indigo — info, ROS 2 elements */
  
  /* Background layers */
  --bg-base: #0F1117;
  --bg-panel: rgba(20, 24, 36, 0.8);
  --bg-hover: rgba(255, 255, 255, 0.05);
  --bg-active: rgba(59, 130, 246, 0.15);
  
  /* Typography */
  --text-primary: rgba(255, 255, 255, 0.92);
  --text-secondary: rgba(255, 255, 255, 0.55);
  --text-muted: rgba(255, 255, 255, 0.3);
  
  /* Spacing */
  --radius-sm: 6px;
  --radius-md: 10px;
  --radius-lg: 16px;
  
  /* Transitions */
  --transition-fast: 150ms ease;
  --transition-smooth: 250ms cubic-bezier(0.4, 0, 0.2, 1);
}
```

### 11.2 Component-Level Requirements

**Left Icon Navigation Bar:**
- Glass background with border-right
- Icons: 40px hit area, icon itself 20px
- Active icon: accent color fill + left 3px accent bar
- Tooltip on hover (show panel name)

**Left Detail Panel:**
- Glass background with blur
- Smooth slide-in animation when panel opens
- Panel header with title and optional collapse button
- Scrollable content area with custom scrollbar

**Program Tree Blocks:**
- Each block type has a distinctive color-coded left border (matching the block circle colors in screenshots: J=blue, L=green, fn=teal, etc.)
- Active/executing block: animated left border pulse + subtle background highlight
- Hover: `--bg-hover` background
- Drag handle: visible on hover

**3D Viewport:**
- Toolbar icons: glass buttons with blur
- Background: dark gradient (not pure black)
- Grid: subtle, low-contrast
- Coordinate axes gizmo: keep in bottom-right corner
- Speed slider: glass pill with accent thumb

**Top Toolbar:**
- Glass bar with `border-bottom: var(--glass-border)`
- Run button: green accent, icon + label
- Stop button: red accent
- Operation Mode (Edit/Simulate/Live): segmented pill selector with animated indicator

**Block Cards in Library:**
- Glass cards with `var(--radius-md)` corners
- Drag handle on right (dots icon)
- Icon on left (colored per block category)
- Two-line layout: block name (primary) + description (secondary/muted)

**Bottom Debug Panel:**
- Fixed height (resizable via drag handle on top edge)
- Tab bar: glass background, active tab has bottom border accent
- Node Graph: render as a `<canvas>` or SVG graph with node boxes in glass style
- "Alive" status badges: pulsing green dot animation

**Live PWM / Controllers section (Debug tab):**
- Bar graphs for J1 CMD, J3 CMD etc.: use CSS custom properties for real-time width animation
- Color: green when nominal, amber when high, red when at limit

### 11.3 Real-Time Data Animations

- Joint state bars in Diagnostics tab: CSS transitions `width 100ms ease`
- Connection status dot: `@keyframes pulse` animation (scale 1.0 → 1.3 → 1.0, 2s loop)
- Executing block highlight: `@keyframes borderPulse` (opacity 1 → 0.4 → 1, 1s loop)
- No animation for safety-critical displays (they must not blink or distract during operation)

### Phase 11 Verification Checklist
```
[ ] All panels use the glass background and blur effect
[ ] Color system is consistent (accent colors only used for their defined purposes)
[ ] Left nav icon bar works correctly (correct panel opens, correct active state)
[ ] Block type colors match the original screenshot color scheme
[ ] Executing block is highlighted with a pulse animation
[ ] Real-time joint state bars update smoothly without flickering
[ ] Responsive: layout works on 1920x1080 and 1366x768
[ ] Text contrast passes WCAG AA (4.5:1 minimum for body text)
[ ] No layout shift when panels open/close
[ ] Transitions are smooth (no janky animations)
```

---

<a name="master-verification-checklist"></a>
## Master Verification Checklist

Run this complete checklist after all phases are done. Treat each item as a test case.

### Simulator Mode — End-to-End Test
```
Precondition: Robot loaded (IRB 6700 or default URDF). Simulator mode selected.

[ ] 1. Open RoboForge → Default program tree loads correctly
[ ] 2. Switch to Blocks view → All block categories are present (Motion, I/O, Flow)
[ ] 3. Add a MoveJ block → Waypoint picker appears in 3D Viewport
[ ] 4. Click a point in 3D Viewport → Waypoint recorded with correct pose
[ ] 5. Click Run on MoveJ block → Robot moves to waypoint in 3D Viewport
[ ] 6. Confirm MoveIt log shows IK solved successfully
[ ] 7. Confirm Gazebo/simulation simulates the motion (or FK-only if Gazebo replaced)
[ ] 8. Confirm 3D Viewport animates smoothly (≥30fps)
[ ] 9. Add a MoveC block → Via-point and end-point can be set (both 3D click and parameter methods)
[ ] 10. Run full PickAndPlace program → All steps execute in order, correct blocks highlighted
[ ] 11. While loop executes correctly (counter increments, tested for 3 iterations)
[ ] 12. Pause mid-execution → Robot stops cleanly after current step
[ ] 13. Resume → Execution continues from correct next step
[ ] 14. Stop → Execution halts, execution state cleared
```

### Live Mode — End-to-End Test (requires hardware or hardware simulator)
```
[ ] 15. Connect controller hardware (or hardware mock)
[ ] 16. Software auto-detects controller on startup
[ ] 17. Handshake completes and Live Mode toggle becomes enabled
[ ] 18. Homing sequence runs (if incremental encoder) or position is read (if absolute)
[ ] 19. 3D Viewport shows robot at its actual homed position
[ ] 20. Toggle to Live Mode
[ ] 21. Run a MoveJ block → Real robot moves (or mock hardware responds)
[ ] 22. /joint_states reflects real encoder readings (verify in ROS2 Monitor tab)
[ ] 23. Disconnect controller mid-run → Software detects within 300ms, stops safely
[ ] 24. Reconnect → Handshake repeats, Live Mode re-enabled after homing
```

### Config & URDF Tests
```
[ ] 25. Open Config tab → All sections visible
[ ] 26. Change robot name → ROS namespace updates
[ ] 27. Upload a STEP file → Conversion runs, detection popup appears
[ ] 28. Verify joints in popup → Confirm → URDF generated and loaded
[ ] 29. 3D Viewport shows imported robot model
[ ] 30. Change a joint limit in Config → URDF regenerates → MoveIt respects new limit
[ ] 31. Change motor type to Stepper → Encoder section updates to show stepper-relevant fields
```

### Layout & UI Tests
```
[ ] 32. ROS 2 Bridge panel is in the LEFT detail panel (NOT on the right side)
[ ] 33. Right panel only shows Properties/Inspector for selected block
[ ] 34. Bottom panel has all 4 tabs: Diagnostics, ROS2 Monitor, Console, Node Graph
[ ] 35. All left nav icons open their correct panels
[ ] 36. Glass visual theme is applied consistently
[ ] 37. No duplicate panels visible simultaneously
```

---

## Appendix A — ROS 2 Topic & Service Reference

```
Published by /roboforge_bridge:
  /joint_states          sensor_msgs/JointState          50Hz
  /tcp_pose              geometry_msgs/PoseStamped       50Hz
  /trajectory            trajectory_msgs/JointTrajectory  on-demand
  /robot_contract        std_msgs/String (JSON)           latched

Subscribed by /roboforge_bridge:
  /cmd_joint             sensor_msgs/JointState           on-demand
  /cmd_pose              geometry_msgs/PoseStamped        on-demand
  /external_trigger      std_msgs/Bool                    on-demand

MoveIt actions (used by bridge):
  /move_group            moveit_msgs/MoveGroup            action
  /execute_trajectory    moveit_msgs/ExecuteTrajectory    action
  /plan_kinematic_path   moveit_msgs/GetMotionPlan        service

ros2_control:
  /joint_trajectory_controller/command   trajectory_msgs/JointTrajectory
  /joint_trajectory_controller/state     control_msgs/JointTrajectoryControllerState
```

## Appendix B — File Structure Reference

```
/project-root
├── frontend/                          ← Browser app
│   ├── src/
│   │   ├── components/
│   │   │   ├── Layout/AppLayout      ← PHASE 1: Fix ROS2Bridge position here
│   │   │   ├── ProgramTree/          ← PHASE 2: Block system
│   │   │   ├── BlockLibrary/         ← PHASE 2: Block types
│   │   │   ├── Viewport3D/           ← PHASE 3: 3D viewer
│   │   │   ├── ROS2Bridge/           ← PHASE 1: Move to left panel
│   │   │   ├── DebugPanel/           ← Stays at bottom
│   │   │   └── ConfigTab/            ← PHASE 8: Robot config UI
│   │   └── styles/
│   │       └── design-system.css     ← PHASE 11: Glass theme vars
├── backend/                          ← Server
│   ├── api/
│   │   ├── program/                  ← execute, compile endpoints
│   │   ├── config/                   ← PHASE 8: config CRUD
│   │   └── urdf/                     ← PHASE 9: import endpoints
│   └── services/
│       ├── ProgramCompiler.ts        ← PHASE 4: IR compiler
│       └── URDFConverter.py          ← PHASE 9: STEP converter
└── ros2/                             ← ROS 2 packages
    ├── roboforge_bridge/             ← Existing bridge node
    ├── roboforge_hardware/           ← PHASE 7: Real hardware interface
    └── roboforge_config/
        ├── config/
        │   ├── kinematics.yaml       ← PHASE 3: IK solver config
        │   ├── joint_limits.yaml     ← PHASE 5: Velocity/accel limits
        │   └── pilz_cartesian_limits.yaml
        └── urdf/                     ← PHASE 9: Generated URDFs stored here
```

---

**END OF SPECIFICATION**  
*Version 1.0 — For use by Antigravity AI development agent*  
*Iterate phase by phase. Verify before proceeding. Do not assume — check the codebase first.*
