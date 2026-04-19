# RoboForge — Master Implementation Plan

**Date:** 2026-04-14
**Goal:** Make offline WPF client 100% functional mirror of online React IDE with SolidWorks-like 3D viewport, full command manager, and verified end-to-end pipeline.

---

## 📋 PHASE-BY-PHASE PLAN

### PHASE 0: COMPREHENSIVE AUDIT (10 min planning → 30 min execution)

**Objective:** Check EVERY file, line-by-line, for errors, missing functionality, and broken connections.

#### 0.1 Online System Audit (React IDE)
```
Files to check:
├── NEW_UI/remix-of-roboflow-studio/src/
│   ├── App.tsx                    ← Entry point, routing
│   ├── pages/Index.tsx            ← Main layout
│   ├── components/
│   │   ├── robot/
│   │   │   ├── Viewport3D.tsx     ← 3D viewport (Three.js + Drei)
│   │   │   ├── IndustrialRobot.tsx← Robot model
│   │   │   ├── PathVisualization.tsx
│   │   │   ├── SceneObjects.tsx
│   │   │   └── ... (all robot components)
│   │   ├── program/
│   │   │   ├── ProgramTree.tsx
│   │   │   ├── BlockEditor.tsx
│   │   │   └── ... (all program components)
│   │   ├── ui/                    ← UI primitives
│   │   └── layout/                ← Layout components
│   ├── store/
│   │   ├── AppState.tsx           ← Main state (773 lines)
│   │   └── RosConnection.ts       ← ROS connection state
│   ├── services/
│   │   └── BackendConnector.ts    ← ROS bridge connection
│   └── engine/
│       ├── IKSolver.ts            ← Local IK solver
│       └── TrajectoryPlanner.ts   ← Trajectory planning
```

**For each file, check:**
- [ ] Does it compile? (TypeScript errors)
- [ ] Does it have actual logic or is it a stub?
- [ ] Are all functions implemented or are they TODO comments?
- [ ] Does it connect to backend (BackendConnector)?
- [ ] Does it update state (AppState)?
- [ ] Does it render correctly (UI components)?

#### 0.2 Offline System Audit (WPF Client)
```
Files to check:
├── src/RoboForge.Wpf/
│   ├── App.xaml.cs                ← Entry point
│   ├── MainWindow.xaml            ← Main UI layout
│   ├── MainWindow.xaml.cs         ← Code-behind
│   ├── AST/AstNodes.cs            ← Program AST
│   ├── Bridge/Ros2BridgeClient.cs ← ROS WebSocket client
│   ├── Core/
│   │   ├── Compiler.cs            ← AST → Instructions
│   │   ├── ConditionEvaluator.cs  ← Expression parser
│   │   ├── ExecutionEngines.cs    ← Ghost/Real engines
│   │   └── StateBus.cs            ← Reactive state
│   ├── FileSystem/ProjectFileHandler.cs
│   ├── Homing/HomingSequence.cs
│   ├── IO/
│   │   ├── DeviceDetection.cs
│   │   └── HandshakeAndConfig.cs
│   ├── Models/RobotModel.cs
│   └── Themes/
```

**For each file, check:**
- [ ] Does it compile? (C# errors)
- [ ] Are all methods implemented?
- [ ] Do button handlers actually DO something (not just log)?
- [ ] Does Ros2BridgeClient actually connect and parse data?
- [ ] Does ExecutionEngine actually execute instructions?
- [ ] Does 3D viewport actually update with robot pose?
- [ ] Does StateBus actually distribute state changes?

---

### PHASE 1: MIRROR ONLINE → OFFLINE FUNCTIONALITY (2-3 hours)

**Objective:** Every feature in React IDE must exist in WPF client.

#### 1.1 Feature Mapping Table

| Online Feature (React) | Offline Status (WPF) | Action Required |
|------------------------|----------------------|-----------------|
| 3D Viewport with Three.js | HelixViewport3D | ✅ Exists, needs SolidWorks toolbar |
| Program Tree (nested blocks) | SceneTree (flat) | ❌ Need nested tree support |
| Block Editor (drag-drop) | Stub | ❌ Need full implementation |
| IK Solver (MoveIt + local) | Stub | ❌ Need actual IK calls |
| Waypoint placement in 3D | Stub | ❌ Need click-to-place |
| Scene Outliner (objects) | Stub | ❌ Need object management |
| Properties Panel | Stub | ❌ Need property grid |
| I/O Panel | Stub | ❌ Need IO signal control |
| Debug Panel | Stub | ❌ Need variable watch |
| ROS 2 Panel | Stub | ❌ Need topic monitor |
| Execute Program (▶ Run) | Stub | ❌ Need actual execution |
| Step Through (⏭ Step) | Stub | ❌ Need step execution |
| Pause/Stop (⏸ ⏹) | Stub | ❌ Need state machine |
| Joint Sliders | ✅ Working | ✅ Done |
| Motor PWM | ✅ Working | ✅ Done |
| Connection Status | ✅ Working | ✅ Done |
| Status Bar | ✅ Working | ✅ Done |

#### 1.2 Priority Order for Implementation

**P0 (Must Work):**
1. Block Editor (drag-drop blocks)
2. Program Execution (Run/Step/Pause/Stop)
3. 3D Viewport robot animation
4. IK Solver (via MoveIt)

**P1 (Should Work):**
5. Waypoint placement in 3D
6. Scene Outliner
7. Properties Panel
8. I/O Panel

**P2 (Nice to Have):**
9. Debug Panel
10. ROS 2 Panel
11. Config Panel
12. Settings

---

### PHASE 2: SOLIDWORKS-LIKE 3D VIEWPORT (2-3 hours)

**Objective:** Make 3D viewport match SolidWorks functionality.

#### 2.1 SolidWorks Feature Mapping

| SolidWorks Feature | Implementation Plan |
|-------------------|---------------------|
| FeatureManager Design Tree | Left panel with assembly tree + feature history |
| CommandManager Ribbon | Top toolbar with tabs (Features, Sketch, Evaluate, etc.) |
| Graphics Area | Center 3D viewport with selection, zoom, pan |
| PropertyManager | Right panel with context-sensitive properties |
| ConfigurationManager | Tab for configuration management |
| Task Pane | Collapsible right panel for additional tools |

#### 2.2 3D Viewport Implementation Details

**CommandManager Tabs:**
```
┌─────────────────────────────────────────────────────────────┐
│  File  │  Edit  │  View  │  Insert  │  Tools  │  Evaluate  │
└─────────────────────────────────────────────────────────────┘

File Tab:
  [New] [Open] [Save] [Save As] [Export] [Print]

Edit Tab:
  [Undo] [Redo] [Cut] [Copy] [Paste] [Select All]

View Tab:
  [Zoom to Fit] [Zoom to Area] [Pan] [Rotate]
  [Front] [Back] [Left] [Right] [Top] [Bottom] [Isometric]
  [Hide/Show Items] [Appearance] [Display Style]

Insert Tab:
  [Sketch] [Extrude] [Revolve] [Sweep] [Loft]
  [Reference Geometry] [Component] [Assembly]

Tools Tab:
  [Measure] [Mass Properties] [Interference Detection]
  [Collision Detection] [Move Component] [Rotate Component]

Evaluate Tab:
  [Measure] [Section View] [Interference Detection]
  [Collision Detection] [Dynamics Analysis]
```

**Graphics Area Features:**
- Selection highlighting (green outline)
- Hover tooltips
- Right-click context menus
- Drag-and-drop components
- Coordinate triad (XYZ)
- Origin axes
- Reference planes (Front, Top, Right)

---

### PHASE 3: END-TO-END PIPELINE VERIFICATION (3-4 hours)

**Objective:** Program → Execute → 3D Viewport → Gazebo → StateBus → UI must all work.

#### 3.1 Test Cases (10x Online + 10x Offline)

**Test 1: Simple MoveJ Execution**
```
Program:
  MoveJ Home → MoveJ Approach → MoveJ Pick

Expected:
  1. Block highlights during execution
  2. Robot moves in 3D viewport
  3. Joint angles update in real-time
  4. Gazebo receives commands (if bridge connected)
```

**Test 2: MoveL with IK**
```
Program:
  MoveL Pick (x=500, y=0, z=900)

Expected:
  1. IK solver computes joint angles
  2. Robot moves to exact position
  3. TCP reaches target coordinates
```

**Test 3: Gripper Control**
```
Program:
  Gripper Close → Wait 0.5s → Gripper Open

Expected:
  1. Gripper closes in 3D viewport
  2. Wait block pauses execution
  3. Gripper opens after delay
```

**Test 4: While Loop**
```
Program:
  While counter < 3:
    MoveJ Pick
    MoveJ Place
    Increment counter

Expected:
  1. Loop executes 3 times
  2. Counter increments correctly
  3. Program completes successfully
```

**Test 5: If/Else Conditional**
```
Program:
  If sensor1 == true:
    MoveJ Pick
  Else:
    MoveJ Home

Expected:
  1. Condition evaluates correctly
  2. Correct branch executes
```

**Test 6: IO Control**
```
Program:
  SetDO DO_Gripper = 1
  WaitDI DI_PartPresent = 1
  SetDO DO_Gripper = 0

Expected:
  1. Digital output sets correctly
  2. Wait for input works
  3. Output clears after input
```

**Test 7: Subroutine Call**
```
Program:
  CallRoutine PickAndPlace

Expected:
  1. Subroutine executes
  2. Returns to main program
  3. Program continues
```

**Test 8: Multi-Waypoint Path**
```
Program:
  MoveJ WP1 → MoveL WP2 → MoveC WP3, WP4 → MoveJ WP5

Expected:
  1. All waypoints reached
  2. Circular arc correct
  3. Smooth transitions
```

**Test 9: Error Handling**
```
Program:
  MoveJ UnreachablePosition

Expected:
  1. IK solver returns error
  2. Program stops with error message
  3. Diagnostics panel shows error
```

**Test 10: Full Pick-and-Place Cycle**
```
Program:
  Initialize:
    MoveJ Home
    SetDO Gripper = OFF

  While TRUE:
    MoveJ Approach
    MoveL Pick
    Gripper Close
    Wait 0.2s
    MoveL Lift
    MoveJ Place
    Gripper Open
    Wait 0.5s

Expected:
  1. Full cycle executes
  2. Loop continues indefinitely
  3. Can pause/stop at any point
```

---

### PHASE 4: ITERATIVE FIXES UNTIL 100% WORKING (4-6 hours)

**Objective:** Screenshot verification, fix issues, repeat until perfect.

#### 4.1 Screenshot Verification Process

```
For each test case:
1. Run program
2. Take screenshot of:
   - Online React IDE
   - Offline WPF client
   - Gazebo VNC GUI
   - Data flow monitor
3. Compare screenshots
4. Note differences
5. Fix differences
6. Repeat until identical
```

#### 4.2 Known Issues to Fix

**Critical:**
- Block editor doesn't exist (need drag-drop implementation)
- Program execution is stub (need state machine)
- 3D viewport doesn't animate (need joint updates)
- IK solver not wired (need MoveIt calls)
- Buttons only log (need actual functionality)

**High Priority:**
- No drag-drop for blocks
- No waypoint placement in 3D
- No selection in viewport
- No context menus
- No property editing

**Medium Priority:**
- No I/O control
- No debug panel
- No ROS 2 panel
- No config panel

---

### PHASE 5: FINAL VERIFICATION (1 hour)

**Objective:** All tests pass, screenshots match, pipeline verified.

#### 5.1 Final Checklist

- [ ] Online React IDE: All 10 test cases pass
- [ ] Offline WPF Client: All 10 test cases pass
- [ ] Gazebo VNC: Receives commands, shows robot movement
- [ ] Bridge: Parses joint states, calls IK/FK
- [ ] StateBus: Distributes state changes correctly
- [ ] Data Flow: Complete pipeline verified
- [ ] Screenshots: Online matches offline
- [ ] Documentation: Updated with final status

---

## 📊 TIME ESTIMATES

| Phase | Estimated Time | Priority |
|-------|---------------|----------|
| Phase 0: Audit | 30 min | P0 |
| Phase 1: Mirror Features | 2-3 hours | P0 |
| Phase 2: SolidWorks Viewport | 2-3 hours | P1 |
| Phase 3: Pipeline Testing | 3-4 hours | P0 |
| Phase 4: Iterative Fixes | 4-6 hours | P0 |
| Phase 5: Final Verification | 1 hour | P0 |
| **TOTAL** | **13-17 hours** | **Must complete** |

---

## 🎯 SUCCESS CRITERIA

**Minimum Viable:**
- ✅ All 10 test cases pass online
- ✅ All 10 test cases pass offline
- ✅ 3D viewport animates with program execution
- ✅ Gazebo receives commands
- ✅ Screenshots match

**Target:**
- ✅ SolidWorks-like 3D viewport
- ✅ Full command manager with all tabs
- ✅ Feature tree, assembly tree
- ✅ Property manager
- ✅ All panels functional
- ✅ End-to-end pipeline verified 10x each

---

*Plan created: 2026-04-14*
*Status: Ready to execute Phase 0*
