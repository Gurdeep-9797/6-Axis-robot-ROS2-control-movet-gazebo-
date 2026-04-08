# AI_WORKFLOW.md — Combined AI Coordination Protocol
### RoboForge v8.2 | Dual-AI Development Workflow

> This document defines how AI-ANTIGRAVITY (online/React) and AI-QWEN (offline/WPF)
> coordinate to produce a unified, production-grade robotics IDE.
>
> **Rule #1**: Read ROBOFORGE_GLOBAL_SYSTEM.md before starting any session.
> **Rule #2**: Mark your completed tasks in GLOBAL_SYSTEM.md, section 9.
> **Rule #3**: Prefix every major code change with `[ANTIGRAVITY]` or `[QWEN]` in comments.

---

## DIVISION OF OWNERSHIP

```
┌────────────────────────────────────────────────────────────────────┐
│                    SHARED BACKEND                                   │
│  Python roboforge_bridge (:9090 WebSocket, :8765 REST)             │
│  ROS 2 Humble + MoveIt 2 + Gazebo Harmonic (Docker)               │
│  API_AND_CONNECTIONS.md defines all contracts                       │
│              ↑                    ↑                                 │
│   [ANTIGRAVITY]               [QWEN]                               │
│   Online Web App              Offline WPF App                       │
│   React + Three.js            .NET 8 + WPF + HelixToolkit          │
│   BackendConnector.ts         ViewModels.cs (WebSocket)             │
│   IKSolver.ts (JS)            C# IKSolverService                   │
│   :8080 dev server            WPF executable                        │
└────────────────────────────────────────────────────────────────────┘
```

---

## PHASE 1 — STABILIZATION (Current)
*Goal: Both apps build, connect, and show correct robot state*

### ANTIGRAVITY Tasks (Online)

**P1 — Fix block execution IK seed bug** ⬅️ HIGHEST PRIORITY
- File: `src/store/AppState.tsx` → `executeBlock()` function
- Problem: Each block execution starts with `[0,0,0,0,0,0]` seeds instead of current joints
- Fix: Pass `jointAngles` as seed to each IK call in the execution chain
- Test: Run 3-block program → robot should move smoothly without config flips

**P2 — Wire ConsolePanel to slot dropdown**
- File: `src/components/robot/layout/LayoutSlot.tsx`
- Add `{ id: 'console', label: 'Console', icon: '🖥' }` to TOOL_OPTIONS
- Add `console: () => <ConsolePanel />` to ToolRegistry
- Update AppState `ToolId` union type to include `'console'`

**P3 — Fix setActivePanel → LayoutSlot wiring**
- File: `src/pages/Index.tsx`
- Problem: `setActivePanel('io')` from TopRibbon doesn't update left slot
- Fix: In WorkspaceContent, add effect:
  ```ts
  useEffect(() => {
    if (activePanel && activePanel !== 'fullscreen3d' && activePanel !== 'settings') {
      setLayoutSlot('explorer', activePanel as ToolId);
    }
  }, [activePanel]);
  ```

**P4 — GlobalStartPoint: user-editable via 3D right-click**
- File: `src/components/robot/Viewport3D.tsx`
- Add "Set as Home" to context menu
- Calls `setGlobalStartPoint(currentJointAngles)` + log entry
- TopRibbon "Home" button already reads `globalStartPoint` ✅

### QWEN Tasks (Offline WPF)

**P1 — Replace left panel placeholder with Program Tree** ⬅️ HIGHEST PRIORITY
- File: `src/RoboForge.Wpf/MainWindow.xaml`
- Replace left panel "SETTINGS" TextBlock with:
  - ItemsControl bound to `ProgramBlocks`
  - Each item shows Type icon + Name + status dot
  - Selected item highlighted in cyan

**P2 — Add Joint Jog Sliders**
- File: `src/RoboForge.Wpf/MainWindow.xaml` (right panel)
- Add 6 Slider controls (J1-J6) below tracking error widget
- Bind to `J1..J6` in ViewModel
- On value change: call offline IK or publish trajectory

**P3 — Wire HelixToolkit to joint angles**
- File: `src/RoboForge.Wpf/MainWindow.xaml.cs`
- Load robot URDF mesh into Viewport3DX
- Subscribe to J1..J6 property changes → update mesh transforms
- Use `Model3DGroup` with rotation transforms per joint

**P4 — Implement offline C# IK**
- File: `src/RoboForge.ROS2Bridge/Ros2BridgeService.cs`
- Replace stub `SolveIKAsync` with real analytical IK (mirror of JS version)
- Same DH params as `MotionTypes.ts`: IRB6700_DH table

---

## PHASE 2 — FEATURE PARITY (Next)
*Goal: Online and offline apps have the same features*

### ANTIGRAVITY
- [ ] Timeline panel: show execution progress with timestamps
- [ ] Path recording: save robot TCP trail as waypoints
- [ ] Export to RAPID / URScript format
- [ ] Multi-robot: separate joint state per robot instance
- [ ] Collision avoidance preview in 3D

### QWEN  
- [ ] I/O toggle grid panel (mirror IOPanel.tsx)
- [ ] Diagnostics panel with tracking error graph
- [ ] Save/load project to .rfp file (JSON)
- [ ] Hardware config UI (mirror HardwareConfigPage.tsx)
- [ ] Block execution with visual status highlight

---

## PHASE 3 — PRODUCTION HARDENING (Future)
*Goal: Vercel deploy + WPF installer package*

### ANTIGRAVITY
- [ ] Environment-aware backend URL (Vercel VITE_ROS_WS_URL)
- [ ] Cloudflare tunnel instructions in Settings page
- [ ] PWA manifest for offline web use
- [ ] Performance: code-split large bundles (1.8MB → <500KB)

### QWEN
- [ ] NSIS/WiX installer for Windows
- [ ] Auto-update via GitHub releases
- [ ] Auto-launch bridge script from WPF
- [ ] gRPC endpoint in Python bridge for WPF native connection

---

## CROSS-CHECK PROTOCOL

After every major feature:

1. **ANTIGRAVITY verifies**: Does the online app match the feature spec?
2. **QWEN verifies**: Is the same data available in WPF via the same ROS topics?
3. **Both verify**: Does `ROBOFORGE_GLOBAL_SYSTEM.md` accurately reflect current state?

### Shared Contracts (NEVER change without both AIs agreeing)
- ROS topic names in section 6 of GLOBAL_SYSTEM.md
- Joint naming: `joint_1` through `joint_6` (must match URDF)
- IK target format: `{x_mm, y_mm, z_mm}` in UI coords → `/1000.0` for ROS
- Program block JSON format (used in save/load)

---

## HOW TO RUN BOTH APPS SIMULTANEOUSLY

```powershell
# Terminal 1 — Online app (React)
cd NEW_UI\remix-of-roboflow-studio
npm run dev
# → http://localhost:8080

# Terminal 2 — Offline app (WPF)  
cd src\RoboForge.Wpf
dotnet run

# Terminal 3 — Backend (in WSL2 or Docker)
docker-compose up -d
# → rosbridge on ws://localhost:9090
# → Gazebo physics running
# → MoveIt /compute_ik service available

# Both apps will auto-connect to the same backend
# Both will display the same robot state from /joint_states
```

---

## DEBUGGING CHECKLIST

When something breaks:
1. Check browser console (F12) for JS errors
2. Check `[Bridge]` prefixed logs — show connection status
3. Check `[IK]` prefixed logs — show solver mode and errors
4. Check WPF console window for C# exceptions
5. Check Docker container logs: `docker logs roboforge_bridge`
6. Verify URDF joint names match code: `grep "joint_" robot.urdf`

---

## FILE MODIFICATION LOG

| Date | File | AI | Change |
|------|------|----|--------|
| 2026-04-09 | layout/LayoutSlot.tsx | ANTIGRAVITY | Fixed stale JSX bug, added proper dropdown |
| 2026-04-09 | store/AppState.tsx | ANTIGRAVITY | Fixed slot defaults (bottom=diagnostics, inspector=config) |
| 2026-04-09 | ROBOFORGE_GLOBAL_SYSTEM.md | ANTIGRAVITY | Created comprehensive system audit |
| 2026-04-09 | AI_WORKFLOW.md | ANTIGRAVITY | Created this coordination file |
