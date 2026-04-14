# RoboForge v8.2 — Offline System Guide

> **Native WPF desktop application with full 3D CAD workspace**

---

## 🚀 Quick Start

### Build and Run

```powershell
# Navigate to WPF project
cd src/RoboForge.Wpf

# Build
dotnet build --configuration Debug

# Run
dotnet run
```

---

## 🖥️ Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    WPF UI LAYER                                  │
│                                                                  │
│  MainWindow.xaml                                                 │
│    ┌─────────────────────────────────────────────────────────┐  │
│    │  LEFT PANEL (360px)          │  RIGHT PANEL (*)         │  │
│    │  ┌─────────────────────┐     │  ┌──────────────────┐   │  │
│    │  │ Scene Tree (45%)    │     │  │ Tab Bar          │   │  │
│    │  │  - Robot Root       │     │  │  [Blocks][Script]│   │  │
│    │  │  - Link_1..6        │     │  ├──────────────────┤   │  │
│    │  │  - Sensors          │     │  │ Block Library    │   │  │
│    │  │  - End Effector     │     │  │  MoveJ  MoveL    │   │  │
│    │  │  - Coordinate Frames│     │  │  SetDO  WaitDI   │   │  │
│    │  ├─────────────────────┤     │  │  Gripper  If/Else│   │  │
│    │  │ 3D Viewport (55%)   │     │  ├──────────────────┤   │  │
│    │  │  HelixViewport3D    │     │  │ Block Canvas     │   │  │
│    │  │  [🏠 Homing]       │     │  │  [MoveJ]         │   │  │
│    │  └─────────────────────┘     │  │  [MoveL]         │   │  │
│    │                              │  │  [GripperClose]  │   │  │
│    │                              │  │  [Wait]          │   │  │
│    │                              │  ├──────────────────┤   │  │
│    │                              │  │ Diagnostics      │   │  │
│    │                              │  │ [▶][⏸][⏹]        │   │  │
│    └─────────────────────────────────────────────────────────┘  │
│                                                                  │
│  STATUS BAR (28px)                                               │
│  [● Connected] [Device: None] [MODE: SIM] [Speed: 50%] [Saved]  │
└─────────────────────────────────────────────────────────────────┘
```

---

## 📦 Project Structure

```
src/RoboForge.Wpf/
│
├── Models/
│   └── RobotModel.cs              # Robot structure models
│       ├── RobotModel             # Root robot model
│       ├── LinkModel              # Rigid body segments
│       ├── JointModel             # Joint configuration
│       ├── SensorModel            # Attached sensors
│       ├── EndEffectorModel       # Tool configuration
│       └── CoordinateFrameModel   # Coordinate frames
│
├── AST/
│   └── AstNodes.cs                # Program AST system
│       ├── AstNode                # Base class
│       ├── ProgramNode            # Root program node
│       ├── BlockNode              # Base for blocks
│       ├── MoveJNode              # Joint space move
│       ├── MoveLNode              # Linear TCP move
│       ├── MoveCNode              # Circular arc move
│       ├── SetDONode              # Set digital output
│       ├── WaitDINode             # Wait for digital input
│       ├── GripperOpenNode        # Open gripper
│       ├── GripperCloseNode       # Close gripper
│       ├── IfNode                 # Conditional branch
│       ├── WhileNode              # Conditional loop
│       ├── ForNode                # Count loop
│       └── StopNode               # Program stop
│
├── Core/
│   ├── StateBus.cs                # Rx.NET state distribution
│   │   └── ExecutionStateUpdate   # State update message
│   ├── Compiler.cs                # AST → InstructionList
│   │   └── Compile()              # Compile program
│   └── ExecutionEngines.cs        # Execution engines
│       ├── IExecutionEngine       # Interface
│       ├── GhostExecutionEngine   # Simulation engine
│       └── RealExecutionEngine    # Hardware engine (stub)
│
├── MainWindow.xaml                # UI layout
└── MainWindow.xaml.cs             # UI logic + subscriptions
```

---

## 🔧 Data Flow Architecture

### The Backbone: State Bus

All communication flows through the **State Bus** (Rx.NET `BehaviorSubject`):

```
┌─────────────┐    ┌───────────────┐    ┌──────────────────┐
│ Block Editor│───▶│ AST (Program) │───▶│ Compiler         │
│ (Drag-drop) │    │ Single source │    │ AST → Instructions│
└─────────────┘    └───────────────┘    └────────┬─────────┘
                                                  │
                                                  ▼
                                    ┌─────────────────────────┐
                                    │ Execution Engine        │
                                    │ (Ghost or Real)         │
                                    └───────────┬─────────────┘
                                                │
                                                ▼
                                    ┌─────────────────────────┐
                                    │ State Bus               │
                                    │ BehaviorSubject         │
                                    │ (ExecutionStateUpdate)  │
                                    └───────────┬─────────────┘
                                                │
                            ┌───────────────────┼───────────────┐
                            ▼                   ▼               ▼
                    ┌───────────────┐   ┌───────────┐   ┌───────────┐
                    │ 3D Viewport   │   │ Block     │   │Diagnostics│
                    │ Joint updates │   │ Highlight │   │ Messages  │
                    └───────────────┘   └───────────┘   └───────────┘
```

### Key Principle
**One source of truth**: The `ProgramAST` object in memory. The block editor and script editor are both **views** of this same AST. Data never flows backward from UI to AST directly — all mutations go through **commands**.

---

## 🎯 Features Implemented

### Scene Graph (TRS Hierarchy)
- ✅ 6 articulated links with Scale → Rotate → Translate transforms
- ✅ `GetLinkTransform(int)` method for gizmo access
- ✅ `GetLinkWorldPosition(int)` for world-space calculations
- ✅ Selection highlighting (cyan #00D4FF)

### Program AST
- ✅ 18 block types defined
- ✅ Depth-first tree traversal
- ✅ Parent-child relationships
- ✅ Node ID system (UUID)

### Compiler
- ✅ AST → flat InstructionList
- ✅ Control flow with jump instructions (While, If/Else)
- ✅ Duration estimation per instruction

### Ghost Execution Engine
- ✅ 60Hz state updates (16ms intervals)
- ✅ Cubic ease-in-out interpolation for smooth motion
- ✅ Virtual IO state simulation
- ✅ Block highlight publishing to StateBus
- ✅ 10x speed for Wait blocks in simulation

### State Bus
- ✅ Rx.NET `BehaviorSubject<ExecutionStateUpdate>`
- ✅ Subscription with disposal support
- ✅ Helper methods: `UpdateJointAngles()`, `UpdateActiveNode()`, `UpdateProgramState()`

### UI
- ✅ 3-column layout (360px | 6px splitter | *)
- ✅ Custom title bar (frameless window)
- ✅ Scene tree with hierarchical nodes
- ✅ 3D viewport (HelixToolkit.Wpf)
- ✅ Block editor with library sidebar
- ✅ Script editor tab
- ✅ Collapsible diagnostics panel
- ✅ Persistent status bar
- ✅ "Startup & Homing" button

---

## 🛠️ Building

### Prerequisites
- .NET 8.0 SDK
- Windows 10/11
- Visual Studio 2022 (optional)

### Commands
```powershell
# Build
dotnet build src/RoboForge.Wpf/RoboForge.Wpf.csproj --configuration Debug

# Run
dotnet run --project src/RoboForge.Wpf/RoboForge.Wpf.csproj

# Publish standalone executable
dotnet publish src/RoboForge.Wpf/RoboForge.Wpf.csproj -c Release -o publish/
```

### Build Status
```
✅ Build Succeeded
✅ 0 Errors
⚠️ 2 Warnings (HelixToolkit compatibility — works fine)
```

---

## 🎮 Controls

### Keyboard Shortcuts
| Key | Action |
|-----|--------|
| `Ctrl+S` | Save project |
| `Ctrl+O` | Open project |
| `Ctrl+Z` | Undo |
| `Ctrl+Y` | Redo |
| `F5` | Run program |
| `F6` | Pause program |
| `Shift+F5` | Stop program |

### Block Library Colors
| Category | Color | Blocks |
|----------|-------|--------|
| Motion | #4A90D9 (Steel Blue) | MoveJ, MoveL, MoveC, MoveAbsJ |
| I/O | #E8A020 (Amber) | SetDO, PulseDO, WaitDI |
| Flow | #8B5CF6 (Violet) | Wait, If/Else, While, For |
| Robot | #22C55E (Green) | GripperOpen, GripperClose |
| Structure | #F43F5E (Rose) | Routine, CallRoutine |

---

## 📋 Roadmap (Next Steps)

### Phase 4: IO Detection & Auto-Configuration
- [ ] USB device enumeration (WMI on Windows, /dev/tty* on Linux)
- [ ] Device fingerprint database (Arduino, ESP32, STM32, etc.)
- [ ] Serial handshake protocol (ROBOFORGE_HELLO/ACK)
- [ ] Pin mapping UI with board schematics

### Phase 5: Panel Docking System
- [ ] DockablePanel class with 4 states (Docked, Auto-hide, Floating, Tabbed)
- [ ] GridSplitter with hover highlighting
- [ ] Drag-to-reorder panel headers
- [ ] Settings persistence

### Phase 6: File System (.rfproj)
- [ ] ZIP-based project format
- [ ] Save/Open operations with atomic writes
- [ ] Auto-save every 5 minutes
- [ ] Recent projects list

### Phase 7: Homing Sequence
- [ ] Pre-flight checks modal
- [ ] Per-joint homing algorithm (Hall sensor + PID settle)
- [ ] Real-time error convergence graph
- [ ] Post-home verification

### Phase 8: Serial Communication
- [ ] SerialCommunicationService on background thread
- [ ] Command queue with async/await
- [ ] Response parsing with cancellation tokens
- [ ] Multi-device support (up to 4 devices)

---

## 🔄 Rollback Instructions

All changes are tracked via Git:

```powershell
# View WPF-specific commits
git log --oneline -- src/RoboForge.Wpf/

# Revert WPF to previous state
git checkout <commit-hash> -- src/RoboForge.Wpf/

# Create backup before major changes
git branch wpf-backup-$(Get-Date -Format "yyyy-MM-dd")
```

---

*Offline System v8.2 | Last Updated: 2026-04-14*
*Build Status: ✅ 0 Errors*
