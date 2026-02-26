using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using System;
using System.Collections.ObjectModel;
using System.Linq;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Input;
using TeachPendant_WPF.Models;
using TeachPendant_WPF.SceneGraph;
using TeachPendant_WPF.Services;

namespace TeachPendant_WPF.ViewModels
{
    /// <summary>
    /// Thin coordinator shell. Instantiates and exposes all sub-ViewModels.
    /// Contains only window-management commands and the startup pipeline.
    /// All domain logic lives in the sub-ViewModels.
    /// </summary>
    public partial class MainViewModel : ObservableObject
    {
        // ── Sub-ViewModels (Bound in XAML via DataContext.XxxVM) ─────
        public GlobalStateViewModel GlobalState { get; }
        public RobotViewModel RobotVM { get; }
        public ProgramViewModel ProgramVM { get; }
        public WorldViewModel WorldVM { get; }
        public SettingsViewModel SettingsVM { get; }
        public WorkTreeViewModel WorkTree { get; }

        // ── Scene Graph (Single Source of Truth) ────────────────────
        public SceneGraphManager SceneGraph { get; }

        // ── Legacy Compatibility (to be removed incrementally) ──────
        [ObservableProperty] private RobotState _currentState;
        [ObservableProperty] private RobotProgram _currentProgram;

        // ── Grid Layout ─────────────────────────────────────────────
        [ObservableProperty] private GridLength _leftNavWidth  = new GridLength(56, GridUnitType.Pixel);
        [ObservableProperty] private GridLength _cmdPaletteWidth = new GridLength(200, GridUnitType.Pixel);
        [ObservableProperty] private GridLength _programWidth    = new GridLength(320, GridUnitType.Pixel);
        [ObservableProperty] private GridLength _viewportWidth   = new GridLength(1, GridUnitType.Star);
        private bool _isWorkAreaExpanded;

        // ── Navigation ──────────────────────────────────────────────
        [ObservableProperty] private string _activeNavPage = "Teaching";

        // ── Command Registry ─────────────────────────────────────────
        public CommandRegistry CommandReg { get; } = new();

        // ── Command Palette ──────────────────────────────────────────
        [ObservableProperty] private bool _isCommandPaletteOpen;
        [ObservableProperty] private string _commandPaletteQuery = string.Empty;
        public ObservableCollection<CommandDefinition> CommandPaletteResults { get; } = new();

        // ── Services ────────────────────────────────────────────────
        private readonly IRobotDriver _driver;
        private readonly ExecutionEngine _engine;
        private readonly DatabaseService _dbService;

        // ── Construction ────────────────────────────────────────────
        public MainViewModel()
        {
            // 1. Core models (legacy, kept for backward compat during migration)
            CurrentState = new RobotState();
            CurrentProgram = new RobotProgram();

            // 2. Scene Graph
            SceneGraph = new SceneGraphManager();
            InitializeRobotInSceneGraph();

            // 3. Database service
            _dbService = new DatabaseService();

            // 4. Sub-ViewModels — MUST be created BEFORE driver/engine connect
            //    because Connect() fires StateUpdated immediately
            GlobalState = new GlobalStateViewModel();
            RobotVM = new RobotViewModel(SceneGraph);
            ProgramVM = new ProgramViewModel();
            WorldVM = new WorldViewModel(SceneGraph);
            SettingsVM = new SettingsViewModel(_dbService);
            WorkTree = new WorkTreeViewModel();

            // 5. Driver — now safe to connect since sub-VMs exist
            _driver = new SimulationDriver();
            _driver.StateUpdated += OnDriverStateUpdated;
            _driver.Connect();

            // 6. Execution engine
            _engine = new ExecutionEngine(_driver);
            _engine.ExecutionStateChanged += (running) =>
            {
                CurrentState.IsRunning = running;
                GlobalState.UpdateExecutionState(running);
            };
            _engine.ActiveLineChanged += (line) =>
            {
                CurrentState.ActiveLineNumber = line;
                ProgramVM.ActiveLineNumber = line;
            };

            // 7. Load data
            _ = LoadInitialDataAsync();

            // 8. Register commands in registry
            RegisterCoreCommands();
        }

        // ── Robot Setup ─────────────────────────────────────────────
        private void InitializeRobotInSceneGraph()
        {
            var robot = new RobotNode { Name = "FR-6DOF" };

            // Define 6 revolute joints matching the URDF
            var jointConfigs = new[]
            {
                (name: "J1", axis: new System.Windows.Media.Media3D.Vector3D(0, 0, 1), min: -170.0, max: 170.0, offset: new System.Windows.Media.Media3D.Vector3D(0, 0, 0)),
                (name: "J2", axis: new System.Windows.Media.Media3D.Vector3D(0, 1, 0), min: -120.0, max: 120.0, offset: new System.Windows.Media.Media3D.Vector3D(0, 0, 152)),
                (name: "J3", axis: new System.Windows.Media.Media3D.Vector3D(0, 1, 0), min: -170.0, max: 170.0, offset: new System.Windows.Media.Media3D.Vector3D(0, 0, 380)),
                (name: "J4", axis: new System.Windows.Media.Media3D.Vector3D(1, 0, 0), min: -180.0, max: 180.0, offset: new System.Windows.Media.Media3D.Vector3D(0, 0, 0)),
                (name: "J5", axis: new System.Windows.Media.Media3D.Vector3D(0, 1, 0), min: -120.0, max: 120.0, offset: new System.Windows.Media.Media3D.Vector3D(0, 0, 325)),
                (name: "J6", axis: new System.Windows.Media.Media3D.Vector3D(1, 0, 0), min: -360.0, max: 360.0, offset: new System.Windows.Media.Media3D.Vector3D(0, 0, 80)),
            };

            foreach (var cfg in jointConfigs)
            {
                var joint = new JointNode
                {
                    Name = cfg.name,
                    Axis = cfg.axis,
                    MinLimit = cfg.min,
                    MaxLimit = cfg.max,
                    OriginOffset = cfg.offset
                };
                robot.AddJoint(joint);
            }

            // Tool frame at the end effector
            var tool = new FrameNode
            {
                Name = "Tool0",
                FrameKind = FrameNode.FrameType.ToolTCP
            };
            robot.SetToolFrame(tool);

            SceneGraph.SetRobot(robot);
        }

        // ── Data Loading ────────────────────────────────────────────
        private async Task LoadInitialDataAsync()
        {
            // Check if this is the first run
            await SettingsVM.CheckFirstRunAsync();

            var programs = await _dbService.GetAllProgramsAsync();
            if (programs.Count > 0)
            {
                var latest = programs[0];
                System.Diagnostics.Debug.WriteLine($"Loaded DB Program: {latest.Name}");
            }
            else
            {
                // Load sample program into the new ProgramViewModel
                ProgramVM.LoadSampleProgram();

                // Also load legacy program for backward compat
                CurrentProgram.LoadSampleProgram();

                await _dbService.CreateNewProgramAsync("Default_Program_01");
            }
        }

        // ── Driver Callback ─────────────────────────────────────────
        private void OnDriverStateUpdated(RobotState updatedState)
        {
            CurrentState.J1 = updatedState.J1;
            CurrentState.J2 = updatedState.J2;
            CurrentState.J3 = updatedState.J3;
            CurrentState.J4 = updatedState.J4;
            CurrentState.J5 = updatedState.J5;
            CurrentState.J6 = updatedState.J6;

            // Sync to scene graph
            RobotVM.ApplyJointAngles(new[]
            {
                updatedState.J1, updatedState.J2, updatedState.J3,
                updatedState.J4, updatedState.J5, updatedState.J6
            });
        }

        // ── Execution Commands ──────────────────────────────────────
        [RelayCommand]
        private async Task StartProgram()
        {
            await _engine.RunProgramAsync(CurrentProgram, CurrentState);
        }

        [RelayCommand]
        private void StopProgram()
        {
            _engine.Stop();
        }

        // ── Window Commands ─────────────────────────────────────────
        [RelayCommand]
        private void CloseWindow() => Application.Current.Shutdown();

        [RelayCommand]
        private void MinimizeWindow()
        {
            if (Application.Current.MainWindow != null)
                Application.Current.MainWindow.WindowState = WindowState.Minimized;
        }

        [RelayCommand]
        private void MaximizeWindow()
        {
            if (Application.Current.MainWindow != null)
            {
                var w = Application.Current.MainWindow;
                w.WindowState = w.WindowState == WindowState.Maximized
                    ? WindowState.Normal
                    : WindowState.Maximized;
            }
        }

        // ── Work Area Toggle ────────────────────────────────────────
        [RelayCommand]
        private void ToggleWorkAreaConfig()
        {
            _isWorkAreaExpanded = !_isWorkAreaExpanded;
            if (_isWorkAreaExpanded)
            {
                LeftNavWidth    = new GridLength(60, GridUnitType.Pixel);
                CmdPaletteWidth = new GridLength(0, GridUnitType.Pixel);
                ProgramWidth    = new GridLength(0, GridUnitType.Pixel);
                ViewportWidth   = new GridLength(1, GridUnitType.Star);
            }
            else
            {
                LeftNavWidth    = new GridLength(220, GridUnitType.Pixel);
                CmdPaletteWidth = new GridLength(220, GridUnitType.Pixel);
                ProgramWidth    = new GridLength(350, GridUnitType.Pixel);
                ViewportWidth   = new GridLength(1, GridUnitType.Star);
            }

            WorldVM.IsWorkAreaConfigActive = _isWorkAreaExpanded;
        }

        // ── Navigation ───────────────────────────────────────────────
        [RelayCommand]
        private void SwitchNavPage(string page)
        {
            ActiveNavPage = page ?? "Teaching";
        }

        // ── New Program (Creates a fresh program file) ──────────────
        [RelayCommand]
        private void NewProgram()
        {
            // Clear legacy program
            CurrentProgram.Instructions.Clear();
            var programName = $"program_{DateTime.Now:yyyyMMdd_HHmmss}.lua";

            // Clear ProgramVM
            ProgramVM.Nodes.Clear();
            ProgramVM.ActiveLineNumber = 0;

            // Reset WorkTree to a fresh program structure
            var workTree = WorkTree;
            // Find and clear the program node, then add default Init
            foreach (var node in workTree.RootNodes)
            {
                if (node.NodeType == WorkTreeNodeType.Program)
                {
                    node.Children.Clear();
                    node.Name = $"Program: {programName}";
                    node.Children.Add(new WorkTreeNode("🏠", "Initialize()", WorkTreeNodeType.ProgramInstr));
                    break;
                }
            }

            // Clear points (user starts fresh)
            foreach (var node in workTree.RootNodes)
            {
                if (node.Name.Contains("Points"))
                {
                    node.Children.Clear();
                    break;
                }
            }
        }

        // ── AddInstruction (Delegates to both ProgramVM and WorkTree) ──
        [RelayCommand]
        private void AddInstruction(string type)
        {
            // Delegate to structured ProgramViewModel
            ProgramVM.AddNodeCommand.Execute(type);

            // Also add to WorkTree
            WorkTree.AddProgramInstructionCommand.Execute(type);

            // Also add to legacy collection for backward compat
            RobotInstruction? newInstr = type?.ToUpper() switch
            {
                "WAIT" => new WaitInstruction(1000),
                "LIN" => new LinInstruction("p1", 50, 0),
                "PTP" => new PtpInstruction("p1", 50),
                "ARC" or "CIRCLE" => new CircleInstruction("p1", "p2", 50),
                "WHILE" => new WhileInstruction("True"),
                "SETDO" => new SetDOInstruction(1, 1),
                "DOFILE" => new NewDoFileInstruction("file.lua"),
                _ => null
            };

            if (newInstr != null)
                CurrentProgram.Instructions.Add(newInstr);
        }

        // ── Legacy JogJoint ─────────────────────────────────────────
        [RelayCommand]
        private void JogJoint(string param)
        {
            RobotVM.JogJointCommand.Execute(param);

            // Also update legacy state
            if (string.IsNullOrEmpty(param) || param.Length < 2) return;
            bool isPositive = param[0] == '+';
            
            string numPart = param.Substring(1);
            if (numPart.StartsWith("J", StringComparison.OrdinalIgnoreCase))
                numPart = numPart.Substring(1);
            if (!int.TryParse(numPart, out int jointNum)) return;

            var positions = new[]
            {
                CurrentState.J1, CurrentState.J2, CurrentState.J3,
                CurrentState.J4, CurrentState.J5, CurrentState.J6
            };
            int idx = jointNum - 1;
            if (idx >= 0 && idx < 6)
            {
                positions[idx] += isPositive ? 1.0 : -1.0;
                _driver.SendJointPositions(positions);
            }
        }

        // ── Save ────────────────────────────────────────────────────
        [RelayCommand]
        private async Task SaveProgramToDb()
        {
            var programs = await _dbService.GetAllProgramsAsync();
            if (programs.Count > 0)
            {
                var prog = programs[0];
                prog.Name = "Updated_" + DateTime.Now.ToString("HHmmss");
                await _dbService.SaveProgramAsync(prog);
            }
        }

        // ── Command Palette ────────────────────────────────────────
        [RelayCommand]
        private void ToggleCommandPalette()
        {
            IsCommandPaletteOpen = !IsCommandPaletteOpen;
            if (IsCommandPaletteOpen)
            {
                CommandPaletteQuery = string.Empty;
                RefreshPaletteResults();
            }
        }

        [RelayCommand]
        private void ExecuteAndClosePalette(CommandDefinition? cmd)
        {
            if (cmd?.Execute != null)
            {
                cmd.Execute();
                IsCommandPaletteOpen = false;
            }
        }

        partial void OnCommandPaletteQueryChanged(string value)
        {
            RefreshPaletteResults();
        }

        private void RefreshPaletteResults()
        {
            CommandPaletteResults.Clear();
            foreach (var cmd in CommandReg.Search(CommandPaletteQuery))
                CommandPaletteResults.Add(cmd);
        }

        // ── Command Registry Setup ──────────────────────────────────
        private void RegisterCoreCommands()
        {
            CommandReg.Register(new CommandDefinition
            {
                Id = "Program.Start", DisplayName = "Start Program", Category = "Program",
                IconGlyph = "▶", Description = "Run the active program",
                DefaultGesture = new KeyGesture(Key.F5),
                Execute = () => StartProgramCommand.Execute(null),
                CanExecute = () => !GlobalState.IsRunning
            });

            CommandReg.Register(new CommandDefinition
            {
                Id = "Program.Stop", DisplayName = "Stop Program", Category = "Program",
                IconGlyph = "⏹", Description = "Stop execution",
                DefaultGesture = new KeyGesture(Key.F5, ModifierKeys.Shift),
                Execute = () => StopProgramCommand.Execute(null),
                CanExecute = () => GlobalState.IsRunning
            });

            CommandReg.Register(new CommandDefinition
            {
                Id = "Program.Save", DisplayName = "Save Program", Category = "Program",
                IconGlyph = "💾", Description = "Save program to database",
                DefaultGesture = new KeyGesture(Key.S, ModifierKeys.Control),
                Execute = () => SaveProgramToDbCommand.Execute(null)
            });

            CommandReg.Register(new CommandDefinition
            {
                Id = "Program.New", DisplayName = "New Program", Category = "Program",
                IconGlyph = "📄", Description = "Create a new empty program",
                DefaultGesture = new KeyGesture(Key.N, ModifierKeys.Control),
                Execute = () => NewProgramCommand.Execute(null)
            });

            CommandReg.Register(new CommandDefinition
            {
                Id = "Program.AddPoint", DisplayName = "Add Point", Category = "Program",
                IconGlyph = "📍", Description = "Teach a new target point",
                DefaultGesture = new KeyGesture(Key.P, ModifierKeys.Control),
                Execute = () => WorkTree.AddPointCommand.Execute(null)
            });

            CommandReg.Register(new CommandDefinition
            {
                Id = "Robot.GoHome", DisplayName = "Go Home", Category = "Robot",
                IconGlyph = "🏠", Description = "Move all joints to zero",
                DefaultGesture = new KeyGesture(Key.H, ModifierKeys.Control),
                Execute = () => RobotVM.GoHomeCommand.Execute(null)
            });

            CommandReg.Register(new CommandDefinition
            {
                Id = "Robot.ToggleMode", DisplayName = "Toggle Real/Sim Mode", Category = "Robot",
                IconGlyph = "🔄", Description = "Switch between simulator and real robot",
                DefaultGesture = new KeyGesture(Key.R, ModifierKeys.Control),
                Execute = () => GlobalState.ToggleModeCommand.Execute(null)
            });

            CommandReg.Register(new CommandDefinition
            {
                Id = "System.Settings", DisplayName = "Open Settings", Category = "System",
                IconGlyph = "⚙", Description = "Open application settings",
                Execute = () => { ActiveNavPage = "Settings"; }
            });

            CommandReg.Register(new CommandDefinition
            {
                Id = "Robot.LoadURDF", DisplayName = "Load Robot URDF", Category = "Robot",
                IconGlyph = "📂", Description = "Load a URDF robot model file",
                Execute = () =>
                {
                    var dialog = new Microsoft.Win32.OpenFileDialog
                    {
                        Filter = "URDF Files (*.urdf)|*.urdf|All Files (*.*)|*.*",
                        Title = "Select Robot URDF Model"
                    };
                    if (dialog.ShowDialog() == true)
                    {
                        // Store path for reload in Settings
                        SettingsVM.UrdfPath = dialog.FileName;
                    }
                }
            });

            CommandReg.Register(new CommandDefinition
            {
                Id = "System.CommandPalette", DisplayName = "Command Palette", Category = "System",
                IconGlyph = "⌘", Description = "Open the command palette",
                DefaultGesture = new KeyGesture(Key.P, ModifierKeys.Control | ModifierKeys.Shift),
                Execute = () => ToggleCommandPalette()
            });

            // Add motion commands
            foreach (var (type, name, icon) in new[]
            {
                ("PTP", "Add PTP Motion", "⤵"),
                ("LIN", "Add Linear Motion", "📏"),
                ("ARC", "Add Arc Motion", "◜"),
                ("WAIT", "Add Wait", "⏱"),
                ("SETDO", "Add Digital Output", "🔌"),
                ("WHILE", "Add Loop", "🔃"),
            })
            {
                var t = type;
                CommandReg.Register(new CommandDefinition
                {
                    Id = $"Program.Add{t}", DisplayName = name, Category = "Program",
                    IconGlyph = icon, Execute = () => AddInstructionCommand.Execute(t)
                });
            }
        }
    }
}
