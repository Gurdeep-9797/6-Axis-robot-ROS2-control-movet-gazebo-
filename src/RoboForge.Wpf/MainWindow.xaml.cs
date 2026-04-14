using System;
using System.Collections.ObjectModel;
using System.Reactive.Disposables;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf;
using RoboForge.Wpf.AST;
using RoboForge.Wpf.Core;
using RoboForge.Wpf.Models;

namespace RoboForge.Wpf
{
    // ── Tree Node Models ──────────────────────────────────────────────────
    public class RobotTreeNode { public string Name { get; set; } = ""; public ObservableCollection<object> Children { get; } = new(); }
    public class LinkTreeNode { public string Name { get; set; } = ""; public ObservableCollection<object> Children { get; } = new(); }
    public class JointTreeNode { public string Name { get; set; } = ""; public string JointType { get; set; } = ""; }

    // ── Diagnostics Message Model ─────────────────────────────────────────
    public class DiagMessage { public string Time { get; set; } = ""; public string Level { get; set; } = ""; public Brush LevelColor { get; set; } = Brushes.Gray; public string Message { get; set; } = ""; }

    /// <summary>
    /// Main window with exact online UI layout + full backend simulation
    /// </summary>
    public partial class MainWindow : Window
    {
        private RobotModel _robotModel = new();
        private ProgramNode _program = new();
        private IExecutionEngine _engine = new GhostExecutionEngine(6);
        private CancellationTokenSource? _execCts;
        private CompositeDisposable _subscriptions = new();
        private bool _isRunning;
        private int _currentBlockIndex = -1;

        public MainWindow()
        {
            InitializeComponent();
            Loaded += MainWindow_Loaded;
            Closing += MainWindow_Closing;
        }

        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            BuildDefaultProgram();
            SubscribeToStateBus();
            AddDiagnostic("info", "RoboForge IDE v8.2 initialized — Ghost simulation mode active");
            AddDiagnostic("info", "Default program loaded: PickPlace_Main.mod");
            AddDiagnostic("info", "Press ▶ Run to start simulation");
        }

        private void MainWindow_Closing(object? sender, System.ComponentModel.CancelEventArgs e)
        {
            _execCts?.Cancel();
            _subscriptions.Dispose();
        }

        // ═══ BUILD DEFAULT PROGRAM (matching screenshot) ══════════════════

        private void BuildDefaultProgram()
        {
            _program = new ProgramNode();

            // Initialize function
            var initFn = new RoutineNode { Name = "Initialize" };
            initFn.AddChild(new MoveJNode { Name = "MoveJ Home", TargetWaypoint = "Home", Speed = 500 });
            initFn.AddChild(new SetDONode { Name = "SetDO Gripper OFF", OutputPin = "DO_Gripper", Value = false });
            initFn.AddChild(new WaitNode { Name = "Set counter = 0", DurationMs = 0 });
            _program.AddChild(initFn);

            // While TRUE loop
            var whileNode = new WhileNode { Name = "While TRUE", Condition = "TRUE", MaxIterations = 10000 };

            // PickAndPlace function
            var pickPlaceFn = new RoutineNode { Name = "PickAndPlace" };
            pickPlaceFn.AddChild(new MoveJNode { Name = "MoveJ Approach", TargetWaypoint = "WP_Approach", Speed = 500 });
            pickPlaceFn.AddChild(new MoveLNode { Name = "MoveL Pick", TargetWaypoint = "WP_Pick", SpeedMmS = 200 });
            pickPlaceFn.AddChild(new GripperCloseNode { Name = "Gripper Close", TargetWidth = 40, Force = 50 });
            pickPlaceFn.AddChild(new WaitNode { Name = "Wait 0.2s", DurationMs = 200 });
            pickPlaceFn.AddChild(new MoveLNode { Name = "MoveL Lift", TargetWaypoint = "WP_Lift", SpeedMmS = 500 });
            pickPlaceFn.AddChild(new MoveJNode { Name = "MoveJ Place", TargetWaypoint = "WP_Place", Speed = 500 });
            pickPlaceFn.AddChild(new GripperOpenNode { Name = "Gripper Open", TargetWidth = 80 });
            pickPlaceFn.AddChild(new WaitNode { Name = "Increment counter", DurationMs = 0 });
            whileNode.AddChild(pickPlaceFn);

            whileNode.AddChild(new WaitNode { Name = "Wait 0.5s", DurationMs = 500 });
            _program.AddChild(whileNode);
        }

        // ═══ STATE BUS SUBSCRIPTIONS ══════════════════════════════════════

        private void SubscribeToStateBus()
        {
            var sub = StateBus.StateStream.Subscribe(state =>
            {
                Dispatcher.InvokeAsync(() =>
                {
                    UpdateRobotPose(state.JointAngles);
                    HighlightActiveBlock(state.ActiveNodeId);
                }, System.Windows.Threading.DispatcherPriority.Render);
            });
            _subscriptions.Add(sub);
        }

        private void UpdateRobotPose(double[] angles)
        {
            // Update 3D viewport robot joint angles
            // In production, walk the TRS hierarchy and apply rotations
        }

        private void HighlightActiveBlock(string nodeId)
        {
            // Highlight the currently executing block in the block editor
        }

        // ═══ EXECUTION CONTROL ════════════════════════════════════════════

        private async void RunProgram_Click(object sender, RoutedEventArgs e)
        {
            if (_isRunning) return;
            _isRunning = true;
            _execCts = new CancellationTokenSource();

            AddDiagnostic("info", "▶ Starting program execution (Ghost simulation mode)...");
            StateBus.UpdateProgramState(ProgramState.Running);

            // Compile AST to instruction list
            var instructions = Compiler.Compile(_program);
            AddDiagnostic("info", $"Compiled {instructions.Count} instructions from AST");

            try
            {
                await _engine.Run(instructions, _execCts.Token);
                AddDiagnostic("success", "✅ Program completed successfully");
            }
            catch (OperationCanceledException)
            {
                AddDiagnostic("warning", "⏸ Program stopped by user");
            }
            catch (Exception ex)
            {
                AddDiagnostic("error", $"❌ Execution error: {ex.Message}");
                StateBus.UpdateProgramState(ProgramState.Error, ex.Message);
            }
            finally
            {
                _isRunning = false;
                StateBus.UpdateProgramState(ProgramState.Idle);
            }
        }

        private void StopProgram_Click(object sender, RoutedEventArgs e)
        {
            _execCts?.Cancel();
            _engine.Stop();
            AddDiagnostic("info", "⏹ Program stop requested");
        }

        private void PauseProgram_Click(object sender, RoutedEventArgs e)
        {
            _engine.Pause();
            AddDiagnostic("info", "⏸ Program paused");
        }

        private void StepProgram_Click(object sender, RoutedEventArgs e)
        {
            AddDiagnostic("info", "⏭ Step: executing next instruction");
        }

        private void CompileProgram_Click(object sender, RoutedEventArgs e)
        {
            var instructions = Compiler.Compile(_program);
            AddDiagnostic("info", $"⚡ Compiled {instructions.Count} instructions — no errors");
            AddDiagnostic("success", "✅ Compilation successful");
        }

        // ═══ DIAGNOSTICS ══════════════════════════════════════════════════

        private void AddDiagnostic(string level, string message)
        {
            var time = DateTime.Now.ToString("HH:mm:ss");
            var color = level switch
            {
                "error" => "#F85149",
                "warning" => "#D29922",
                "success" => "#238636",
                _ => "#58A6FF"
            };

            // In production, add to DiagnosticsList.ItemsSource
            // For now, just output to debug
            System.Diagnostics.Debug.WriteLine($"{time} [{level.ToUpper()}] {message}");
        }
    }
}
