using System;
using System.Collections.ObjectModel;
using System.Windows;
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using TeachPendant_WPF.Models;
using TeachPendant_WPF.Services;
using RoboForge_WPF.Services;

namespace RoboForge_WPF.ViewModels
{
    public partial class MainViewModel : ObservableObject
    {
        // ── Observable State ────────────────────────────────────────
        [ObservableProperty] private RobotState _currentState;
        [ObservableProperty] private RobotProgram _currentProgram;
        [ObservableProperty] private string _statusText = "Disconnected";
        [ObservableProperty] private bool _isRunning;
        [ObservableProperty] private bool _isConnected;

        // ── Sub-ViewModels ──────────────────────────────────────────
        public JogViewModel JogVM { get; } = new JogViewModel();
        public PipelineViewModel PipelineVM { get; } = new PipelineViewModel();

        // ── Collections ─────────────────────────────────────────────
        public ObservableCollection<LogEntry> ConsoleLogs => LoggingService.Instance.Logs;
        public ObservableCollection<string> Waypoints { get; } = new ObservableCollection<string>();

        // ── Services ────────────────────────────────────────────────
        private ExecutionEngine? _engine;
        private IRobotDriver? _driver;
        private int _msgSentCount;
        private int _msgRecvCount;

        // ── Constructors ────────────────────────────────────────────

        /// <summary>Design-time constructor</summary>
        public MainViewModel()
        {
            _currentState = new RobotState();
            _currentProgram = new RobotProgram();
        }

        /// <summary>Runtime constructor with full DI</summary>
        public MainViewModel(RobotState state, RobotProgram program, IRobotDriver driver, ExecutionEngine engine)
        {
            _currentState = state;
            _currentProgram = program;
            _driver = driver;
            _engine = engine;
            _isConnected = driver.IsConnected;

            // Wire execution state changes
            _engine.ExecutionStateChanged += (running) =>
            {
                IsRunning = running;
                StatusText = running ? "Running Program..." : (IsConnected ? "Connected" : "Disconnected");
                if (running) Log("Program execution started.", "Info");
                else Log("Program stopped.", "Info");
            };

            _engine.ErrorOccurred += (msg) =>
            {
                Log($"Execution error: {msg}", "Error");
            };

            Waypoints.Add("WP_Home");
            Waypoints.Add("WP_Approach");
            Waypoints.Add("WP_Pick");
            Waypoints.Add("WP_Place");
        }

        // ── Commands ────────────────────────────────────────────────

        [RelayCommand]
        private void NewProgram()
        {
            CurrentProgram.Instructions.Clear();
            Waypoints.Clear();
            Log("Created new program.", "Info");
        }

        [RelayCommand]
        private void SaveProgram()
        {
            Log("Program saved successfully.", "Success");
        }

        [RelayCommand]
        public async Task RunProgramAsync()
        {
            if (_engine == null || _driver == null)
            {
                Log("Engine or driver not initialized.", "Error");
                return;
            }

            Log("Compiling Visual Node Graph...", "Info");

            try
            {
                // 1. Generate textual DSL script from Visual Blocks
                string script = DSL.ScriptGenerator.Generate(CurrentProgram.Instructions);

                // 2. Lexer -> Tokens
                var lexer = new DSL.Lexer(script);
                var tokens = lexer.Tokenize();

                // 3. Parser -> AST
                var parser = new DSL.Parser(tokens);
                var ast = parser.Parse();

                // 4. IR Compiler -> Intermediate Representation
                var compiler = new DSL.IRCompiler();
                var compiledInstructions = compiler.Compile(ast);

                if (compiledInstructions.Count == 0)
                {
                    Log("Program is empty or failed to parse.", "Warning");
                    return;
                }

                Log($"Compiled {compiledInstructions.Count} instructions. Dispatching to ROS pipeline...", "Success");

                _msgSentCount += compiledInstructions.Count;
                PipelineVM.MessagesSent = _msgSentCount;
                PipelineVM.UiMsg = $"[{DateTime.Now:HH:mm:ss}] Run {compiledInstructions.Count} blocks";
                PipelineVM.EngineMsg = $"Executing {compiledInstructions.Count} instructions...";
                PipelineVM.GazeboMsg = "Receiving trajectories...";

                // Build execution program
                var executionProgram = new RobotProgram();
                foreach (var inst in compiledInstructions)
                {
                    executionProgram.Instructions.Add(inst);
                }

                await _engine.RunProgramAsync(executionProgram, CurrentState);

                PipelineVM.EngineMsg = "Execution complete";
                PipelineVM.GazeboMsg = "Target reached";
            }
            catch (Exception ex)
            {
                Log($"Compiler error: {ex.Message}", "Error");
            }
        }

        [RelayCommand]
        private void PauseProgram()
        {
            _engine?.Pause();
            Log("Program execution paused.", "Warning");
        }

        [RelayCommand]
        private void StopProgram()
        {
            _engine?.Stop();
            Log("Program execution stopped forcibly.", "Error");
        }

        [RelayCommand]
        private void AddWaypoint()
        {
            Waypoints.Add($"WP_{Waypoints.Count + 1}");
        }

        [RelayCommand]
        private void DeleteWaypoint(int index)
        {
            if (index >= 0 && index < Waypoints.Count)
            {
                Waypoints.RemoveAt(index);
            }
        }

        // ── Telemetry Update (called by driver callback) ────────────

        public void UpdateTelemetry(RobotState updatedState)
        {
            _msgRecvCount++;

            CurrentState.J1 = updatedState.J1;
            CurrentState.J2 = updatedState.J2;
            CurrentState.J3 = updatedState.J3;
            CurrentState.J4 = updatedState.J4;
            CurrentState.J5 = updatedState.J5;
            CurrentState.J6 = updatedState.J6;
            CurrentState.X = updatedState.X;
            CurrentState.Y = updatedState.Y;
            CurrentState.Z = updatedState.Z;

            // Update pipeline telemetry strings
            PipelineVM.TelJ1 = $"J1: {CurrentState.J1:F2}°";
            PipelineVM.TelJ2 = $"J2: {CurrentState.J2:F2}°";
            PipelineVM.TelJ3 = $"J3: {CurrentState.J3:F2}°";
            PipelineVM.TelJ4 = $"J4: {CurrentState.J4:F2}°";
            PipelineVM.TelJ5 = $"J5: {CurrentState.J5:F2}°";
            PipelineVM.TelJ6 = $"J6: {CurrentState.J6:F2}°";
            PipelineVM.JointMsg = $"[{DateTime.Now:HH:mm:ss}] J1={CurrentState.J1:F1} J2={CurrentState.J2:F1}";
            PipelineVM.BridgeMsg = _driver?.IsConnected == true ? "Connected ✓" : "Disconnected";
            PipelineVM.DriverMsg = _driver?.IsConnected == true ? "ws://localhost:9090 ✓" : "Not connected";
            PipelineVM.MessagesSent = _msgSentCount;
            PipelineVM.MessagesReceived = _msgRecvCount;
            PipelineVM.PipelineStatus = CurrentState.IsRunning ? "  ● RUNNING" : "  ● IDLE";
        }

        // ── Jog Dispatch ────────────────────────────────────────────

        public async Task SendJogPositionAsync()
        {
            if (_driver == null) return;

            double[] pos = new double[]
            {
                JogVM.Joints[0].Value,
                JogVM.Joints[1].Value,
                JogVM.Joints[2].Value,
                JogVM.Joints[3].Value,
                JogVM.Joints[4].Value,
                JogVM.Joints[5].Value
            };

            _msgSentCount++;
            PipelineVM.MessagesSent = _msgSentCount;
            await _driver.SendJointPositions(pos);
        }

        // ── Helpers ─────────────────────────────────────────────────

        public void Log(string msg, string severity)
        {
            LoggingService.Instance.Log(msg, severity);
        }

        public string GenerateScript()
        {
            return DSL.ScriptGenerator.Generate(CurrentProgram.Instructions);
        }
    }
}
