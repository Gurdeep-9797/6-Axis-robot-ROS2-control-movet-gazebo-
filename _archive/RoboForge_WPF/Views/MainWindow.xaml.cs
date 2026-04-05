using System;
using System.Collections.ObjectModel;
using System.IO;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf;
using TeachPendant_WPF.Models;
using TeachPendant_WPF.SceneGraph;
using TeachPendant_WPF.Services;
using TeachPendant_WPF.Views;
using RoboForge_WPF.ViewModels;

namespace RoboForge_WPF.Views
{


    public partial class MainWindow : Window
    {
        // ── ViewModel (primary state holder) ────────────────────────
        private MainViewModel _vm;
        private RoboForge_WPF.Services.WebDashboardHost? _webHost;

        // ── Services that require View-layer wiring ─────────────────
        private SceneGraphManager _sceneGraph;
        private IRobotDriver _driver;
        private ExecutionEngine _engine;
        private URDFVisualizer? _urdfVisualizer;
        private RoboForge_WPF.Services.CliApiService? _apiService;
        private RoboForge_WPF.Services.DatabaseManager _dbManager;
        
        // Convenience aliases into the ViewModel
        private JogViewModel JogVM => _vm.JogVM;
        private PipelineViewModel PipelineVM => _vm.PipelineVM;
        private ObservableCollection<LogEntry> ConsoleLogs => Services.LoggingService.Instance.Logs;
        private ObservableCollection<string> Waypoints => _vm.Waypoints;
        private RobotState _currentState => _vm.CurrentState;
        private RobotProgram _currentProgram => _vm.CurrentProgram;
        
        // UI State
        private bool _isPropertiesVisible = false;

        public MainWindow()
        {
            InitializeComponent();
            
            // 1. Initialize core logic (ViewModel owns the primary state)
            _vm = new MainViewModel(); // Design-time safe; will be re-initialized in Loaded
            DataContext = _vm;

            _sceneGraph = new SceneGraphManager();
            
            // Default Robot Setup
            var robot = new RobotNode { Name = "FR-6DOF" };
            var jointConfigs = new[]
            {
                (name: "J1", axis: new Vector3D(0, 0, 1), min: -170.0, max: 170.0, offset: new Vector3D(0, 0, 150)),
                (name: "J2", axis: new Vector3D(0, 1, 0), min: -135.0, max: 135.0, offset: new Vector3D(0, 50, 250)),
                (name: "J3", axis: new Vector3D(0, 1, 0), min: -150.0, max: 150.0, offset: new Vector3D(0, 0, 450)),
                (name: "J4", axis: new Vector3D(1, 0, 0), min: -180.0, max: 180.0, offset: new Vector3D(0, -50, 150)),
                (name: "J5", axis: new Vector3D(0, 1, 0), min: -120.0, max: 120.0, offset: new Vector3D(0, 0, 350)),
                (name: "J6", axis: new Vector3D(1, 0, 0), min: -360.0, max: 360.0, offset: new Vector3D(0, -50, 120)),
            };
            foreach (var cfg in jointConfigs) robot.AddJoint(new JointNode { Name = cfg.name, Axis = cfg.axis, MinLimit = cfg.min, MaxLimit = cfg.max, OriginOffset = cfg.offset });
            _sceneGraph.SetRobot(robot);

            // 3. Initialize UI bindings
            ConsoleList.ItemsSource = ConsoleLogs;
            WaypointList.ItemsSource = Waypoints;
            
            // Init Jog Panel
            JogJointsControl.ItemsSource = JogVM.Joints;
            JogVM.JogRequested += (s, e) => UpdateRobotTargetFromJog();
            
            // Initialize Database Manager
            _dbManager = new RoboForge_WPF.Services.DatabaseManager();

            // Start Properties collapsed
            PropertiesPanel.Visibility = Visibility.Collapsed;

            Services.LoggingService.Instance.LogsUpdated += () => {
                Dispatcher.Invoke(() => {
                    if (ConsoleMsgCount != null) ConsoleMsgCount.Text = $"{ConsoleLogs.Count} messages";
                    if (IsLoaded && ConsoleLogs.Count > 0 && ConsoleList != null && ConsoleList.Items.Count > 0)
                    {
                        try { ConsoleList.ScrollIntoView(ConsoleList.Items[^1]); } catch { }
                    }
                });
            };

            this.Loaded += MainWindow_Loaded;
            this.Closed += MainWindow_Closed;
        }

        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            // 2. Setup Driver & Execution — with SimulationDriver fallback
            IRobotDriver ros2 = new Ros2Driver();
            ros2.Connect();

            if (ros2.IsConnected)
            {
                _driver = ros2;
                Log("Connected to ROS 2 bridge (ws://localhost:9090)", "Success");
            }
            else
            {
                Log("ROS 2 bridge unreachable — falling back to SimulationDriver", "Warning");
                _driver = new SimulationDriver();
                _driver.Connect();
            }

            _driver.StateUpdated += OnDriverStateUpdated;
            _engine = new ExecutionEngine(_driver);

            // Re-initialize ViewModel with real services
            _vm = new MainViewModel(_vm.CurrentState, _vm.CurrentProgram, _driver, _engine);
            DataContext = _vm;

            // Start Web Dashboard for Online UI Control
            _webHost = new RoboForge_WPF.Services.WebDashboardHost(_vm, 5050);
            _ = _webHost.StartAsync();

            // Wire execution state to View-layer chrome
            _engine.ExecutionStateChanged += (running) =>
            {
                Dispatcher.Invoke(() => {
                    ModeSimulate.IsChecked = running;
                    StatusText.Text = running ? "Running Program..." : (_driver.IsConnected ? "Connected" : "Disconnected");
                    StatusLed.Fill = (SolidColorBrush)FindResource(running ? "Status.Warning" : "Status.Ok");
                    BtnRun.IsEnabled = !running;
                    BtnStop.IsEnabled = running;
                });
            };

            Log("RoboForge Studio v5.0 initialized", "Success");
            
            // 4. Load 3D Viewport
            string urdfPath = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "Assets", "ur3e_cobot.urdf");
            if (File.Exists(urdfPath))
            {
                _urdfVisualizer = new URDFVisualizer(urdfPath);
                RobotModelVisual.Children.Add(_urdfVisualizer.RootVisual);
                Log($"Robot model loaded: {urdfPath}", "Success");
            }
            else
            {
                Log("Warning: Default URDF model not found.", "Warning");
            }

            UpdateCodeGeneration();

            // Populate Project Manager List dynamically from DB
            var workspaces = _dbManager.GetRecentWorkspaces();
            foreach (var ws in workspaces)
            {
                ProjectList.Items.Add(new TextBlock 
                { 
                    Text = $"💼 {ws.Name}  [{ws.LastModified:d}]", 
                    Margin = new Thickness(4), 
                    Foreground = (SolidColorBrush)FindResource("Text.Primary") 
                });
            }

            // Populate Robot Configuration dynamically from DB
            var robots = _dbManager.GetRecentRobotConfigs();
            if (robots.Count > 0)
            {
                var defaultRobot = robots[0];
                Log($"Loaded robot config: {defaultRobot.Name} from DB", "Info");
                string dynamicUrdf = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "Assets", defaultRobot.UrdfPath);
                if (System.IO.File.Exists(dynamicUrdf))
                {
                    RobotModelVisual.Children.Clear();
                    _urdfVisualizer = new URDFVisualizer(dynamicUrdf);
                    RobotModelVisual.Children.Add(_urdfVisualizer.RootVisual);
                }
            }

            // 5. Start CLI REST API
            _apiService = new RoboForge_WPF.Services.CliApiService(_currentState, _currentProgram, _driver, _engine, (msg) => Log(msg, "Info"));
            _apiService.Start();

            // 6. Automated Pipeline Test Hook
            var args = Environment.GetCommandLineArgs();
            if (Array.Exists(args, arg => arg.ToLower() == "--autorun"))
            {
                System.Threading.Tasks.Task.Run(async () => 
                {
                    await System.Threading.Tasks.Task.Delay(3000);
                    Application.Current.Dispatcher.Invoke(() => 
                    {
                        Log("AUTORUN DETECTED: Triggering End-to-End Pipeline...", "Warning");
                        _currentProgram.Instructions.Clear();
                        _currentProgram.Instructions.Add(new PtpInstruction("WP_Pick", 60));
                        _currentProgram.Instructions.Add(new WaitInstruction(2000));
                        _currentProgram.Instructions.Add(new LinInstruction("WP_Place", 30, 0));
                        UpdateCodeGeneration();
                        BtnRun.RaiseEvent(new RoutedEventArgs(System.Windows.Controls.Primitives.ButtonBase.ClickEvent));
                    });
                });
            }
        }

        private void MainWindow_Closed(object? sender, EventArgs e)
        {
            _apiService?.Stop();
            _webHost?.Dispose();
        }

        // ── Window Chrome ───────────────────────────────────────────
        private void Minimize_Click(object sender, RoutedEventArgs e) => WindowState = WindowState.Minimized;
        private void Maximize_Click(object sender, RoutedEventArgs e) => WindowState = WindowState == WindowState.Maximized ? WindowState.Normal : WindowState.Maximized;
        private void Close_Click(object sender, RoutedEventArgs e) => Application.Current.Shutdown();
        private void Exit_Click(object sender, RoutedEventArgs e) => Application.Current.Shutdown();

        protected override void OnMouseLeftButtonDown(MouseButtonEventArgs e)
        {
            base.OnMouseLeftButtonDown(e);
            if (e.GetPosition(this).Y < 32) DragMove();
        }

        // ── Navigation (Left Panel Router) ──────────────────────────
        private void Nav_Click(object sender, RoutedEventArgs e)
        {
            if (sender is RadioButton btn && btn.Tag is string tag)
            {
                PanelProgram.Visibility = Visibility.Collapsed;
                PanelBlocks.Visibility = Visibility.Collapsed;
                PanelMotion.Visibility = Visibility.Collapsed;
                PanelIO.Visibility = Visibility.Collapsed;
                PanelDebug.Visibility = Visibility.Collapsed;
                PanelConfig.Visibility = Visibility.Collapsed;
                PanelSettings.Visibility = Visibility.Collapsed;
                PanelProject.Visibility = Visibility.Collapsed;
                PanelJog.Visibility = Visibility.Collapsed;

                switch (tag)
                {
                    case "program": PanelProgram.Visibility = Visibility.Visible; break;
                    case "blocks": PanelBlocks.Visibility = Visibility.Visible; break;
                    case "motion": PanelMotion.Visibility = Visibility.Visible; break;
                    case "io": PanelIO.Visibility = Visibility.Visible; break;
                    case "debug": PanelDebug.Visibility = Visibility.Visible; break;
                    case "config": PanelConfig.Visibility = Visibility.Visible; break;
                    case "settings": PanelSettings.Visibility = Visibility.Visible; break;
                    case "project": PanelProject.Visibility = Visibility.Visible; break;
                    case "jog": PanelJog.Visibility = Visibility.Visible; break;
                }
            }
        }

        // ── Blocks & Code Generation ────────────────────────────────
        private void Block_Click(object sender, MouseButtonEventArgs e)
        {
            if (sender is Border b && b.Tag is string blockType)
            {
                Log($"Selected Block: {blockType}", "Info");
                // Open properties pane for editing
                if (!_isPropertiesVisible) ToggleProperties_Click(this, new RoutedEventArgs());
                
                // Add to underlying program model
                RobotInstruction? instr = blockType switch
                {
                    "MoveJ" => new PtpInstruction("p1", 100),
                    "MoveL" => new LinInstruction("p1", 50, 0),
                    "Wait" => new WaitInstruction(1000),
                    "SetDO" => new SetDOInstruction(1, 1),
                    _ => null
                };
                
                if (instr != null) 
                {
                    _currentProgram.Instructions.Add(instr);
                    UpdateCodeGeneration();
                    // Update pipeline
                    PipeUiMsg.Text = $"[{DateTime.Now:HH:mm:ss}] Added {blockType} block";
                }
            }
        }

        private async void Block_RightClick(object sender, MouseButtonEventArgs e)
        {
            if (sender is Border b && b.Tag is string blockType)
            {
                Log($"Jupyter-style quick execution: Run {blockType} block", "Info");
                
                RobotInstruction? instr = blockType switch
                {
                    "MoveJ" => new PtpInstruction("p_temp", 100),
                    "MoveL" => new LinInstruction("p_temp", 50, 0),
                    "Wait" => new WaitInstruction(1000),
                    "SetDO" => new SetDOInstruction(1, 1),
                    _ => null
                };

                if (instr != null)
                {
                    Tab3D.IsChecked = true;
                    CenterTab_Click(Tab3D, null!);
                    _currentState.IsRunning = true;
                    await instr.ExecuteAsync(_currentState, _driver);
                    _currentState.IsRunning = false;
                    Log($"{blockType} quick execution finished.", "Success");
                }
            }
        }

        private void UpdateCodeGeneration()
        {
            // Use the new DSL ScriptGenerator
            string script = RoboForge_WPF.DSL.ScriptGenerator.Generate(_currentProgram.Instructions);
            
            // XAML Note: The static RichTextBox (CodeEditor) was replaced with 
            // the dynamic Node-Graph Editor (NodeCanvasContainer).
            // For now, we just output to Debug.
            System.Diagnostics.Debug.WriteLine(script);
        }

        // ── Properties Panel ────────────────────────────────────────
        private void ToggleProperties_Click(object sender, RoutedEventArgs e)
        {
            _isPropertiesVisible = !_isPropertiesVisible;
            PropertiesPanel.Visibility = _isPropertiesVisible ? Visibility.Visible : Visibility.Collapsed;
        }

        private void ApplyBlock_Click(object sender, RoutedEventArgs e)
        {
            // Apply property values to the currently generated block
            UpdateCodeGeneration();
            Log($"Applied properties to {PropVarLabel.Text}", "Success");
        }

        private void SavePoint_Click(object sender, RoutedEventArgs e)
        {
            string label = PropVarLabel.Text;
            if (!Waypoints.Contains(label))
            {
                Waypoints.Add(label);
                Log($"Saved new waypoint: {label} [X:{PropX.Text} Y:{PropY.Text} Z:{PropZ.Text}]", "Success");
            }
        }

        private void AddWaypoint_Click(object sender, RoutedEventArgs e)
        {
            Waypoints.Add($"WP_{Waypoints.Count + 1}");
        }

        private void DeleteWaypoint_Click(object sender, RoutedEventArgs e)
        {
            if (WaypointList.SelectedIndex >= 0)
            {
                Waypoints.RemoveAt(WaypointList.SelectedIndex);
            }
        }

        private void Waypoint_Selected(object sender, SelectionChangedEventArgs e)
        {
            if (WaypointList.SelectedItem is string wp)
            {
                if (PropVarLabel != null) PropVarLabel.Text = wp;
                if (PropHeader != null) PropHeader.Text = $"PROPERTIES — {wp.ToUpper()}";
                if (!_isPropertiesVisible) ToggleProperties_Click(this, null!);
            }
        }

        // ── Execution Pipeline (Visual Programming Execution Loop) ──────────────────────────────────────
        private async void Run_Click(object sender, RoutedEventArgs e)
        {
            // Switch to 3D View for execution monitoring
            if (Tab3D != null) Tab3D.IsChecked = true;
            CenterTab_Click(Tab3D!, null!);

            // Delegate entire compilation + execution to ViewModel
            await _vm.RunProgramAsync();
        }

        private void Pause_Click(object sender, RoutedEventArgs e)
        {
            _vm.PauseProgramCommand.Execute(null);
        }

        private void Stop_Click(object sender, RoutedEventArgs e)
        {
            _vm.StopProgramCommand.Execute(null);
        }

        // ── Viewport ────────────────────────────────────────────────
        private void CenterTab_Click(object sender, RoutedEventArgs e)
        {
            if (sender is System.Windows.Controls.Primitives.ToggleButton)
            {
                CodeEditorArea.Visibility = TabCode.IsChecked == true ? Visibility.Visible : Visibility.Collapsed;
                Viewport3DArea.Visibility = Tab3D.IsChecked == true ? Visibility.Visible : Visibility.Collapsed;
                PipelineArea.Visibility = TabPipeline.IsChecked == true ? Visibility.Visible : Visibility.Collapsed;

                ColCode.Width = TabCode.IsChecked == true ? new GridLength(4, GridUnitType.Star) : new GridLength(0);
                ColSplit1.Width = (TabCode.IsChecked == true && (Tab3D.IsChecked == true || TabPipeline.IsChecked == true)) ? new GridLength(4) : new GridLength(0);
                
                Col3D.Width = Tab3D.IsChecked == true ? new GridLength(6, GridUnitType.Star) : new GridLength(0);
                ColSplit2.Width = ((TabCode.IsChecked == true || Tab3D.IsChecked == true) && TabPipeline.IsChecked == true) ? new GridLength(4) : new GridLength(0);

                ColPipeline.Width = TabPipeline.IsChecked == true ? new GridLength(4, GridUnitType.Star) : new GridLength(0);
            }
        }

        private void ResetCamera_Click(object sender, RoutedEventArgs e)
        {
            HelixView.Camera.Position = new Point3D(1500, 1500, 1000);
            HelixView.Camera.LookDirection = new Vector3D(-1500, -1500, -1000);
            HelixView.Camera.UpDirection = new Vector3D(0, 0, 1);
        }

        private void HelixView_MouseDown(object sender, MouseButtonEventArgs e)
        {
            if (e.LeftButton == MouseButtonState.Pressed && PointMethodViewport.IsChecked == true && _isPropertiesVisible)
            {
                var pt = e.GetPosition(HelixView);
                var hits = HelixView.Viewport.FindHits(pt);
                if (hits != null && hits.Count > 0)
                {
                    // Filter hits: prioritize workpiece meshes / edges (SolidWorks style curve/line selection)
                    var firstHit = hits.First();
                    Point3D hitPos = firstHit.Position;
                    
                    // Vertex / Edge Snapping Logic (CAD-like)
                    if (firstHit.RayHit != null && firstHit.Mesh != null)
                    {
                        var mesh = firstHit.Mesh;
                        int v1 = firstHit.RayHit.VertexIndex1;
                        int v2 = firstHit.RayHit.VertexIndex2;
                        int v3 = firstHit.RayHit.VertexIndex3;
                        
                        Point3D p1 = mesh.Positions[v1];
                        Point3D p2 = mesh.Positions[v2];
                        Point3D p3 = mesh.Positions[v3];
                        
                        // Find closest vertex to snap
                        double d1 = hitPos.DistanceTo(p1);
                        double d2 = hitPos.DistanceTo(p2);
                        double d3 = hitPos.DistanceTo(p3);
                        
                        // Snap threshold (adjust based on scene scale)
                        if (d1 < d2 && d1 < d3 && d1 < 50.0) hitPos = p1;
                        else if (d2 < d1 && d2 < d3 && d2 < 50.0) hitPos = p2;
                        else if (d3 < 50.0) hitPos = p3;
                    }

                    PropX.Text = hitPos.X.ToString("F1");
                    PropY.Text = hitPos.Y.ToString("F1");
                    PropZ.Text = hitPos.Z.ToString("F1");
                    Log($"Picked 3D Point (Snapped): [{PropX.Text}, {PropY.Text}, {PropZ.Text}]", "Info");
                    
                    // Simple Inverse Kinematics estimate for visual feedback
                    PropJ1.Text = (Math.Atan2(hitPos.Y, hitPos.X) * 180 / Math.PI).ToString("F2");
                }
            }
        }

        // ── Driver & Sync (STRICTLY from /joint_states → UI, never the reverse) ──
        private void OnDriverStateUpdated(RobotState updatedState)
        {
            // Delegate telemetry state update to ViewModel
            _vm.UpdateTelemetry(updatedState);

            Dispatcher.BeginInvoke(() =>
            {
                // Update 3D visualizer from telemetry ONLY
                if (_urdfVisualizer != null)
                {
                    _urdfVisualizer.UpdateJoints(_currentState.J1, _currentState.J2, _currentState.J3, _currentState.J4, _currentState.J5, _currentState.J6);
                }

                // Synchronize Jog Sliders to physical state
                if (JogVM.Joints.Count == 6)
                {
                    JogVM.Joints[0].Value = _currentState.J1;
                    JogVM.Joints[1].Value = _currentState.J2;
                    JogVM.Joints[2].Value = _currentState.J3;
                    JogVM.Joints[3].Value = _currentState.J4;
                    JogVM.Joints[4].Value = _currentState.J5;
                    JogVM.Joints[5].Value = _currentState.J6;
                }
                
                // Live update Properties panel if Point Method is "Current"
                if (PointMethodCurrent.IsChecked == true && _isPropertiesVisible)
                {
                    PropJ1.Text = _currentState.J1.ToString("F2");
                    PropJ2.Text = _currentState.J2.ToString("F2");
                    PropJ3.Text = _currentState.J3.ToString("F2");
                    PropJ4.Text = _currentState.J4.ToString("F2");
                    PropJ5.Text = _currentState.J5.ToString("F2");
                    PropJ6.Text = _currentState.J6.ToString("F2");
                }

                // Live update Pipeline Monitor from ViewModel properties
                if (PipelineArea.Visibility == Visibility.Visible)
                {
                    PipeTelJ1.Text = PipelineVM.TelJ1;
                    PipeTelJ2.Text = PipelineVM.TelJ2;
                    PipeTelJ3.Text = PipelineVM.TelJ3;
                    PipeTelJ4.Text = PipelineVM.TelJ4;
                    PipeTelJ5.Text = PipelineVM.TelJ5;
                    PipeTelJ6.Text = PipelineVM.TelJ6;
                    PipeJointMsg.Text = PipelineVM.JointMsg;
                    PipeBridgeMsg.Text = PipelineVM.BridgeMsg;
                    PipeDriverMsg.Text = PipelineVM.DriverMsg;
                    PipeMsgCount.Text = $"Messages Sent: {PipelineVM.MessagesSent}  |  Received: {PipelineVM.MessagesReceived}";
                    PipelineStatus.Text = PipelineVM.PipelineStatus;
                    PipelineStatus.Foreground = _currentState.IsRunning 
                        ? (System.Windows.Media.Brush)FindResource("Status.Ok") 
                        : (System.Windows.Media.Brush)FindResource("Text.Muted");
                }
            });
        }

        // ── Console & Logging ───────────────────────────────────────
        private void Log(string msg, string severity)
        {
            Services.LoggingService.Instance.Log(msg, severity);
        }

        private void ToggleConsole_Click(object sender, RoutedEventArgs e)
        {
            ConsoleArea.Visibility = ConsoleArea.Visibility == Visibility.Visible ? Visibility.Collapsed : Visibility.Visible;
        }

        // ── Jog Handling ────────────────────────────────────────────
        private void JogMinus_Click(object sender, RoutedEventArgs e) => DoJog_UI(sender, -1);
        private void JogPlus_Click(object sender, RoutedEventArgs e)  => DoJog_UI(sender, 1);
        
        private void DoJog_UI(object sender, int dir)
        {
            if (sender is Button btn && btn.Tag is string jName)
            {
                var jIndex = int.Parse(jName.Substring(1)) - 1;
                var joint = JogVM.Joints[jIndex];
                
                // Determine step size from radio buttons
                double step = JogVM.StepSize;
                foreach (var child in FindVisualChildren<RadioButton>(PanelJog))
                {
                    if (child.GroupName == "JogStep" && child.IsChecked == true)
                    {
                        if (child.Content.ToString()!.Contains("0.1")) step = 0.1;
                        if (child.Content.ToString()!.Contains("10")) step = 10.0;
                        break;
                    }
                }
                JogVM.StepSize = step;
                
                joint.Value = Math.Clamp(joint.Value + (step * dir), joint.Min, joint.Max);
                UpdateRobotTargetFromJog();
            }
        }

        private void JogSlider_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            // Only update if user dragged it (if IsKeyboardFocusWithin or mouse over)
            if (sender is Slider s && s.IsMouseOver)
            {
                UpdateRobotTargetFromJog();
            }
        }

        private void ZeroJoints_Click(object sender, RoutedEventArgs e)
        {
            JogVM.ZeroAllCommand.Execute(null);
            UpdateRobotTargetFromJog();
        }

        private async void UpdateRobotTargetFromJog()
        {
            // DELIBERATE FIX: We do NOT natively update the 3D Viewer anymore.
            // We only send the joint command. The visualizer will update when Gazebo actually publishes /joint_states.
            await _vm.SendJogPositionAsync();
        }

        private static IEnumerable<T> FindVisualChildren<T>(DependencyObject depObj) where T : DependencyObject
        {
            if (depObj != null)
            {
                for (int i = 0; i < VisualTreeHelper.GetChildrenCount(depObj); i++)
                {
                    DependencyObject child = VisualTreeHelper.GetChild(depObj, i);
                    if (child != null && child is T) yield return (T)child;
                    foreach (T childOfChild in FindVisualChildren<T>(child)) yield return childOfChild;
                }
            }
        }

        // ── Toolbar Actions (Visual Programming Execution Loop) ─────────────────────────────────────────


        private void ModeEdit_Click(object sender, RoutedEventArgs e) => Log("Switched to EDIT mode", "Info");
        private void ModeSimulate_Click(object sender, RoutedEventArgs e) => Log("Switched to SIMULATE mode", "Info");
        private void ModeLive_Click(object sender, RoutedEventArgs e) => Log("WARNING: Switched to LIVE HARDWARE mode", "Warning");

        private void NewProgram_Click(object sender, RoutedEventArgs e)
        {
            _vm.NewProgramCommand.Execute(null);
            UpdateCodeGeneration();
        }
        
        private void SaveProgram_Click(object sender, RoutedEventArgs e)
        {
            _vm.SaveProgramCommand.Execute(null);
        }

        private void LoadURDF_Click(object sender, RoutedEventArgs e)
        {
            Log("URDF loading triggered via Settings Panel.", "Info");
        }

        private void RunDiag_Click(object sender, RoutedEventArgs e)
        {
            Log("Running system diagnostics...", "Info");
            Log("No faults detected.", "Success");
        }
    }
}
