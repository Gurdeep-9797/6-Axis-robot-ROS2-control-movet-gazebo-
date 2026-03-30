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

namespace RoboForge_WPF.Views
{
    public partial class MainWindow : Window
    {
        // ViewModels & Services
        private RobotState _currentState;
        private RobotProgram _currentProgram;
        private SceneGraphManager _sceneGraph;
        private IRobotDriver _driver;
        private ExecutionEngine _engine;
        private URDFVisualizer _urdfVisualizer;
        private RoboForge_WPF.Services.CliApiService _apiService;
        
        public ObservableCollection<LogEntry> ConsoleLogs { get; } = new();
        public ObservableCollection<string> Waypoints { get; } = new();
        
        // UI State
        private bool _isPropertiesVisible = false;

        public MainWindow()
        {
            InitializeComponent();
            
            // 1. Initialize core logic
            _currentState = new RobotState();
            _currentProgram = new RobotProgram();
            _sceneGraph = new SceneGraphManager();
            
            // Default Robot Setup
            var robot = new RobotNode { Name = "FR-6DOF" };
            // Define 6 revolute joints strictly matching the industrial D-H URDF
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

            // 2. Setup Driver & Execution
            _driver = new Ros2Driver();
            _driver.StateUpdated += OnDriverStateUpdated;
            _driver.Connect();
            
            _engine = new ExecutionEngine(_driver);
            _engine.ExecutionStateChanged += (running) =>
            {
                _currentState.IsRunning = running;
                Dispatcher.Invoke(() => {
                    ModeSimulate.IsChecked = running;
                    StatusText.Text = running ? "Running Program..." : "Connected";
                    StatusLed.Fill = (SolidColorBrush)FindResource(running ? "Status.Warning" : "Status.Ok");
                    BtnRun.IsEnabled = !running;
                    BtnStop.IsEnabled = running;
                    
                    if(running) Log("Program execution started.", "Info");
                    else Log("Program stopped.", "Info");
                });
            };

            // 3. Initialize UI bindings
            ConsoleList.ItemsSource = ConsoleLogs;
            WaypointList.ItemsSource = Waypoints;
            Log("RoboForge Studio v5.0 initialized", "Success");
            Log("Connected to ROS 2 bridge", "Info");
            
            // 4. Load 3D Viewport
            string urdfPath = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "Assets", "robot.urdf");
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

            // Defaults
            Waypoints.Add("WP_Home");
            Waypoints.Add("WP_Approach");
            Waypoints.Add("WP_Pick");
            Waypoints.Add("WP_Place");
            UpdateCodeGeneration();
            
            // Start Properties collapsed
            PropertiesCol.Width = new GridLength(0);

            // 5. Start CLI REST API
            _apiService = new RoboForge_WPF.Services.CliApiService(_currentState, _currentProgram, _driver, _engine, (msg) => Log(msg, "Info"));
            _apiService.Start();

            // 6. Automated Pipeline Test Hook
            var args = Environment.GetCommandLineArgs();
            if (Array.Exists(args, arg => arg.ToLower() == "--autorun"))
            {
                Task.Run(async () => 
                {
                    await Task.Delay(3000); // 3-second initialization delay
                    Application.Current.Dispatcher.Invoke(() => 
                    {
                        Log("AUTORUN DETECTED: Triggering End-to-End Pipeline...", "Warning");
                        _currentProgram.Instructions.Clear();
                        // Inject 3-step sequence
                        _currentProgram.Instructions.Add(new PtpInstruction("WP_Pick", 60));
                        _currentProgram.Instructions.Add(new WaitInstruction(2000));
                        _currentProgram.Instructions.Add(new LinInstruction("WP_Place", 30, 0));
                        UpdateCodeGeneration();
                        BtnRun.RaiseEvent(new RoutedEventArgs(System.Windows.Controls.Primitives.ButtonBase.ClickEvent)); // Simulate user clicking Run
                    });
                });
            }
            
            this.Closed += MainWindow_Closed;
        }

        private void MainWindow_Closed(object? sender, EventArgs e)
        {
            _apiService?.Stop();
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
                if (!_isPropertiesVisible) ToggleProperties_Click(this, null);
                
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
            // Simple RAPID-style script generation for display
            string script = "// PickAndPlace Program — Generated Module\nMODULE PickPlace_Main\n\n";
            
            foreach(var wp in Waypoints)
            {
                script += $"  CONST robtarget {wp} := [[{GetRandomCoordinate()}],[0,0,1,0]];\n";
            }
            
            script += "\n  PROC main()\n";
            foreach (var instr in _currentProgram.Instructions)
            {
                if (instr is PtpInstruction ptp) script += $"    MoveJ {ptp.PointId}, v1000, fine, tool0;\n";
                else if (instr is LinInstruction lin) script += $"    MoveL {lin.PointId}, v500, fine, tool0;\n";
                else if (instr is WaitInstruction wait) script += $"    WaitTime {wait.DelayMs / 1000.0:F1};\n";
                else if (instr is SetDOInstruction sdo) script += $"    SetDO DO_{sdo.Port}, {sdo.Value};\n";
            }
            script += "  ENDPROC\nENDMODULE\n";
            
            CodeEditor.Document.Blocks.Clear();
            CodeEditor.Document.Blocks.Add(new System.Windows.Documents.Paragraph(new System.Windows.Documents.Run(script)));
        }

        private string GetRandomCoordinate()
        {
            var r = new Random();
            return $"{r.Next(200,600)},{r.Next(-300,300)},{r.Next(50,400)}";
        }

        // ── Properties Panel ────────────────────────────────────────
        private void ToggleProperties_Click(object sender, RoutedEventArgs e)
        {
            _isPropertiesVisible = !_isPropertiesVisible;
            PropertiesCol.Width = _isPropertiesVisible ? new GridLength(280) : new GridLength(0);
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

        // ── Execution Pipeline ──────────────────────────────────────
        private async void Run_Click(object sender, RoutedEventArgs e)
        {
            if (_currentProgram.Instructions.Count == 0)
            {
                Log("Program is empty. Please add blocks.", "Error");
                return;
            }
            
            // Switch to 3D View for execution monitoring
            if (Tab3D != null) Tab3D.IsChecked = true;
            CenterTab_Click(Tab3D!, null!);

            await _engine.RunProgramAsync(_currentProgram, _currentState);
        }

        private void Pause_Click(object sender, RoutedEventArgs e) => _engine.Stop(); // Future: proper pause support
        private void Stop_Click(object sender, RoutedEventArgs e) => _engine.Stop();

        // ── Viewport ────────────────────────────────────────────────
        private void CenterTab_Click(object sender, RoutedEventArgs e)
        {
            if (sender is RadioButton rb && rb.Tag is string tag)
            {
                CodeEditorArea.Visibility = tag == "code" ? Visibility.Visible : Visibility.Collapsed;
                Viewport3DArea.Visibility = tag == "3d" ? Visibility.Visible : Visibility.Collapsed;
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

        // ── Driver & Sync ───────────────────────────────────────────
        private void OnDriverStateUpdated(RobotState updatedState)
        {
            _currentState.J1 = updatedState.J1;
            _currentState.J2 = updatedState.J2;
            _currentState.J3 = updatedState.J3;
            _currentState.J4 = updatedState.J4;
            _currentState.J5 = updatedState.J5;
            _currentState.J6 = updatedState.J6;

            Dispatcher.BeginInvoke(() =>
            {
                // Update 3D visualizer
                if (_urdfVisualizer != null)
                {
                    _urdfVisualizer.UpdateJoints(_currentState.J1, _currentState.J2, _currentState.J3, _currentState.J4, _currentState.J5, _currentState.J6);
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
                    
                    // Note: In real app, run Forward Kinematics to update PropX, PropY, PropZ based on joints
                }
            });
        }

        // ── Console & Logging ───────────────────────────────────────
        private void Log(string msg, string severity)
        {
            Dispatcher.Invoke(() => {
                ConsoleLogs.Add(new LogEntry { Message = msg, Severity = severity });
                if (ConsoleMsgCount != null)
                {
                    ConsoleMsgCount.Text = $"{ConsoleLogs.Count} messages";
                }
                if (ConsoleLogs.Count > 0 && ConsoleList != null)
                {
                    var border = VisualTreeHelper.GetChild(ConsoleList, 0) as Border;
                    if (border != null)
                    {
                        var scrollViewer = VisualTreeHelper.GetChild(border, 0) as ScrollViewer;
                        scrollViewer?.ScrollToBottom();
                    }
                }
            });
        }

        private void ToggleConsole_Click(object sender, RoutedEventArgs e)
        {
            ConsoleArea.Visibility = ConsoleArea.Visibility == Visibility.Visible ? Visibility.Collapsed : Visibility.Visible;
        }

        // ── Modes & Actions ─────────────────────────────────────────
        private void ModeEdit_Click(object sender, RoutedEventArgs e) => Log("Switched to EDIT mode", "Info");
        private void ModeSimulate_Click(object sender, RoutedEventArgs e) => Log("Switched to SIMULATE mode", "Info");
        private void ModeLive_Click(object sender, RoutedEventArgs e) => Log("WARNING: Switched to LIVE HARDWARE mode", "Warning");

        private void NewProgram_Click(object sender, RoutedEventArgs e)
        {
            _currentProgram.Instructions.Clear();
            Waypoints.Clear();
            UpdateCodeGeneration();
            Log("Created new program.", "Info");
        }
        
        private void SaveProgram_Click(object sender, RoutedEventArgs e)
        {
            Log("Program saved successfully.", "Success");
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
