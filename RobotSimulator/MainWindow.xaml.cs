using System;
using System.Collections.Generic;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Windows.Input;
using System.Net.WebSockets;
using System.Text;
using System.Text.Json;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf;
using MahApps.Metro.Controls;
using Microsoft.Win32;
using RobotSimulator.Core.Import;
using RobotSimulator.Core.Kinematics;
using RobotSimulator.Core.Models;
using RobotSimulator.Core.Api;
using RobotSimulator.Core.ROS;
using RobotSimulator.Core.Trajectory;

namespace RobotSimulator
{
    /// <summary>
    /// Professional Digital Twin Simulator for 6-Axis Robot.
    /// Acts as a frontend for ROS 2 / MoveIt.
    /// </summary>
    public partial class MainWindow : MetroWindow
    {
        // Robot model and kinematics
        private RobotModel _robotModel = null!;
        private KinematicsEngine _kinematics = null!;
        private bool _isInitialized = false;
        private bool _suppressSliderEvents = false;
        
        // Joint state
        private readonly double[] _jointAngles = new double[6];
        private readonly Slider[] _sliders;
        private readonly TextBlock[] _sliderLabels;
        
        // 3D model: Actual Robot
        private readonly Transform3DGroup[] _linkTransforms = new Transform3DGroup[7];
        private readonly RotateTransform3D[] _jointRotations = new RotateTransform3D[6];
        private Model3DGroup _robotGroup = null!;
        
        // 3D model: Ghost Robot (Target)
        private readonly Transform3DGroup[] _ghostLinkTransforms = new Transform3DGroup[7];
        private readonly RotateTransform3D[] _ghostJointRotations = new RotateTransform3D[6];
        private Model3DGroup _ghostGroup = null!;
        
        // Materials
        private DiffuseMaterial _orangeMat = new(new SolidColorBrush(Color.FromRgb(255, 140, 0)));
        private DiffuseMaterial _whiteMat = new(new SolidColorBrush(Color.FromRgb(230, 230, 230)));
        private DiffuseMaterial _greyMat = new(new SolidColorBrush(Color.FromRgb(100, 100, 105)));
        private DiffuseMaterial _blueMat = new(new SolidColorBrush(Color.FromRgb(0, 122, 204)));
        private DiffuseMaterial _ghostMat = new(new SolidColorBrush(Color.FromArgb(80, 0, 255, 0))); // Transparent Green
        
        // Program
        private RobotProgram _program = new();
        
        // Serial Hardware (Legacy Debug)
        private RobotSimulator.Core.Hardware.SerialHardwareInterface _serial = new();

        // ROS / MoveIt Client
        private readonly RosMoveItClient _moveit = new();
        private bool _rosConnected = false;
        private ApiServer? _apiServer;
        
        public MainWindow()
        {
            InitializeComponent();
            
            _sliders = new[] { sliJ1, sliJ2, sliJ3, sliJ4, sliJ5, sliJ6 };
            _sliderLabels = new[] { lblJ1, lblJ2, lblJ3, lblJ4, lblJ5, lblJ6 };
            
            Loaded += Window_Loaded;
            Closing += Window_Closing;
        }

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            try
            {
                // Create robot model with exact URDF dimensions
                _robotModel = URDFParser.CreateABBIRB120();
                _kinematics = new KinematicsEngine(_robotModel);
                
                // Build articulated robot (Actual + Ghost)
                BuildArticulatedRobot();
                
                // Setup visual properties
                viewPort3d.RotateGesture = new MouseGesture(MouseAction.RightClick);
                viewPort3d.PanGesture = new MouseGesture(MouseAction.MiddleClick);
                
                // Build environment (floor, table, fences)
                BuildEnvironment();
                
                // Initial update
                UpdateRobotVisualization();
                UpdateDisplays();
                
                // Clean startup state
                statusText.Text = "Ready - Simulator Initialized";
                rosStatusText.Text = "DISCONNECTED";
                rosIndicator.Fill = Brushes.Red;
                
                _isInitialized = true;
                
                // Auto-scan devices on load
                RefreshDeviceList();
                
                // Start API Server for CLI/AI control
                try 
                {
                    StartApiServer();
                }
                catch (Exception ex)
                {
                    var logPath = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "api_error.log");
                    File.WriteAllText(logPath, $"{DateTime.Now}: API Server failed to start.\n{ex}\n");
                    statusText.Text = "API Server Failed to Start (See api_error.log)";
                }
            }
            catch (Exception ex)
            {
                var logPath = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "crash.log");
                File.WriteAllText(logPath, $"{DateTime.Now}\n{ex}\n");
                MessageBox.Show($"Error loading simulator: {ex.Message}\n\nSee crash.log for details.", "Init Error", MessageBoxButton.OK, MessageBoxImage.Error);
            }
        }
        
        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            _apiServer?.Stop();
            _moveit.Disconnect();
            if (_serial.IsConnected) _serial.Disconnect();
        }

        #region 3D Robot Construction
        
        private void BuildArticulatedRobot()
        {
            _robotGroup = new Model3DGroup();
            _ghostGroup = new Model3DGroup();

            // --- ACTUAL ROBOT ---
            BuildRobotChain(_robotGroup, _linkTransforms, _jointRotations, _orangeMat, _whiteMat, _greyMat);
            robotVisual.Content = _robotGroup;
            
            // --- GHOST ROBOT ---
            BuildRobotChain(_ghostGroup, _ghostLinkTransforms, _ghostJointRotations, _ghostMat, _ghostMat, _ghostMat);
            ghostVisual.Content = _ghostGroup;
        }

        private void BuildRobotChain(Model3DGroup group, Transform3DGroup[] transforms, RotateTransform3D[] rotations, 
                                   Material matBase, Material matArm, Material matWrist)
        {
            // Link lengths (meters) from URDF
            double l1 = 0.29, l2 = 0.27, l3 = 0.302, l4 = 0.072, l5 = 0.072, l6 = 0.050;

            // --- BASE ---
            var mk = new MeshBuilder();
            // Main base plate
            mk.AddBox(new Point3D(0, 0, 0.05), 0.25, 0.25, 0.1); 
            // Pedestal
            mk.AddCylinder(new Point3D(0, 0, 0.1), new Point3D(0, 0, 0.2), 0.10, 32);
            // Cable connectors (detail)
            mk.AddBox(new Point3D(-0.12, 0, 0.08), 0.05, 0.10, 0.05);
            group.Children.Add(new GeometryModel3D(mk.ToMesh(), matBase));

            // --- LINK 1 (Turret / J1) ---
            rotations[0] = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(0, 0, 1), 0));
            transforms[0] = new Transform3DGroup();
            transforms[0].Children.Add(rotations[0]);
            
            var m1 = new MeshBuilder();
            // Turret cylinder
            m1.AddCylinder(new Point3D(0, 0, 0), new Point3D(0, 0, l1 - 0.05), 0.11, 32);
            // Shoulder mount (offset)
            m1.AddBox(new Point3D(0.05, 0, l1 - 0.1), 0.15, 0.18, 0.15);
            // Motor housing J2
            m1.AddCylinder(new Point3D(0, 0.1, l1 - 0.05), new Point3D(0, 0.18, l1 - 0.05), 0.08, 16);
            AddLink(group, m1, matBase, transforms[0]);

            // --- LINK 2 (Lower Arm / J2) ---
            rotations[1] = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(0, 1, 0), 0));
            transforms[1] = new Transform3DGroup();
            transforms[1].Children.Add(rotations[1]);
            transforms[1].Children.Add(new TranslateTransform3D(0, 0, l1));
            transforms[1].Children.Add(transforms[0]);
            
            var m2 = new MeshBuilder();
            // Arm structure (tapered box approximated)
            m2.AddBox(new Point3D(0, 0, l2/2), 0.12, 0.12, l2 + 0.1);
            // Counterweight cylinder
            m2.AddCylinder(new Point3D(0, 0, -0.05), new Point3D(0, 0, 0.05), 0.10, 16);
            // Elbow joint housing
            m2.AddCylinder(new Point3D(0, 0.08, l2), new Point3D(0, 0.14, l2), 0.07, 16);
            AddLink(group, m2, matBase, transforms[1]);

            // --- LINK 3 (Upper Arm / J3) ---
            rotations[2] = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(0, 1, 0), 0));
            transforms[2] = new Transform3DGroup();
            transforms[2].Children.Add(rotations[2]);
            transforms[2].Children.Add(new TranslateTransform3D(0, 0, l2));
            transforms[2].Children.Add(transforms[1]);
            
            var m3 = new MeshBuilder();
            // Arm boom
            m3.AddBox(new Point3D(0.05, 0, l3/2), 0.20, 0.10, l3 + 0.05);
            // Motor housing J4
            m3.AddCylinder(new Point3D(0.1, 0, 0.05), new Point3D(0.25, 0, 0.05), 0.06, 16);
            AddLink(group, m3, matBase, transforms[2]);

            // --- LINK 4 (Forearm / J4) ---
            rotations[3] = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(1, 0, 0), 0));
            transforms[3] = new Transform3DGroup();
            transforms[3].Children.Add(rotations[3]);
            transforms[3].Children.Add(new TranslateTransform3D(0, 0, l3));
            transforms[3].Children.Add(transforms[2]);
            
            var m4 = new MeshBuilder();
            // Main cylinder
            m4.AddCylinder(new Point3D(0, 0, 0), new Point3D(l4 + l5, 0, 0), 0.06, 24);
            // Motor J5 bump
            m4.AddBox(new Point3D(0.05, 0, 0.05), 0.08, 0.06, 0.08);
            AddLink(group, m4, matArm, transforms[3]);

            // --- LINK 5 (Wrist / J5) ---
            rotations[4] = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(0, 1, 0), 0));
            transforms[4] = new Transform3DGroup();
            transforms[4].Children.Add(rotations[4]);
            transforms[4].Children.Add(new TranslateTransform3D(l4, 0, 0)); // Corrected offset logic
            transforms[4].Children.Add(transforms[3]);
            
            var m5 = new MeshBuilder();
            // Wrist joint
            m5.AddBox(new Point3D(0.03, 0, 0), 0.06, 0.08, 0.06);
            AddLink(group, m5, matArm, transforms[4]);

            // --- LINK 6 (Flange / J6) ---
            rotations[5] = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(1, 0, 0), 0));
            transforms[5] = new Transform3DGroup();
            transforms[5].Children.Add(rotations[5]);
            transforms[5].Children.Add(new TranslateTransform3D(l5, 0, 0));
            transforms[5].Children.Add(transforms[4]);
            
            var m6 = new MeshBuilder();
            // Mounting Flange (Silver/Grey)
            m6.AddCylinder(new Point3D(0, 0, 0), new Point3D(0.01, 0, 0), 0.04, 24);
            // Locating pin
            m6.AddCylinder(new Point3D(0.01, 0.02, 0), new Point3D(0.015, 0.02, 0), 0.005, 8);
            AddLink(group, m6, matWrist, transforms[5]);
            
            // Tool/TCP
            transforms[6] = new Transform3DGroup();
            transforms[6].Children.Add(new TranslateTransform3D(l6, 0, 0)); // Final offset
            transforms[6].Children.Add(transforms[5]);
            
            // TCP Marker (Blue Sphere)
            var mtcp = new MeshBuilder();
            mtcp.AddSphere(new Point3D(0,0,0), 0.005);
            group.Children.Add(new GeometryModel3D(mtcp.ToMesh(), _blueMat) { Transform = transforms[6] });
        }
        
        private void AddLink(Model3DGroup group, MeshBuilder mb, Material mat, Transform3D trans)
        {
            var geom = new GeometryModel3D(mb.ToMesh(), mat);
            geom.Transform = trans;
            group.Children.Add(geom);
        }

        private void UpdateRobotVisualization()
        {
            // Update Rotation Transforms (Chain propagates automatically)
            for (int i = 0; i < 6; i++)
            {
                var rotation = (AxisAngleRotation3D)_jointRotations[i].Rotation;
                rotation.Angle = _jointAngles[i] * 180.0 / Math.PI;
            }
            
            // Update Cartesian Readout
            var (x, y, z) = _kinematics.GetEndEffectorPosition(_jointAngles);
            var (r, p, yaw) = _kinematics.GetEndEffectorOrientation(_jointAngles);
            
            txtX.Text = $"X: {x * 1000:F1}";
            txtY.Text = $"Y: {y * 1000:F1}";
            txtZ.Text = $"Z: {z * 1000:F1}";
            txtR.Text = $"R: {r * 180/Math.PI:F1}°";
            txtP.Text = $"P: {p * 180/Math.PI:F1}°";
            txtYaw.Text = $"Y: {yaw * 180/Math.PI:F1}°";
            
            // Sync Serial if connected (Legacy)
            if (_serial.IsConnected)
            {
                _serial.SendJointAngles(_jointAngles);
            }
        }
        
        private void SetGhostAngles(double[] angles)
        {
            if (angles == null || angles.Length < 6) return;
            
            for (int i = 0; i < 6; i++)
            {
                var rotation = (AxisAngleRotation3D)_ghostJointRotations[i].Rotation;
                rotation.Angle = angles[i] * 180.0 / Math.PI;
            }
        }

        #endregion

        #region User Interaction

        private void Slider_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (!_isInitialized || _suppressSliderEvents) return;
            
            // Sliders drive the 'Actual' robot locally for teaching/jogging
            // UNLESS strict sync is on and we are connected to ROS
            if (_rosConnected && chkSync.IsChecked == true && _moveit.IsConnected)
            {
                // In strict sync mode, sliders only propose a command, 
                // but the 3D model waits for /joint_states from ROS.
                // However, for smooth dragging, we usually visualize the command immediately 
                // and let the ghost/actual split handle it. 
                // For simplicity here: Sliders update local model immediately.
            }
            
            _jointAngles[0] = sliJ1.Value * Math.PI / 180.0;
            _jointAngles[1] = sliJ2.Value * Math.PI / 180.0;
            _jointAngles[2] = sliJ3.Value * Math.PI / 180.0;
            _jointAngles[3] = sliJ4.Value * Math.PI / 180.0;
            _jointAngles[4] = sliJ5.Value * Math.PI / 180.0;
            _jointAngles[5] = sliJ6.Value * Math.PI / 180.0;
            
            UpdateRobotVisualization();
            UpdateDisplays();
        }
        
        private void UpdateDisplays()
        {
            for (int i = 0; i < 6; i++)
            {
                _sliderLabels[i].Text = $"{_jointAngles[i] * 180.0 / Math.PI:F1}°";
            }
        }

        private void ToggleCoords_Click(object sender, RoutedEventArgs e)
        {
            viewPort3d.ShowCoordinateSystem = (chkCoords.IsChecked == true);
        }

        private void ScanDevices_Click(object sender, RoutedEventArgs e)
        {
            RefreshDeviceList();
        }
        
        private void RefreshDeviceList()
        {
            deviceList.Items.Clear();
            var ports = SerialPort.GetPortNames();
            foreach (var port in ports)
            {
                deviceList.Items.Add($"{port} (Serial Device)");
            }
            
            if (deviceList.Items.Count == 0)
                deviceList.Items.Add("No USB devices found");
            else
                statusText.Text = $"Found {ports.Length} devices";
        }

        private void EStop_Click(object sender, RoutedEventArgs e)
        {
            // Stop logic
            statusText.Text = "EMERGENCY STOP TRIGGERED";
            rosStatusText.Text = "E-STOP";
            rosIndicator.Fill = Brushes.Red;
            
            if (_rosConnected)
            {
                // Sending an empty trajectory usually aborts execution in ROS controllers
                _ = _moveit.ExecuteTrajectoryAsync(new double[0][]);
            }
            
            if (_serial.IsConnected)
            {
                _serial.Disconnect(); // Sever connection
            }
        }

        private void ImportSTL_Click(object sender, RoutedEventArgs e)
        {
            var dlg = new OpenFileDialog { Filter = "STL Files|*.stl|All Files|*.*" };
            if (dlg.ShowDialog() == true)
            {
                try
                {
                    var reader = new StLReader();
                    var model = reader.Read(dlg.FileName);
                    var visual = new ModelVisual3D();
                    visual.Content = model;
                    
                    if (model is Model3DGroup group)
                    {
                        // Apply material to all children if possible, 
                        // or just leave as is (HelixToolkit default is usually grey)
                        foreach(var child in group.Children)
                        {
                            if (child is GeometryModel3D geom)
                                geom.Material = _greyMat;
                        }
                    }

                    // Add to environment
                    var envGroup = environmentVisual.Content as Model3DGroup ?? new Model3DGroup();
                    envGroup.Children.Add(visual.Content);
                    environmentVisual.Content = envGroup;
                    
                    statusText.Text = $"Imported {Path.GetFileName(dlg.FileName)}";
                }
                catch (Exception ex)
                {
                    MessageBox.Show($"Import failed: {ex.Message}");
                }
            }
        }
        
        // --- Teach Program Handlers ---
        
         private void AddPoint_Click(object sender, RoutedEventArgs e)
        {
            var (x, y, z) = _kinematics.GetEndEffectorPosition(_jointAngles);
            var (r, p, yw) = _kinematics.GetEndEffectorOrientation(_jointAngles);
            
            var point = _program.AddPoint((double[])_jointAngles.Clone(), new Point3D(x, y, z), r, p, yw);
            RefreshPointList();
            // Draw marker
            // (Simplified: Rebuild all for now)
            // RebuildPointMarkers(); // Need to implement marker visualization in V2 style if desired
        }

        private void ClearPoints_Click(object sender, RoutedEventArgs e)
        {
            _program.Clear();
            RefreshPointList();
        }

        private void RefreshPointList()
        {
            pointList.Items.Clear();
            foreach (var pt in _program.Points)
            {
                pointList.Items.Add($"P{pt.Id} ({pt.CartesianPosition.X*1000:F0}, {pt.CartesianPosition.Y*1000:F0}, {pt.CartesianPosition.Z*1000:F0})");
            }
        }

        private async void GotoPoint_Click(object sender, RoutedEventArgs e)
        {
            if (pointList.SelectedIndex < 0) return;
            var pt = _program.Points[pointList.SelectedIndex];
            
            if (_rosConnected && _moveit.IsConnected)
            {
                statusText.Text = $"Planning to P{pt.Id} (MoveIt)...";
                
                // Show Ghost at target immediately
                if (chkGhost.IsChecked == true)
                {
                    SetGhostAngles(pt.JointAngles);
                }
                
                var traj = await _moveit.PlanToJointGoalAsync(_jointAngles, pt.JointAngles);
                if (traj != null && traj.Length > 0)
                {
                    // Execute
                    await _moveit.ExecuteTrajectoryAsync(traj);
                    statusText.Text = $"Execution Started ({traj.Length} pts)";
                }
                else
                {
                    statusText.Text = "Planning Failed!";
                }
            }
            else
            {
                // Direct Move
                ApplyJoints(pt.JointAngles);
                statusText.Text = $"Moved to P{pt.Id} (Direct)";
            }
        }
        
        private async void RunProgram_Click(object sender, RoutedEventArgs e)
        {
             // Simple sequential execution
             foreach(var pt in _program.Points)
             {
                 if (_rosConnected)
                 {
                     var traj = await _moveit.PlanToJointGoalAsync(_jointAngles, pt.JointAngles);
                     if (traj != null) await _moveit.ExecuteTrajectoryAsync(traj);
                     // Need await for completion? Real implementation would use execution_state feedback.
                     // For now, simple delay estimate.
                     await Task.Delay(2000); 
                 }
             }
        }

        private void ApplyJoints(double[] angles)
        {
             for(int i=0; i<6; i++) {
                 _jointAngles[i] = angles[i];
                 _sliders[i].Value = angles[i] * 180.0 / Math.PI;
             }
             UpdateRobotVisualization();
             UpdateDisplays();
        }

        #endregion

        #region ROS & Devices

        private async void ConnectRos_Click(object sender, RoutedEventArgs e)
        {
            if (_rosConnected)
            {
                _moveit.Disconnect();
                _rosConnected = false;
                rosStatusText.Text = "DISCONNECTED";
                rosIndicator.Fill = Brushes.Red;
            }
            else
            {
                statusText.Text = "Connecting ROS...";
                _moveit.OnJointStateReceived += OnRosJointStateReceived;
                bool success = await _moveit.ConnectAsync(rosUri.Text);
                
                if (success)
                {
                    _rosConnected = true;
                    rosStatusText.Text = "CONNECTED";
                    rosIndicator.Fill = Brushes.LightGreen;
                    statusText.Text = "ROS Connected. Digital Twin Active.";
                }
                else
                {
                    statusText.Text = "Connection Failed";
                }
            }
        }

        private void OnRosJointStateReceived(double[] positions)
        {
            Dispatcher.Invoke(() =>
            {
                // Strict Sync: REAL robot model follows ROS feedback only
                if (chkSync.IsChecked == true)
                {
                    _suppressSliderEvents = true;
                    for (int i = 0; i < 6; i++)
                    {
                        _jointAngles[i] = positions[i];
                        _sliders[i].Value = positions[i] * 180.0 / Math.PI;
                    }
                    _suppressSliderEvents = false;
                    UpdateRobotVisualization();
                    UpdateDisplays();
                }
                
                // Update Latency UI (Mock)
                rosLatencyText.Text = "Latency: 12ms";
            });
        }
        
        private void ConnectSerial_Click(object sender, RoutedEventArgs e)
        {
             var port = deviceList.SelectedItem as string;
             if(port == null) return;
             // Clean port name "COM3 (Device)" -> "COM3"
             var cleanPort = port.Split(' ')[0];
             _serial.Connect(cleanPort);
             btnSerialConnect.IsEnabled = false;
             statusText.Text = $"Legacy Serial Connected to {cleanPort}";
        }

        #endregion

        #region Environment

        private void BuildEnvironment()
        {
            var envGroup = new Model3DGroup();
            var floorMat = new DiffuseMaterial(new SolidColorBrush(Color.FromRgb(60, 60, 65)));
            var tableMat = new DiffuseMaterial(new SolidColorBrush(Color.FromRgb(90, 90, 95)));
            var fenceMat = new DiffuseMaterial(new SolidColorBrush(Color.FromArgb(120, 255, 200, 0)));
            var gridMat  = new DiffuseMaterial(new SolidColorBrush(Color.FromArgb(40, 200, 200, 200)));

            // Floor
            var floor = new MeshBuilder();
            floor.AddBox(new Point3D(0, 0, -0.005), 3.0, 3.0, 0.01);
            envGroup.Children.Add(new GeometryModel3D(floor.ToMesh(), floorMat));

            // Floor Grid Lines (thin boxes as grid)
            for (double g = -1.5; g <= 1.5; g += 0.25)
            {
                var gx = new MeshBuilder();
                gx.AddBox(new Point3D(g, 0, 0.001), 0.003, 3.0, 0.002);
                envGroup.Children.Add(new GeometryModel3D(gx.ToMesh(), gridMat));

                var gy = new MeshBuilder();
                gy.AddBox(new Point3D(0, g, 0.001), 3.0, 0.003, 0.002);
                envGroup.Children.Add(new GeometryModel3D(gy.ToMesh(), gridMat));
            }

            // Worktable
            var table = new MeshBuilder();
            table.AddBox(new Point3D(0.6, 0, 0.35), 0.8, 0.5, 0.03); // Top
            table.AddBox(new Point3D(0.25, -0.20, 0.17), 0.05, 0.05, 0.34); // Leg 1
            table.AddBox(new Point3D(0.95, -0.20, 0.17), 0.05, 0.05, 0.34); // Leg 2
            table.AddBox(new Point3D(0.25,  0.20, 0.17), 0.05, 0.05, 0.34); // Leg 3
            table.AddBox(new Point3D(0.95,  0.20, 0.17), 0.05, 0.05, 0.34); // Leg 4
            envGroup.Children.Add(new GeometryModel3D(table.ToMesh(), tableMat));

            // Safety Fence Posts (4 corners of the work cell)
            var fence = new MeshBuilder();
            fence.AddCylinder(new Point3D(-1.2, -1.2, 0), new Point3D(-1.2, -1.2, 1.0), 0.025, 12);
            fence.AddCylinder(new Point3D( 1.2, -1.2, 0), new Point3D( 1.2, -1.2, 1.0), 0.025, 12);
            fence.AddCylinder(new Point3D(-1.2,  1.2, 0), new Point3D(-1.2,  1.2, 1.0), 0.025, 12);
            fence.AddCylinder(new Point3D( 1.2,  1.2, 0), new Point3D( 1.2,  1.2, 1.0), 0.025, 12);
            // Horizontal rails
            fence.AddCylinder(new Point3D(-1.2, -1.2, 0.5), new Point3D( 1.2, -1.2, 0.5), 0.015, 8);
            fence.AddCylinder(new Point3D(-1.2,  1.2, 0.5), new Point3D( 1.2,  1.2, 0.5), 0.015, 8);
            fence.AddCylinder(new Point3D(-1.2, -1.2, 0.5), new Point3D(-1.2,  1.2, 0.5), 0.015, 8);
            fence.AddCylinder(new Point3D( 1.2, -1.2, 0.5), new Point3D( 1.2,  1.2, 0.5), 0.015, 8);
            fence.AddCylinder(new Point3D(-1.2, -1.2, 1.0), new Point3D( 1.2, -1.2, 1.0), 0.015, 8);
            fence.AddCylinder(new Point3D(-1.2,  1.2, 1.0), new Point3D( 1.2,  1.2, 1.0), 0.015, 8);
            fence.AddCylinder(new Point3D(-1.2, -1.2, 1.0), new Point3D(-1.2,  1.2, 1.0), 0.015, 8);
            fence.AddCylinder(new Point3D( 1.2, -1.2, 1.0), new Point3D( 1.2,  1.2, 1.0), 0.015, 8);
            envGroup.Children.Add(new GeometryModel3D(fence.ToMesh(), fenceMat));

            // Small workpiece on table (demo cylinder)
            var workpiece = new MeshBuilder();
            workpiece.AddCylinder(new Point3D(0.6, 0, 0.365), new Point3D(0.6, 0, 0.46), 0.04, 24);
            var wpMat = new DiffuseMaterial(new SolidColorBrush(Color.FromRgb(180, 180, 190)));
            envGroup.Children.Add(new GeometryModel3D(workpiece.ToMesh(), wpMat));

            environmentVisual.Content = envGroup;
        }

        #endregion

        #region API Server

        private void StartApiServer()
        {
            _apiServer = new ApiServer(8085);
            
            // Wire status queries
            _apiServer.GetJointAngles = () => (double[])_jointAngles.Clone();
            _apiServer.GetTcpPosition = () => _kinematics.GetEndEffectorPosition(_jointAngles);
            _apiServer.GetTcpOrientation = () => _kinematics.GetEndEffectorOrientation(_jointAngles);
            _apiServer.GetRosConnected = () => _rosConnected;
            _apiServer.GetStatusText = () => statusText.Text;
            
            // Wire control actions
            _apiServer.SetJointAngles = (angles) => ApplyJoints(angles);
            _apiServer.HomeRobot = () => ApplyJoints(new double[6]);
            _apiServer.AddWaypoint = () => AddPoint_Click(this, new RoutedEventArgs());
            _apiServer.ClearProgram = () => ClearPoints_Click(this, new RoutedEventArgs());
            _apiServer.RunProgram = () => RunProgram_Click(this, new RoutedEventArgs());
            _apiServer.EmergencyStop = () => EStop_Click(this, new RoutedEventArgs());
            _apiServer.ConnectRos = () => ConnectRos_Click(this, new RoutedEventArgs());
            
            // Wire program query
            _apiServer.GetProgramPoints = () =>
            {
                return _program.Points.Select(p =>
                    $"P{p.Id} ({p.CartesianPosition.X * 1000:F0}, {p.CartesianPosition.Y * 1000:F0}, {p.CartesianPosition.Z * 1000:F0})").ToArray();
            };
            
            _apiServer.Start();
            statusText.Text = "Ready - API Server on port 8085";
        }

        #endregion
    }
}
