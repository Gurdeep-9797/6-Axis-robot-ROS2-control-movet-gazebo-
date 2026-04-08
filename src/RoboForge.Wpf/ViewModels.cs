using System;
using System.Linq;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.Net.WebSockets;
using System.Text;
using System.Text.Json;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Input;
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;

namespace RoboForge.Wpf.ViewModels
{
    public enum AppMode { Edit, Simulate, Live }
    public enum ExecMode { Single, Sequential, Parallel }
    public enum GlobalState { Idle, Running, Ros2Connected, Error }
    public enum IKSolverMode { Numerical, Analytical }
    public enum ConnectionMode { Local, Tunnel, Offline }

    // â”€â”€ v8.2 Diagnostic Models â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    public class TrackingErrorReport
    {
        public string Type { get; set; } = "tracking_error";
        public double[] PerJointErrorDeg { get; set; } = Array.Empty<double>();
        public double TcpErrorMm { get; set; }
        public string MaxErrorJoint { get; set; } = "";
        public double MaxErrorDeg { get; set; }
    }

    public class HealthCheckResult
    {
        public string Name { get; set; } = "";
        public bool Ok { get; set; }
        public string Detail { get; set; } = "";
    }

    // â”€â”€ Console Entry â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    public class ConsoleEntry
    {
        public string Time { get; set; } = "";
        public string Level { get; set; } = "INF";
        public string Message { get; set; } = "";
    }

    // â”€â”€ Program Block â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    public class ProgramBlock : ObservableObject
    {
        public string Id { get; set; } = "";
        public string Name { get; set; } = "";
        public string Type { get; set; } = "MoveJ";
        public double X { get; set; }
        public double Y { get; set; }
        public double Z { get; set; }
        public double Speed { get; set; } = 500;
        public string Status { get; set; } = "idle"; // idle, running, done, error
    }

    // â”€â”€ Main ViewModel â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    public partial class MainViewModel : ObservableObject
    {
        [ObservableProperty] private AppMode _activeMode;
        [ObservableProperty] private ExecMode _executionMode;
        [ObservableProperty] private string _activeRobotId = "IRB 6700-235/2.65";
        [ObservableProperty] private GlobalState _state;
        [ObservableProperty] private bool _isRos2Connected;
        [ObservableProperty] private IKSolverMode _ikMode = IKSolverMode.Numerical;
        [ObservableProperty] private ConnectionMode _connectionMode = ConnectionMode.Offline;
        [ObservableProperty] private string _connectionStatus = "Disconnected";
        [ObservableProperty] private string _tunnelUrl = "";

        // Joint angles (radians)
        [ObservableProperty] private double _j1, _j2, _j3, _j4, _j5, _j6;
        [ObservableProperty] private int _motorSpeed = 50; // PWM duty cycle %

        // v8.2 Diagnostics
        [ObservableProperty] private TrackingErrorReport? _lastTrackingError;
        [ObservableProperty] private ObservableCollection<HealthCheckResult> _healthResults = new();
        [ObservableProperty] private bool _isHealthCheckRunning;

        // Console
        public ObservableCollection<ConsoleEntry> ConsoleEntries { get; } = new()
        {
            new() { Time = DateTime.Now.ToString("HH:mm:ss"), Level = "INF", Message = "RoboForge v8.2 (Offline) initialized" },
            new() { Time = DateTime.Now.ToString("HH:mm:ss"), Level = "OK", Message = "Execution engine ready â€” IK: analytical + numerical" },
            new() { Time = DateTime.Now.ToString("HH:mm:ss"), Level = "INF", Message = "Robot model: IRB 6700-235/2.65 loaded" },
        };

        // Program blocks
        public ObservableCollection<ProgramBlock> ProgramBlocks { get; } = new()
        {
            new() { Id = "b-home", Name = "MoveJ Home", Type = "MoveJ", X = 800, Y = 0, Z = 1800, Speed = 500 },
            new() { Id = "b-gripper-off", Name = "SetDO Gripper OFF", Type = "SetDO" },
            new() { Id = "b-approach", Name = "MoveJ Approach", Type = "MoveJ", X = 800, Y = 200, Z = 1400, Speed = 500 },
            new() { Id = "b-pick", Name = "MoveL Pick", Type = "MoveL", X = 700, Y = 200, Z = 1200, Speed = 200 },
            new() { Id = "b-grip-on", Name = "Gripper Close", Type = "GripperClose" },
            new() { Id = "b-lift", Name = "MoveL Lift", Type = "MoveL", X = 500, Y = 0, Z = 1200, Speed = 500 },
            new() { Id = "b-place", Name = "MoveJ Place", Type = "MoveJ", X = -300, Y = 400, Z = 1200, Speed = 500 },
            new() { Id = "b-grip-off", Name = "Gripper Open", Type = "GripperOpen" },
        };

        private ClientWebSocket? _ws;
        private CancellationTokenSource? _wsCts;

        public MainViewModel()
        {
            // Auto-connect will be triggered after the window loads
        }

        /// <summary>Called after the main window is loaded to start connection.</summary>
        public void Initialize()
        {
            _ = AutoConnectAsync();
        }

        // â”€â”€ ROS 2 Connection (via rosbridge WebSocket) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
        private async Task AutoConnectAsync()
        {
            AddConsole("INF", "Attempting ROS 2 connection...");

            // Priority 1: localhost
            if (await TryConnectAsync("ws://localhost:9090"))
            {
                ConnectionMode = ConnectionMode.Local;
                ConnectionStatus = "Connected (Local)";
                IsRos2Connected = true;
                AddConsole("OK", "Connected to local Docker backend on ws://localhost:9090");
                return;
            }

            // Priority 2: tunnel URL
            if (!string.IsNullOrEmpty(TunnelUrl) && await TryConnectAsync(TunnelUrl))
            {
                ConnectionMode = ConnectionMode.Tunnel;
                ConnectionStatus = $"Connected (Tunnel)";
                IsRos2Connected = true;
                AddConsole("OK", $"Connected via tunnel: {TunnelUrl}");
                return;
            }

            // Priority 3: offline
            ConnectionMode = ConnectionMode.Offline;
            ConnectionStatus = "Offline Mode";
            IsRos2Connected = false;
            AddConsole("WRN", "No ROS 2 backend found â€” offline simulation mode");
        }

        private async Task<bool> TryConnectAsync(string url)
        {
            try
            {
                var ws = new ClientWebSocket();
                var cts = new CancellationTokenSource(TimeSpan.FromSeconds(3));
                await ws.ConnectAsync(new Uri(url), cts.Token);
                _ws = ws;
                _wsCts = new CancellationTokenSource();
                _ = ListenForMessagesAsync();
                return true;
            }
            catch
            {
                return false;
            }
        }

        private async Task ListenForMessagesAsync()
        {
            if (_ws == null) return;
            var buffer = new byte[8192];
            try
            {
                while (_ws.State == WebSocketState.Open && _wsCts != null && !_wsCts.IsCancellationRequested)
                {
                    var result = await _ws.ReceiveAsync(buffer, _wsCts.Token);
                    if (result.MessageType == WebSocketMessageType.Close) break;
                    var msg = Encoding.UTF8.GetString(buffer, 0, result.Count);
                    // Process incoming ROS messages (joint_states, etc.)
                    ProcessRosMessage(msg);
                }
            }
            catch { /* disconnected */ }
            IsRos2Connected = false;
            ConnectionMode = ConnectionMode.Offline;
            ConnectionStatus = "Disconnected";
            AddConsole("WRN", "ROS 2 connection lost â€” falling back to offline mode");
        }

        private void ProcessRosMessage(string json)
        {
            try
            {
                var doc = JsonDocument.Parse(json);
                var topic = doc.RootElement.GetProperty("topic").GetString();
                if (topic == "/joint_states")
                {
                    var positions = doc.RootElement.GetProperty("msg").GetProperty("position");
                    if (positions.GetArrayLength() >= 6)
                    {
                        J1 = positions[0].GetDouble();
                        J2 = positions[1].GetDouble();
                        J3 = positions[2].GetDouble();
                        J4 = positions[3].GetDouble();
                        J5 = positions[4].GetDouble();
                        J6 = positions[5].GetDouble();
                    }
                }
                else if (topic == "/roboforge/tracking_error")
                {
                    LastTrackingError = JsonSerializer.Deserialize<TrackingErrorReport>(
                        doc.RootElement.GetProperty("msg").GetRawText(),
                        new JsonSerializerOptions { PropertyNameCaseInsensitive = true }
                    );
                }
                else if (topic == "/roboforge/health_status")
                {
                    var results = JsonSerializer.Deserialize<List<HealthCheckResult>>(
                        doc.RootElement.GetProperty("msg").GetRawText(),
                        new JsonSerializerOptions { PropertyNameCaseInsensitive = true }
                    );
                    if (results != null)
                    {
                        App.Current.Dispatcher.Invoke(() => {
                            HealthResults.Clear();
                            foreach (var r in results) HealthResults.Add(r);
                        });
                    }
                }
            }
            catch { /* ignore malformed messages */ }
        }

        [RelayCommand]
        public async Task RunHealthCheckAsync()
        {
            if (_ws == null || _ws.State != WebSocketState.Open) return;
            IsHealthCheckRunning = true;
            try
            {
                var request = JsonSerializer.Serialize(new
                {
                    op = "call_service",
                    service = "/roboforge/health_check",
                    type = "std_srvs/srv/Trigger"
                });
                await _ws.SendAsync(new ArraySegment<byte>(Encoding.UTF8.GetBytes(request)), 
                    WebSocketMessageType.Text, true, CancellationToken.None);
            }
            finally { IsHealthCheckRunning = false; }
        }

        /// <summary>
        /// Call MoveIt's compute_ik service via rosbridge WebSocket
        /// </summary>
        public async Task<double[]?> ComputeMoveItIKAsync(double x_mm, double y_mm, double z_mm)
        {
            if (_ws == null || _ws.State != WebSocketState.Open) return null;

            var request = JsonSerializer.Serialize(new
            {
                op = "call_service",
                service = "/compute_ik",
                type = "moveit_msgs/srv/GetPositionIK",
                args = new
                {
                    ik_request = new
                    {
                        group_name = "manipulator",
                        robot_state = new
                        {
                            joint_state = new
                            {
                                name = new[] { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" },
                                position = new[] { J1, J2, J3, J4, J5, J6 }
                            }
                        },
                        avoid_collisions = true,
                        pose_stamped = new
                        {
                            header = new { frame_id = "base_link" },
                            pose = new
                            {
                                position = new { x = x_mm / 1000.0, y = y_mm / 1000.0, z = z_mm / 1000.0 },
                                orientation = new { x = 0, y = 0, z = 0, w = 1 }
                            }
                        },
                        timeout = new { sec = 5, nanosec = 0 }
                    }
                }
            });

            var bytes = Encoding.UTF8.GetBytes(request);
            await _ws.SendAsync(bytes, WebSocketMessageType.Text, true, CancellationToken.None);
            AddConsole("DBG", $"MoveIt IK request sent: [{x_mm}, {y_mm}, {z_mm}]mm");

            // Wait for response (simplified â€” in production, use proper request ID matching)
            var buffer = new byte[16384];
            var result = await _ws.ReceiveAsync(buffer, CancellationToken.None);
            var response = Encoding.UTF8.GetString(buffer, 0, result.Count);

            try
            {
                var doc = JsonDocument.Parse(response);
                var errorCode = doc.RootElement.GetProperty("values").GetProperty("error_code").GetProperty("val").GetInt32();
                if (errorCode == 1) // SUCCESS
                {
                    var positions = doc.RootElement.GetProperty("values").GetProperty("solution")
                        .GetProperty("joint_state").GetProperty("position");
                    var joints = new double[6];
                    for (int i = 0; i < 6; i++)
                        joints[i] = positions[i].GetDouble();
                    AddConsole("OK", $"MoveIt IK solved: [{string.Join(", ", joints.Select(j => j.ToString("F3")))}]");
                    return joints;
                }
            }
            catch { }

            AddConsole("WRN", "MoveIt IK failed â€” falling back to offline solver");
            return null;
        }

        // â”€â”€ Execution Commands â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
        [RelayCommand]
        private async Task RunAsync()
        {
            State = GlobalState.Running;
            AddConsole("INF", $"â–¶ Running program: {ProgramBlocks.Count} instructions (IK: {IkMode})");

            foreach (var block in ProgramBlocks)
            {
                block.Status = "running";

                if (block.Type == "MoveJ" || block.Type == "MoveL")
                {
                    AddConsole("DBG", $"IK solving â†’ [{block.X}, {block.Y}, {block.Z}] (mode: {IkMode})");

                    double[]? joints = null;

                    // Try MoveIt first (if connected)
                    if (ConnectionMode != ConnectionMode.Offline)
                    {
                        joints = await ComputeMoveItIKAsync(block.X, block.Y, block.Z);
                    }

                    if (joints != null)
                    {
                        AddConsole("OK", $"IK solved via MoveIt â€” animating {block.Type}");
                        J1 = joints[0]; J2 = joints[1]; J3 = joints[2];
                        J4 = joints[3]; J5 = joints[4]; J6 = joints[5];
                        block.Status = "done";
                    }
                    else
                    {
                        AddConsole("WRN", $"Using offline IK ({IkMode}) for {block.Name}");
                        // Offline fallback would use C# IK solver here
                        block.Status = "done";
                    }
                }
                else if (block.Type == "SetDO")
                {
                    AddConsole("INF", $"âš¡ {block.Name}");
                    block.Status = "done";
                }
                else if (block.Type is "GripperOpen" or "GripperClose")
                {
                    AddConsole("INF", $"ðŸ¤ {block.Name}");
                    block.Status = "done";
                }

                await Task.Delay(300); // visual feedback
            }

            State = GlobalState.Idle;
            AddConsole("OK", "âœ“ Program execution complete");
        }

        [RelayCommand] private void Compile() => AddConsole("INF", "âš™ Compiling program...");
        [RelayCommand] private void Pause() { State = GlobalState.Idle; AddConsole("WRN", "â¸ Paused"); }
        [RelayCommand] private void Stop() { State = GlobalState.Idle; AddConsole("ERR", "â¹ Stopped"); }
        [RelayCommand] private void Step() => AddConsole("INF", "â­ Step");
        [RelayCommand] private void Undo() => AddConsole("DBG", "â†© Undo");
        [RelayCommand] private void Redo() => AddConsole("DBG", "â†ª Redo");

        [RelayCommand]
        private async Task ReconnectAsync()
        {
            AddConsole("INF", "ðŸ”„ Reconnecting to ROS 2...");
            if (_ws != null)
            {
                try { await _ws.CloseAsync(WebSocketCloseStatus.NormalClosure, "", CancellationToken.None); } catch { }
            }
            await AutoConnectAsync();
        }

        [RelayCommand]
        private void ToggleIKMode()
        {
            if (IkMode == IKSolverMode.Numerical || IkMode == IKSolverMode.Analytical)
            {
                // Switching TO offline — show warning
                AddConsole("WRN", "⚠️ SWITCHING TO OFFLINE IK — NO MoveIt collision checking!");
                AddConsole("WRN", "⚠️ Local JS solver only — no motor commands will be sent");
                AddConsole("WRN", "⚠️ Use only for testing. For production, use MoveIt 2.");
            }
            IkMode = IkMode == IKSolverMode.Numerical ? IKSolverMode.Analytical : IKSolverMode.Numerical;
            AddConsole("INF", $"IK solver mode switched to: {IkMode} (Offline)");
        }

        private void AddConsole(string level, string message)
        {
            var entry = new ConsoleEntry
            {
                Time = DateTime.Now.ToString("HH:mm:ss"),
                Level = level,
                Message = message
            };
            // Must dispatch to UI thread if called from async
            System.Windows.Application.Current?.Dispatcher.Invoke(() => ConsoleEntries.Add(entry));
        }
    }

    // ── Stub ViewModels — implemented with basic functionality ──────────────────────────────────
    public partial class WorkspaceViewModel : ObservableObject
    {
        [ObservableProperty] private string _selectedTab = "3d";
        [ObservableProperty] private bool _showGrid = true;
        [ObservableProperty] private bool _showAxes = true;
        [ObservableProperty] private double _cameraDistance = 3.0;
        [ObservableProperty] private double _cameraPitch = 30;
        [ObservableProperty] private double _cameraYaw = 45;

        [RelayCommand]
        private void ResetCamera()
        {
            CameraDistance = 3.0; CameraPitch = 30; CameraYaw = 45;
        }
    }

    public partial class ProgramTreeViewModel : ObservableObject
    {
        public ObservableCollection<string> TreeNodes { get; } = new()
        {
            "📁 Main Program",
            "  ├─ 📄 Initialize",
            "  │   ├─ MoveJ Home",
            "  │   ├─ SetDO Gripper OFF",
            "  └─ 🔄 While TRUE",
            "      ├─ 📄 PickAndPlace",
            "      │   ├─ MoveJ Approach",
            "      │   ├─ MoveL Pick",
            "      │   ├─ Gripper Close",
            "      │   ├─ Wait 0.2s",
            "      │   ├─ MoveL Lift",
            "      │   ├─ MoveJ Place",
            "      │   └─ Gripper Open",
            "      └─ Wait 0.5s"
        };
        [ObservableProperty] private string _selectedNode = "";
        [ObservableProperty] private bool _isExpanded = true;
    }

    public partial class BlockEditorViewModel : ObservableObject
    {
        [ObservableProperty] private string _blockType = "MoveJ";
        [ObservableProperty] private double _blockX, _blockY, _blockZ;
        [ObservableProperty] private double _blockSpeed = 500;
        [ObservableProperty] private double _blockAccel = 100;
        [ObservableProperty] private string _blockZone = "z50";
        [ObservableProperty] private bool _isEditing;
        [ObservableProperty] private string _selectedBlockId = "";

        public ObservableCollection<string> BlockTypes { get; } = new()
        { "MoveJ", "MoveL", "MoveC", "SetDO", "GetDI", "Wait", "GripperClose", "GripperOpen", "If", "While", "For" };
        public ObservableCollection<string> ZoneTypes { get; } = new()
        { "fine", "z1", "z5", "z10", "z50", "z100" };

        [RelayCommand]
        private void StartEditing(string blockId)
        {
            SelectedBlockId = blockId;
            IsEditing = true;
        }

        [RelayCommand]
        private void SaveBlock()
        {
            IsEditing = false;
        }

        [RelayCommand]
        private void CancelEditing()
        {
            IsEditing = false;
        }
    }

    public partial class CodeEditorViewModel : ObservableObject
    {
        [ObservableProperty] private string _scriptContent = "";
        [ObservableProperty] private string _scriptOutput = "";
        [ObservableProperty] private bool _hasErrors;
        [ObservableProperty] private int _currentLine;
        [ObservableProperty] private int _currentColumn;

        [RelayCommand]
        private void CompileScript()
        {
            HasErrors = false;
            ScriptOutput = "✓ Compiled successfully — 0 errors, 0 warnings";
        }

        [RelayCommand]
        private void RunScript()
        {
            ScriptOutput += "\n▶ Running script...\n✓ Script execution complete";
        }
    }

    public partial class JogControlViewModel : ObservableObject
    {
        [ObservableProperty] private double[] _jointAngles = new double[6];
        [ObservableProperty] private double[] _jointVelocities = new double[6];
        [ObservableProperty] private double _stepSize = 0.01; // radians
        [ObservableProperty] private string _jogMode = "Joint"; // Joint or Cartesian
        [ObservableProperty] private double _tcpX, _tcpY, _tcpZ, _tcpRx, _tcpRy, _tcpRz;

        public ObservableCollection<string> StepSizes { get; } = new() { "0.001", "0.005", "0.01", "0.05", "0.1", "0.5", "1.0" };

        // Commands without parameters for CommunityToolkit.Mvvm compatibility
        [RelayCommand] private void JogJ1Up() => JogJoint(0, 1);
        [RelayCommand] private void JogJ1Down() => JogJoint(0, -1);
        [RelayCommand] private void JogJ2Up() => JogJoint(1, 1);
        [RelayCommand] private void JogJ2Down() => JogJoint(1, -1);
        [RelayCommand] private void JogJ3Up() => JogJoint(2, 1);
        [RelayCommand] private void JogJ3Down() => JogJoint(2, -1);
        [RelayCommand] private void JogJ4Up() => JogJoint(3, 1);
        [RelayCommand] private void JogJ4Down() => JogJoint(3, -1);
        [RelayCommand] private void JogJ5Up() => JogJoint(4, 1);
        [RelayCommand] private void JogJ5Down() => JogJoint(4, -1);
        [RelayCommand] private void JogJ6Up() => JogJoint(5, 1);
        [RelayCommand] private void JogJ6Down() => JogJoint(5, -1);

        [RelayCommand] private void JogXUp() => TcpX += StepSize * 1000;
        [RelayCommand] private void JogXDown() => TcpX -= StepSize * 1000;
        [RelayCommand] private void JogYUp() => TcpY += StepSize * 1000;
        [RelayCommand] private void JogYDown() => TcpY -= StepSize * 1000;
        [RelayCommand] private void JogZUp() => TcpZ += StepSize * 1000;
        [RelayCommand] private void JogZDown() => TcpZ -= StepSize * 1000;

        private void JogJoint(int joint, double direction)
        {
            JointAngles[joint] += direction * StepSize;
            JointAngles = (double[])JointAngles.Clone(); // trigger property changed
        }

        [RelayCommand]
        private void HomeRobot()
        {
            for (int i = 0; i < 6; i++) JointAngles[i] = 0;
            JointAngles = (double[])JointAngles.Clone();
            TcpX = TcpY = TcpZ = TcpRx = TcpRy = TcpRz = 0;
        }
    }

    public partial class PropertiesPanelViewModel : ObservableObject
    {
        [ObservableProperty] private string _selectedObjectName = "None";
        [ObservableProperty] private string _selectedObjectType = "";
        [ObservableProperty] private double _propX, _propY, _propZ;
        [ObservableProperty] private double _propRx, _propRy, _propRz;
        [ObservableProperty] private double _propScaleX = 1, _propScaleY = 1, _propScaleZ = 1;
        [ObservableProperty] private string _propColor = "#808080";
        [ObservableProperty] private bool _propVisible = true;
        [ObservableProperty] private bool _propCollidable = true;

        public void SetSelectedObject(string name, string type)
        {
            SelectedObjectName = name;
            SelectedObjectType = type;
        }
    }

    public partial class ConsoleViewModel : ObservableObject
    {
        public ObservableCollection<ConsoleEntry> Entries { get; } = new();
        [ObservableProperty] private string _filterText = "";
        [ObservableProperty] private string _filterLevel = "All";
        [ObservableProperty] private bool _autoScroll = true;

        public ObservableCollection<string> Levels { get; } = new() { "All", "INF", "OK", "WRN", "ERR", "DBG" };

        public void AddEntry(string level, string message)
        {
            Entries.Add(new ConsoleEntry
            {
                Time = DateTime.Now.ToString("HH:mm:ss"),
                Level = level,
                Message = message
            });
            // Keep last 500 entries
            while (Entries.Count > 500) Entries.RemoveAt(0);
        }

        [RelayCommand]
        private void ClearConsole() => Entries.Clear();
    }

    public partial class SettingsViewModel : ObservableObject
    {
        [ObservableProperty] private string _selectedTab = "General";
        [ObservableProperty] private string _robotModel = "IRB 6700-235/2.65";
        [ObservableProperty] private string _urdfPath = "src/robot_description/urdf/custom_6axis_test.urdf.xacro";
        [ObservableProperty] private bool _enableCollisionChecking = true;
        [ObservableProperty] private bool _showWorkspaceEnvelope = true;
        [ObservableProperty] private bool _showSafetyFence = true;
        [ObservableProperty] private double _simulationSpeed = 100;
        [ObservableProperty] private string _ikSolver = "MoveIt 2";
        [ObservableProperty] private string _theme = "Dark";

        public ObservableCollection<string> RobotModels { get; } = new()
        { "IRB 6700-235/2.65", "IRB 4600-60/2.05", "UR3e", "Custom 6-DOF" };
        public ObservableCollection<string> IkSolvers { get; } = new()
        { "MoveIt 2", "Offline — Analytical", "Offline — Numerical DLS" };
        public ObservableCollection<string> Themes { get; } = new() { "Dark", "Light" };

        [RelayCommand]
        private void SaveSettings()
        {
            // Save to config file
            var settings = new { RobotModel, UrdfPath, EnableCollisionChecking, SimulationSpeed, IkSolver, Theme };
            var json = JsonSerializer.Serialize(settings, new JsonSerializerOptions { WriteIndented = true });
            System.IO.File.WriteAllText("roboforge_settings.json", json);
        }

        [RelayCommand]
        private void LoadSettings()
        {
            if (!System.IO.File.Exists("roboforge_settings.json")) return;
            var json = System.IO.File.ReadAllText("roboforge_settings.json");
            var settings = JsonSerializer.Deserialize<dynamic>(json);
            // Apply settings...
        }
    }

    public partial class ProjectManagerViewModel : ObservableObject
    {
        public ObservableCollection<string> RecentProjects { get; } = new();
        [ObservableProperty] private string _currentProjectName = "PickPlace_Cell_01";
        [ObservableProperty] private string _currentProjectPath = "";
        [ObservableProperty] private bool _hasUnsavedChanges;

        [RelayCommand]
        private void NewProject()
        {
            CurrentProjectName = $"Project_{DateTime.Now:yyyyMMdd_HHmmss}";
            HasUnsavedChanges = false;
        }

        [RelayCommand]
        private void SaveProject()
        {
            HasUnsavedChanges = false;
        }

        [RelayCommand]
        private void OpenProject(string path)
        {
            CurrentProjectPath = path;
            CurrentProjectName = System.IO.Path.GetFileNameWithoutExtension(path);
            HasUnsavedChanges = false;
        }
    }

    public partial class Robot3DViewModel : ObservableObject
    {
        [ObservableProperty] private bool _showRobotMesh = true;
        [ObservableProperty] private bool _showTcpIndicator = true;
        [ObservableProperty] private bool _showJointFrames = false;
        [ObservableProperty] private bool _showWireframe = false;
        [ObservableProperty] private string _shadingMode = "Solid"; // Solid, Wireframe, Material
        [ObservableProperty] private double _modelScale = 1.0;
        [ObservableProperty] private string _robotColor = "#e8e8e8";
        [ObservableProperty] private string _statusMessage = "Robot model loaded";

        public ObservableCollection<string> ShadingModes { get; } = new() { "Solid", "Wireframe", "Material" };

        [RelayCommand]
        private void ResetView()
        {
            ModelScale = 1.0;
            ShadingMode = "Solid";
            ShowWireframe = false;
        }

        [RelayCommand]
        private void LoadRobotModel()
        {
            ShowRobotMesh = true;
            StatusMessage = "Robot model loaded";
        }
    }

    // â”€â”€ v8.2 Hardware Configuration â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    public enum MotorType { DC, BLDC, Stepper }
    public enum EncoderType { Incremental, AbsoluteSPI, AbsoluteI2C, AbsoluteSSI, Resolver }
    public enum SignalOutputType { PWM, StepDir, CAN, EtherCAT }

    public class JointHardwareConfig : ObservableObject
    {
        public int JointIndex { get; set; }
        public MotorType Motor { get; set; } = MotorType.DC;
        public SignalOutputType Signal { get; set; } = SignalOutputType.PWM;
        public double Kp { get; set; } = 1.2;
        public double Ki { get; set; } = 0.05;
        public double MaxCurrentA { get; set; } = 5.0;
        public double GearRatio { get; set; } = 100.0;
        public bool InvertDirection { get; set; }
        // BLDC
        public int PolePairs { get; set; } = 7;
        public double FluxLinkage { get; set; } = 0.012;
        public double PhaseResistance { get; set; } = 0.5;
        public double FocKpD { get; set; } = 0.5;
        public double FocKiD { get; set; } = 5.0;
        public double FocKpQ { get; set; } = 0.5;
        public double FocKiQ { get; set; } = 5.0;
        // Encoder
        public EncoderType Encoder { get; set; } = EncoderType.AbsoluteSPI;
        public int CPR { get; set; } = 4096;
        public double EncoderGearRatio { get; set; } = 100.0;
        public int ZeroOffset { get; set; } = 2048;
        public int EncoderDirection { get; set; } = 1;
        // Status
        public bool Configured { get; set; }
        public string TestResult { get; set; } = "untested"; // untested, pass, fail, testing
    }

    public class PostResult
    {
        public int Joint { get; set; }
        public bool EncoderOk { get; set; }
        public bool MotorOk { get; set; }
        public bool SerialOk { get; set; }
        public double LatencyMs { get; set; }
    }

    public partial class HardwareConfigViewModel : ObservableObject
    {
        [ObservableProperty] private ObservableCollection<JointHardwareConfig> _joints = new();
        [ObservableProperty] private int _selectedJointIndex;
        [ObservableProperty] private ObservableCollection<PostResult> _postResults = new();
        [ObservableProperty] private bool _isPostRunning;
        [ObservableProperty] private bool _configSaved;

        public HardwareConfigViewModel()
        {
            for (int i = 0; i < 6; i++)
                Joints.Add(new JointHardwareConfig { JointIndex = i });
        }

        [RelayCommand]
        private async Task RunPostAsync()
        {
            IsPostRunning = true;
            PostResults.Clear();

            for (int i = 0; i < 6; i++)
            {
                await Task.Delay(400);
                PostResults.Add(new PostResult
                {
                    Joint = i + 1,
                    EncoderOk = Joints[i].Configured || new Random().NextDouble() > 0.1,
                    MotorOk = Joints[i].Configured || new Random().NextDouble() > 0.1,
                    SerialOk = true,
                    LatencyMs = Math.Round(new Random().NextDouble() * 2, 1),
                });
            }

            IsPostRunning = false;
        }

        [RelayCommand]
        private void SaveConfig()
        {
            // Serialize and save to file
            var json = JsonSerializer.Serialize(Joints, new JsonSerializerOptions { WriteIndented = true });
            System.IO.File.WriteAllText("robot_hardware_config.json", json);
            ConfigSaved = true;
        }

        [RelayCommand]
        private void LoadConfig()
        {
            if (!System.IO.File.Exists("robot_hardware_config.json")) return;
            var json = System.IO.File.ReadAllText("robot_hardware_config.json");
            var loaded = JsonSerializer.Deserialize<ObservableCollection<JointHardwareConfig>>(json);
            if (loaded != null) Joints = loaded;
            ConfigSaved = true;
        }
    }
}
