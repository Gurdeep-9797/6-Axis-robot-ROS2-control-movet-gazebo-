using System;
using System.IO;
using System.Net;
using System.Text;
using System.Text.Json;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;

namespace RobotSimulator.Core.Api
{
    /// <summary>
    /// Lightweight embedded HTTP API server for AI/CLI control.
    /// Runs on http://localhost:8085/api/ inside the WPF application.
    /// </summary>
    public class ApiServer
    {
        private readonly HttpListener _listener;
        private CancellationTokenSource? _cts;
        private readonly int _port;
        
        // Delegates for dispatching commands to MainWindow (UI thread)
        public Func<double[]>? GetJointAngles { get; set; }
        public Func<(double x, double y, double z)>? GetTcpPosition { get; set; }
        public Func<(double r, double p, double y)>? GetTcpOrientation { get; set; }
        public Func<bool>? GetRosConnected { get; set; }
        public Func<string>? GetStatusText { get; set; }
        
        public Action<double[]>? SetJointAngles { get; set; }
        public Action? HomeRobot { get; set; }
        public Action? AddWaypoint { get; set; }
        public Action? ClearProgram { get; set; }
        public Action? RunProgram { get; set; }
        public Action? EmergencyStop { get; set; }
        public Action? ConnectRos { get; set; }
        public Func<string[]>? GetProgramPoints { get; set; }
        
        public ApiServer(int port = 8085)
        {
            _port = port;
            _listener = new HttpListener();
            _listener.Prefixes.Add($"http://localhost:{port}/api/");
        }
        
        public void Start()
        {
            _cts = new CancellationTokenSource();
            _listener.Start();
            _ = Task.Run(() => ListenLoop(_cts.Token));
        }
        
        public void Stop()
        {
            _cts?.Cancel();
            if (_listener.IsListening) _listener.Stop();
        }
        
        private async Task ListenLoop(CancellationToken ct)
        {
            while (!ct.IsCancellationRequested && _listener.IsListening)
            {
                try
                {
                    var ctx = await _listener.GetContextAsync();
                    _ = Task.Run(() => HandleRequest(ctx), ct);
                }
                catch (Exception) when (ct.IsCancellationRequested)
                {
                    break;
                }
                catch (ObjectDisposedException)
                {
                    break;
                }
            }
        }
        
        private void HandleRequest(HttpListenerContext ctx)
        {
            var req = ctx.Request;
            var resp = ctx.Response;
            
            // CORS headers for browser-based AI tools
            resp.AppendHeader("Access-Control-Allow-Origin", "*");
            resp.AppendHeader("Access-Control-Allow-Methods", "GET, POST, DELETE, OPTIONS");
            resp.AppendHeader("Access-Control-Allow-Headers", "Content-Type");
            
            if (req.HttpMethod == "OPTIONS")
            {
                resp.StatusCode = 200;
                resp.Close();
                return;
            }
            
            try
            {
                var path = req.Url?.AbsolutePath?.ToLower() ?? "";
                var method = req.HttpMethod;
                
                object? result = null;
                int statusCode = 200;
                
                switch (path)
                {
                    case "/api/status":
                        result = HandleStatus();
                        break;
                        
                    case "/api/joints":
                        if (method == "POST")
                            result = HandleSetJoints(req);
                        else
                            result = HandleGetJoints();
                        break;
                        
                    case "/api/home":
                        if (method == "POST")
                            result = HandleHome();
                        else
                            statusCode = 405;
                        break;
                        
                    case "/api/teach":
                        if (method == "POST")
                            result = HandleTeach();
                        else
                            statusCode = 405;
                        break;
                        
                    case "/api/run":
                        if (method == "POST")
                            result = HandleRun();
                        else
                            statusCode = 405;
                        break;
                        
                    case "/api/estop":
                        if (method == "POST")
                            result = HandleEStop();
                        else
                            statusCode = 405;
                        break;
                        
                    case "/api/program":
                        if (method == "DELETE")
                            result = HandleClearProgram();
                        else
                            result = HandleGetProgram();
                        break;
                        
                    case "/api/connect/ros":
                        if (method == "POST")
                            result = HandleConnectRos();
                        else
                            statusCode = 405;
                        break;
                        
                    default:
                        statusCode = 404;
                        result = new { error = "Not found", endpoints = new[] {
                            "GET  /api/status",
                            "GET  /api/joints",
                            "POST /api/joints {j1,j2,j3,j4,j5,j6}",
                            "POST /api/home",
                            "POST /api/teach",
                            "POST /api/run",
                            "POST /api/estop",
                            "GET  /api/program",
                            "DELETE /api/program",
                            "POST /api/connect/ros"
                        }};
                        break;
                }
                
                resp.StatusCode = statusCode;
                resp.ContentType = "application/json";
                var json = JsonSerializer.Serialize(result ?? new { ok = true }, new JsonSerializerOptions { WriteIndented = true });
                var buf = Encoding.UTF8.GetBytes(json);
                resp.OutputStream.Write(buf, 0, buf.Length);
            }
            catch (Exception ex)
            {
                resp.StatusCode = 500;
                var err = Encoding.UTF8.GetBytes(JsonSerializer.Serialize(new { error = ex.Message }));
                resp.OutputStream.Write(err, 0, err.Length);
            }
            finally
            {
                resp.Close();
            }
        }
        
        // --- Handlers ---
        
        private object HandleStatus()
        {
            var angles = GetJointAngles?.Invoke() ?? new double[6];
            var pos = GetTcpPosition?.Invoke() ?? (0.0, 0.0, 0.0);
            var ori = GetTcpOrientation?.Invoke() ?? (0.0, 0.0, 0.0);
            return new
            {
                ros_connected = GetRosConnected?.Invoke() ?? false,
                status = GetStatusText?.Invoke() ?? "Unknown",
                joints_deg = new { j1 = angles[0] * 180 / Math.PI, j2 = angles[1] * 180 / Math.PI, j3 = angles[2] * 180 / Math.PI, j4 = angles[3] * 180 / Math.PI, j5 = angles[4] * 180 / Math.PI, j6 = angles[5] * 180 / Math.PI },
                tcp_mm = new { x = pos.x * 1000, y = pos.y * 1000, z = pos.z * 1000 },
                tcp_deg = new { roll = ori.r * 180 / Math.PI, pitch = ori.p * 180 / Math.PI, yaw = ori.y * 180 / Math.PI },
                api_port = _port
            };
        }
        
        private object HandleGetJoints()
        {
            var angles = GetJointAngles?.Invoke() ?? new double[6];
            return new { j1 = angles[0] * 180 / Math.PI, j2 = angles[1] * 180 / Math.PI, j3 = angles[2] * 180 / Math.PI, j4 = angles[3] * 180 / Math.PI, j5 = angles[4] * 180 / Math.PI, j6 = angles[5] * 180 / Math.PI };
        }
        
        private object HandleSetJoints(HttpListenerRequest req)
        {
            using var reader = new StreamReader(req.InputStream);
            var body = reader.ReadToEnd();
            var doc = JsonDocument.Parse(body);
            var root = doc.RootElement;
            
            var angles = new double[6];
            if (root.TryGetProperty("j1", out var v1)) angles[0] = v1.GetDouble() * Math.PI / 180;
            if (root.TryGetProperty("j2", out var v2)) angles[1] = v2.GetDouble() * Math.PI / 180;
            if (root.TryGetProperty("j3", out var v3)) angles[2] = v3.GetDouble() * Math.PI / 180;
            if (root.TryGetProperty("j4", out var v4)) angles[3] = v4.GetDouble() * Math.PI / 180;
            if (root.TryGetProperty("j5", out var v5)) angles[4] = v5.GetDouble() * Math.PI / 180;
            if (root.TryGetProperty("j6", out var v6)) angles[5] = v6.GetDouble() * Math.PI / 180;
            
            Application.Current.Dispatcher.Invoke(() => SetJointAngles?.Invoke(angles));
            return new { ok = true, moved_to_deg = new { j1 = angles[0] * 180 / Math.PI, j2 = angles[1] * 180 / Math.PI, j3 = angles[2] * 180 / Math.PI, j4 = angles[3] * 180 / Math.PI, j5 = angles[4] * 180 / Math.PI, j6 = angles[5] * 180 / Math.PI } };
        }
        
        private object HandleHome()
        {
            Application.Current.Dispatcher.Invoke(() => HomeRobot?.Invoke());
            return new { ok = true, action = "homed" };
        }
        
        private object HandleTeach()
        {
            Application.Current.Dispatcher.Invoke(() => AddWaypoint?.Invoke());
            return new { ok = true, action = "waypoint_added" };
        }
        
        private object HandleRun()
        {
            Application.Current.Dispatcher.Invoke(() => RunProgram?.Invoke());
            return new { ok = true, action = "program_started" };
        }
        
        private object HandleEStop()
        {
            Application.Current.Dispatcher.Invoke(() => EmergencyStop?.Invoke());
            return new { ok = true, action = "emergency_stop" };
        }
        
        private object HandleGetProgram()
        {
            var points = GetProgramPoints?.Invoke() ?? Array.Empty<string>();
            return new { count = points.Length, points };
        }
        
        private object HandleClearProgram()
        {
            Application.Current.Dispatcher.Invoke(() => ClearProgram?.Invoke());
            return new { ok = true, action = "program_cleared" };
        }
        
        private object HandleConnectRos()
        {
            Application.Current.Dispatcher.Invoke(() => ConnectRos?.Invoke());
            return new { ok = true, action = "ros_connect_requested" };
        }
    }
}
