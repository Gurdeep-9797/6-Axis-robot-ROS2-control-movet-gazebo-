using System;
using System.Net.WebSockets;
using System.Text;
using System.Text.Json;
using System.Threading;
using System.Threading.Tasks;

namespace RobotSimulator.Core.ROS
{
    /// <summary>
    /// MoveIt client that communicates via the rosbridge WebSocket protocol.
    /// Delegates IK solving, motion planning, and trajectory execution to MoveIt 2.
    /// 
    /// Rosbridge Protocol Reference:
    ///   - call_service: {"op":"call_service","service":"...","args":{...}}
    ///   - publish:      {"op":"publish","topic":"...","msg":{...}}
    /// </summary>
    public class RosMoveItClient
    {
        private ClientWebSocket? _socket;
        private CancellationTokenSource? _cts;
        private bool _connected = false;
        private string _lastError = string.Empty;
        
        // Pending response tracking
        private TaskCompletionSource<JsonElement>? _pendingServiceResponse;
        private readonly SemaphoreSlim _serviceLock = new(1, 1);
        
        public bool IsConnected => _connected && _socket?.State == WebSocketState.Open;
        public string LastError => _lastError;
        
        public event Action<string>? OnLog;
        public event Action<double[]>? OnJointStateReceived;

        /// <summary>
        /// Connect to rosbridge WebSocket server.
        /// </summary>
        public async Task<bool> ConnectAsync(string uri, CancellationToken ct = default)
        {
            try
            {
                _socket = new ClientWebSocket();
                _cts = CancellationTokenSource.CreateLinkedTokenSource(ct);
                await _socket.ConnectAsync(new Uri(uri), _cts.Token);
                _connected = true;
                
                // Subscribe to /joint_states for feedback
                await SubscribeAsync("/joint_states", "sensor_msgs/msg/JointState");
                
                // Start receive loop
                _ = ReceiveLoopAsync();
                
                Log("Connected to rosbridge");
                return true;
            }
            catch (Exception ex)
            {
                _lastError = ex.Message;
                Log($"Connection failed: {ex.Message}");
                return false;
            }
        }
        
        /// <summary>
        /// Disconnect from rosbridge.
        /// </summary>
        public void Disconnect()
        {
            _cts?.Cancel();
            _socket?.Dispose();
            _socket = null;
            _connected = false;
            Log("Disconnected from rosbridge");
        }

        /// <summary>
        /// Request IK solution from MoveIt's /compute_ik service.
        /// Returns joint angles if successful, null otherwise.
        /// </summary>
        public async Task<double[]?> ComputeIKAsync(
            double x, double y, double z,
            double qx, double qy, double qz, double qw,
            double[] seedState)
        {
            if (!IsConnected) return null;
            
            var args = new
            {
                ik_request = new
                {
                    group_name = "robot_arm",
                    robot_state = new
                    {
                        joint_state = new
                        {
                            name = new[] { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" },
                            position = seedState
                        }
                    },
                    pose_stamped = new
                    {
                        header = new { frame_id = "base_link" },
                        pose = new
                        {
                            position = new { x, y, z },
                            orientation = new { x = qx, y = qy, z = qz, w = qw }
                        }
                    },
                    timeout = new { sec = 5, nanosec = 0 },
                    attempts = 10
                }
            };
            
            Log("Sending IK request to MoveIt...");
            var response = await CallServiceAsync("/compute_ik", "moveit_msgs/srv/GetPositionIK", args);
            
            if (response == null)
            {
                Log("IK request timed out");
                return null;
            }
            
            try
            {
                // Parse response: {error_code: {val: 1}, solution: {joint_state: {position: [...]}}}
                var errorCode = response.Value.GetProperty("error_code").GetProperty("val").GetInt32();
                if (errorCode != 1) // MoveIt SUCCESS = 1
                {
                    Log($"IK failed with error code: {errorCode}");
                    return null;
                }
                
                var positions = response.Value
                    .GetProperty("solution")
                    .GetProperty("joint_state")
                    .GetProperty("position");
                
                var result = new double[6];
                int i = 0;
                foreach (var p in positions.EnumerateArray())
                {
                    if (i < 6) result[i++] = p.GetDouble();
                }
                
                Log($"IK solved: [{string.Join(", ", Array.ConvertAll(result, a => $"{a * 180.0 / Math.PI:F1}°"))}]");
                return result;
            }
            catch (Exception ex)
            {
                Log($"IK response parse error: {ex.Message}");
                return null;
            }
        }

        /// <summary>
        /// Request motion plan from MoveIt, targeting a joint-space goal.
        /// Returns trajectory points if successful, null otherwise.
        /// </summary>
        public async Task<double[][]?> PlanToJointGoalAsync(double[] currentJoints, double[] goalJoints)
        {
            if (!IsConnected) return null;
            
            var jointNames = new[] { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" };
            
            // Build joint constraints for the goal
            var jointConstraints = new object[6];
            for (int i = 0; i < 6; i++)
            {
                jointConstraints[i] = new
                {
                    joint_name = jointNames[i],
                    position = goalJoints[i],
                    tolerance_above = 0.01,
                    tolerance_below = 0.01,
                    weight = 1.0
                };
            }
            
            var args = new
            {
                motion_plan_request = new
                {
                    workspace_parameters = new
                    {
                        header = new { frame_id = "base_link" },
                        min_corner = new { x = -1.0, y = -1.0, z = -1.0 },
                        max_corner = new { x = 1.0, y = 1.0, z = 1.0 }
                    },
                    start_state = new
                    {
                        joint_state = new
                        {
                            name = jointNames,
                            position = currentJoints
                        }
                    },
                    goal_constraints = new[]
                    {
                        new { joint_constraints = jointConstraints }
                    },
                    group_name = "robot_arm",
                    num_planning_attempts = 5,
                    allowed_planning_time = 5.0,
                    max_velocity_scaling_factor = 0.5,
                    max_acceleration_scaling_factor = 0.5,
                    pipeline_id = "ompl",
                    planner_id = "RRTConnect"
                }
            };
            
            Log("Sending Plan request to MoveIt (RRTConnect)...");
            var response = await CallServiceAsync("/plan_kinematic_path", "moveit_msgs/srv/GetMotionPlan", args);
            
            if (response == null)
            {
                Log("Plan request timed out");
                return null;
            }
            
            try
            {
                var errorCode = response.Value
                    .GetProperty("motion_plan_response")
                    .GetProperty("error_code")
                    .GetProperty("val")
                    .GetInt32();
                    
                if (errorCode != 1)
                {
                    Log($"Planning failed with error code: {errorCode}");
                    return null;
                }
                
                var points = response.Value
                    .GetProperty("motion_plan_response")
                    .GetProperty("trajectory")
                    .GetProperty("joint_trajectory")
                    .GetProperty("points");
                
                var trajectory = new System.Collections.Generic.List<double[]>();
                foreach (var pt in points.EnumerateArray())
                {
                    var positions = pt.GetProperty("positions");
                    var jointPos = new double[6];
                    int i = 0;
                    foreach (var p in positions.EnumerateArray())
                    {
                        if (i < 6) jointPos[i++] = p.GetDouble();
                    }
                    trajectory.Add(jointPos);
                }
                
                Log($"Plan received: {trajectory.Count} waypoints");
                return trajectory.ToArray();
            }
            catch (Exception ex)
            {
                Log($"Plan response parse error: {ex.Message}");
                return null;
            }
        }

        /// <summary>
        /// Execute a trajectory by publishing it to /planned_trajectory.
        /// The HardwareBridgeNode will relay it to the controller.
        /// </summary>
        public async Task ExecuteTrajectoryAsync(double[][] trajectory)
        {
            if (!IsConnected || trajectory == null || trajectory.Length == 0) return;
            
            var jointNames = new[] { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" };
            
            // Build trajectory points with time_from_start
            var points = new object[trajectory.Length];
            for (int i = 0; i < trajectory.Length; i++)
            {
                double timeSec = (i + 1) * 0.02; // 50Hz control rate
                int sec = (int)timeSec;
                int nanosec = (int)((timeSec - sec) * 1e9);
                
                points[i] = new
                {
                    positions = trajectory[i],
                    velocities = new double[6],
                    time_from_start = new { sec, nanosec }
                };
            }
            
            var msg = new
            {
                op = "publish",
                topic = "/planned_trajectory",
                msg = new
                {
                    header = new
                    {
                        stamp = new { sec = DateTimeOffset.UtcNow.ToUnixTimeSeconds(), nanosec = 0 },
                        frame_id = "base_link"
                    },
                    joint_names = jointNames,
                    points
                }
            };
            
            await SendAsync(msg);
            Log($"Published trajectory with {trajectory.Length} points to /planned_trajectory");
        }
        
        /// <summary>
        /// Convert Euler angles (roll, pitch, yaw) to quaternion for MoveIt pose goals.
        /// </summary>
        public static (double qx, double qy, double qz, double qw) EulerToQuaternion(
            double roll, double pitch, double yaw)
        {
            double cy = Math.Cos(yaw * 0.5);
            double sy = Math.Sin(yaw * 0.5);
            double cp = Math.Cos(pitch * 0.5);
            double sp = Math.Sin(pitch * 0.5);
            double cr = Math.Cos(roll * 0.5);
            double sr = Math.Sin(roll * 0.5);

            double qw = cr * cp * cy + sr * sp * sy;
            double qx = sr * cp * cy - cr * sp * sy;
            double qy = cr * sp * cy + sr * cp * sy;
            double qz = cr * cp * sy - sr * sp * cy;

            return (qx, qy, qz, qw);
        }

        #region rosbridge Protocol
        
        private async Task<JsonElement?> CallServiceAsync(string service, string type, object args)
        {
            await _serviceLock.WaitAsync();
            try
            {
                _pendingServiceResponse = new TaskCompletionSource<JsonElement>();
                
                var msg = new
                {
                    op = "call_service",
                    service,
                    type,
                    args
                };
                
                await SendAsync(msg);
                
                // Wait for response with timeout
                using var timeoutCts = new CancellationTokenSource(TimeSpan.FromSeconds(10));
                timeoutCts.Token.Register(() => _pendingServiceResponse?.TrySetCanceled());
                
                try
                {
                    return await _pendingServiceResponse.Task;
                }
                catch (OperationCanceledException)
                {
                    return null;
                }
            }
            finally
            {
                _pendingServiceResponse = null;
                _serviceLock.Release();
            }
        }
        
        private async Task SubscribeAsync(string topic, string type)
        {
            var msg = new { op = "subscribe", topic, type };
            await SendAsync(msg);
        }
        
        private async Task SendAsync(object message)
        {
            if (_socket?.State != WebSocketState.Open) return;
            
            var json = JsonSerializer.Serialize(message);
            var bytes = Encoding.UTF8.GetBytes(json);
            await _socket.SendAsync(bytes, WebSocketMessageType.Text, true, 
                _cts?.Token ?? CancellationToken.None);
        }
        
        private async Task ReceiveLoopAsync()
        {
            var buffer = new byte[65536]; // 64KB for large trajectory responses
            
            while (_socket?.State == WebSocketState.Open)
            {
                try
                {
                    var result = await _socket.ReceiveAsync(buffer, _cts?.Token ?? CancellationToken.None);
                    if (result.MessageType == WebSocketMessageType.Text)
                    {
                        var json = Encoding.UTF8.GetString(buffer, 0, result.Count);
                        ProcessMessage(json);
                    }
                }
                catch (OperationCanceledException) { break; }
                catch { /* ignore */ }
            }
        }
        
        private void ProcessMessage(string json)
        {
            try
            {
                using var doc = JsonDocument.Parse(json);
                var root = doc.RootElement;
                var op = root.GetProperty("op").GetString();
                
                if (op == "service_response")
                {
                    // Service call response
                    if (root.TryGetProperty("values", out var values))
                    {
                        _pendingServiceResponse?.TrySetResult(values.Clone());
                    }
                }
                else if (op == "publish")
                {
                    // Topic message
                    if (root.TryGetProperty("topic", out var topic) && topic.GetString() == "/joint_states")
                    {
                        if (root.TryGetProperty("msg", out var msg) && msg.TryGetProperty("position", out var pos))
                        {
                            var positions = new double[6];
                            int i = 0;
                            foreach (var p in pos.EnumerateArray())
                            {
                                if (i < 6) positions[i++] = p.GetDouble();
                            }
                            OnJointStateReceived?.Invoke(positions);
                        }
                    }
                }
            }
            catch { /* ignore parse errors */ }
        }
        
        #endregion
        
        private void Log(string message)
        {
            OnLog?.Invoke($"[MoveIt] {message}");
            App.LogMessage($"[MoveIt] {message}");
        }
    }
}
