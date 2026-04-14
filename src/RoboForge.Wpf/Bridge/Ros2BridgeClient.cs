// ── ROS2 Bridge: WebSocket Client for Gazebo/MoveIt Integration ──────────
using System;
using System.Collections.Generic;
using System.Net.WebSockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using Newtonsoft.Json;
using RoboForge.Wpf.Core;

namespace RoboForge.Wpf.Bridge
{
    /// <summary>
    /// ROS2 Bridge WebSocket Client
    /// Connects to roboforge_bridge (ws://localhost:9090) to:
    /// - Subscribe to /joint_states from Gazebo simulator
    /// - Call /compute_ik and /compute_fk on MoveIt
    /// - Publish trajectory commands to /joint_trajectory_command
    /// - Monitor execution state
    /// 
    /// This ensures the 3D viewport reflects actual Gazebo simulation output,
    /// not fake/random angles. All kinematics are computed by MoveIt, not locally.
    /// </summary>
    public class Ros2BridgeClient : IDisposable
    {
        private ClientWebSocket? _webSocket;
        private readonly string _bridgeUrl;
        private bool _isConnected;
        private readonly Dictionary<string, Action<string>> _serviceCallbacks = new();
        private int _messageId = 0;
        private CancellationTokenSource? _receiveCts;
        
        // Subscribers for ROS2 topics
        public event Action<double[]>? JointStatesReceived;
        public event Action<string, bool>? IoStateReceived;
        public event Action<ProgramState>? ExecutionStateChanged;

        public Ros2BridgeClient(string bridgeUrl = "ws://localhost:9090")
        {
            _bridgeUrl = bridgeUrl;
        }

        public bool IsConnected => _isConnected && _webSocket?.State == WebSocketState.Open;

        /// <summary>
        /// Connect to the ROS2 bridge WebSocket
        /// </summary>
        public async Task<bool> ConnectAsync(CancellationToken ct = default)
        {
            try
            {
                _webSocket = new ClientWebSocket();
                _webSocket.Options.KeepAliveInterval = TimeSpan.FromSeconds(30);
                
                await _webSocket.ConnectAsync(new Uri(_bridgeUrl), ct);
                _isConnected = true;
                
                // Subscribe to joint states
                await SubscribeTopicAsync("/joint_states", ct);
                
                // Start receive loop
                _receiveCts = new CancellationTokenSource();
                _ = Task.Run(() => ReceiveLoop(_receiveCts.Token), ct);
                
                return true;
            }
            catch
            {
                _isConnected = false;
                return false;
            }
        }

        /// <summary>
        /// Disconnect from the bridge
        /// </summary>
        public async Task DisconnectAsync()
        {
            _receiveCts?.Cancel();
            
            if (_webSocket != null && _webSocket.State == WebSocketState.Open)
            {
                await _webSocket.CloseAsync(WebSocketCloseStatus.NormalClosure, "Disconnecting", CancellationToken.None);
            }
            
            _isConnected = false;
            _webSocket?.Dispose();
            _webSocket = null;
        }

        /// <summary>
        /// Subscribe to a ROS2 topic via rosbridge
        /// </summary>
        private async Task SubscribeTopicAsync(string topic, CancellationToken ct)
        {
            var subscribeMsg = new
            {
                op = "subscribe",
                topic = topic,
                type = GetTopicType(topic)
            };
            
            await SendJsonAsync(subscribeMsg, ct);
        }

        private string GetTopicType(string topic)
        {
            return topic switch
            {
                "/joint_states" => "sensor_msgs/msg/JointState",
                "/planned_trajectory" => "trajectory_msgs/msg/JointTrajectory",
                _ => "std_msgs/msg/String"
            };
        }

        /// <summary>
        /// Receive loop: parse incoming WebSocket messages
        /// </summary>
        private async Task ReceiveLoop(CancellationToken ct)
        {
            var buffer = new byte[4096];
            
            try
            {
                while (!ct.IsCancellationRequested && _webSocket?.State == WebSocketState.Open)
                {
                    var result = await _webSocket.ReceiveAsync(new ArraySegment<byte>(buffer), ct);
                    
                    if (result.MessageType == WebSocketMessageType.Text)
                    {
                        var message = Encoding.UTF8.GetString(buffer, 0, result.Count);
                        await ProcessMessageAsync(message, ct);
                    }
                }
            }
            catch (OperationCanceledException)
            {
                // Expected on disconnect
            }
            catch
            {
                _isConnected = false;
            }
        }

        /// <summary>
        /// Process incoming ROS2 bridge messages
        /// </summary>
        private async Task ProcessMessageAsync(string json, CancellationToken ct)
        {
            try
            {
                using var doc = System.Text.Json.JsonDocument.Parse(json);
                var root = doc.RootElement;
                
                if (!root.TryGetProperty("op", out var opElement))
                    return;
                
                var op = opElement.GetString();
                
                switch (op)
                {
                    case "publish":
                        await HandlePublishAsync(root, ct);
                        break;
                    
                    case "service_response":
                        await HandleServiceResponseAsync(root, ct);
                        break;
                    
                    case "status":
                        await HandleStatusAsync(root, ct);
                        break;
                }
            }
            catch
            {
                // Ignore malformed messages
            }
        }

        /// <summary>
        /// Handle topic publications (joint states, IO, etc.)
        /// </summary>
        private async Task HandlePublishAsync(System.Text.Json.JsonElement root, CancellationToken ct)
        {
            if (!root.TryGetProperty("topic", out var topicElement))
                return;
            
            var topic = topicElement.GetString();
            
            if (topic == "/joint_states" && root.TryGetProperty("msg", out var jointMsgElement))
            {
                // Parse joint states from Gazebo
                var jointStates = ParseJointStates(jointMsgElement);
                JointStatesReceived?.Invoke(jointStates);
            }
            else if (topic != null && topic.StartsWith("/io/"))
            {
                // Parse IO state
                var pinName = topic.Substring(4);
                if (root.TryGetProperty("msg", out var ioMsgElement) && ioMsgElement.TryGetProperty("data", out var dataElement))
                {
                    var value = dataElement.GetBoolean();
                    IoStateReceived?.Invoke(pinName, value);
                }
            }
        }

        /// <summary>
        /// Parse joint states from Gazebo's sensor_msgs/JointState message
        /// </summary>
        private double[] ParseJointStates(System.Text.Json.JsonElement msgElement)
        {
            var angles = new double[6];
            
            if (msgElement.TryGetProperty("position", out var posArray))
            {
                for (int i = 0; i < Math.Min(6, posArray.GetArrayLength()); i++)
                {
                    angles[i] = posArray[i].GetDouble();
                }
            }
            
            return angles;
        }

        /// <summary>
        /// Handle service responses (IK, FK, health check)
        /// </summary>
        private async Task HandleServiceResponseAsync(System.Text.Json.JsonElement root, CancellationToken ct)
        {
            if (!root.TryGetProperty("id", out var idElement))
                return;
            
            var id = idElement.GetString();
            if (string.IsNullOrEmpty(id))
                return;
            
            if (_serviceCallbacks.TryGetValue(id, out var callback))
            {
                callback(root.GetRawText());
                _serviceCallbacks.Remove(id);
            }
        }

        /// <summary>
        /// Handle status messages
        /// </summary>
        private async Task HandleStatusAsync(System.Text.Json.JsonElement root, CancellationToken ct)
        {
            if (root.TryGetProperty("msg", out var msgElement) && msgElement.TryGetProperty("level", out var levelElement))
            {
                var level = levelElement.GetInt32();
                // Level 0 = OK, 1 = Warning, 2 = Error
            }
        }

        /// <summary>
        /// Call a ROS2 service via rosbridge
        /// </summary>
        public async Task<string?> CallServiceAsync(string service, object request, CancellationToken ct = default)
        {
            if (_webSocket?.State != WebSocketState.Open)
                return null;
            
            var id = $"svc_{++_messageId}";
            var tcs = new TaskCompletionSource<string?>();
            
            _serviceCallbacks[id] = (json) => tcs.TrySetResult(json);
            
            var message = new
            {
                op = "call_service",
                service = service,
                id = id,
                args = request
            };
            
            await SendJsonAsync(message, ct);
            
            // Wait for response with timeout
            using var timeoutCts = new CancellationTokenSource(TimeSpan.FromSeconds(5));
            using var linkedCts = CancellationTokenSource.CreateLinkedTokenSource(ct, timeoutCts.Token);
            
            try
            {
                return await tcs.Task.WaitAsync(linkedCts.Token);
            }
            catch
            {
                _serviceCallbacks.Remove(id);
                return null;
            }
        }

        /// <summary>
        /// Compute IK via MoveIt /compute_ik service
        /// </summary>
        public async Task<double[]?> ComputeIkAsync(
            double x, double y, double z,
            double qx, double qy, double qz, double qw,
            string groupName = "robot_arm",
            CancellationToken ct = default)
        {
            var request = new
            {
                ik_request = new
                {
                    group_name = groupName,
                    avoid_collisions = true,
                    pose_stamped = new
                    {
                        header = new { frame_id = "base_link" },
                        pose = new
                        {
                            position = new { x, y, z },
                            orientation = new { x = qx, y = qy, z = qz, w = qw }
                        }
                    },
                    timeout = new { sec = 5, nanosec = 0 }
                }
            };
            
            var response = await CallServiceAsync("/compute_ik", request, ct);
            if (response == null)
                return null;
            
            try
            {
                using var doc = System.Text.Json.JsonDocument.Parse(response);
                var root = doc.RootElement;
                
                // Check error code
                if (root.TryGetProperty("values", out var valuesElement) &&
                    valuesElement.TryGetProperty("error_code", out var errorCodeElement) &&
                    errorCodeElement.TryGetProperty("val", out var valElement))
                {
                    var errorCode = valElement.GetInt32();
                    if (errorCode != 1) // 1 = SUCCESS
                        return null;
                }
                
                // Extract joint positions
                if (root.TryGetProperty("values", out var valuesEl) &&
                    valuesEl.TryGetProperty("joint_state", out var jointStateEl) &&
                    jointStateEl.TryGetProperty("position", out var posArray))
                {
                    var angles = new double[posArray.GetArrayLength()];
                    for (int i = 0; i < angles.Length; i++)
                    {
                        angles[i] = posArray[i].GetDouble();
                    }
                    return angles;
                }
            }
            catch
            {
                return null;
            }
            
            return null;
        }

        /// <summary>
        /// Compute FK via MoveIt /compute_fk service
        /// </summary>
        public async Task<(double x, double y, double z, double qw, double qx, double qy, double qz)?> ComputeFkAsync(
            double[] jointAngles,
            string groupName = "robot_arm",
            CancellationToken ct = default)
        {
            var request = new
            {
                fk_request = new
                {
                    group_name = groupName,
                    joint_state = new
                    {
                        position = jointAngles
                    }
                }
            };
            
            var response = await CallServiceAsync("/compute_fk", request, ct);
            if (response == null)
                return null;
            
            try
            {
                using var doc = System.Text.Json.JsonDocument.Parse(response);
                var root = doc.RootElement;
                
                if (root.TryGetProperty("values", out var valuesEl) &&
                    valuesEl.TryGetProperty("pose_stamped", out var poseEl) &&
                    poseEl.TryGetProperty("pose", out var poseData))
                {
                    var pos = poseData.GetProperty("position");
                    var orient = poseData.GetProperty("orientation");
                    
                    return (
                        pos.GetProperty("x").GetDouble(),
                        pos.GetProperty("y").GetDouble(),
                        pos.GetProperty("z").GetDouble(),
                        orient.GetProperty("w").GetDouble(),
                        orient.GetProperty("x").GetDouble(),
                        orient.GetProperty("y").GetDouble(),
                        orient.GetProperty("z").GetDouble()
                    );
                }
            }
            catch
            {
                return null;
            }
            
            return null;
        }

        /// <summary>
        /// Publish trajectory command to ROS2
        /// </summary>
        public async Task PublishTrajectoryAsync(
            double[][] waypoints,
            double[] durations,
            CancellationToken ct = default)
        {
            var points = new List<object>();
            double timeAccum = 0;
            
            for (int i = 0; i < waypoints.Length; i++)
            {
                timeAccum += durations[i];
                points.Add(new
                {
                    positions = waypoints[i],
                    velocities = new double[waypoints[i].Length],
                    time_from_start = new { sec = (int)timeAccum, nanosec = 0 }
                });
            }
            
            var message = new
            {
                op = "publish",
                topic = "/joint_trajectory_command",
                msg = new
                {
                    header = new { frame_id = "base_link" },
                    joint_names = new[] { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" },
                    points = points.ToArray()
                }
            };
            
            await SendJsonAsync(message, ct);
        }

        /// <summary>
        /// Send JSON message via WebSocket
        /// </summary>
        private async Task SendJsonAsync(object message, CancellationToken ct)
        {
            if (_webSocket?.State != WebSocketState.Open)
                return;
            
            var json = JsonConvert.SerializeObject(message);
            var bytes = Encoding.UTF8.GetBytes(json);
            await _webSocket.SendAsync(new ArraySegment<byte>(bytes), WebSocketMessageType.Text, true, ct);
        }

        public void Dispose()
        {
            DisconnectAsync().Wait();
        }
    }
}
