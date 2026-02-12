using System;
using System.Net.WebSockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

namespace RobotSimulator.Core.Communication
{
    /// <summary>
    /// ROS Bridge WebSocket client for ROS 2 communication.
    /// Connects to rosbridge_server running in ROS environment.
    /// </summary>
    public class RosBridgeClient : IDisposable
    {
        private ClientWebSocket? _socket;
        private readonly string _serverUri;
        private CancellationTokenSource? _cts;
        private bool _connected;
        
        public event Action<JointStateMessage>? OnJointStateReceived;
        public event Action<string>? OnStatusChanged;
        public event Action<string>? OnError;
        
        public bool IsConnected => _connected;

        public RosBridgeClient(string serverUri = "ws://localhost:9090")
        {
            _serverUri = serverUri;
        }

        public async Task ConnectAsync()
        {
            try
            {
                _socket = new ClientWebSocket();
                _cts = new CancellationTokenSource();
                
                OnStatusChanged?.Invoke($"Connecting to {_serverUri}...");
                
                await _socket.ConnectAsync(new Uri(_serverUri), _cts.Token);
                _connected = true;
                
                OnStatusChanged?.Invoke("Connected to ROS Bridge");
                
                // Start receiving messages
                _ = ReceiveLoopAsync();
                
                // Subscribe to joint states
                await SubscribeAsync("/joint_states", "sensor_msgs/msg/JointState");
            }
            catch (Exception ex)
            {
                _connected = false;
                OnError?.Invoke($"Connection failed: {ex.Message}");
            }
        }

        public async Task DisconnectAsync()
        {
            if (_socket != null && _connected)
            {
                _cts?.Cancel();
                await _socket.CloseAsync(WebSocketCloseStatus.NormalClosure, "Closing", CancellationToken.None);
                _connected = false;
                OnStatusChanged?.Invoke("Disconnected");
            }
        }

        public async Task SubscribeAsync(string topic, string messageType)
        {
            var msg = new
            {
                op = "subscribe",
                topic = topic,
                type = messageType
            };
            await SendAsync(JsonConvert.SerializeObject(msg));
        }

        public async Task PublishJointCommandAsync(string[] jointNames, double[] positions)
        {
            var msg = new
            {
                op = "publish",
                topic = "/joint_command",
                msg = new
                {
                    name = jointNames,
                    position = positions
                }
            };
            await SendAsync(JsonConvert.SerializeObject(msg));
        }

        public async Task PublishTrajectoryAsync(string[] jointNames, double[][] positions, double[] timeFromStart)
        {
            var points = new object[positions.Length];
            for (int i = 0; i < positions.Length; i++)
            {
                points[i] = new
                {
                    positions = positions[i],
                    time_from_start = new { sec = (int)timeFromStart[i], nanosec = (int)((timeFromStart[i] % 1) * 1e9) }
                };
            }

            var msg = new
            {
                op = "publish",
                topic = "/joint_trajectory",
                msg = new
                {
                    joint_names = jointNames,
                    points = points
                }
            };
            await SendAsync(JsonConvert.SerializeObject(msg));
        }

        private async Task SendAsync(string message)
        {
            if (_socket == null || !_connected) return;
            
            var bytes = Encoding.UTF8.GetBytes(message);
            await _socket.SendAsync(new ArraySegment<byte>(bytes), 
                WebSocketMessageType.Text, true, _cts?.Token ?? CancellationToken.None);
        }

        private async Task ReceiveLoopAsync()
        {
            var buffer = new byte[8192];
            var sb = new StringBuilder();
            
            while (_socket?.State == WebSocketState.Open && !(_cts?.Token.IsCancellationRequested ?? true))
            {
                try
                {
                    var result = await _socket.ReceiveAsync(new ArraySegment<byte>(buffer), _cts!.Token);
                    
                    if (result.MessageType == WebSocketMessageType.Close)
                    {
                        _connected = false;
                        OnStatusChanged?.Invoke("Connection closed by server");
                        break;
                    }
                    
                    sb.Append(Encoding.UTF8.GetString(buffer, 0, result.Count));
                    
                    if (result.EndOfMessage)
                    {
                        ProcessMessage(sb.ToString());
                        sb.Clear();
                    }
                }
                catch (OperationCanceledException)
                {
                    break;
                }
                catch (Exception ex)
                {
                    OnError?.Invoke($"Receive error: {ex.Message}");
                }
            }
        }

        private void ProcessMessage(string json)
        {
            try
            {
                var obj = JObject.Parse(json);
                var topic = obj["topic"]?.ToString();
                
                if (topic == "/joint_states")
                {
                    var msg = obj["msg"];
                    if (msg != null)
                    {
                        var jointState = new JointStateMessage
                        {
                            Names = msg["name"]?.ToObject<string[]>() ?? Array.Empty<string>(),
                            Positions = msg["position"]?.ToObject<double[]>() ?? Array.Empty<double>(),
                            Velocities = msg["velocity"]?.ToObject<double[]>() ?? Array.Empty<double>()
                        };
                        OnJointStateReceived?.Invoke(jointState);
                    }
                }
            }
            catch (Exception ex)
            {
                OnError?.Invoke($"Parse error: {ex.Message}");
            }
        }

        public void Dispose()
        {
            _cts?.Cancel();
            _socket?.Dispose();
        }
    }

    public class JointStateMessage
    {
        public string[] Names { get; set; } = Array.Empty<string>();
        public double[] Positions { get; set; } = Array.Empty<double>();
        public double[] Velocities { get; set; } = Array.Empty<double>();
    }
}
