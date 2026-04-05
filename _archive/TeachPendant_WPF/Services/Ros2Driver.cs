using System;
using System.Diagnostics;
using System.Net.WebSockets;
using System.Text;
using System.Text.Json;
using System.Threading;
using System.Threading.Tasks;
using TeachPendant_WPF.Models;

namespace TeachPendant_WPF.Services
{
    public class Ros2Driver : IRobotDriver
    {
        private ClientWebSocket? _webSocket;
        private CancellationTokenSource? _cts;
        private RobotState _currentState = new RobotState();
        private readonly string _uri;
        private bool _isConnected;
        private int _currentSimSec = 0;
        private uint _currentSimNano = 0;

        public event Action<RobotState>? StateUpdated;

        public bool IsConnected => _isConnected && _webSocket?.State == WebSocketState.Open;

        public Ros2Driver(string uri = "ws://localhost:9090")
        {
            _uri = uri;
        }

        public void Connect()
        {
            if (IsConnected) return;

            _webSocket = new ClientWebSocket();
            _cts = new CancellationTokenSource();
            _isConnected = true;

            Task.Run(async () =>
            {
                try
                {
                    await _webSocket.ConnectAsync(new Uri(_uri), _cts.Token);
                    Debug.WriteLine("Connected to ROS2 Bridge");

                    // 1. Subscribe to joint states (Input from Gazebo)
                    var subscribeMsg = new
                    {
                        op = "subscribe",
                        topic = "/joint_states",
                        type = "sensor_msgs/JointState"
                    };
                    await CustomSend(subscribeMsg);

                    // 2. Subscribe to /clock for sim time sync
                    var clockSubscribeMsg = new
                    {
                        op = "subscribe",
                        topic = "/clock",
                        type = "rosgraph_msgs/Clock"
                    };
                    await CustomSend(clockSubscribeMsg);

                    // 3. Advertise trajectory topic (Output from WPF)
                    var advertiseMsg = new
                    {
                        op = "advertise",
                        topic = "/robot_arm_controller/joint_trajectory",
                        type = "trajectory_msgs/JointTrajectory"
                    };
                    await CustomSend(advertiseMsg);

                    _ = ReceiveLoop();
                }
                catch (Exception ex)
                {
                    Debug.WriteLine($"Failed to connect ROS2 Bridge: {ex.Message}");
                    _isConnected = false;
                }
            });
        }

        private async Task CustomSend(object data)
        {
            if (!IsConnected) return;
            string json = JsonSerializer.Serialize(data);
            var bytes = Encoding.UTF8.GetBytes(json);
            await _webSocket.SendAsync(new ArraySegment<byte>(bytes), WebSocketMessageType.Text, true, _cts.Token);
        }

        public void Disconnect()
        {
            _isConnected = false;
            _cts?.Cancel();
            try
            {
                if (_webSocket?.State == WebSocketState.Open)
                {
                    var unadvertise = new { op = "unadvertise", topic = "/robot_arm_controller/joint_trajectory" };
                    CustomSend(unadvertise).Wait(500);
                    _webSocket?.CloseAsync(WebSocketCloseStatus.NormalClosure, "Client disconnecting", CancellationToken.None).Wait(1000);
                }
            }
            catch { }
            finally
            {
                _webSocket?.Dispose();
                _webSocket = null;
            }
        }

        public RobotState GetCurrentState() => _currentState;

        public async Task SendJointPositions(double[] anglesDeg)
        {
            // By default, just execute immediately with a small 0.1s time horizon
            await ExecuteTrajectoryPoint(anglesDeg, 0.1);
        }

        public async Task SendJointTorques(double[] torques) { await Task.CompletedTask; }

        public async Task ExecuteTrajectoryPoint(double[] anglesDeg, double timeFromStartSec)
        {
            if (!IsConnected || anglesDeg.Length < 6) return;

            // Convert degrees back to radians for ROS
            double[] rads = new double[6];
            for(int i=0; i<6; i++) {
                rads[i] = anglesDeg[i] * (Math.PI / 180.0);
            }

            int sec = (int)Math.Floor(timeFromStartSec);
            int nanos = (int)((timeFromStartSec - sec) * 1e9);

            var publishMsg = new
            {
                op = "publish",
                topic = "/robot_arm_controller/joint_trajectory",
                msg = new
                {
                    header = new { 
                        stamp = new { sec = _currentSimSec, nanosec = _currentSimNano },
                        frame_id = "" 
                    },
                    joint_names = new[] { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" },
                    points = new[]
                    {
                        new
                        {
                            positions = rads,
                            time_from_start = new { sec = sec, nanosec = nanos }
                        }
                    }
                }
            };

            await CustomSend(publishMsg);
        }

        private async Task ReceiveLoop()
        {
            var buffer = new byte[8192];
            while (!_cts.Token.IsCancellationRequested && IsConnected)
            {
                try
                {
                    var result = await _webSocket.ReceiveAsync(new ArraySegment<byte>(buffer), _cts.Token);
                    if (result.MessageType == WebSocketMessageType.Close)
                    {
                        Disconnect();
                        break;
                    }

                    var message = Encoding.UTF8.GetString(buffer, 0, result.Count);
                    using (JsonDocument doc = JsonDocument.Parse(message))
                    {
                        var root = doc.RootElement;
                        if (root.TryGetProperty("op", out var opProp) && opProp.GetString() == "publish")
                        {
                            if (root.TryGetProperty("topic", out var topicProp))
                            {
                                if (topicProp.GetString() == "/clock" && root.TryGetProperty("msg", out var clkMsg) && clkMsg.TryGetProperty("clock", out var clkData))
                                {
                                    if (clkData.TryGetProperty("sec", out var secProp)) _currentSimSec = secProp.GetInt32();
                                    if (clkData.TryGetProperty("nanosec", out var nanoProp)) _currentSimNano = nanoProp.GetUInt32();
                                }
                                else if (topicProp.GetString() == "/joint_states" && root.TryGetProperty("msg", out var msgProp) && msgProp.TryGetProperty("position", out var posArray))
                                {
                                    if (posArray.GetArrayLength() >= 6)
                                    {
                                        // ROS2 /joint_states provides radians. Convert to Degrees for the Teach Pendant UI.
                                        _currentState.J1 = posArray[0].GetDouble() * (180.0 / Math.PI);
                                        _currentState.J2 = posArray[1].GetDouble() * (180.0 / Math.PI);
                                        _currentState.J3 = posArray[2].GetDouble() * (180.0 / Math.PI);
                                        _currentState.J4 = posArray[3].GetDouble() * (180.0 / Math.PI);
                                        _currentState.J5 = posArray[4].GetDouble() * (180.0 / Math.PI);
                                        _currentState.J6 = posArray[5].GetDouble() * (180.0 / Math.PI);

                                        StateUpdated?.Invoke(_currentState);
                                    }
                                }
                            }
                        }
                    }
                }
                catch (Exception)
                {
                    break;
                }
            }
        }
        public async Task<bool> WaitForJointStateAsync(double[] targetAngles, double toleranceDeg, int timeoutMs = 10000)
        {
            if (targetAngles.Length != 6) return false;

            var sw = Stopwatch.StartNew();
            while (sw.ElapsedMilliseconds < timeoutMs)
            {
                if (!_isConnected) return false;

                bool reached = true;
                double[] current = new[] { _currentState.J1, _currentState.J2, _currentState.J3, _currentState.J4, _currentState.J5, _currentState.J6 };

                for (int i = 0; i < 6; i++)
                {
                    if (Math.Abs(current[i] - targetAngles[i]) > toleranceDeg)
                    {
                        reached = false;
                        break;
                    }
                }

                if (reached) return true;

                // Wait a bit before checking again
                await Task.Delay(50);
            }

            return false; // Timeout
        }
    }
}
