using System;
using System.Collections.Generic;
using System.IO;
using System.Net;
using System.Text;
using System.Text.Json;
using System.Threading.Tasks;
using System.Windows;
using TeachPendant_WPF.Models;
using TeachPendant_WPF.Services;

namespace RoboForge_WPF.Services
{
    public class CliApiService
    {
        private HttpListener _listener;
        private RobotState _state;
        private RobotProgram _program;
        private IRobotDriver _driver;
        private ExecutionEngine _engine;
        private bool _isRunning;
        private Action<string> _logCallback;

        public CliApiService(RobotState state, RobotProgram program, IRobotDriver driver, ExecutionEngine engine, Action<string> logCallback)
        {
            _state = state;
            _program = program;
            _driver = driver;
            _engine = engine;
            _logCallback = logCallback;
            _listener = new HttpListener();
            _listener.Prefixes.Add("http://127.0.0.1:5050/api/");
        }

        public void Start()
        {
            try
            {
                _listener.Start();
                _isRunning = true;
                Task.Run(() => ListenLoop());
                _logCallback?.Invoke("CLI API Server started on http://127.0.0.1:5050/api/");
            }
            catch (Exception ex)
            {
                _logCallback?.Invoke($"Failed to start CLI API: {ex.Message}");
            }
        }

        public void Stop()
        {
            _isRunning = false;
            if (_listener.IsListening)
            {
                _listener.Stop();
                _listener.Close();
            }
        }

        private async Task ListenLoop()
        {
            while (_isRunning && _listener.IsListening)
            {
                try
                {
                    var context = await _listener.GetContextAsync();
                    _ = Task.Run(() => ProcessRequest(context));
                }
                catch (ObjectDisposedException) { /* Expected on Stop */ }
                catch (Exception ex)
                {
                    _logCallback?.Invoke($"API Error: {ex.Message}");
                }
            }
        }

        private async Task ProcessRequest(HttpListenerContext context)
        {
            var req = context.Request;
            var res = context.Response;
            res.ContentType = "application/json";

            try
            {
                string path = req.Url.AbsolutePath.ToLower();
                string method = req.HttpMethod.ToUpper();

                if (method == "GET" && path == "/api/state")
                {
                    var stateJson = JsonSerializer.Serialize(new {
                        J1 = _state.J1, J2 = _state.J2, J3 = _state.J3,
                        J4 = _state.J4, J5 = _state.J5, J6 = _state.J6,
                        IsRunning = _state.IsRunning
                    });
                    await SendResponseAsync(res, 200, stateJson);
                }
                else if (method == "POST" && path == "/api/jog")
                {
                    using var reader = new StreamReader(req.InputStream, req.ContentEncoding);
                    var body = await reader.ReadToEndAsync();
                    var jogCmd = JsonSerializer.Deserialize<JogPayload>(body);
                    
                    if (jogCmd != null && jogCmd.Joints != null && jogCmd.Joints.Length == 6)
                    {
                        _driver.SendJointPositions(jogCmd.Joints);
                        _logCallback?.Invoke("API: Jog command sent to ROS2 driver.");
                        await SendResponseAsync(res, 200, "{\"status\": \"Jog executing\"}");
                    }
                    else
                    {
                        await SendResponseAsync(res, 400, "{\"error\": \"Invalid joints array\"}");
                    }
                }
                else if (method == "POST" && path == "/api/execute")
                {
                    using var reader = new StreamReader(req.InputStream, req.ContentEncoding);
                    var body = await reader.ReadToEndAsync();
                    var scriptPayload = JsonSerializer.Deserialize<List<ScriptBlockPayload>>(body);

                    if (scriptPayload != null)
                    {
                        Application.Current.Dispatcher.Invoke(() => 
                        {
                            _program.Instructions.Clear();
                            foreach(var item in scriptPayload)
                            {
                                RobotInstruction instr = item.Type switch
                                {
                                    "MoveJ" => new PtpInstruction(item.Target ?? "p1", item.Speed > 0 ? item.Speed : 100),
                                    "MoveL" => new LinInstruction(item.Target ?? "p1", item.Speed > 0 ? item.Speed : 50, 0),
                                    "Wait" => new WaitInstruction((int)(item.Delay > 0 ? item.Delay : 1000)),
                                    "SetDO" => new SetDOInstruction(item.Port, item.Value),
                                    _ => null
                                };
                                if (instr != null) _program.Instructions.Add(instr);
                            }
                            _logCallback?.Invoke($"API: Received {scriptPayload.Count} blocks. Starting execution pipeline...");
                            _ = _engine.RunProgramAsync(_program, _state);
                        });
                        await SendResponseAsync(res, 200, "{\"status\": \"Script Execution Started via WPF -> ROS pipeline\"}");
                    }
                    else
                    {
                        await SendResponseAsync(res, 400, "{\"error\": \"Invalid script payload\"}");
                    }
                }
                else
                {
                    await SendResponseAsync(res, 404, "{\"error\": \"Route not found\"}");
                }
            }
            catch (Exception ex)
            {
                await SendResponseAsync(res, 500, $"{{\"error\": \"Server error: {ex.Message}\"}}");
            }
        }

        private async Task SendResponseAsync(HttpListenerResponse res, int statusCode, string body)
        {
            res.StatusCode = statusCode;
            var buffer = Encoding.UTF8.GetBytes(body);
            res.ContentLength64 = buffer.Length;
            await res.OutputStream.WriteAsync(buffer, 0, buffer.Length);
            res.Close();
        }
        
        private class JogPayload
        {
            public double[]? Joints { get; set; }
        }
        
        private class ScriptBlockPayload
        {
            public string? Type { get; set; }
            public string? Target { get; set; }
            public double Speed { get; set; }
            public int Delay { get; set; }
            public int Port { get; set; }
            public int Value { get; set; }
        }
    }
}
