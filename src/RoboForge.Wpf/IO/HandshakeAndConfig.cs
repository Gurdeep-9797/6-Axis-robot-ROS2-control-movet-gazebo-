// ── Section 4.3: Handshake Protocol & Configuration ───────────────────────
using System;
using System.IO.Ports;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using Newtonsoft.Json;

namespace RoboForge.Wpf.IO
{
    /// <summary>
    /// Handshake protocol for RoboForge-compatible device firmware.
    /// Protocol: Text-based over serial with newline-terminated messages.
    /// 
    /// Host sends:    "ROBOFORGE_HELLO\n"
    /// Device replies: "ROBOFORGE_ACK:{deviceType}:{version}:{pinCount}\n"
    /// 
    /// Configuration:
    /// Host sends:    "CONFIG:{json_config}\n"
    /// Device replies: "CONFIG_OK\n" or "CONFIG_ERR:{reason}\n"
    /// </summary>
    public static class HandshakeProtocol
    {
        private const string HelloMessage = "ROBOFORGE_HELLO\n";
        private const string AckPrefix = "ROBOFORGE_ACK:";
        private const string ConfigPrefix = "CONFIG:";
        private const int DefaultTimeoutMs = 2000;
        private const int DefaultBaud = 115200;

        /// <summary>
        /// Attempt handshake with a device on the specified serial port.
        /// Returns device info string if successful, null if timeout or no response.
        /// </summary>
        public static async Task<DeviceHandshakeInfo?> TryHandshakeAsync(
            string portName, int baud = DefaultBaud, int timeoutMs = DefaultTimeoutMs, CancellationToken ct = default)
        {
            using var serial = new SerialPort(portName, baud);
            serial.ReadTimeout = timeoutMs;
            serial.WriteTimeout = timeoutMs;

            try
            {
                serial.Open();
                serial.DiscardInBuffer();
                serial.DiscardOutBuffer();

                // Send hello
                serial.Write(HelloMessage);

                // Wait for response with timeout
                var sw = System.Diagnostics.Stopwatch.StartNew();
                var response = new StringBuilder();

                while (sw.ElapsedMilliseconds < timeoutMs && !ct.IsCancellationRequested)
                {
                    if (serial.BytesToRead > 0)
                    {
                        var bytes = new byte[serial.BytesToRead];
                        serial.Read(bytes, 0, bytes.Length);
                        response.Append(Encoding.UTF8.GetString(bytes));

                        if (response.ToString().Contains('\n'))
                            break;
                    }
                    await Task.Delay(10, ct);
                }

                serial.Close();

                var responseStr = response.ToString().Trim();
                return ParseAckResponse(responseStr);
            }
            catch
            {
                // Device not responding, port unavailable, or unknown firmware
                return null;
            }
        }

        /// <summary>
        /// Parse the ACK response string into structured device info.
        /// Format: "ROBOFORGE_ACK:{deviceType}:{version}:{pinCount}"
        /// </summary>
        private static DeviceHandshakeInfo? ParseAckResponse(string response)
        {
            if (!response.StartsWith(AckPrefix))
                return null;

            var parts = response.Substring(AckPrefix.Length).Split(':');
            if (parts.Length < 3) return null;

            return new DeviceHandshakeInfo
            {
                DeviceType = parts[0],
                FirmwareVersion = parts[1],
                PinCount = int.TryParse(parts[2], out var pins) ? pins : 0,
                IsRoboForgeCompatible = true
            };
        }

        /// <summary>
        /// Send IO configuration to device.
        /// The config is serialized as JSON and sent as "CONFIG:{json}\n"
        /// Device stores this in EEPROM for persistence across power cycles.
        /// </summary>
        public static async Task<bool> SendConfigurationAsync(
            string portName, IoConfiguration config, int baud = DefaultBaud, CancellationToken ct = default)
        {
            using var serial = new SerialPort(portName, baud);
            serial.WriteTimeout = 5000;
            serial.ReadTimeout = 5000;

            try
            {
                serial.Open();

                // Serialize config to JSON
                var json = JsonConvert.SerializeObject(config);
                var message = ConfigPrefix + json + "\n";

                serial.Write(Encoding.UTF8.GetBytes(message), 0, Encoding.UTF8.GetByteCount(message));

                // Wait for CONFIG_OK response
                var sw = System.Diagnostics.Stopwatch.StartNew();
                var response = new StringBuilder();

                while (sw.ElapsedMilliseconds < 3000 && !ct.IsCancellationRequested)
                {
                    if (serial.BytesToRead > 0)
                    {
                        var bytes = new byte[serial.BytesToRead];
                        serial.Read(bytes, 0, bytes.Length);
                        response.Append(Encoding.UTF8.GetString(bytes));

                        if (response.ToString().Contains('\n'))
                            break;
                    }
                    await Task.Delay(10, ct);
                }

                serial.Close();
                return response.ToString().StartsWith("CONFIG_OK");
            }
            catch
            {
                return false;
            }
        }
    }

    /// <summary>Device information returned by handshake</summary>
    public class DeviceHandshakeInfo
    {
        public string DeviceType { get; set; } = "";
        public string FirmwareVersion { get; set; } = "";
        public int PinCount { get; set; }
        public bool IsRoboForgeCompatible { get; set; }
    }

    /// <summary>
    /// Complete IO configuration for a device.
    /// Serialized to JSON and sent to device firmware for EEPROM storage.
    /// </summary>
    public class IoConfiguration
    {
        public string DeviceId { get; set; } = "";
        public int DeviceIndex { get; set; } // 0-3 for multi-device support
        public List<PinMapping> PinMappings { get; set; } = new();
        public Dictionary<string, string> Settings { get; set; } = new();

        /// <summary>
        /// Serialize this configuration to JSON for transmission to device.
        /// </summary>
        public string ToJson() => JsonConvert.SerializeObject(this, Formatting.Indented);

        /// <summary>
        /// Deserialize from JSON received from saved project file.
        /// </summary>
        public static IoConfiguration? FromJson(string json)
        {
            try { return JsonConvert.DeserializeObject<IoConfiguration>(json); }
            catch { return null; }
        }
    }

    /// <summary>Maps a physical device pin to a robot system function</summary>
    public class PinMapping
    {
        public int PinNumber { get; set; }
        public string Function { get; set; } = ""; // MotorDir, MotorPWM, EncoderA, EncoderB, HallSensor, LimitSwitch, DO, DI
        public string JointName { get; set; } = ""; // J1, J2, J3, J4, J5, J6
        public string SignalType { get; set; } = ""; // Direction, PWM, Quadrature, Digital
        public bool ActiveHigh { get; set; } = true;
        public int DebounceMs { get; set; }
        public double CalibrationOffset { get; set; }
        public double CalibrationScale { get; set; } = 1.0;
    }

    /// <summary>
    /// IO Link: virtual wire between a digital output on one device
    /// and a trigger on another device, routed through the host PC.
    /// Enables inter-device synchronization for large robots.
    /// </summary>
    public class IoLink
    {
        public string LinkId { get; set; } = Guid.NewGuid().ToString();
        public int SourceDeviceIndex { get; set; }
        public int SourcePin { get; set; }
        public int TargetDeviceIndex { get; set; }
        public string TargetFunction { get; set; } = ""; // Trigger, Enable, Reset
        public string TargetJoint { get; set; } = "";
        public bool InvertLogic { get; set; }
    }
}
