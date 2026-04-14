// ── Section 4: IO Detection & Auto-Configuration ──────────────────────────
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Management;
using System.Text.RegularExpressions;
using CommunityToolkit.Mvvm.ComponentModel;

namespace RoboForge.Wpf.IO
{
    /// <summary>Represents a known microcontroller device with USB VID/PID fingerprint</summary>
    public class KnownDevice
    {
        public string Name { get; set; } = "";
        public ushort VendorId { get; set; }
        public ushort ProductId { get; set; }
        public int DefaultBaud { get; set; } = 115200;
        public int DigitalPins { get; set; }
        public int AnalogPins { get; set; }
        public int[] PwmPins { get; set; } = Array.Empty<int>();
        public string I2cPins { get; set; } = "";
        public string SpiPins { get; set; } = "";
        public string Notes { get; set; } = "";
        public bool Is3v3Logic { get; set; }
        public bool HasWiFi { get; set; }
        public bool HasBLE { get; set; }
        public bool RequiresFirmwareQuery { get; set; }

        public string HexId => $"VID:{VendorId:X4} PID:{ProductId:X4}";
    }

    /// <summary>
    /// Complete device fingerprint database covering Arduino, ESP32, ESP8266, STM32, Teensy, Pi Pico
    /// </summary>
    public static class DeviceDatabase
    {
        private static readonly List<KnownDevice> _devices = new()
        {
            // ── Arduino Family ──────────────────────────────────────────────
            new KnownDevice {
                Name = "Arduino Uno R3", VendorId = 0x2341, ProductId = 0x0043,
                DigitalPins = 14, AnalogPins = 6,
                PwmPins = new[] { 3, 5, 6, 9, 10, 11 },
                I2cPins = "SDA=A4, SCL=A5", SpiPins = "MOSI=11, MISO=12, SCK=13",
                Notes = "ATmega328P, 16MHz, 5V logic"
            },
            new KnownDevice {
                Name = "Arduino Mega 2560", VendorId = 0x2341, ProductId = 0x0010,
                DigitalPins = 54, AnalogPins = 16,
                PwmPins = Enumerable.Range(2, 12).Concat(new[] { 44, 45, 46 }).ToArray(),
                I2cPins = "SDA=20, SCL=21", SpiPins = "MOSI=51, MISO=50, SCK=52",
                Notes = "ATmega2560, 16MHz, 5V logic — large project support"
            },
            new KnownDevice {
                Name = "Arduino Nano (Genuine)", VendorId = 0x2341, ProductId = 0x0043,
                DigitalPins = 14, AnalogPins = 8,
                PwmPins = new[] { 3, 5, 6, 9, 10, 11 },
                I2cPins = "SDA=A4, SCL=A5", SpiPins = "MOSI=11, MISO=12, SCK=13",
                Notes = "ATmega328P, compact form factor — same VID/PID as Uno"
            },
            new KnownDevice {
                Name = "Arduino Nano (CH340 Clone)", VendorId = 0x1A86, ProductId = 0x7523,
                DigitalPins = 14, AnalogPins = 8,
                PwmPins = new[] { 3, 5, 6, 9, 10, 11 },
                I2cPins = "SDA=A4, SCL=A5", SpiPins = "MOSI=11, MISO=12, SCK=13",
                Notes = "CH340 USB-UART chip — extremely common in Chinese clones"
            },
            new KnownDevice {
                Name = "Arduino Leonardo", VendorId = 0x2341, ProductId = 0x8036,
                DigitalPins = 20, AnalogPins = 12,
                PwmPins = new[] { 3, 5, 6, 9, 10, 11, 13 },
                I2cPins = "SDA=2, SCL=3", SpiPins = "MOSI=16, MISO=14, SCK=15",
                Notes = "ATmega32U4, native USB HID capability"
            },
            new KnownDevice {
                Name = "Arduino Due", VendorId = 0x2341, ProductId = 0x003E,
                DigitalPins = 54, AnalogPins = 12,
                PwmPins = Enumerable.Range(2, 12).ToArray(),
                I2cPins = "SDA=20, SCL=21", SpiPins = "MOSI=75, MISO=74, SCK=76",
                Notes = "AT91SAM3X8E ARM Cortex-M3, 84MHz — 3.3V LOGIC — flag warning for 5V signals",
                Is3v3Logic = true
            },

            // ── ESP32 Family ────────────────────────────────────────────────
            new KnownDevice {
                Name = "ESP32 (CP2102)", VendorId = 0x10C4, ProductId = 0xEA60,
                DigitalPins = 34, AnalogPins = 18,
                PwmPins = Enumerable.Range(0, 34).ToArray(),
                I2cPins = "SDA=21, SCL=22 (default)", SpiPins = "VSPI/HSPI",
                Notes = "Dual-core 240MHz, WiFi + BLE. ADC2 conflicts with WiFi — flag this",
                HasWiFi = true, HasBLE = true
            },
            new KnownDevice {
                Name = "ESP32 (CH340)", VendorId = 0x1A86, ProductId = 0x7523,
                DigitalPins = 34, AnalogPins = 18,
                PwmPins = Enumerable.Range(0, 34).ToArray(),
                I2cPins = "SDA=21, SCL=22 (default)", SpiPins = "VSPI/HSPI",
                Notes = "Same VID/PID as Arduino Nano clone — distinguish by firmware query",
                HasWiFi = true, HasBLE = true, RequiresFirmwareQuery = true
            },
            new KnownDevice {
                Name = "ESP32-S3", VendorId = 0x303A, ProductId = 0x1001,
                DigitalPins = 34, AnalogPins = 18,
                PwmPins = Enumerable.Range(0, 34).ToArray(),
                I2cPins = "SDA=8, SCL=9 (default)", SpiPins = "VSPI/FSPI",
                Notes = "Dual-core 240MHz, native USB, USB HID capable",
                HasWiFi = true, HasBLE = true
            },

            // ── ESP8266 Family ──────────────────────────────────────────────
            new KnownDevice {
                Name = "ESP8266 NodeMCU (CP2102)", VendorId = 0x10C4, ProductId = 0xEA60,
                DigitalPins = 17, AnalogPins = 1,
                PwmPins = Enumerable.Range(0, 17).ToArray(),
                I2cPins = "SDA=D2, SCL=D1 (default)", SpiPins = "HSPI",
                Notes = "Single-core 80/160MHz, WiFi. Same VID/PID as ESP32 CP2102 — firmware query needed",
                HasWiFi = true, RequiresFirmwareQuery = true
            },
            new KnownDevice {
                Name = "ESP8266 NodeMCU (CH340)", VendorId = 0x1A86, ProductId = 0x7523,
                DigitalPins = 17, AnalogPins = 1,
                PwmPins = Enumerable.Range(0, 17).ToArray(),
                I2cPins = "SDA=D2, SCL=D1 (default)", SpiPins = "HSPI",
                Notes = "Same VID/PID as Arduino Nano — firmware query needed",
                HasWiFi = true, RequiresFirmwareQuery = true
            },

            // ── Raspberry Pi Pico ───────────────────────────────────────────
            new KnownDevice {
                Name = "Raspberry Pi Pico", VendorId = 0x2E8A, ProductId = 0x0005,
                DigitalPins = 30, AnalogPins = 3,
                PwmPins = Enumerable.Range(0, 30).ToArray(),
                I2cPins = "SDA=4/6/14/18/20/26, SCL=5/7/15/19/21/27", SpiPins = "GP0-GP7/16-GP23",
                Notes = "RP2040 dual-core ARM Cortex-M0+, 133MHz. MicroPython or C SDK"
            },
            new KnownDevice {
                Name = "Raspberry Pi Pico W", VendorId = 0x2E8A, ProductId = 0x0005,
                DigitalPins = 30, AnalogPins = 3,
                PwmPins = Enumerable.Range(0, 30).ToArray(),
                I2cPins = "SDA=4/6/14/18/20/26, SCL=5/7/15/19/21/27", SpiPins = "GP0-GP7/16-GP23",
                Notes = "Same VID/PID as Pico — distinguish by firmware query. WiFi capable",
                HasWiFi = true, RequiresFirmwareQuery = true
            },

            // ── STM32 Family ────────────────────────────────────────────────
            new KnownDevice {
                Name = "STM32 (ST-Link V2)", VendorId = 0x0483, ProductId = 0x374B,
                DigitalPins = 0, AnalogPins = 0,
                Notes = "Advanced device requiring separate toolchain (STM32CubeIDE, OpenOCD)"
            },

            // ── Teensy Family ───────────────────────────────────────────────
            new KnownDevice {
                Name = "Teensy 4.0", VendorId = 0x16C0, ProductId = 0x0483,
                DigitalPins = 34, AnalogPins = 14,
                PwmPins = Enumerable.Range(0, 34).ToArray(),
                I2cPins = "SDA=18/17, SCL=19/16", SpiPins = "MOSI=11, MISO=12, SCK=13",
                Notes = "ARM Cortex-M7, 600MHz — excellent for high-speed PWM and encoders"
            },
            new KnownDevice {
                Name = "Teensy 4.1", VendorId = 0x16C0, ProductId = 0x0487,
                DigitalPins = 46, AnalogPins = 14,
                PwmPins = Enumerable.Range(0, 46).ToArray(),
                I2cPins = "SDA=18/17, SCL=19/16", SpiPins = "MOSI=11, MISO=12, SCK=13",
                Notes = "Same as 4.0 with Ethernet header — 600MHz ARM Cortex-M7"
            },
        };

        /// <summary>Get all known devices</summary>
        public static IReadOnlyList<KnownDevice> AllDevices => _devices;

        /// <summary>Look up device by VID/PID. Returns null if unknown.</summary>
        public static KnownDevice? Lookup(ushort vid, ushort pid) =>
            _devices.FirstOrDefault(d => d.VendorId == vid && d.ProductId == pid);

        /// <summary>Find all devices matching a VID (for devices with multiple PIDs)</summary>
        public static IEnumerable<KnownDevice> LookupByVendor(ushort vid) =>
            _devices.Where(d => d.VendorId == vid);

        /// <summary>Get a friendly name for an unknown device</summary>
        public static string UnknownDeviceName(ushort vid, ushort pid) =>
            $"Unknown USB Device (VID:{vid:X4} PID:{pid:X4})";
    }

    /// <summary>Represents a detected USB device connected to the system</summary>
    public partial class DetectedDevice : ObservableObject
    {
        [ObservableProperty] private string _portName = "";
        [ObservableProperty] private string _friendlyName = "";
        [ObservableProperty] private ushort _vendorId;
        [ObservableProperty] private ushort _productId;
        [ObservableProperty] private KnownDevice? _knownDevice;
        [ObservableProperty] private string _status = "Detected";
        [ObservableProperty] private string _firmwareVersion = "";
        [ObservableProperty] private bool _handshakeSuccess;
        [ObservableProperty] private ObservableCollection<PinAssignment> _pinAssignments = new();
        [ObservableProperty] private bool _isExpanded;

        public string DisplayName => _knownDevice?.Name ?? DeviceDatabase.UnknownDeviceName(_vendorId, _productId);
        public bool IsKnown => _knownDevice != null;
        public bool Is3v3Warning => _knownDevice?.Is3v3Logic == true;
    }

    /// <summary>Maps a physical pin to a function in the robot system</summary>
    public partial class PinAssignment : ObservableObject
    {
        [ObservableProperty] private int _pinNumber;
        [ObservableProperty] private string _function = "Unassigned";
        [ObservableProperty] private string _jointName = "";
        [ObservableProperty] private string _signalType = ""; // MotorDir, MotorPWM, EncoderA, EncoderB, HallSensor, LimitSwitch
    }

    /// <summary>
    /// Detects and enumerates connected USB/serial devices.
    /// Uses WMI on Windows, /dev/tty* on Linux/macOS.
    /// </summary>
    public static class UsbDeviceEnumerator
    {
        /// <summary>
        /// Enumerate all connected USB serial devices and identify them by VID/PID.
        /// Returns a list of detected devices with their known device info.
        /// </summary>
        public static List<DetectedDevice> EnumerateDevices()
        {
            var devices = new List<DetectedDevice>();

            if (OperatingSystem.IsWindows())
            {
                devices.AddRange(EnumerateWindowsDevices());
            }
            else if (OperatingSystem.IsLinux())
            {
                devices.AddRange(EnumerateLinuxDevices());
            }
            else if (OperatingSystem.IsMacOS())
            {
                devices.AddRange(EnumerateMacOSDevices());
            }

            // Cross-reference with known device database
            foreach (var device in devices)
            {
                device.KnownDevice = DeviceDatabase.Lookup(device.VendorId, device.ProductId);
                device.FriendlyName = device.KnownDevice?.Name ??
                    DeviceDatabase.UnknownDeviceName(device.VendorId, device.ProductId);
            }

            return devices;
        }

        private static IEnumerable<DetectedDevice> EnumerateWindowsDevices()
        {
            var devices = new List<DetectedDevice>();

            try
            {
                // Method 1: Get serial port names from System.IO.Ports
                var portNames = SerialPort.GetPortNames();

                // Method 2: Query WMI for USB serial devices with VID/PID
                using var searcher = new ManagementObjectSearcher(
                    "SELECT * FROM Win32_PnPEntity WHERE (Name LIKE '%(COM%)' OR Name LIKE '%USB%') AND (PNPClass = 'Ports' OR PNPClass = 'USB')");

                var wmiDevices = searcher.Get().Cast<ManagementObject>();

                foreach (var wmi in wmiDevices)
                {
                    var name = wmi["Name"]?.ToString() ?? "";
                    var deviceId = wmi["DeviceID"]?.ToString() ?? "";

                    // Extract COM port from name (e.g., "Arduino Uno (COM4)" → "COM4")
                    var comMatch = System.Text.RegularExpressions.Regex.Match(name, @"\(COM(\d+)\)");
                    if (!comMatch.Success) continue;

                    var portNum = int.Parse(comMatch.Groups[1].Value);
                    var portName = $"COM{portNum}";

                    // Extract VID/PID from DeviceID (e.g., "USB\VID_2341+PID_0043\...")
                    var vidMatch = System.Text.RegularExpressions.Regex.Match(deviceId, @"VID_([0-9A-Fa-f]{4})");
                    var pidMatch = System.Text.RegularExpressions.Regex.Match(deviceId, @"PID_([0-9A-Fa-f]{4})");

                    if (!vidMatch.Success || !pidMatch.Success) continue;

                    devices.Add(new DetectedDevice
                    {
                        PortName = portName,
                        FriendlyName = name,
                        VendorId = Convert.ToUInt16(vidMatch.Groups[1].Value, 16),
                        ProductId = Convert.ToUInt16(pidMatch.Groups[1].Value, 16),
                        Status = "Connected"
                    });
                }
            }
            catch (Exception ex)
            {
                // WMI query failed — fall back to SerialPort.GetPortNames() only
                foreach (var portName in SerialPort.GetPortNames())
                {
                    devices.Add(new DetectedDevice
                    {
                        PortName = portName,
                        FriendlyName = portName,
                        Status = "Detected (VID/PID unavailable)",
                        KnownDevice = null
                    });
                }
            }

            return devices;
        }

        private static IEnumerable<DetectedDevice> EnumerateLinuxDevices()
        {
            var devices = new List<DetectedDevice>();
            var patterns = new[] { "/dev/ttyUSB*", "/dev/ttyACM*", "/dev/ttyAMA*" };

            foreach (var pattern in patterns)
            {
                try
                {
                    var dir = Path.GetDirectoryName(pattern)!;
                    var searchPattern = Path.GetFileName(pattern);
                    var ports = Directory.GetFiles(dir, searchPattern);

                    foreach (var port in ports)
                    {
                        devices.Add(new DetectedDevice
                        {
                            PortName = port,
                            FriendlyName = port,
                            Status = "Connected"
                        });
                    }
                }
                catch
                {
                    // Pattern doesn't exist or permission denied
                }
            }

            return devices;
        }

        private static IEnumerable<DetectedDevice> EnumerateMacOSDevices()
        {
            var devices = new List<DetectedDevice>();
            var patterns = new[] { "/dev/tty.*", "/dev/cu.*" };

            foreach (var pattern in patterns)
            {
                try
                {
                    var dir = Path.GetDirectoryName(pattern)!;
                    var searchPattern = Path.GetFileName(pattern);
                    var ports = Directory.GetFiles(dir, searchPattern);

                    foreach (var port in ports)
                    {
                        devices.Add(new DetectedDevice
                        {
                            PortName = port,
                            FriendlyName = port,
                            Status = "Connected"
                        });
                    }
                }
                catch
                {
                    // Pattern doesn't exist or permission denied
                }
            }

            return devices;
        }
    }
}
