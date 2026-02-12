using System;
using System.IO.Ports;
using System.Text;
using System.Threading;

namespace RobotSimulator.Core.Hardware
{
    public class SerialHardwareInterface : IDisposable
    {
        private SerialPort _serialPort;
        private bool _isConnected = false;
        
        public bool IsConnected => _isConnected;
        public string CurrentPort => _serialPort?.PortName ?? "None";

        public event Action<string> OnDataReceived;
        public event Action<string> OnStatusChanged;

        public void Connect(string portName, int baudRate = 115200)
        {
            if (_isConnected) Disconnect();

            try
            {
                _serialPort = new SerialPort(portName, baudRate);
                _serialPort.DataReceived += SerialPort_DataReceived;
                _serialPort.Open();
                _isConnected = true;
                OnStatusChanged?.Invoke($"Connected to {portName}");
            }
            catch (Exception ex)
            {
                OnStatusChanged?.Invoke($"Error connecting to {portName}: {ex.Message}");
                _isConnected = false;
            }
        }

        public void Disconnect()
        {
            if (_serialPort != null && _serialPort.IsOpen)
            {
                _serialPort.Close();
                _serialPort.Dispose();
            }
            _isConnected = false;
            OnStatusChanged?.Invoke("Disconnected");
        }

        public void SendJointAngles(double[] angles)
        {
            if (!_isConnected || angles.Length < 6) return;

            // Format: <J0:val,J1:val,J2:val,J3:val,J4:val,J5:val>
            var sb = new StringBuilder();
            sb.Append("<");
            for (int i = 0; i < 6; i++)
            {
                sb.Append($"J{i}:{angles[i]:F2}");
                if (i < 5) sb.Append(",");
            }
            sb.Append(">");

            try
            {
                _serialPort.Write(sb.ToString());
            }
            catch (Exception ex)
            {
                OnStatusChanged?.Invoke($"Send Error: {ex.Message}");
                Disconnect();
            }
        }

        private void SerialPort_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            try
            {
                string data = _serialPort.ReadExisting();
                OnDataReceived?.Invoke(data);
            }
            catch { }
        }

        public string[] GetAvailablePorts()
        {
            return SerialPort.GetPortNames();
        }

        public void Dispose()
        {
            Disconnect();
        }
    }
}
