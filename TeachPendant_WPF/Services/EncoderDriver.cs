using System;
using System.Threading;
using System.Threading.Tasks;
using TeachPendant_WPF.Models;

namespace TeachPendant_WPF.Services
{
    /// <summary>
    /// Encoder-based driver for real robot hardware.
    /// Reads encoder counts, converts to joint angles via gear ratio + resolution,
    /// computes FK, and updates TCP.
    /// Implements IRobotDriver — swappable with SimulationDriver via Strategy pattern.
    /// </summary>
    public class EncoderDriver : IRobotDriver
    {
        private RobotState _currentState = new RobotState();
        private CancellationTokenSource? _pollingCts;

        // ── Configuration ───────────────────────────────────────────

        public int EncoderResolution { get; set; } = 4096;   // counts per revolution
        public double GearRatio { get; set; } = 100.0;        // gear reduction
        public string CommunicationMode { get; set; } = "EtherCAT"; // EtherCAT, USB, etc.

        // ── IRobotDriver Interface ──────────────────────────────────

        public bool IsConnected { get; private set; }
        public event Action<RobotState>? StateUpdated;

        public void Connect()
        {
            IsConnected = true;
            // Start polling encoder values
            _pollingCts = new CancellationTokenSource();
            _ = PollEncodersAsync(_pollingCts.Token);
        }

        public void Disconnect()
        {
            IsConnected = false;
            _pollingCts?.Cancel();
        }

        public RobotState GetCurrentState() => _currentState;

        public Task SendJointPositions(double[] anglesDeg)
        {
            // In Real mode, this sends position commands to motor controllers
            // For now, update internal state (hardware interface will be added later)
            if (anglesDeg.Length >= 6)
            {
                _currentState.J1 = anglesDeg[0];
                _currentState.J2 = anglesDeg[1];
                _currentState.J3 = anglesDeg[2];
                _currentState.J4 = anglesDeg[3];
                _currentState.J5 = anglesDeg[4];
                _currentState.J6 = anglesDeg[5];
                StateUpdated?.Invoke(_currentState);
            }
            return Task.CompletedTask;
        }

        public Task SendJointTorques(double[] torques)
        {
            // Torque mode command to motor controllers
            return Task.CompletedTask;
        }

        public Task ExecuteTrajectoryPoint(double[] anglesDeg, double timeFromStartSec)
        {
            return SendJointPositions(anglesDeg);
        }

        // ── Encoder Reading ─────────────────────────────────────────

        /// <summary>
        /// Convert raw encoder count to joint angle in degrees.
        /// angle = (count / (resolution * gearRatio)) * 360
        /// </summary>
        public double EncoderCountToAngle(long encoderCount)
        {
            return (encoderCount / (EncoderResolution * GearRatio)) * 360.0;
        }

        /// <summary>
        /// Convert joint angle in degrees to encoder count.
        /// </summary>
        public long AngleToEncoderCount(double angleDeg)
        {
            return (long)((angleDeg / 360.0) * EncoderResolution * GearRatio);
        }

        /// <summary>
        /// Background polling loop that reads encoder values and updates state.
        /// In a real implementation, this reads from EtherCAT / serial / USB.
        /// </summary>
        private async Task PollEncodersAsync(CancellationToken ct)
        {
            while (!ct.IsCancellationRequested)
            {
                try
                {
                    // Read encoder values from hardware
                    // Placeholder: In production, replace with actual hardware API calls
                    long[] encoderCounts = ReadEncoderHardware();

                    if (encoderCounts.Length >= 6)
                    {
                        _currentState.J1 = EncoderCountToAngle(encoderCounts[0]);
                        _currentState.J2 = EncoderCountToAngle(encoderCounts[1]);
                        _currentState.J3 = EncoderCountToAngle(encoderCounts[2]);
                        _currentState.J4 = EncoderCountToAngle(encoderCounts[3]);
                        _currentState.J5 = EncoderCountToAngle(encoderCounts[4]);
                        _currentState.J6 = EncoderCountToAngle(encoderCounts[5]);

                        StateUpdated?.Invoke(_currentState);
                    }

                    await Task.Delay(20, ct); // 50 Hz polling
                }
                catch (OperationCanceledException)
                {
                    break;
                }
                catch (Exception ex)
                {
                    System.Diagnostics.Debug.WriteLine($"Encoder read error: {ex.Message}");
                    await Task.Delay(100, ct);
                }
            }
        }

        /// <summary>
        /// Placeholder for actual hardware encoder reading.
        /// Replace with EtherCAT PDO reads, serial protocol, etc.
        /// </summary>
        private long[] ReadEncoderHardware()
        {
            // Return current state as encoder counts (for testing)
            return new long[]
            {
                AngleToEncoderCount(_currentState.J1),
                AngleToEncoderCount(_currentState.J2),
                AngleToEncoderCount(_currentState.J3),
                AngleToEncoderCount(_currentState.J4),
                AngleToEncoderCount(_currentState.J5),
                AngleToEncoderCount(_currentState.J6),
            };
        }
    }
}
