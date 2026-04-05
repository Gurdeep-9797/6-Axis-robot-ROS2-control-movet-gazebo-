using System;
using System.Diagnostics;
using System.Threading.Tasks;
using TeachPendant_WPF.Models;

namespace TeachPendant_WPF.Services
{
    public class SimulationDriver : IRobotDriver
    {
        private RobotState _currentState = new RobotState();
        private bool _isConnected;
        private double[] _targetJoints = new double[6];

        public bool IsConnected => _isConnected;

        public event Action<RobotState>? StateUpdated;

        public void Connect()
        {
            _isConnected = true;
            // Native simulation loop
            Task.Run(async () =>
            {
                while (_isConnected)
                {
                    SimulateStep();
                    StateUpdated?.Invoke(_currentState);
                    await Task.Delay(50); // 20 Hz
                }
            });
        }

        public void Disconnect()
        {
            _isConnected = false;
        }

        public RobotState GetCurrentState() => _currentState;

        public Task SendJointPositions(double[] anglesDeg)
        {
            if (anglesDeg.Length == 6)
            {
                _targetJoints = (double[])anglesDeg.Clone();
            }
            return Task.CompletedTask;
        }

        public Task SendJointTorques(double[] torques) => Task.CompletedTask;

        public Task ExecuteTrajectoryPoint(double[] anglesDeg, double timeFromStartSec)
        {
            return SendJointPositions(anglesDeg);
        }

        public async Task<bool> WaitForJointStateAsync(double[] targetAngles, double toleranceDeg, int timeoutMs = 10000)
        {
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

                await Task.Delay(50);
            }
            return false;
        }

        private void SimulateStep()
        {
            // Move current state towards target state step by step (simple linear interpolation)
            double maxStep = 1.5; // degrees per tick
            
            _currentState.J1 = StepTowards(_currentState.J1, _targetJoints[0], maxStep);
            _currentState.J2 = StepTowards(_currentState.J2, _targetJoints[1], maxStep);
            _currentState.J3 = StepTowards(_currentState.J3, _targetJoints[2], maxStep);
            _currentState.J4 = StepTowards(_currentState.J4, _targetJoints[3], maxStep);
            _currentState.J5 = StepTowards(_currentState.J5, _targetJoints[4], maxStep);
            _currentState.J6 = StepTowards(_currentState.J6, _targetJoints[5], maxStep);
            
            // Simple forward kinematics approx for XYZ
            _currentState.X = _currentState.J1 * 2.0;
            _currentState.Y = _currentState.J2 * 2.0;
            _currentState.Z = 300 + _currentState.J3 * 2.0;
        }

        private double StepTowards(double current, double target, double maxStep)
        {
            double diff = target - current;
            if (Math.Abs(diff) <= maxStep) return target;
            return current + Math.Sign(diff) * maxStep;
        }
    }
}
