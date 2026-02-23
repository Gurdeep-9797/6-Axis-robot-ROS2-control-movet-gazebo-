using System;
using System.Threading.Tasks;
using TeachPendant_WPF.Models;

namespace TeachPendant_WPF.Services
{
    public class SimulationDriver : IRobotDriver
    {
        private RobotState _currentState = new RobotState();
        public bool IsConnected { get; private set; } = false;

        public event Action<RobotState> StateUpdated;

        public void Connect()
        {
            IsConnected = true;
            // In a real simulator we might start a background thread to compute physics
            StateUpdated?.Invoke(_currentState);
        }

        public void Disconnect()
        {
            IsConnected = false;
        }

        public RobotState GetCurrentState()
        {
            return _currentState;
        }

        public Task SendJointPositions(double[] anglesDeg)
        {
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
            // Simulate torque application 
            return Task.CompletedTask;
        }

        public Task ExecuteTrajectoryPoint(double[] anglesDeg, double timeFromStartSec)
        {
            return SendJointPositions(anglesDeg);
        }
    }
}
