// ── State Bus: Real-time state distribution via Reactive Extensions ────────
using System;
using System.Collections.Generic;
using System.Reactive.Subjects;
using System.Windows.Media.Media3D;
using RoboForge.Wpf.AST;

namespace RoboForge.Wpf.Core
{
    /// <summary>Execution states for the program</summary>
    public enum ProgramState { Idle, Running, Paused, Stopped, Error, Homing }

    /// <summary>State update published on the state bus every control cycle</summary>
    public class ExecutionStateUpdate
    {
        public string ActiveInstructionId { get; set; } = "";
        public string ActiveNodeId { get; set; } = ""; // Maps back to AST node
        public double[] JointAngles { get; set; } = Array.Empty<double>();
        public Vector3D TcpPosition { get; set; }
        public Quaternion TcpRotation { get; set; }
        public Dictionary<string, bool> IoStates { get; set; } = new();
        public Dictionary<string, double> AnalogValues { get; set; } = new();
        public double ExecutionSpeed { get; set; } = 1.0;
        public ProgramState ProgramState { get; set; } = ProgramState.Idle;
        public string? ErrorMessage { get; set; }
        public DateTime Timestamp { get; set; } = DateTime.Now;
    }

    /// <summary>
    /// Central state bus — all components publish to and subscribe from this.
    /// Uses Reactive Extensions for type-safe pub/sub with disposal support.
    /// </summary>
    public static class StateBus
    {
        private static readonly BehaviorSubject<ExecutionStateUpdate> _stateSubject =
            new(new ExecutionStateUpdate());

        /// <summary>Observable stream of state updates. Subscribe to receive updates.</summary>
        public static IObservable<ExecutionStateUpdate> StateStream => _stateSubject;

        /// <summary>Publish a new state update to all subscribers</summary>
        public static void Publish(ExecutionStateUpdate update) => _stateSubject.OnNext(update);

        /// <summary>Get the current state synchronously</summary>
        public static ExecutionStateUpdate CurrentState => _stateSubject.Value;

        /// <summary>Quick helper to update just joint angles</summary>
        public static void UpdateJointAngles(double[] angles)
        {
            var current = _stateSubject.Value;
            var update = new ExecutionStateUpdate
            {
                ActiveInstructionId = current.ActiveInstructionId,
                ActiveNodeId = current.ActiveNodeId,
                JointAngles = angles,
                TcpPosition = current.TcpPosition,
                TcpRotation = current.TcpRotation,
                IoStates = new Dictionary<string, bool>(current.IoStates),
                AnalogValues = new Dictionary<string, double>(current.AnalogValues),
                ExecutionSpeed = current.ExecutionSpeed,
                ProgramState = current.ProgramState,
                ErrorMessage = current.ErrorMessage,
            };
            _stateSubject.OnNext(update);
        }

        /// <summary>Quick helper to update active block highlight</summary>
        public static void UpdateActiveNode(string nodeId)
        {
            var current = _stateSubject.Value;
            var update = new ExecutionStateUpdate
            {
                ActiveInstructionId = current.ActiveInstructionId,
                ActiveNodeId = nodeId,
                JointAngles = current.JointAngles,
                TcpPosition = current.TcpPosition,
                TcpRotation = current.TcpRotation,
                IoStates = current.IoStates,
                AnalogValues = current.AnalogValues,
                ExecutionSpeed = current.ExecutionSpeed,
                ProgramState = current.ProgramState,
                ErrorMessage = current.ErrorMessage,
            };
            _stateSubject.OnNext(update);
        }

        /// <summary>Quick helper to update program state</summary>
        public static void UpdateProgramState(ProgramState state, string? error = null)
        {
            var current = _stateSubject.Value;
            var update = new ExecutionStateUpdate
            {
                ActiveInstructionId = current.ActiveInstructionId,
                ActiveNodeId = current.ActiveNodeId,
                JointAngles = current.JointAngles,
                TcpPosition = current.TcpPosition,
                TcpRotation = current.TcpRotation,
                IoStates = current.IoStates,
                AnalogValues = current.AnalogValues,
                ExecutionSpeed = current.ExecutionSpeed,
                ProgramState = state,
                ErrorMessage = error,
            };
            _stateSubject.OnNext(update);
        }
    }
}
