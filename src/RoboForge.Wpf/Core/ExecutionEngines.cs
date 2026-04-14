// ── Execution Engines: Real and Ghost/Simulation ──────────────────────────
using System;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
using RoboForge.Wpf.AST;

namespace RoboForge.Wpf.Core
{
    /// <summary>Interface for execution engines (real hardware or simulation)</summary>
    public interface IExecutionEngine
    {
        Task Run(List<Instruction> instructions, CancellationToken ct);
        void Pause();
        void Resume();
        void Stop();
        ProgramState State { get; }
    }

    /// <summary>
    /// Ghost/Simulation Execution Engine
    /// Simulates robot motion using forward kinematics — no hardware required.
    /// Publishes state updates to the StateBus at 60Hz for smooth 3D viewport animation.
    /// </summary>
    public class GhostExecutionEngine : IExecutionEngine
    {
        private ProgramState _state = ProgramState.Idle;
        public ProgramState State => _state;
        private readonly int _jointCount;
        private readonly double[] _currentAngles;

        public GhostExecutionEngine(int jointCount = 6)
        {
            _jointCount = jointCount;
            _currentAngles = new double[jointCount];
        }

        public async Task Run(List<Instruction> instructions, CancellationToken ct)
        {
            _state = ProgramState.Running;
            StateBus.UpdateProgramState(ProgramState.Running);

            int pc = 0; // Program counter
            var stateUpdateTimer = TimeSpan.FromMilliseconds(16); // ~60Hz

            while (pc < instructions.Count && !ct.IsCancellationRequested)
            {
                var instr = instructions[pc];

                // Publish state update with active block highlight
                StateBus.UpdateActiveNode(instr.SourceNodeId);

                switch (instr.InstructionType)
                {
                    case NodeType.MoveJ:
                    case NodeType.MoveL:
                        await SimulateMotion(instr, ct);
                        break;

                    case NodeType.Wait:
                        var duration = instr.Parameters.TryGetValue("durationMs", out var d)
                            ? (double)d / 1000.0 : 1.0;
                        // In simulation, wait at 10x speed (or 1ms in "instant" mode)
                        var simDelay = Math.Max(1, duration * 100);
                        await Task.Delay((int)simDelay, ct);
                        break;

                    case NodeType.SetDO:
                    case NodeType.PulseDO:
                        // In simulation, just update virtual IO state
                        var pin = instr.Parameters.GetValueOrDefault("outputPin", "")?.ToString() ?? "";
                        var val = instr.Parameters.GetValueOrDefault("value", false);
                        var ioDict = StateBus.CurrentState.IoStates;
                        ioDict[pin] = (bool)val;
                        StateBus.Publish(new ExecutionStateUpdate
                        {
                            ActiveNodeId = instr.SourceNodeId,
                            IoStates = ioDict,
                            JointAngles = _currentAngles,
                            ProgramState = ProgramState.Running,
                        });
                        break;

                    case NodeType.GripperOpen:
                    case NodeType.GripperClose:
                        await Task.Delay(200, ct); // Simulated gripper action
                        break;

                    case NodeType.While:
                        // Check condition — in simulation, assume true unless specified
                        var condition = instr.Parameters.GetValueOrDefault("condition", "true")?.ToString() ?? "true";
                        if (condition == "false")
                        {
                            // Jump to end of loop
                            if (instr.Parameters.TryGetValue("jumpIndex", out var jumpIdx))
                                pc = (int)jumpIdx;
                        }
                        break;

                    case NodeType.Break:
                        // This is actually a loop-end jump marker
                        if (instr.Parameters.TryGetValue("jumpTo", out var loopStart))
                            pc = (int)loopStart;
                        else
                            pc++; // Normal increment
                        continue;

                    case NodeType.If:
                        var ifCond = instr.Parameters.GetValueOrDefault("condition", "true")?.ToString() ?? "true";
                        if (ifCond == "false")
                        {
                            if (instr.Parameters.TryGetValue("elseJump", out var elseJump))
                                pc = (int)elseJump;
                        }
                        break;

                    case NodeType.Stop:
                        _state = ProgramState.Stopped;
                        StateBus.UpdateProgramState(ProgramState.Stopped);
                        return;

                    default:
                        break;
                }

                // Advance program counter (unless already modified by control flow)
                if (instr.InstructionType != NodeType.While && instr.InstructionType != NodeType.If && instr.InstructionType != NodeType.Break)
                    pc++;

                // Throttle to ~60Hz state updates
                await Task.Delay(stateUpdateTimer, ct);
            }

            _state = ProgramState.Idle;
            StateBus.UpdateProgramState(ProgramState.Idle);
        }

        /// <summary>Simulate motion by interpolating joint angles smoothly</summary>
        private async Task SimulateMotion(Instruction instr, CancellationToken ct)
        {
            // Get target angles from waypoint or parameters
            var targetAngles = GetTargetAngles(instr);
            var duration = instr.EstimatedDuration;
            var steps = (int)(duration * 60); // 60Hz interpolation

            var startAngles = (double[])_currentAngles.Clone();

            for (int i = 0; i <= steps && !ct.IsCancellationRequested; i++)
            {
                double t = steps > 0 ? (double)i / steps : 1.0;
                // Smooth easing: cubic ease-in-out
                t = t < 0.5 ? 4 * t * t * t : 1 - Math.Pow(-2 * t + 2, 3) / 2;

                for (int j = 0; j < _jointCount; j++)
                {
                    _currentAngles[j] = startAngles[j] + (targetAngles[j] - startAngles[j]) * t;
                }

                StateBus.UpdateJointAngles(_currentAngles);
                await Task.Delay(16, ct);
            }
        }

        private double[] GetTargetAngles(Instruction instr)
        {
            // In simulation, generate plausible target angles based on block type
            var targets = new double[_jointCount];
            var rand = new Random(instr.SourceNodeId.GetHashCode());

            if (instr.InstructionType == NodeType.MoveJ || instr.InstructionType == NodeType.MoveL)
            {
                // Generate random-ish but deterministic joint angles
                targets[0] = rand.NextDouble() * 60 - 30;
                targets[1] = rand.NextDouble() * 40 - 20;
                targets[2] = rand.NextDouble() * 60 - 30;
                targets[3] = rand.NextDouble() * 40 - 20;
                targets[4] = rand.NextDouble() * 40 - 20;
                targets[5] = rand.NextDouble() * 60 - 30;
            }
            return targets;
        }

        public void Pause() { _state = ProgramState.Paused; StateBus.UpdateProgramState(ProgramState.Paused); }
        public void Resume() { _state = ProgramState.Running; StateBus.UpdateProgramState(ProgramState.Running); }
        public void Stop() { _state = ProgramState.Stopped; StateBus.UpdateProgramState(ProgramState.Stopped); }
    }

    /// <summary>
    /// Real Execution Engine (stub — communicates with hardware via serial)
    /// In production, this sends serial commands to the robot controller.
    /// </summary>
    public class RealExecutionEngine : IExecutionEngine
    {
        private ProgramState _state = ProgramState.Idle;
        public ProgramState State => _state;

        public async Task Run(List<Instruction> instructions, CancellationToken ct)
        {
            _state = ProgramState.Running;
            StateBus.UpdateProgramState(ProgramState.Running);

            int pc = 0;
            while (pc < instructions.Count && !ct.IsCancellationRequested)
            {
                var instr = instructions[pc];
                StateBus.UpdateActiveNode(instr.SourceNodeId);

                // TODO: Send serial commands to hardware
                // await SerialService.SendCommandAsync(instr, ct);

                // For now, treat as simulation
                await Task.Delay(100, ct);
                pc++;
            }

            _state = ProgramState.Idle;
            StateBus.UpdateProgramState(ProgramState.Idle);
        }

        public void Pause() => _state = ProgramState.Paused;
        public void Resume() => _state = ProgramState.Running;
        public void Stop() => _state = ProgramState.Stopped;
    }
}
