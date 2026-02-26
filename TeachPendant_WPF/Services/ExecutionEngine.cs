using System;
using System.Threading;
using System.Threading.Tasks;
using TeachPendant_WPF.Models;

namespace TeachPendant_WPF.Services
{
    /// <summary>
    /// Execution engine with a proper Finite State Machine.
    /// States: Idle → Running → Paused → Stopped → Error
    /// Runs on a separate thread from the UI.
    /// </summary>
    public class ExecutionEngine
    {
        private readonly IRobotDriver _driver;
        private CancellationTokenSource? _cancellationTokenSource;

        // ── FSM State ───────────────────────────────────────────────

        public enum ExecutionState
        {
            Idle,
            Running,
            Paused,
            Stopped,
            Error
        }

        private ExecutionState _state = ExecutionState.Idle;
        public ExecutionState State
        {
            get => _state;
            private set
            {
                _state = value;
                StateChanged?.Invoke(value);
            }
        }

        private readonly ManualResetEventSlim _pauseEvent = new(true);

        // ── Events ──────────────────────────────────────────────────

        public event Action<int>? ActiveLineChanged;
        public event Action<bool>? ExecutionStateChanged;
        public event Action<ExecutionState>? StateChanged;
        public event Action<string>? ErrorOccurred;

        // ── Construction ────────────────────────────────────────────

        public ExecutionEngine(IRobotDriver driver)
        {
            _driver = driver;
        }

        // ── Execution ───────────────────────────────────────────────

        public async Task RunProgramAsync(RobotProgram program, RobotState state)
        {
            if (State == ExecutionState.Running) return;

            _cancellationTokenSource = new CancellationTokenSource();
            State = ExecutionState.Running;
            state.IsRunning = true;
            ExecutionStateChanged?.Invoke(true);

            try
            {
                for (int i = 0; i < program.Instructions.Count; i++)
                {
                    // Check cancellation
                    if (_cancellationTokenSource.Token.IsCancellationRequested)
                    {
                        State = ExecutionState.Stopped;
                        break;
                    }

                    // Wait if paused
                    _pauseEvent.Wait(_cancellationTokenSource.Token);

                    var instruction = program.Instructions[i];
                    state.ActiveLineNumber = i;
                    ActiveLineChanged?.Invoke(i);

                    // Handle breakpoints
                    if (instruction.IsBreakpoint)
                    {
                        State = ExecutionState.Paused;
                        _pauseEvent.Reset();
                        _pauseEvent.Wait(_cancellationTokenSource.Token);
                        State = ExecutionState.Running;
                    }

                    // Execute the instruction
                    await instruction.ExecuteAsync(state);
                }

                if (State == ExecutionState.Running)
                    State = ExecutionState.Idle;
            }
            catch (OperationCanceledException)
            {
                State = ExecutionState.Stopped;
            }
            catch (Exception ex)
            {
                State = ExecutionState.Error;
                ErrorOccurred?.Invoke(ex.Message);
                Console.WriteLine($"Execution Error: {ex.Message}");
            }
            finally
            {
                state.IsRunning = false;
                ExecutionStateChanged?.Invoke(false);
            }
        }

        // ── Control Commands ────────────────────────────────────────

        public void Pause()
        {
            if (State == ExecutionState.Running)
            {
                State = ExecutionState.Paused;
                _pauseEvent.Reset();
            }
        }

        public void Resume()
        {
            if (State == ExecutionState.Paused)
            {
                State = ExecutionState.Running;
                _pauseEvent.Set();
            }
        }

        public void Stop()
        {
            _cancellationTokenSource?.Cancel();
            _pauseEvent.Set(); // Unblock if paused
            State = ExecutionState.Stopped;
        }
    }
}
