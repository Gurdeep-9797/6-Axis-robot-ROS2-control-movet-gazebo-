using System;
using System.Threading;
using System.Threading.Tasks;
using TeachPendant_WPF.Models;

namespace TeachPendant_WPF.Services
{
    public class ExecutionEngine
    {
        private readonly IRobotDriver _driver;
        private CancellationTokenSource _cancellationTokenSource;

        public event Action<int> ActiveLineChanged;
        public event Action<bool> ExecutionStateChanged;

        public ExecutionEngine(IRobotDriver driver)
        {
            _driver = driver;
        }

        public async Task RunProgramAsync(RobotProgram program, RobotState state)
        {
            if (state.IsRunning) return;

            _cancellationTokenSource = new CancellationTokenSource();
            state.IsRunning = true;
            ExecutionStateChanged?.Invoke(true);

            try
            {
                for (int i = 0; i < program.Instructions.Count; i++)
                {
                    if (_cancellationTokenSource.Token.IsCancellationRequested)
                    {
                        break;
                    }

                    var instruction = program.Instructions[i];
                    state.ActiveLineNumber = i;
                    ActiveLineChanged?.Invoke(i);

                    // Pause if breakpoint
                    if (instruction.IsBreakpoint)
                    {
                        // Logic for breakpoint yielding could go here
                    }

                    await instruction.ExecuteAsync(state);
                }
            }
            catch (Exception ex)
            {
                // Handle execution error
                Console.WriteLine($"Execution Error: {ex.Message}");
            }
            finally
            {
                state.IsRunning = false;
                ExecutionStateChanged?.Invoke(false);
            }
        }

        public void Stop()
        {
            _cancellationTokenSource?.Cancel();
        }
    }
}
