using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;

namespace TeachPendant_WPF.ViewModels
{
    /// <summary>
    /// Global application state visible in the Top Status Bar.
    /// Binds to: Mode toggle, speed override, alarms, tool name,
    /// work object name, running/stopped indicator.
    /// </summary>
    public partial class GlobalStateViewModel : ObservableObject
    {
        // ── Mode ────────────────────────────────────────────────────

        public enum OperatingMode { Simulator, Real }

        [ObservableProperty] private OperatingMode _currentMode = OperatingMode.Simulator;
        [ObservableProperty] private string _modeDisplayText = "Simulator";

        // ── Execution State ─────────────────────────────────────────

        [ObservableProperty] private bool _isRunning;
        [ObservableProperty] private string _executionStateText = "Stopped";

        // ── Speed Override ──────────────────────────────────────────

        private double _speedOverridePercent = 100.0;
        public double SpeedOverridePercent
        {
            get => _speedOverridePercent;
            set
            {
                _speedOverridePercent = System.Math.Clamp(value, 1.0, 100.0);
                OnPropertyChanged();
            }
        }

        // ── Tool & Work Object ──────────────────────────────────────

        [ObservableProperty] private string _activeToolName = "Tool0";
        [ObservableProperty] private string _activeWorkObjectName = "Wobj0";

        // ── Alarm ───────────────────────────────────────────────────

        [ObservableProperty] private bool _hasAlarm;
        [ObservableProperty] private string _alarmMessage = string.Empty;

        // ── Units ───────────────────────────────────────────────────

        [ObservableProperty] private string _unitSystem = "mm";

        // ── Commands ────────────────────────────────────────────────

        public event System.Action<OperatingMode>? ModeChangeRequested;

        [RelayCommand]
        private void ToggleMode()
        {
            var targetMode = CurrentMode == OperatingMode.Simulator 
                ? OperatingMode.Real 
                : OperatingMode.Simulator;
            
            ModeChangeRequested?.Invoke(targetMode);
        }

        public void SetMode(OperatingMode mode)
        {
            CurrentMode = mode;
            ModeDisplayText = mode == OperatingMode.Simulator ? "Simulator" : "Real";
        }

        [RelayCommand]
        private void ClearAlarm()
        {
            HasAlarm = false;
            AlarmMessage = string.Empty;
        }

        public void SetAlarm(string message)
        {
            HasAlarm = true;
            AlarmMessage = message;
        }

        public void UpdateExecutionState(bool running)
        {
            IsRunning = running;
            ExecutionStateText = running ? "Running" : "Stopped";
        }
    }
}
