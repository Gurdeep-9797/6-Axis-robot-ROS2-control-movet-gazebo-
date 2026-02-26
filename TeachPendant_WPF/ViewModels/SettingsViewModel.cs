using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using TeachPendant_WPF.Services;

namespace TeachPendant_WPF.ViewModels
{
    /// <summary>
    /// Manages the initialization wizard and persistent configuration.
    /// On first launch: opens setup wizard.
    /// On subsequent launches: loads cached config from SQLite.
    /// </summary>
    public partial class SettingsViewModel : ObservableObject
    {
        private readonly DatabaseService _dbService;

        // ── Wizard State ────────────────────────────────────────────

        [ObservableProperty] private bool _isWizardOpen;
        [ObservableProperty] private int _wizardStep;   // 0-based step index
        [ObservableProperty] private int _totalSteps = 10;

        // ── Configuration Values ────────────────────────────────────

        // Step 1: Units
        [ObservableProperty] private string _unitSystem = "mm";

        // Step 2: Robot Model
        [ObservableProperty] private string _robotModel = "FR-6DOF";

        // Step 3: Encoder Specs
        [ObservableProperty] private int _encoderResolution = 4096;
        [ObservableProperty] private double _gearRatio = 100.0;

        // Step 4: Communication Mode
        [ObservableProperty] private string _communicationMode = "EtherCAT";

        // Step 5: Joint Limits
        [ObservableProperty] private double _j1Min = -170, _j1Max = 170;
        [ObservableProperty] private double _j2Min = -120, _j2Max = 120;
        [ObservableProperty] private double _j3Min = -170, _j3Max = 170;
        [ObservableProperty] private double _j4Min = -180, _j4Max = 180;
        [ObservableProperty] private double _j5Min = -120, _j5Max = 120;
        [ObservableProperty] private double _j6Min = -360, _j6Max = 360;

        // Step 6: Mounting Orientation
        [ObservableProperty] private string _mountingOrientation = "Floor";

        // Step 7: Base Frame Offset
        [ObservableProperty] private double _baseOffsetX, _baseOffsetY, _baseOffsetZ;

        // Step 8: Tool TCP
        [ObservableProperty] private double _toolX, _toolY, _toolZ;
        [ObservableProperty] private double _toolRx, _toolRy, _toolRz;

        // Step 9: Work Object
        [ObservableProperty] private double _wobjX, _wobjY, _wobjZ;

        // Step 10: Simulation Default
        [ObservableProperty] private bool _defaultToSimulation = true;

        // ── Robot URDF Path ──────────────────────────────────────────
        [ObservableProperty] private string _urdfPath = string.Empty;

        // ── Encoder Advanced Config ──────────────────────────────────
        [ObservableProperty] private int _countsPerRevolution = 4096;
        [ObservableProperty] private bool _directionInvert;
        [ObservableProperty] private double _offsetCalibration;

        // ── Computed ────────────────────────────────────────────────

        public bool IsFirstStep => WizardStep == 0;
        public bool IsLastStep => WizardStep == TotalSteps - 1;
        public string StepTitle => WizardStep switch
        {
            0 => "Units",
            1 => "Robot Model",
            2 => "Encoder Specs",
            3 => "Communication Mode",
            4 => "Joint Limits",
            5 => "Mounting Orientation",
            6 => "Base Frame Offset",
            7 => "Tool TCP",
            8 => "Work Object",
            9 => "Simulation Default",
            _ => "Setup"
        };

        // ── Construction ────────────────────────────────────────────

        public SettingsViewModel(DatabaseService dbService)
        {
            _dbService = dbService;
        }

        // ── Commands ────────────────────────────────────────────────

        [RelayCommand]
        private void NextStep()
        {
            if (WizardStep < TotalSteps - 1)
            {
                WizardStep++;
                OnPropertyChanged(nameof(IsFirstStep));
                OnPropertyChanged(nameof(IsLastStep));
                OnPropertyChanged(nameof(StepTitle));
            }
        }

        [RelayCommand]
        private void PreviousStep()
        {
            if (WizardStep > 0)
            {
                WizardStep--;
                OnPropertyChanged(nameof(IsFirstStep));
                OnPropertyChanged(nameof(IsLastStep));
                OnPropertyChanged(nameof(StepTitle));
            }
        }

        [RelayCommand]
        private async System.Threading.Tasks.Task FinishWizard()
        {
            // Save all config to database
            await SaveConfigToDatabase();
            IsWizardOpen = false;
        }

        [RelayCommand]
        private void CancelWizard()
        {
            IsWizardOpen = false;
        }

        // ── Persistence ─────────────────────────────────────────────

        public async System.Threading.Tasks.Task LoadConfigFromDatabase()
        {
            // In Phase C/D we will implement full deserialization from RobotConfigEntity
            // For now, use defaults
            IsWizardOpen = false;
        }

        public async System.Threading.Tasks.Task SaveConfigToDatabase()
        {
            // In Phase C/D we will implement full serialization to RobotConfigEntity
            // For now, stub
            await System.Threading.Tasks.Task.CompletedTask;
        }

        /// <summary>
        /// Called on application startup.
        /// If no config exists, opens the wizard.
        /// </summary>
        public async System.Threading.Tasks.Task CheckFirstRunAsync()
        {
            // Check DB for existing config
            // If none → open wizard
            // For now, always load defaults
            bool hasConfig = false; // Will query DB in Phase C
            if (!hasConfig)
            {
                IsWizardOpen = true;
                WizardStep = 0;
            }
            else
            {
                await LoadConfigFromDatabase();
            }
        }
    }
}
