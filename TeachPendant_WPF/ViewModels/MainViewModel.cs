using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Input;
using TeachPendant_WPF.Models;
using TeachPendant_WPF.Services;

namespace TeachPendant_WPF.ViewModels
{
    public partial class MainViewModel : ObservableObject
    {
        [ObservableProperty]
        private RobotState _currentState;

        [ObservableProperty]
        private RobotProgram _currentProgram;

        private readonly IRobotDriver _driver;
        private readonly ExecutionEngine _engine;
        private readonly DatabaseService _dbService;

        // Dynamic 3D Expand Settings
        [ObservableProperty] private GridLength _leftNavWidth = new GridLength(220, GridUnitType.Pixel);
        [ObservableProperty] private GridLength _cmdPaletteWidth = new GridLength(220, GridUnitType.Pixel);
        [ObservableProperty] private GridLength _programWidth = new GridLength(350, GridUnitType.Pixel);
        [ObservableProperty] private GridLength _viewportWidth = new GridLength(1, GridUnitType.Star);
        private bool _isWorkAreaExpanded = false;

        public MainViewModel()
        {
            // Initialize Core Models
            CurrentState = new RobotState();
            CurrentProgram = new RobotProgram();
            
            _dbService = new DatabaseService();

            // Setup Services (Defaulting to Simulation for now)
            _driver = new SimulationDriver();
            _driver.StateUpdated += OnStateUpdated;
            _driver.Connect();

            _engine = new ExecutionEngine(_driver);
            _engine.ExecutionStateChanged += (isRunning) => CurrentState.IsRunning = isRunning;
            _engine.ActiveLineChanged += (line) => CurrentState.ActiveLineNumber = line;

            // Load data asynchronously without blocking constructor
            _ = LoadInitialDataAsync();
        }

        private async Task LoadInitialDataAsync()
        {
            var programs = await _dbService.GetAllProgramsAsync();
            if (programs.Count > 0)
            {
                var latest = programs[0];
                var fullProgram = await _dbService.GetProgramByIdAsync(latest.Id);
                // In Phase 4 we will convert NodeEntity into WPF instruction Nodes here
                System.Diagnostics.Debug.WriteLine($"Loaded DB Program: {fullProgram?.Name}");
            }
            else
            {
                // Fallback to sample logic
                CurrentProgram.LoadSampleProgram();
                await _dbService.CreateNewProgramAsync("Default_Program_01");
            }
        }

        [RelayCommand]
        public async Task SaveProgramToDb()
        {
            var programs = await _dbService.GetAllProgramsAsync();
            if (programs.Count > 0)
            {
                var prog = programs[0];
                prog.Name = "Updated_Program_" + System.DateTime.Now.ToString("HHmmss");
                await _dbService.SaveProgramAsync(prog);
            }
        }

        private void OnStateUpdated(RobotState updatedState)
        {
            // Update the bound properties
            CurrentState.J1 = updatedState.J1;
            CurrentState.J2 = updatedState.J2;
            CurrentState.J3 = updatedState.J3;
            CurrentState.J4 = updatedState.J4;
            CurrentState.J5 = updatedState.J5;
            CurrentState.J6 = updatedState.J6;
        }

        [RelayCommand]
        public void ToggleMode()
        {
            CurrentState.IsMultiAxisJog = !CurrentState.IsMultiAxisJog;
            OnPropertyChanged(nameof(CurrentState));
        }

        [RelayCommand]
        public async Task StartProgram()
        {
            await _engine.RunProgramAsync(CurrentProgram, CurrentState);
        }

        [RelayCommand]
        public void StopProgram()
        {
            _engine.Stop();
        }

        [RelayCommand]
        public void CloseWindow()
        {
            System.Windows.Application.Current.Shutdown();
        }

        [RelayCommand]
        public void AddInstruction(string type)
        {
            RobotInstruction newInstr = null;
            switch (type?.ToUpper())
            {
                case "WAIT": newInstr = new WaitInstruction(1000); break;
                case "LIN": newInstr = new LinInstruction("p1", 50, 0); break;
                case "PTP": newInstr = new PtpInstruction("p1", 50); break;
                case "ARC":
                case "CIRCLE": newInstr = new CircleInstruction("p1", "p2", 50); break;
                case "WHILE": newInstr = new WhileInstruction("True"); break;
                case "SETDO": newInstr = new SetDOInstruction(1, 1); break;
                case "DOFILE": newInstr = new NewDoFileInstruction("file.lua"); break;
            }

            if (newInstr != null)
            {
                CurrentProgram.Instructions.Add(newInstr);
            }
        }

        [RelayCommand]
        public void MinimizeWindow()
        {
            if (System.Windows.Application.Current.MainWindow != null)
                System.Windows.Application.Current.MainWindow.WindowState = System.Windows.WindowState.Minimized;
        }

        [RelayCommand]
        public void MaximizeWindow()
        {
            if (System.Windows.Application.Current.MainWindow != null)
            {
                var window = System.Windows.Application.Current.MainWindow;
                if (window.WindowState == System.Windows.WindowState.Maximized)
                    window.WindowState = System.Windows.WindowState.Normal;
                else
                    window.WindowState = System.Windows.WindowState.Maximized;
            }
        }

        [RelayCommand]
        public void ToggleWorkAreaConfig()
        {
            _isWorkAreaExpanded = !_isWorkAreaExpanded;
            if (_isWorkAreaExpanded)
            {
                // Collapse left panels, give 3D maximum space
                LeftNavWidth = new GridLength(60, GridUnitType.Pixel);
                CmdPaletteWidth = new GridLength(0, GridUnitType.Pixel);
                ProgramWidth = new GridLength(0, GridUnitType.Pixel);
                ViewportWidth = new GridLength(1, GridUnitType.Star);
            }
            else
            {
                // Restore proportions
                LeftNavWidth = new GridLength(220, GridUnitType.Pixel);
                CmdPaletteWidth = new GridLength(160, GridUnitType.Pixel);
                ProgramWidth = new GridLength(400, GridUnitType.Pixel);
                ViewportWidth = new GridLength(1, GridUnitType.Star);
            }
        }

        [RelayCommand]
        public void JogJoint(string param)
        {
            if (param.Length < 2) return;
            
            int joint = int.Parse(param.Substring(1));
            bool isPositive = param[0] == '+';
            double step = 1.0; // 1 degree step

            var currentPositions = new double[] { CurrentState.J1, CurrentState.J2, CurrentState.J3, CurrentState.J4, CurrentState.J5, CurrentState.J6 };
            
            if (isPositive)
                currentPositions[joint - 1] += step;
            else
                currentPositions[joint - 1] -= step;

            _driver.SendJointPositions(currentPositions);
        }
    }
}
