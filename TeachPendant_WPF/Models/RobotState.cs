using CommunityToolkit.Mvvm.ComponentModel;

namespace TeachPendant_WPF.Models
{
    public partial class RobotState : ObservableObject
    {
        // 6-Axis Joint Angles in degrees
        [ObservableProperty] private double _j1;
        [ObservableProperty] private double _j2;
        [ObservableProperty] private double _j3;
        [ObservableProperty] private double _j4;
        [ObservableProperty] private double _j5;
        [ObservableProperty] private double _j6;

        // TCP Coordinates
        public double X { get; set; }
        public double Y { get; set; }
        public double Z { get; set; }
        public double RX { get; set; }
        public double RY { get; set; }
        public double RZ { get; set; }

        // Motion Settings
        public double SpeedPercent { get; set; } = 100.0;
        public double Acceleration { get; set; } = 180.0;
        public double Threshold { get; set; } = 30.0;
        
        // Mode parameters
        public bool IsMultiAxisJog { get; set; } = false;

        // Force/Torque (FT) Data
        public double Fx { get; set; }
        public double Fy { get; set; }
        public double Fz { get; set; }
        public double Tx { get; set; }
        public double Ty { get; set; }
        public double Tz { get; set; }

        // Execution Status
        // Execution Status
        [ObservableProperty] private bool _isRunning = false;
        [ObservableProperty] private int _activeLineNumber = 0;
    }
}
