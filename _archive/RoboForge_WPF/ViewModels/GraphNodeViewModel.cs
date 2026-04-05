using System;
using System.ComponentModel;
using System.Windows;
using TeachPendant_WPF.Models;

namespace RoboForge_WPF.ViewModels
{
    public class GraphNodeViewModel : INotifyPropertyChanged
    {
        public event PropertyChangedEventHandler? PropertyChanged;

        private double _x;
        public double X { get => _x; set { _x = value; OnPropertyChanged(nameof(X)); } }

        private double _y;
        public double Y { get => _y; set { _y = value; OnPropertyChanged(nameof(Y)); } }

        public string Title { get; set; } = "";
        public string Subtitle { get; set; } = "";
        
        public string NodeType { get; set; } = "MoveJ"; // Start, MoveJ, MoveL, SetDO
        
        // Associated instruction for the backend pipeline execution
        public RobotInstruction? NodeInstruction { get; set; }

        private bool _isActive;
        public bool IsActive { get => _isActive; set { _isActive = value; OnPropertyChanged(nameof(IsActive)); } }

        protected void OnPropertyChanged(string name) => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(name));
    }
}
