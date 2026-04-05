using System;
using System.Collections.ObjectModel;
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;

namespace RoboForge_WPF.ViewModels
{
    public partial class JointViewModel : ObservableObject
    {
        [ObservableProperty] private string _name = "";
        [ObservableProperty] private double _min = -180.0;
        [ObservableProperty] private double _max = 180.0;
        
        private double _value;
        public double Value
        {
            get => _value;
            set
            {
                if (SetProperty(ref _value, value))
                {
                    ValueChanged?.Invoke(this, EventArgs.Empty);
                }
            }
        }

        public event EventHandler? ValueChanged;
    }

    public partial class JogViewModel : ObservableObject
    {
        public ObservableCollection<JointViewModel> Joints { get; } = new ObservableCollection<JointViewModel>();

        [ObservableProperty] private double _stepSize = 1.0;

        public event EventHandler? JogRequested;

        public JogViewModel()
        {
            for (int i = 1; i <= 6; i++)
            {
                var joint = new JointViewModel { Name = $"J{i}", Value = 0 };
                joint.ValueChanged += (s, e) => JogRequested?.Invoke(this, EventArgs.Empty);
                Joints.Add(joint);
            }
        }

        [RelayCommand]
        private void JogMinus(string jointName) => DoJog(jointName, -1);

        [RelayCommand]
        private void JogPlus(string jointName) => DoJog(jointName, 1);

        private void DoJog(string jName, int dir)
        {
            if (string.IsNullOrEmpty(jName) || !jName.StartsWith("J")) return;
            if (int.TryParse(jName.Substring(1), out int jIndex))
            {
                var joint = Joints[jIndex - 1];
                joint.Value = Math.Clamp(joint.Value + (StepSize * dir), joint.Min, joint.Max);
            }
        }

        [RelayCommand]
        private void ZeroAll()
        {
            foreach (var j in Joints)
            {
                j.Value = 0;
            }
        }
    }
}
