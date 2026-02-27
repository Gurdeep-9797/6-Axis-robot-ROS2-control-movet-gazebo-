using System;
using System.Collections.ObjectModel;
using System.Linq;
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using TeachPendant_WPF.SceneGraph;
using TeachPendant_WPF.Models;

namespace TeachPendant_WPF.ViewModels
{
    /// <summary>
    /// Manages the robot kinematic chain, joint sliders, FK/IK calls,
    /// and the connection to SceneGraph.RobotNode.
    /// </summary>
    public partial class RobotViewModel : ObservableObject
    {
        private readonly SceneGraphManager _sceneGraph;

        // ── Joint Data (Bound to UI Sliders) ────────────────────────

        public ObservableCollection<JointSliderItem> JointSliders { get; } = new();

        // ── TCP Readout ─────────────────────────────────────────────

        [ObservableProperty] private double _tcpX;
        [ObservableProperty] private double _tcpY;
        [ObservableProperty] private double _tcpZ;
        [ObservableProperty] private double _tcpRx;
        [ObservableProperty] private double _tcpRy;
        [ObservableProperty] private double _tcpRz;

        // ── Force/Torque Readout ────────────────────────────────────

        [ObservableProperty] private double _fx;
        [ObservableProperty] private double _fy;
        [ObservableProperty] private double _fz;

        // ── Speed Settings ──────────────────────────────────────────

        [ObservableProperty] private double _speedPercent = 100.0;
        [ObservableProperty] private double _acceleration = 180.0;

        // ── IK Reachability ─────────────────────────────────────────

        [ObservableProperty] private bool _isTargetReachable = true;

        // ── Construction ────────────────────────────────────────────

        public RobotViewModel(SceneGraphManager sceneGraph)
        {
            _sceneGraph = sceneGraph;

            // Initialize joint slider items from the robot node
            if (_sceneGraph.Robot != null)
            {
                InitializeSliders(_sceneGraph.Robot);
            }
        }

        public void InitializeSliders(RobotNode robot)
        {
            JointSliders.Clear();
            for (int i = 0; i < robot.DOF; i++)
            {
                var joint = robot.Joints[i];
                var slider = new JointSliderItem
                {
                    Name = $"J{i + 1}",
                    Min = joint.MinLimit,
                    Max = joint.MaxLimit,
                    Angle = joint.CurrentAngle,
                    JointIndex = i
                };

                // When slider value changes, update the scene graph joint
                slider.PropertyChanged += (s, e) =>
                {
                    if (e.PropertyName == nameof(JointSliderItem.Angle))
                    {
                        var item = (JointSliderItem)s!;
                        if (_sceneGraph.Robot != null && item.JointIndex < _sceneGraph.Robot.DOF)
                        {
                            _sceneGraph.Robot.Joints[item.JointIndex].CurrentAngle = item.Angle;
                            UpdateTCPReadout();
                        }
                    }
                };

                JointSliders.Add(slider);
            }
        }

        // ── Joint Angle Application ─────────────────────────────────

        /// <summary>
        /// Apply a complete set of joint angles (from execution engine or encoder).
        /// Updates both the scene graph and the UI slider readouts.
        /// </summary>
        public void ApplyJointAngles(double[] angles)
        {
            if (_sceneGraph.Robot == null) return;

            _sceneGraph.Robot.ApplyJointAngles(angles);

            // Sync sliders
            for (int i = 0; i < Math.Min(angles.Length, JointSliders.Count); i++)
            {
                JointSliders[i].Angle = angles[i];
            }

            UpdateTCPReadout();
        }

        /// <summary>
        /// Read current joint angles from the scene graph.
        /// </summary>
        public double[] GetJointAngles()
        {
            return _sceneGraph.Robot?.GetJointAngles() ?? Array.Empty<double>();
        }

        // ── TCP Update ──────────────────────────────────────────────

        private void UpdateTCPReadout()
        {
            if (_sceneGraph.Robot == null) return;

            var tcp = _sceneGraph.Robot.GetTCPPosition();
            TcpX = tcp.X;
            TcpY = tcp.Y;
            TcpZ = tcp.Z;

            // Rotation extraction is simplified here;
            // full quaternion → Euler will be implemented in Phase F
        }

        // ── Commands ────────────────────────────────────────────────

        [RelayCommand]
        private void JogJoint(string param)
        {
            if (string.IsNullOrEmpty(param) || param.Length < 2) return;
            if (_sceneGraph.Robot == null) return;

            bool isPositive = param[0] == '+';
            
            // Handle formats: "+J1", "-J2", "+1", "-1"
            string numPart = param.Substring(1);
            if (numPart.StartsWith("J", StringComparison.OrdinalIgnoreCase))
                numPart = numPart.Substring(1);

            if (!int.TryParse(numPart, out int jointNum)) return;

            int idx = jointNum - 1;
            if (idx < 0 || idx >= JointSliders.Count) return;

            double step = 1.0; // 1 degree step
            JointSliders[idx].Angle += isPositive ? step : -step;
        }

        [RelayCommand]
        private void SetInitialPosition()
        {
            // Capture current joint angles as the initial position node
            // This will be used by ProgramViewModel to create an InitializeNode
        }

        [RelayCommand]
        private void GoHome()
        {
            ApplyJointAngles(new double[] { 0, 0, 0, 0, 0, 0 });
        }
    }

    /// <summary>
    /// Data item for a single joint slider in the Motion Parameters panel.
    /// </summary>
    public partial class JointSliderItem : ObservableObject
    {
        [ObservableProperty] private string _name = string.Empty;
        [ObservableProperty] private double _min = -180;
        [ObservableProperty] private double _max = 180;
        [ObservableProperty] private double _angle;

        public int JointIndex { get; set; }
    }
}
