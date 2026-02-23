using System.ComponentModel;
using System.IO;
using System.Windows;
using TeachPendant_WPF.ViewModels;

namespace TeachPendant_WPF.Views
{
    public partial class MainWindow : Window
    {
        private URDFVisualizer _urdfVisualizer;
        private MainViewModel _viewModel;

        public MainWindow()
        {
            InitializeComponent();
            
            _viewModel = new MainViewModel();
            DataContext = _viewModel;

            // Initialize 3D Visualizer and attach to the Helix Viewport
            string urdfPath = System.IO.Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "Assets", "robot.urdf");
            _urdfVisualizer = new URDFVisualizer(urdfPath);
            RobotModelVisual.Children.Add(_urdfVisualizer.RootVisual);

            // Subscribe to RobotState changes to update 3D Kinematics in real-time
            _viewModel.CurrentState.PropertyChanged += CurrentState_PropertyChanged;
        }

        private void CurrentState_PropertyChanged(object? sender, PropertyChangedEventArgs e)
        {
            var state = _viewModel.CurrentState;
            _urdfVisualizer.UpdateJoints(state.J1, state.J2, state.J3, state.J4, state.J5, state.J6);
        }
    }
}
