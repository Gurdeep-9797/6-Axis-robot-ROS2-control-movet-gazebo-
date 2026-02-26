using System;
using System.ComponentModel;
using System.IO;
using System.Windows;
using System.Windows.Input;
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
            string urdfPath = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "Assets", "robot.urdf");
            _urdfVisualizer = new URDFVisualizer(urdfPath);
            RobotModelVisual.Children.Add(_urdfVisualizer.RootVisual);

            // ── CRITICAL: Subscribe to BOTH CurrentState and JointSlider changes ──
            // Legacy pipeline: CurrentState.PropertyChanged → UpdateJoints
            _viewModel.CurrentState.PropertyChanged += CurrentState_PropertyChanged;

            // New pipeline: JointSlider.Angle changes → sync to CurrentState → UpdateJoints
            foreach (var slider in _viewModel.RobotVM.JointSliders)
            {
                slider.PropertyChanged += JointSlider_PropertyChanged;
            }

            // Also re-subscribe when sliders collection changes (e.g. after URDF reload)
            _viewModel.RobotVM.JointSliders.CollectionChanged += (s, e) =>
            {
                if (e.NewItems != null)
                {
                    foreach (JointSliderItem item in e.NewItems)
                        item.PropertyChanged += JointSlider_PropertyChanged;
                }
            };

            // ── Register keyboard shortcuts ──
            RegisterKeyBindings();
        }

        // ── Window Drag (fixes custom chrome not draggable) ──────────
        protected override void OnMouseLeftButtonDown(MouseButtonEventArgs e)
        {
            base.OnMouseLeftButtonDown(e);
            // Only drag from the top bar area (first 48px)
            if (e.GetPosition(this).Y < 48)
            {
                DragMove();
            }
        }

        // ── Joint Slider → 3D Viewport Pipeline ─────────────────────
        private void JointSlider_PropertyChanged(object? sender, PropertyChangedEventArgs e)
        {
            if (e.PropertyName != nameof(JointSliderItem.Angle)) return;

            // Sync all slider values to CurrentState (legacy) so the 3D view updates
            var sliders = _viewModel.RobotVM.JointSliders;
            var state = _viewModel.CurrentState;

            if (sliders.Count >= 6)
            {
                state.J1 = sliders[0].Angle;
                state.J2 = sliders[1].Angle;
                state.J3 = sliders[2].Angle;
                state.J4 = sliders[3].Angle;
                state.J5 = sliders[4].Angle;
                state.J6 = sliders[5].Angle;
            }

            // Direct 3D update for immediate feedback
            UpdateRobotVisual();
        }

        private void CurrentState_PropertyChanged(object? sender, PropertyChangedEventArgs e)
        {
            UpdateRobotVisual();
        }

        private void UpdateRobotVisual()
        {
            var state = _viewModel.CurrentState;
            Dispatcher.BeginInvoke(() =>
            {
                _urdfVisualizer.UpdateJoints(
                    state.J1, state.J2, state.J3,
                    state.J4, state.J5, state.J6);
            });
        }

        // ── Keyboard Shortcuts ──────────────────────────────────────
        private void RegisterKeyBindings()
        {
            // F5 → Start Program
            InputBindings.Add(new KeyBinding(
                _viewModel.StartProgramCommand, Key.F5, ModifierKeys.None));

            // Shift+F5 → Stop Program
            InputBindings.Add(new KeyBinding(
                _viewModel.StopProgramCommand, Key.F5, ModifierKeys.Shift));

            // Ctrl+S → Save
            InputBindings.Add(new KeyBinding(
                _viewModel.SaveProgramToDbCommand, Key.S, ModifierKeys.Control));

            // Ctrl+P → Add Point
            InputBindings.Add(new KeyBinding(
                _viewModel.WorkTree.AddPointCommand, Key.P, ModifierKeys.Control));

            // Ctrl+N → New Program
            InputBindings.Add(new KeyBinding(
                _viewModel.NewProgramCommand, Key.N, ModifierKeys.Control));

            // Ctrl+R → Toggle Real/Sim
            InputBindings.Add(new KeyBinding(
                _viewModel.GlobalState.ToggleModeCommand, Key.R, ModifierKeys.Control));

            // Ctrl+H → Go Home
            InputBindings.Add(new KeyBinding(
                _viewModel.RobotVM.GoHomeCommand, Key.H, ModifierKeys.Control));

            // Ctrl+Shift+P → Command Palette
            InputBindings.Add(new KeyBinding(
                _viewModel.ToggleCommandPaletteCommand, Key.P, ModifierKeys.Control | ModifierKeys.Shift));
        }
    }
}
