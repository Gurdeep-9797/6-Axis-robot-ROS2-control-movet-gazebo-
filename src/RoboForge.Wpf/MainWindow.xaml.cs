using System;
using System.Collections.ObjectModel;
using System.Collections.Specialized;
using System.Reactive.Disposables;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using System.Windows.Shapes;
using HelixToolkit.Wpf;
using RoboForge.Wpf.AST;
using RoboForge.Wpf.Core;
using RoboForge.Wpf.Models;

namespace RoboForge.Wpf
{
    // ── Tree Node Models for Scene Tree ────────────────────────────────────
    public class RobotTreeNode
    {
        public string Name { get; set; } = "";
        public ObservableCollection<object> Children { get; } = new();
    }
    public class LinkTreeNode
    {
        public string Name { get; set; } = "";
        public ObservableCollection<object> Children { get; } = new();
    }
    public class JointTreeNode
    {
        public string Name { get; set; } = "";
        public string JointType { get; set; } = "";
    }

    // ── Diagnostics Message Model ──────────────────────────────────────────
    public class DiagMessage
    {
        public string Time { get; set; } = "";
        public string Level { get; set; } = "";
        public Brush LevelColor { get; set; } = Brushes.Gray;
        public string Message { get; set; } = "";
    }

    /// <summary>
    /// Main window code-behind — wires together scene tree, block editor, execution engine, and state bus.
    /// </summary>
    public partial class MainWindow : Window
    {
        private RobotModel _robotModel = new();
        private ProgramNode _program = new();
        private IExecutionEngine _engine = new GhostExecutionEngine(6);
        private CancellationTokenSource? _execCts;
        private CompositeDisposable _subscriptions = new();
        private bool _isRunning;

        public MainWindow()
        {
            InitializeComponent();
            Loaded += MainWindow_Loaded;
            Closing += MainWindow_Closing;
        }

        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            BuildSceneTree();
            BuildBlockEditor();
            SubscribeToStateBus();
            AddDiagnostic("info", "RoboForge IDE initialized — Ghost mode active");
            AddDiagnostic("info", "Scene tree loaded with default 6-axis robot");
        }

        private void MainWindow_Closing(object? sender, System.ComponentModel.CancelEventArgs e)
        {
            _execCts?.Cancel();
            _subscriptions.Dispose();
        }

        // ═══ SCENE TREE ═════════════════════════════════════════════════════

        private void BuildSceneTree()
        {
            var root = new RobotTreeNode { Name = _robotModel.Name };

            // Add links and joints
            for (int i = 1; i <= 6; i++)
            {
                var link = new LinkTreeNode { Name = $"Link_{i}" };
                var joint = new JointTreeNode { Name = $"Joint_{i}", JointType = "Revolute" };
                link.Children.Add(joint);
                root.Children.Add(link);
            }

            // Add sensors
            var sensorsNode = new LinkTreeNode { Name = "Sensors" };
            root.Children.Add(sensorsNode);

            // Add end effector
            root.Children.Add(new JointTreeNode { Name = "End Effector", JointType = "Gripper" });

            // Add coordinate frames
            var framesNode = new LinkTreeNode { Name = "Coordinate Frames" };
            framesNode.Children.Add(new JointTreeNode { Name = "world", JointType = "Frame" });
            framesNode.Children.Add(new JointTreeNode { Name = "base", JointType = "Frame" });
            framesNode.Children.Add(new JointTreeNode { Name = "tool0", JointType = "Frame" });
            root.Children.Add(framesNode);

            SceneTree.Items.Add(root);
            // SceneTree.SelectedItem = root; // Read-only, skip for now
        }

        private void SceneTree_SelectedItemChanged(object sender, RoutedPropertyChangedEventArgs<object> e)
        {
            // Highlight selected node in 3D viewport
            AddDiagnostic("info", $"Selected: {e.NewValue?.GetType().Name}");
        }

        // ═══ BLOCK EDITOR ═══════════════════════════════════════════════════

        private void BuildBlockEditor()
        {
            BlockCanvas.Children.Clear();

            // Add some default blocks
            AddBlockToCanvas("MoveJ", "#4A90D9", "Move to home position", "Speed: 50% | Zone: z50");
            AddBlockToCanvas("MoveL", "#4A90D9", "Linear approach to part", "Speed: 200mm/s | Blend: 10mm");
            AddBlockToCanvas("GripperClose", "#22C55E", "Grasp the part", "Width: 0mm | Force: 30N");
            AddBlockToCanvas("MoveL", "#4A90D9", "Retract with part", "Speed: 150mm/s");
            AddBlockToCanvas("Wait", "#8B5CF6", "Wait for conveyor", "Duration: 500ms");
            AddBlockToCanvas("GripperOpen", "#22C55E", "Release the part", "Width: 80mm");
        }

        private void AddBlockToCanvas(string name, string color, string description, string parameters)
        {
            var border = new Border
            {
                Background = new SolidColorBrush(Color.FromArgb(20, 255, 255, 255)),
                BorderBrush = new SolidColorBrush((Color)ColorConverter.ConvertFromString(color)),
                BorderThickness = new Thickness(0, 0, 0, 3),
                CornerRadius = new CornerRadius(6),
                Margin = new Thickness(0, 4, 0, 4),
                Padding = new Thickness(12),
                Cursor = System.Windows.Input.Cursors.Hand,
                ToolTip = description,
            };

            var grid = new Grid();
            grid.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(50) });
            grid.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(1, GridUnitType.Star) });
            grid.ColumnDefinitions.Add(new ColumnDefinition { Width = GridLength.Auto });

            // Color badge
            var badge = new Border
            {
                Background = new SolidColorBrush((Color)ColorConverter.ConvertFromString(color)),
                CornerRadius = new CornerRadius(4),
                Width = 36, Height = 24,
                Child = new TextBlock { Text = name.Substring(0, Math.Min(3, name.Length)), Foreground = Brushes.White, FontSize = 10, FontWeight = FontWeights.SemiBold, HorizontalAlignment = HorizontalAlignment.Center, VerticalAlignment = VerticalAlignment.Center }
            };
            Grid.SetColumn(badge, 0);
            grid.Children.Add(badge);

            // Content
            var content = new StackPanel { Margin = new Thickness(12, 0, 0, 0) };
            content.Children.Add(new TextBlock { Text = name, Foreground = Brushes.White, FontSize = 13, FontWeight = FontWeights.SemiBold });
            content.Children.Add(new TextBlock { Text = parameters, Foreground = new SolidColorBrush(Color.FromArgb(255, 113, 113, 122)), FontSize = 10, Margin = new Thickness(0, 2, 0, 0) });
            Grid.SetColumn(content, 1);
            grid.Children.Add(content);

            // Execution indicator (hidden by default)
            var indicator = new Ellipse { Width = 8, Height = 8, Fill = Brushes.Transparent, Margin = new Thickness(8, 0, 0, 0) };
            Grid.SetColumn(indicator, 2);
            grid.Children.Add(indicator);

            border.Child = grid;
            border.MouseEnter += (s, e) => border.Background = new SolidColorBrush(Color.FromArgb(40, 255, 255, 255));
            border.MouseLeave += (s, e) => border.Background = new SolidColorBrush(Color.FromArgb(20, 255, 255, 255));
            border.MouseDown += (s, e) => { /* Select block */ };

            BlockCanvas.Children.Add(border);
        }

        // ═══ STATE BUS SUBSCRIPTIONS ════════════════════════════════════════

        private void SubscribeToStateBus()
        {
            // Subscribe to state bus for 3D viewport updates
            var sub = StateBus.StateStream.Subscribe(state =>
            {
                Dispatcher.InvokeAsync(() =>
                {
                    UpdateRobotPose(state.JointAngles);
                    HighlightActiveBlock(state.ActiveNodeId);
                    UpdateStatusBar(state);
                }, System.Windows.Threading.DispatcherPriority.Render);
            });
            _subscriptions.Add(sub);
        }

        private void UpdateRobotPose(double[] angles)
        {
            // Find the robot model in the viewport and update joint transforms
            // This is a simplified version — in production, walk the scene graph
            if (RobotViewport.Children.Count > 3 && RobotViewport.Children[3] is ModelVisual3D robotModel)
            {
                // Update joint rotations based on angles
                // In production, this walks the TRS hierarchy
            }
        }

        private void HighlightActiveBlock(string nodeId)
        {
            // Reset all blocks
            foreach (var child in BlockCanvas.Children)
            {
                if (child is Border border)
                    border.BorderThickness = new Thickness(0, 0, 0, 3);
            }

            // Highlight active block (simplified — match by position for now)
            // In production, each block would store its NodeId
        }

        private void UpdateStatusBar(ExecutionStateUpdate state)
        {
            // Status bar is updated via direct property binding in XAML
            // For now, just update diagnostics
        }

        // ═══ EXECUTION CONTROL ══════════════════════════════════════════════

        private async void RunProgram_Click(object sender, RoutedEventArgs e)
        {
            if (_isRunning) return;
            _isRunning = true;
            _execCts = new CancellationTokenSource();

            AddDiagnostic("info", "Starting program execution (Ghost mode)...");
            StateBus.UpdateProgramState(ProgramState.Running);

            // Compile AST to instruction list
            var instructions = Compiler.Compile(_program);
            AddDiagnostic("info", $"Compiled {instructions.Count} instructions");

            try
            {
                await _engine.Run(instructions, _execCts.Token);
                AddDiagnostic("success", "Program completed successfully");
            }
            catch (OperationCanceledException)
            {
                AddDiagnostic("warning", "Program stopped by user");
            }
            catch (Exception ex)
            {
                AddDiagnostic("error", $"Execution error: {ex.Message}");
                StateBus.UpdateProgramState(ProgramState.Error, ex.Message);
            }
            finally
            {
                _isRunning = false;
                StateBus.UpdateProgramState(ProgramState.Idle);
            }
        }

        private void StopProgram_Click(object sender, RoutedEventArgs e)
        {
            _execCts?.Cancel();
            _engine.Stop();
            AddDiagnostic("info", "Program stop requested");
        }

        // ═══ TAB SWITCHING ══════════════════════════════════════════════════

        private void TabBlocks_Click(object sender, RoutedEventArgs e)
        {
            BlockEditorPanel.Visibility = Visibility.Visible;
            ScriptEditorPanel.Visibility = Visibility.Collapsed;
            TabBlocks.Background = new SolidColorBrush((Color)ColorConverter.ConvertFromString("#27272A"));
            TabBlocks.Foreground = new SolidColorBrush((Color)ColorConverter.ConvertFromString("#E4E4E7"));
            TabScript.Background = Brushes.Transparent;
            TabScript.Foreground = new SolidColorBrush((Color)ColorConverter.ConvertFromString("#71717A"));
        }

        private void TabScript_Click(object sender, RoutedEventArgs e)
        {
            BlockEditorPanel.Visibility = Visibility.Collapsed;
            ScriptEditorPanel.Visibility = Visibility.Visible;
            TabScript.Background = new SolidColorBrush((Color)ColorConverter.ConvertFromString("#27272A"));
            TabScript.Foreground = new SolidColorBrush((Color)ColorConverter.ConvertFromString("#E4E4E7"));
            TabBlocks.Background = Brushes.Transparent;
            TabBlocks.Foreground = new SolidColorBrush((Color)ColorConverter.ConvertFromString("#71717A"));
        }

        // ═══ HOMING ═════════════════════════════════════════════════════════

        private void HomingButton_Click(object sender, RoutedEventArgs e)
        {
            AddDiagnostic("info", "Homing sequence initiated — checking pre-flight...");
            // In production, this opens the full homing modal described in Section 8
        }

        // ═══ DIAGNOSTICS ════════════════════════════════════════════════════

        private void AddDiagnostic(string level, string message)
        {
            var time = DateTime.Now.ToString("HH:mm:ss");
            var color = level switch
            {
                "error" => "#EF4444",
                "warning" => "#E8A020",
                "success" => "#22C55E",
                _ => "#3B82F6"
            };

            DiagnosticsList.Items.Add(new DiagMessage
            {
                Time = time,
                Level = level.ToUpper(),
                LevelColor = new SolidColorBrush((Color)ColorConverter.ConvertFromString(color)),
                Message = message,
            });

            // Auto-scroll to bottom
            var scrollViewer = FindVisualChild<ScrollViewer>(DiagnosticsList);
            scrollViewer?.ScrollToBottom();
        }

        private void CollapseDiagnostics_Click(object sender, RoutedEventArgs e)
        {
            // Toggle diagnostics panel visibility
            // In production, animate the collapse
        }

        // ═══ TITLE BAR ══════════════════════════════════════════════════════

        private void Minimize_Click(object sender, RoutedEventArgs e) => WindowState = WindowState.Minimized;
        private void Maximize_Click(object sender, RoutedEventArgs e) => WindowState = WindowState == WindowState.Maximized ? WindowState.Normal : WindowState.Maximized;
        private void Close_Click(object sender, RoutedEventArgs e) => Close();

        // ═══ UTILITY ════════════════════════════════════════════════════════

        private static T? FindVisualChild<T>(DependencyObject parent) where T : DependencyObject
        {
            for (int i = 0; i < VisualTreeHelper.GetChildrenCount(parent); i++)
            {
                var child = VisualTreeHelper.GetChild(parent, i);
                if (child is T typed) return typed;
                var result = FindVisualChild<T>(child);
                if (result != null) return result;
            }
            return null;
        }
    }
}
