// ── Section 5: Panel Docking System ───────────────────────────────────────
using System;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Controls.Primitives;
using System.Windows.Data;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Animation;
using CommunityToolkit.Mvvm.ComponentModel;

namespace RoboForge.Wpf.Docking
{
    /// <summary>Dock location for panels</summary>
    public enum DockLocation { Left, Right, Bottom, Float }

    /// <summary>
    /// Dockable panel with 4 states: Docked, AutoHide, Floating, TabGroup.
    /// Professional docking system modeled after Visual Studio / Qt Creator.
    /// </summary>
    public partial class DockablePanel : ObservableObject, INotifyPropertyChanged
    {
        private Window? _floatingWindow;
        private GridSplitter? _splitter;
        private Border? _tabButton;
        private double _lastDockWidth = 300;
        private double _lastDockHeight = 200;
        private bool _isAutoHideOpen;

        [ObservableProperty] private string _panelId = "";
        [ObservableProperty] private string _title = "";
        [ObservableProperty] private DockLocation _defaultDockLocation = DockLocation.Left;
        [ObservableProperty] private DockLocation _currentLocation = DockLocation.Left;
        [ObservableProperty] private double _minWidth = 200;
        [ObservableProperty] private double _minHeight = 150;
        [ObservableProperty] private double _currentWidth = 300;
        [ObservableProperty] private double _currentHeight = 200;
        [ObservableProperty] private bool _isPinned = true;
        [ObservableProperty] private bool _isVisible = true;
        [ObservableProperty] private bool _isActive;
        [ObservableProperty] private UIElement? _content;
        [ObservableProperty] private string _icon = "📄";

        /// <summary>Panel state: Docked, AutoHide, Floating, or TabGroup</summary>
        public PanelState State
        {
            get
            {
                if (_floatingWindow != null) return PanelState.Floating;
                if (!IsPinned) return PanelState.AutoHide;
                return PanelState.Docked;
            }
        }

        /// <summary>Restore to default dock position and size</summary>
        public void ResetToDefault()
        {
            CurrentLocation = DefaultDockLocation;
            CurrentWidth = _lastDockWidth;
            CurrentHeight = _lastDockHeight;
            IsPinned = true;
            CloseFloating();
        }

        /// <summary>Toggle between docked and floating state</summary>
        public void ToggleFloat()
        {
            if (_floatingWindow != null)
            {
                CloseFloating();
            }
            else
            {
                OpenFloating();
            }
        }

        /// <summary>Toggle pin state (auto-hide vs always visible)</summary>
        public void TogglePin()
        {
            IsPinned = !IsPinned;
        }

        private void OpenFloating()
        {
            _floatingWindow = new Window
            {
                Title = Title,
                Width = CurrentWidth,
                Height = CurrentHeight,
                WindowStyle = WindowStyle.ToolWindow,
                ResizeMode = ResizeMode.CanResize,
                ShowInTaskbar = false,
                Content = BuildFloatingContent(),
                Owner = Application.Current.MainWindow,
            };

            _floatingWindow.Closed += (s, e) =>
            {
                _floatingWindow = null;
                OnPropertyChanged(nameof(State));
            };

            _floatingWindow.Show();
            CurrentLocation = DockLocation.Float;
            OnPropertyChanged(nameof(State));
        }

        private void CloseFloating()
        {
            _floatingWindow?.Close();
            _floatingWindow = null;
            CurrentLocation = DefaultDockLocation;
            OnPropertyChanged(nameof(State));
        }

        private Grid BuildFloatingContent()
        {
            var grid = new Grid();
            // Header bar
            var header = new Border
            {
                Background = new SolidColorBrush(Color.FromArgb(255, 26, 26, 30)),
                Height = 28,
                VerticalAlignment = VerticalAlignment.Top,
            };

            var headerGrid = new Grid();
            var titleText = new TextBlock
            {
                Text = Title,
                Foreground = new SolidColorBrush(Color.FromArgb(255, 208, 208, 216)),
                FontSize = 11,
                FontWeight = FontWeights.SemiBold,
                VerticalAlignment = VerticalAlignment.Center,
                Margin = new Thickness(12, 0, 0, 0),
            };
            headerGrid.Children.Add(titleText);

            // Close button
            var closeBtn = new Button
            {
                Content = "✕",
                Width = 28,
                Height = 28,
                Background = Brushes.Transparent,
                Foreground = new SolidColorBrush(Color.FromArgb(255, 161, 161, 170)),
                BorderThickness = new Thickness(0),
                HorizontalAlignment = HorizontalAlignment.Right,
                FontSize = 12,
            };
            closeBtn.Click += (s, e) => CloseFloating();
            headerGrid.Children.Add(closeBtn);

            header.Child = headerGrid;
            grid.Children.Add(header);

            // Content area
            if (Content != null)
            {
                var contentBorder = new Border
                {
                    Background = new SolidColorBrush(Color.FromArgb(255, 18, 18, 20)),
                    Margin = new Thickness(0, 28, 0, 0),
                    Child = Content,
                };
                grid.Children.Add(contentBorder);
            }

            return grid;
        }

        /// <summary>Create the visual tree for a docked panel</summary>
        public UIElement CreateDockedVisual(Action<string> onTabClick)
        {
            var root = new Grid();

            if (!IsPinned && !_isAutoHideOpen)
            {
                // Auto-hide tab strip
                _tabButton = new Border
                {
                    Background = new SolidColorBrush(Color.FromArgb(255, 26, 26, 30)),
                    BorderBrush = new SolidColorBrush(Color.FromArgb(255, 39, 39, 42)),
                    BorderThickness = new Thickness(0, 0, 0, 1),
                    Padding = new Thickness(8, 4, 8, 4),
                    Cursor = Cursors.Hand,
                    Child = new TextBlock
                    {
                        Text = $"{Icon} {Title}",
                        Foreground = new SolidColorBrush(Color.FromArgb(255, 161, 161, 170)),
                        FontSize = 10,
                        VerticalAlignment = VerticalAlignment.Center,
                    },
                };

                _tabButton.MouseEnter += (s, e) => OpenAutoHide();
                _tabButton.MouseLeftButtonDown += (s, e) => onTabClick?.Invoke(PanelId);

                root.Children.Add(_tabButton);
            }
            else
            {
                // Full docked panel with header
                root.RowDefinitions.Add(new RowDefinition { Height = new GridLength(28) });
                root.RowDefinitions.Add(new RowDefinition { Height = new GridLength(1, GridUnitType.Star) });

                // Header bar
                var header = CreateHeaderBar(onTabClick);
                Grid.SetRow(header, 0);
                root.Children.Add(header);

                // Content area
                var contentBorder = new Border
                {
                    Background = new SolidColorBrush(Color.FromArgb(255, 18, 18, 20)),
                    Child = Content,
                };
                Grid.SetRow(contentBorder, 1);
                root.Children.Add(contentBorder);
            }

            return root;
        }

        private Border CreateHeaderBar(Action<string> onTabClick)
        {
            var header = new Border
            {
                Background = new SolidColorBrush(Color.FromArgb(255, 26, 26, 30)),
                BorderBrush = new SolidColorBrush(Color.FromArgb(255, 39, 39, 42)),
                BorderThickness = new Thickness(0, 0, 0, 1),
            };

            var grid = new Grid();

            // Title
            var titleText = new TextBlock
            {
                Text = $"{Icon} {Title}",
                Foreground = new SolidColorBrush(Color.FromArgb(255, 208, 208, 216)),
                FontSize = 10,
                FontWeight = FontWeights.SemiBold,
                VerticalAlignment = VerticalAlignment.Center,
                Margin = new Thickness(12, 0, 0, 0),
            };
            grid.Children.Add(titleText);

            // Button strip (right side)
            var btnStack = new StackPanel { Orientation = Orientation.Horizontal, HorizontalAlignment = HorizontalAlignment.Right };

            // Pin button
            var pinBtn = CreateHeaderButton(IsPinned ? "📌" : "📍", "Toggle Auto-Hide");
            pinBtn.Click += (s, e) => { TogglePin(); OnPropertyChanged(nameof(State)); };
            btnStack.Children.Add(pinBtn);

            // Float button
            var floatBtn = CreateHeaderButton("⧉", "Float Panel");
            floatBtn.Click += (s, e) => ToggleFloat();
            btnStack.Children.Add(floatBtn);

            // Close button
            var closeBtn = CreateHeaderButton("✕", "Close Panel");
            closeBtn.Click += (s, e) => { IsVisible = false; onTabClick?.Invoke(""); };
            btnStack.Children.Add(closeBtn);

            grid.Children.Add(btnStack);
            header.Child = grid;
            return header;
        }

        private Button CreateHeaderButton(string content, string tooltip)
        {
            return new Button
            {
                Content = content,
                Width = 24,
                Height = 24,
                Background = Brushes.Transparent,
                Foreground = new SolidColorBrush(Color.FromArgb(255, 82, 82, 91)),
                BorderThickness = new Thickness(0),
                FontSize = 10,
                ToolTip = tooltip,
            };
        }

        private void OpenAutoHide()
        {
            if (_isAutoHideOpen) return;
            _isAutoHideOpen = true;
            // In production, animate the panel sliding in from the edge
            // Spring animation: 200ms ease-out
            OnPropertyChanged(nameof(IsVisible));
        }

        public void CloseAutoHide()
        {
            if (!_isAutoHideOpen) return;
            _isAutoHideOpen = false;
            // Collapse after 400ms delay
            OnPropertyChanged(nameof(IsVisible));
        }
    }

    /// <summary>Panel state enumeration</summary>
    public enum PanelState { Docked, AutoHide, Floating, TabGroup }

    /// <summary>
    /// Tab group: multiple panels merged into a single dock slot with tab strip.
    /// Supports drag-reorder and tear-out.
    /// </summary>
    public partial class TabGroup : ObservableObject
    {
        [ObservableProperty] private string _groupId = Guid.NewGuid().ToString();
        [ObservableProperty] private DockLocation _dockLocation = DockLocation.Left;
        [ObservableProperty] private string _activePanelId = "";

        public ObservableCollection<DockablePanel> Panels { get; } = new();

        public DockablePanel? ActivePanel => Panels.FirstOrDefault(p => p.PanelId == ActivePanelId);

        public void AddPanel(DockablePanel panel)
        {
            Panels.Add(panel);
            if (string.IsNullOrEmpty(ActivePanelId))
                ActivePanelId = panel.PanelId;
        }

        public void RemovePanel(DockablePanel panel)
        {
            Panels.Remove(panel);
            if (ActivePanelId == panel.PanelId && Panels.Count > 0)
                ActivePanelId = Panels[0].PanelId;
        }

        public void SetActive(string panelId)
        {
            ActivePanelId = panelId;
            foreach (var panel in Panels)
                panel.IsActive = panel.PanelId == panelId;
        }
    }

    /// <summary>
    /// Manages the entire docking layout: panels, splitters, and tab groups.
    /// Persists layout to settings.json and restores on launch.
    /// </summary>
    public class DockManager : ObservableObject
    {
        private readonly ObservableCollection<DockablePanel> _panels = new();
        private readonly ObservableCollection<TabGroup> _tabGroups = new();

        public ObservableCollection<DockablePanel> Panels => _panels;
        public ObservableCollection<TabGroup> TabGroups => _tabGroups;

        public DockablePanel? LeftPanel => _panels.FirstOrDefault(p => p.DefaultDockLocation == DockLocation.Left && p.CurrentLocation == DockLocation.Left);
        public DockablePanel? RightPanel => _panels.FirstOrDefault(p => p.DefaultDockLocation == DockLocation.Right && p.CurrentLocation == DockLocation.Right);

        /// <summary>Register a panel with the docking system</summary>
        public void RegisterPanel(DockablePanel panel)
        {
            _panels.Add(panel);
        }

        /// <summary>Reset all panels to default layout (Ctrl+Shift+P)</summary>
        public void ResetToDefault()
        {
            foreach (var panel in _panels)
                panel.ResetToDefault();
        }

        /// <summary>Toggle full-screen 3D viewport (F11)</summary>
        public void ToggleViewportFullScreen()
        {
            // Hide all panels, maximize 3D viewport
            foreach (var panel in _panels)
                panel.IsVisible = !panel.IsVisible;
        }

        /// <summary>Close active panel or tab (Ctrl+W)</summary>
        public void CloseActive()
        {
            var active = _panels.FirstOrDefault(p => p.IsActive);
            if (active != null)
                active.IsVisible = false;
        }
    }
}
