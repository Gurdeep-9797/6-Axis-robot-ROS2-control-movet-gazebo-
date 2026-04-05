using CommunityToolkit.Mvvm.ComponentModel;
using System;
using System.Windows.Media;

namespace RoboForge_WPF.ViewModels
{
    public partial class PipelineViewModel : ObservableObject
    {
        [ObservableProperty] private string _pipelineStatus = "  ● IDLE";
        [ObservableProperty] private Brush _pipelineStatusColor;

        [ObservableProperty] private string _uiMsg = "Waiting...";
        [ObservableProperty] private string _engineMsg = "Idle";
        [ObservableProperty] private string _driverMsg = "Disconnected";
        [ObservableProperty] private string _bridgeMsg = "Not connected";
        [ObservableProperty] private string _gazeboMsg = "Not running";
        [ObservableProperty] private string _jointMsg = "No data";

        [ObservableProperty] private string _telJ1 = "J1: 0.00°";
        [ObservableProperty] private string _telJ2 = "J2: 0.00°";
        [ObservableProperty] private string _telJ3 = "J3: 0.00°";
        [ObservableProperty] private string _telJ4 = "J4: 0.00°";
        [ObservableProperty] private string _telJ5 = "J5: 0.00°";
        [ObservableProperty] private string _telJ6 = "J6: 0.00°";

        [ObservableProperty] private int _messagesSent = 0;
        [ObservableProperty] private int _messagesReceived = 0;

        public PipelineViewModel()
        {
            // Set default colors directly since we can't reliably resolve DynamicResource here without Application.Current overhead
            _pipelineStatusColor = new SolidColorBrush(Color.FromRgb(156, 163, 175)); // Text.Muted
        }
    }
}
