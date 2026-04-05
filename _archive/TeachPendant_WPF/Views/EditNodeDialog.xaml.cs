using System;
using System.Text.RegularExpressions;
using System.Windows;

namespace TeachPendant_WPF.Views
{
    public partial class EditNodeDialog : Window
    {
        public string NodeValue
        {
            get => $"P({TxtX.Text}, {TxtY.Text}, {TxtZ.Text}) | {CmbFeed.Text}";
            set => ParseInputAndPopulate(value);
        }

        public EditNodeDialog(string currentValue)
        {
            InitializeComponent();
            ParseInputAndPopulate(currentValue);
            TxtX.Focus();
        }

        private void ParseInputAndPopulate(string val)
        {
            // Simple generic parser for legacy values, e.g. "P(120, -50, 20) | F50"
            // If it doesn't match standard format, just zero it.
            TxtX.Text = "0.0"; TxtY.Text = "0.0"; TxtZ.Text = "0.0";
            TxtRx.Text = "0.0"; TxtRy.Text = "0.0"; TxtRz.Text = "0.0";
            
            try
            {
                var match = Regex.Match(val, @"P\(([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\)");
                if (match.Success)
                {
                    TxtX.Text = match.Groups[1].Value;
                    TxtY.Text = match.Groups[2].Value;
                    TxtZ.Text = match.Groups[3].Value;
                }
            }
            catch { }
        }

        private void RecordPosition_Click(object sender, RoutedEventArgs e)
        {
            // Dynamically query the architectural ViewModel running below this window!
            if (App.Current.MainWindow.DataContext is ViewModels.MainViewModel mainVM)
            {
                var rvm = mainVM.RobotVM;
                if (rvm != null)
                {
                    TxtX.Text = Math.Round(rvm.TcpX, 2).ToString();
                    TxtY.Text = Math.Round(rvm.TcpY, 2).ToString();
                    TxtZ.Text = Math.Round(rvm.TcpZ, 2).ToString();
                    // Temporary zeros for rotations until FK kinematics natively supports quaternions to euler
                    TxtRx.Text = "0.0";
                    TxtRy.Text = "0.0";
                    TxtRz.Text = "0.0";
                    
                    // Flash success visually?
                    MessageBox.Show("Workspace Coordinates successfully captured from physical pipeline!", "Coordinate Record", MessageBoxButton.OK, MessageBoxImage.Information);
                }
            }
        }

        private void Save_Click(object sender, RoutedEventArgs e)
        {
            DialogResult = true;
            Close();
        }

        private void Cancel_Click(object sender, RoutedEventArgs e)
        {
            DialogResult = false;
            Close();
        }
    }
}
