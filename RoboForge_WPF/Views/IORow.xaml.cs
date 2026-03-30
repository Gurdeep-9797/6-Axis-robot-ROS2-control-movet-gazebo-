using System;
using System.Windows;
using System.Windows.Controls;

namespace RoboForge_WPF.Views
{
    public partial class IORow : UserControl
    {
        public static readonly DependencyProperty LabelProperty = DependencyProperty.Register("Label", typeof(string), typeof(IORow), new PropertyMetadata("IO_00"));
        public string Label
        {
            get { return (string)GetValue(LabelProperty); }
            set { SetValue(LabelProperty, value); }
        }

        public static readonly DependencyProperty IsOutputProperty = DependencyProperty.Register("IsOutput", typeof(bool), typeof(IORow), new PropertyMetadata(false, OnIsOutputChanged));
        public bool IsOutput
        {
            get { return (bool)GetValue(IsOutputProperty); }
            set { SetValue(IsOutputProperty, value); }
        }

        public IORow()
        {
            InitializeComponent();
            this.Loaded += (s, e) => { LabelText.Text = Label; };
        }

        private static void OnIsOutputChanged(DependencyObject d, DependencyPropertyChangedEventArgs e)
        {
            if (d is IORow row)
            {
                bool isOut = (bool)e.NewValue;
                row.ToggleBtn.Visibility = isOut ? Visibility.Visible : Visibility.Collapsed;
                row.Indicator.Visibility = isOut ? Visibility.Collapsed : Visibility.Visible;
            }
        }

        private void ToggleBtn_Click(object sender, RoutedEventArgs e)
        {
            // Update backend state
        }
    }
}
