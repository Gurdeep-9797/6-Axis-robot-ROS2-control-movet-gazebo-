using System;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;

namespace TeachPendant_WPF.Views
{
    public partial class AxisJogControl : UserControl
    {
        public static readonly DependencyProperty AxisNameProperty =
            DependencyProperty.Register("AxisName", typeof(string), typeof(AxisJogControl), new PropertyMetadata("J1"));

        public string AxisName
        {
            get => (string)GetValue(AxisNameProperty);
            set => SetValue(AxisNameProperty, value);
        }

        public static readonly DependencyProperty CommandAngleProperty =
            DependencyProperty.Register("CommandAngle", typeof(double), typeof(AxisJogControl), new PropertyMetadata(0.0));

        public double CommandAngle
        {
            get => (double)GetValue(CommandAngleProperty);
            set => SetValue(CommandAngleProperty, value);
        }

        public static readonly DependencyProperty ActualAngleProperty =
            DependencyProperty.Register("ActualAngle", typeof(double), typeof(AxisJogControl), new PropertyMetadata(0.0));

        public double ActualAngle
        {
            get => (double)GetValue(ActualAngleProperty);
            set => SetValue(ActualAngleProperty, value);
        }

        public static readonly DependencyProperty MinLimitProperty =
            DependencyProperty.Register("MinLimit", typeof(double), typeof(AxisJogControl), new PropertyMetadata(-180.0));

        public double MinLimit
        {
            get => (double)GetValue(MinLimitProperty);
            set => SetValue(MinLimitProperty, value);
        }

        public static readonly DependencyProperty MaxLimitProperty =
            DependencyProperty.Register("MaxLimit", typeof(double), typeof(AxisJogControl), new PropertyMetadata(180.0));

        public double MaxLimit
        {
            get => (double)GetValue(MaxLimitProperty);
            set => SetValue(MaxLimitProperty, value);
        }

        public AxisJogControl()
        {
            InitializeComponent();
            TxtAxisName.SetBinding(TextBlock.TextProperty, new System.Windows.Data.Binding("AxisName") { Source = this });
            
            SldPosition.SetBinding(Slider.ValueProperty, new System.Windows.Data.Binding("CommandAngle") { Source = this, Mode = System.Windows.Data.BindingMode.TwoWay, UpdateSourceTrigger = System.Windows.Data.UpdateSourceTrigger.PropertyChanged });
            SldPosition.SetBinding(Slider.MinimumProperty, new System.Windows.Data.Binding("MinLimit") { Source = this });
            SldPosition.SetBinding(Slider.MaximumProperty, new System.Windows.Data.Binding("MaxLimit") { Source = this });
            
            TxtValue.SetBinding(TextBox.TextProperty, new System.Windows.Data.Binding("ActualAngle") { Source = this, StringFormat = "N1", Mode = System.Windows.Data.BindingMode.OneWay });

            BtnMinus.Click += (s, e) => CommandAngle -= 1.0;
            BtnPlus.Click += (s, e) => CommandAngle += 1.0;
        }
    }
}
