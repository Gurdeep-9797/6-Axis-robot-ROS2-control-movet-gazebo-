using System.Collections.ObjectModel;
using System.ComponentModel;
using System.Runtime.CompilerServices;
using System.Threading.Tasks;

namespace TeachPendant_WPF.Models
{
    public abstract class RobotInstruction : INotifyPropertyChanged
    {
        public bool IsBreakpoint { get; set; } = false;
        
        private bool _isSelected = false;
        public bool IsSelected 
        { 
            get => _isSelected; 
            set { _isSelected = value; OnPropertyChanged(); } 
        }

        private int _index;
        public int Index
        {
            get => _index;
            set { _index = value; OnPropertyChanged(); }
        }

        public abstract string DisplayText { get; }
        public abstract string NodeType { get; } // e.g., "Motion", "Logic", "Structure"
        public abstract string IconText { get; } // Unicode character for UI representation

        public ObservableCollection<RobotInstruction> Children { get; set; } = new ObservableCollection<RobotInstruction>();
        public bool HasChildren => Children.Count > 0;

        public abstract Task ExecuteAsync(RobotState state);

        public event PropertyChangedEventHandler PropertyChanged;
        protected void OnPropertyChanged([CallerMemberName] string name = null)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(name));
        }
    }

    public class WaitInstruction : RobotInstruction
    {
        public int DelayMs { get; set; }

        public WaitInstruction(int delayMs)
        {
            DelayMs = delayMs;
        }

        public override string DisplayText => $"WaitMs({DelayMs})";
        public override string NodeType => "Logic";
        public override string IconText => "⏱";

        public override async Task ExecuteAsync(RobotState state)
        {
            await Task.Delay(DelayMs);
        }
    }

    public class SetDOInstruction : RobotInstruction
    {
        public int Port { get; set; }
        public int Value { get; set; }

        public SetDOInstruction(int port, int value)
        {
            Port = port;
            Value = value;
        }

        public override string DisplayText => $"SetDO({Port},{Value},0,0)";
        public override string NodeType => "Logic";
        public override string IconText => "🔌";

        public override Task ExecuteAsync(RobotState state)
        {
            // Set IO bit logic here
            return Task.CompletedTask;
        }
    }

    public class LinInstruction : RobotInstruction
    {
        public string PointId { get; set; }
        public double Speed { get; set; }
        public int Blending { get; set; }

        public LinInstruction(string pointId, double speed, int blending)
        {
            PointId = pointId;
            Speed = speed;
            Blending = blending;
        }

        public override string DisplayText => $"Lin({PointId},{Speed},{Blending},0,0)";
        public override string NodeType => "Motion";
        public override string IconText => "📏";

        public override async Task ExecuteAsync(RobotState state)
        {
            // FAKE KINEMATICS FOR UI DEMO: Sweep joints smoothly over 1.5 seconds
            int steps = 30;
            int delayMs = 50;

            System.Random r = new System.Random(PointId.GetHashCode());
            double targetJ1 = r.Next(-45, 45);
            double targetJ2 = r.Next(-30, 30);
            double targetJ3 = r.Next(-45, 45);
            double targetJ4 = r.Next(-90, 90);
            double targetJ5 = r.Next(-45, 45);

            double startJ1 = state.J1;
            double startJ2 = state.J2;
            double startJ3 = state.J3;
            double startJ4 = state.J4;
            double startJ5 = state.J5;

            for (int i = 1; i <= steps; i++)
            {
                if (!state.IsRunning) break;
                
                double t = (double)i / steps;
                double smoothT = t * t * (3 - 2 * t); // Smoothstep

                state.J1 = startJ1 + (targetJ1 - startJ1) * smoothT;
                state.J2 = startJ2 + (targetJ2 - startJ2) * smoothT;
                state.J3 = startJ3 + (targetJ3 - startJ3) * smoothT;
                state.J4 = startJ4 + (targetJ4 - startJ4) * smoothT;
                state.J5 = startJ5 + (targetJ5 - startJ5) * smoothT;
                
                // Simulate some force data variance
                state.Fx = r.NextDouble() * 5.0;
                state.Fy = r.NextDouble() * 5.0;
                state.Fz = r.NextDouble() * -10.0;

                await Task.Delay(delayMs);
            }
        }
    }
    
    public class PtpInstruction : RobotInstruction
    {
        public string PointId { get; set; }
        public double Speed { get; set; }

        public PtpInstruction(string pointId, double speed)
        {
            PointId = pointId;
            Speed = speed;
        }

        public override string DisplayText => $"PTP({PointId},{Speed},0,0,0)";
        public override string NodeType => "Motion";
        public override string IconText => "🎯";

        public override async Task ExecuteAsync(RobotState state)
        {
            // FAKE KINEMATICS FOR UI DEMO: Sweep joints smoothly over 2.0 seconds
            int steps = 40;
            int delayMs = 50; 

            System.Random r = new System.Random(PointId.GetHashCode());
            double targetJ1 = r.Next(-90, 90);
            double targetJ2 = r.Next(-45, 45);
            double targetJ3 = r.Next(-90, 90);
            double targetJ4 = r.Next(-180, 180);
            double targetJ5 = r.Next(-90, 90);
            double targetJ6 = r.Next(-180, 180);

            double startJ1 = state.J1;
            double startJ2 = state.J2;
            double startJ3 = state.J3;
            double startJ4 = state.J4;
            double startJ5 = state.J5;
            double startJ6 = state.J6;

            for (int i = 1; i <= steps; i++)
            {
                if (!state.IsRunning) break;
                
                double t = (double)i / steps;
                double smoothT = t * t * (3 - 2 * t);

                state.J1 = startJ1 + (targetJ1 - startJ1) * smoothT;
                state.J2 = startJ2 + (targetJ2 - startJ2) * smoothT;
                state.J3 = startJ3 + (targetJ3 - startJ3) * smoothT;
                state.J4 = startJ4 + (targetJ4 - startJ4) * smoothT;
                state.J5 = startJ5 + (targetJ5 - startJ5) * smoothT;
                state.J6 = startJ6 + (targetJ6 - startJ6) * smoothT;
                
                // Track Cartesian approx
                state.X = state.J1 * 2.0;
                state.Y = state.J2 * 2.0;
                state.Z = 300 + state.J3 * 2.0;

                await Task.Delay(delayMs);
            }
        }
    }

    public class CircleInstruction : RobotInstruction
    {
        public string PointId1 { get; set; }
        public string PointId2 { get; set; }
        public double Speed { get; set; }

        public CircleInstruction(string pointId1, string pointId2, double speed)
        {
            PointId1 = pointId1;
            PointId2 = pointId2;
            Speed = speed;
        }

        public override string DisplayText => $"Circle({PointId1},{PointId2},{Speed})";
        public override string NodeType => "Motion";
        public override string IconText => "⭕";

        public override Task ExecuteAsync(RobotState state)
        {
            return Task.CompletedTask;
        }
    }
    
    public class NewDoFileInstruction : RobotInstruction
    {
        public string FilePath { get; set; }

        public NewDoFileInstruction(string filePath)
        {
            FilePath = filePath;
        }

        public override string DisplayText => $"NewDofile(\"{FilePath}\")";
        public override string NodeType => "Structure";
        public override string IconText => "📄";

        public override Task ExecuteAsync(RobotState state)
        {
            return Task.CompletedTask;
        }
    }

    public class DoFileEndInstruction : RobotInstruction
    {
        public override string DisplayText => "DofileEnd()";
        public override string NodeType => "Structure";
        public override string IconText => "🏁";

        public override Task ExecuteAsync(RobotState state)
        {
            return Task.CompletedTask;
        }
    }

    public class WhileInstruction : RobotInstruction
    {
        public string Condition { get; set; }

        public WhileInstruction(string condition)
        {
            Condition = condition;
        }

        public override string DisplayText => $"While ({Condition})";
        public override string NodeType => "Structure";
        public override string IconText => "🔃";

        public override async Task ExecuteAsync(RobotState state)
        {
            // Placeholder execute for While loop simulation
            foreach(var child in Children)
            {
                await child.ExecuteAsync(state);
            }
        }
    }
}
