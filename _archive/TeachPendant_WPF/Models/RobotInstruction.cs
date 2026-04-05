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

        public abstract Task ExecuteAsync(RobotState state, TeachPendant_WPF.Services.IRobotDriver driver);

        public event PropertyChangedEventHandler? PropertyChanged;
        protected void OnPropertyChanged([CallerMemberName] string? name = null)
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

        public override async Task ExecuteAsync(RobotState state, TeachPendant_WPF.Services.IRobotDriver driver)
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

        public override Task ExecuteAsync(RobotState state, TeachPendant_WPF.Services.IRobotDriver driver)
        {
            // Set IO bit logic here via driver if supported
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

        public override async Task ExecuteAsync(RobotState state, TeachPendant_WPF.Services.IRobotDriver driver)
        {
            if (!state.IsRunning) return;

            var targetJoints = TeachPendant_WPF.Services.WaypointStore.Instance.GetWaypoint(PointId);
            if (targetJoints == null)
            {
                // Fallback to current position if waypoint not found
                targetJoints = new[] { state.J1, state.J2, state.J3, state.J4, state.J5, state.J6 };
            }

            // Send actual joint command
            await driver.SendJointPositions(targetJoints);

            // CLOSED-LOOP VALIDATION: wait until UI state reflects that physical robot reached the target
            bool reached = await driver.WaitForJointStateAsync(targetJoints, toleranceDeg: 0.5, timeoutMs: 15000);
            
            if (!reached)
            {
                System.Diagnostics.Debug.WriteLine($"[WARNING] LinInstruction timeout or interrupted while moving to {PointId}");
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

        public override async Task ExecuteAsync(RobotState state, TeachPendant_WPF.Services.IRobotDriver driver)
        {
            if (!state.IsRunning) return;

            var targetJoints = TeachPendant_WPF.Services.WaypointStore.Instance.GetWaypoint(PointId);
            if (targetJoints == null)
            {
                // Fallback to current position if waypoint not found
                targetJoints = new[] { state.J1, state.J2, state.J3, state.J4, state.J5, state.J6 };
            }

            // Send actual joint command
            await driver.SendJointPositions(targetJoints);

            // CLOSED-LOOP VALIDATION: wait until UI state reflects that physical robot reached the target
            bool reached = await driver.WaitForJointStateAsync(targetJoints, toleranceDeg: 0.5, timeoutMs: 15000);
            
            if (!reached)
            {
                System.Diagnostics.Debug.WriteLine($"[WARNING] PtpInstruction timeout or interrupted while moving to {PointId}");
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

        public override Task ExecuteAsync(RobotState state, TeachPendant_WPF.Services.IRobotDriver driver)
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

        public override Task ExecuteAsync(RobotState state, TeachPendant_WPF.Services.IRobotDriver driver)
        {
            return Task.CompletedTask;
        }
    }

    public class DoFileEndInstruction : RobotInstruction
    {
        public override string DisplayText => "DofileEnd()";
        public override string NodeType => "Structure";
        public override string IconText => "🏁";

        public override Task ExecuteAsync(RobotState state, TeachPendant_WPF.Services.IRobotDriver driver)
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

        public override async Task ExecuteAsync(RobotState state, TeachPendant_WPF.Services.IRobotDriver driver)
        {
            // Placeholder execute for While loop simulation
            foreach(var child in Children)
            {
                await child.ExecuteAsync(state, driver);
            }
        }
    }
}
