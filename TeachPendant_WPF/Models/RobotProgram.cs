using System.Collections.ObjectModel;
using System.ComponentModel;
using System.Runtime.CompilerServices;

namespace TeachPendant_WPF.Models
{
    public class RobotProgram : INotifyPropertyChanged
    {
        private string _fileName = "luexpo.lua";
        public string FileName
        {
            get => _fileName;
            set
            {
                if (_fileName != value)
                {
                    _fileName = value;
                    OnPropertyChanged();
                }
            }
        }

        public ObservableCollection<RobotInstruction> Instructions { get; } = new ObservableCollection<RobotInstruction>();

        public event PropertyChangedEventHandler PropertyChanged;
        protected void OnPropertyChanged([CallerMemberName] string name = null)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(name));
        }

        // Helper method to add instructions mimicking the UI structure
        public void LoadSampleProgram()
        {
            Instructions.Clear();
            Instructions.Add(new WaitInstruction(5000));
            Instructions.Add(new SetDOInstruction(3, 0));
            Instructions.Add(new NewDoFileInstruction("/fruser/WELDO..."));
            Instructions.Add(new DoFileEndInstruction());
            Instructions.Add(new LinInstruction("l1", 60, 0));
            Instructions.Add(new LinInstruction("l7", 100, -1));
            Instructions.Add(new LinInstruction("l8", 100, -1));
            Instructions.Add(new NewDoFileInstruction("/fruser/WELDO..."));
            Instructions.Add(new DoFileEndInstruction());
            Instructions.Add(new LinInstruction("l9", 10, -1));
            Instructions.Add(new NewDoFileInstruction("/fruser/WELDO..."));
            Instructions.Add(new DoFileEndInstruction());
            Instructions.Add(new LinInstruction("l10", 70, -1));
            
            UpdateIndices();
        }

        public void UpdateIndices()
        {
            // The screenshot starts counting at line 8
            int startIndex = 8;
            foreach (var instr in Instructions)
            {
                instr.Index = startIndex++;
            }
        }
    }
}
