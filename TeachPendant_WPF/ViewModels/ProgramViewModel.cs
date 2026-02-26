using System;
using System.Collections.ObjectModel;
using System.Linq;
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using TeachPendant_WPF.Models;

namespace TeachPendant_WPF.ViewModels
{
    /// <summary>
    /// Manages the structured WorkTree programming model.
    /// Replaces raw Lua text with typed ProgramNode objects.
    /// </summary>
    public partial class ProgramViewModel : ObservableObject
    {
        // ── Program ─────────────────────────────────────────────────

        [ObservableProperty] private string _programName = "luexpo.lua";

        /// <summary>
        /// The structured work tree of program nodes.
        /// </summary>
        public ObservableCollection<ProgramNode> Nodes { get; } = new();

        // ── Selection ───────────────────────────────────────────────

        private ProgramNode? _selectedNode;
        public ProgramNode? SelectedNode
        {
            get => _selectedNode;
            set
            {
                _selectedNode = value;
                OnPropertyChanged();
                OnPropertyChanged(nameof(HasSelection));
            }
        }

        public bool HasSelection => _selectedNode != null;

        // ── Active Execution Line ───────────────────────────────────

        [ObservableProperty] private int _activeLineNumber;

        // ── Commands ────────────────────────────────────────────────

        [RelayCommand]
        private void AddNode(string type)
        {
            ProgramNode? node = type?.ToUpper() switch
            {
                "INITIALIZE" => new ProgramNode
                {
                    NodeType = ProgramNodeType.Initialize,
                    DisplayText = "Initialize()",
                    IconText = "🏠"
                },
                "MOVETOPOINT" or "PTP" => new ProgramNode
                {
                    NodeType = ProgramNodeType.MoveToPoint,
                    DisplayText = "MoveToPoint(p1)",
                    IconText = "🎯",
                    MotionType = MotionType.PTP
                },
                "LIN" => new ProgramNode
                {
                    NodeType = ProgramNodeType.MoveToPoint,
                    DisplayText = "Lin(p1,100,-1,0,0)",
                    IconText = "📏",
                    MotionType = MotionType.Linear
                },
                "ARC" or "CIRCLE" => new ProgramNode
                {
                    NodeType = ProgramNodeType.ArcMove,
                    DisplayText = "Arc(p1,p2,50)",
                    IconText = "⭕",
                    MotionType = MotionType.Arc
                },
                "CURVETRACE" => new ProgramNode
                {
                    NodeType = ProgramNodeType.CurveTrace,
                    DisplayText = "CurveTrace(curve1)",
                    IconText = "〰️"
                },
                "WAIT" => new ProgramNode
                {
                    NodeType = ProgramNodeType.Wait,
                    DisplayText = "WaitMs(1000)",
                    IconText = "⏱",
                    WaitTimeMs = 1000
                },
                "IO" or "SETDO" => new ProgramNode
                {
                    NodeType = ProgramNodeType.IOAction,
                    DisplayText = "SetDO(1,1)",
                    IconText = "🔌"
                },
                "LOOP" or "WHILE" => new ProgramNode
                {
                    NodeType = ProgramNodeType.Loop,
                    DisplayText = "While(True)",
                    IconText = "🔃"
                },
                "SUBROUTINE" or "DOFILE" => new ProgramNode
                {
                    NodeType = ProgramNodeType.Subroutine,
                    DisplayText = "DoFile(/fruser/...)",
                    IconText = "📄"
                },
                _ => null
            };

            if (node != null)
            {
                node.Index = Nodes.Count + 1;
                Nodes.Add(node);
                UpdateIndices();
            }
        }

        [RelayCommand]
        private void RemoveSelectedNode()
        {
            if (_selectedNode != null)
            {
                Nodes.Remove(_selectedNode);
                SelectedNode = null;
                UpdateIndices();
            }
        }

        [RelayCommand]
        private void MoveNodeUp()
        {
            if (_selectedNode == null) return;
            int idx = Nodes.IndexOf(_selectedNode);
            if (idx > 0)
            {
                Nodes.Move(idx, idx - 1);
                UpdateIndices();
            }
        }

        [RelayCommand]
        private void MoveNodeDown()
        {
            if (_selectedNode == null) return;
            int idx = Nodes.IndexOf(_selectedNode);
            if (idx < Nodes.Count - 1)
            {
                Nodes.Move(idx, idx + 1);
                UpdateIndices();
            }
        }

        // ── Helpers ─────────────────────────────────────────────────

        private void UpdateIndices()
        {
            for (int i = 0; i < Nodes.Count; i++)
            {
                Nodes[i].Index = i + 1;
            }
        }

        /// <summary>
        /// Load the default sample program matching the FR-HMI reference.
        /// </summary>
        public void LoadSampleProgram()
        {
            Nodes.Clear();
            AddNode("WAIT");
            AddNode("SETDO");
            AddNode("DOFILE");
            AddNode("LIN");
            AddNode("LIN");
            AddNode("LIN");
            AddNode("DOFILE");
            AddNode("LIN");
            AddNode("DOFILE");
            AddNode("LIN");
        }
    }

    // ── Program Node Model ──────────────────────────────────────────

    public enum ProgramNodeType
    {
        Initialize,
        MoveToPoint,
        CurveTrace,
        ArcMove,
        IOAction,
        Wait,
        Loop,
        Subroutine
    }

    public enum MotionType
    {
        PTP,
        Linear,
        Arc,
        Spline,
        CurveFollow,
        SurfaceNormal
    }

    public partial class ProgramNode : ObservableObject
    {
        [ObservableProperty] private int _index;
        [ObservableProperty] private string _displayText = string.Empty;
        [ObservableProperty] private string _iconText = string.Empty;
        [ObservableProperty] private ProgramNodeType _nodeType;

        // ── Motion Parameters ───────────────────────────────────────

        [ObservableProperty] private MotionType _motionType = MotionType.Linear;
        [ObservableProperty] private double _speed = 100.0;
        [ObservableProperty] private double _accelerationParam = 180.0;
        [ObservableProperty] private double _blendRadius;
        [ObservableProperty] private string _orientationStrategy = "Fixed";

        // ── Pose Reference ──────────────────────────────────────────

        [ObservableProperty] private Guid _poseReferenceId;
        [ObservableProperty] private double _targetX;
        [ObservableProperty] private double _targetY;
        [ObservableProperty] private double _targetZ;
        [ObservableProperty] private double _targetRx;
        [ObservableProperty] private double _targetRy;
        [ObservableProperty] private double _targetRz;

        // ── Wait / IO Parameters ────────────────────────────────────

        [ObservableProperty] private int _waitTimeMs;
        [ObservableProperty] private int _ioPort;
        [ObservableProperty] private int _ioValue;

        // ── Subroutine ──────────────────────────────────────────────

        [ObservableProperty] private string _filePath = string.Empty;

        // ── Children (for loops) ────────────────────────────────────

        public ObservableCollection<ProgramNode> Children { get; } = new();
    }
}
