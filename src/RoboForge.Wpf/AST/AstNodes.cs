// ── AST (Abstract Syntax Tree) System ─────────────────────────────────────
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using CommunityToolkit.Mvvm.ComponentModel;

namespace RoboForge.Wpf.AST
{
    /// <summary>Node types in the program AST</summary>
    public enum NodeType
    {
        Program, Routine, Block, Sequence,
        MoveJ, MoveL, MoveC, MoveAbsJ, SearchL, HoningCycle,
        SetDO, PulseDO, GetDI, WaitDI, SetAO, GetAI,
        Wait, If, Else, While, For, Break, Return,
        RoutineDef, CallRoutine,
        GripperOpen, GripperClose, ToolChange, ResetError, Stop,
        LoopEnd  // Internal instruction type for loop-end jump (separate from Break)
    }

    /// <summary>Base class for all AST nodes</summary>
    public abstract partial class AstNode : ObservableObject
    {
        public string NodeId { get; } = Guid.NewGuid().ToString();
        [ObservableProperty] private NodeType _nodeType;
        [ObservableProperty] private string _name = "";
        [ObservableProperty] private AstNode? _parent;
        [ObservableProperty] private bool _isExecuting;
        [ObservableProperty] private bool _hasError;
        [ObservableProperty] private string _errorMessage = "";

        public ObservableCollection<AstNode> Children { get; } = new();
        public Dictionary<string, object> Properties { get; } = new();

        public void AddChild(AstNode child)
        {
            child.Parent = this;
            Children.Add(child);
        }

        public void RemoveChild(AstNode child)
        {
            child.Parent = null;
            Children.Remove(child);
        }

        public void InsertChild(int index, AstNode child)
        {
            child.Parent = this;
            Children.Insert(index, child);
        }

        public AstNode? FindById(string id)
        {
            if (NodeId == id) return this;
            foreach (var child in Children)
            {
                var found = child.FindById(id);
                if (found != null) return found;
            }
            return null;
        }

        public IEnumerable<AstNode> GetAllBlocks()
        {
            if (NodeType != NodeType.Program && NodeType != NodeType.Routine && NodeType != NodeType.Sequence)
                yield return this;
            foreach (var child in Children)
                foreach (var block in child.GetAllBlocks())
                    yield return block;
        }
    }

    public partial class ProgramNode : AstNode
    {
        public ProgramNode() { NodeType = NodeType.Program; Name = "Main Program"; }
    }

    public partial class RoutineNode : AstNode
    {
        public RoutineNode() { NodeType = NodeType.Routine; }
        public List<(string name, string type)> Parameters { get; } = new();
        public string ReturnType { get; set; } = "void";
    }

    public abstract partial class BlockNode : AstNode
    {
        [ObservableProperty] private double _speed = 50;
        [ObservableProperty] private double _acceleration = 20;
        [ObservableProperty] private double _blendRadius;
    }

    public partial class MoveJNode : BlockNode
    {
        public MoveJNode() { NodeType = NodeType.MoveJ; Name = "MoveJ"; }
        [ObservableProperty] private string _targetWaypoint = "";
        [ObservableProperty] private string _zone = "z50";
    }

    public partial class MoveLNode : BlockNode
    {
        public MoveLNode() { NodeType = NodeType.MoveL; Name = "MoveL"; }
        [ObservableProperty] private string _targetWaypoint = "";
        [ObservableProperty] private double _speedMmS = 200;
        [ObservableProperty] private double _accelMmS2 = 2000;
        [ObservableProperty] private string _orientationMethod = "SLERP";
    }

    public partial class MoveCNode : BlockNode
    {
        public MoveCNode() { NodeType = NodeType.MoveC; Name = "MoveC"; }
        [ObservableProperty] private string _viaWaypoint = "";
        [ObservableProperty] private string _endWaypoint = "";
        [ObservableProperty] private double _speedMmS = 200;
    }

    public partial class MoveAbsJNode : BlockNode
    {
        public MoveAbsJNode() { NodeType = NodeType.MoveAbsJ; Name = "MoveAbsJ"; }
        public double[] JointAngles { get; set; } = new double[6];
    }

    public partial class SearchLNode : BlockNode
    {
        public SearchLNode() { NodeType = NodeType.SearchL; Name = "SearchL"; }
        [ObservableProperty] private double _directionX, _directionY, _directionZ = 1;
        [ObservableProperty] private double _maxDistance = 100;
        [ObservableProperty] private double _searchSpeed = 20;
        [ObservableProperty] private string _triggerPin = "";
        [ObservableProperty] private bool _triggerHigh = true;
    }

    public partial class SetDONode : BlockNode
    {
        public SetDONode() { NodeType = NodeType.SetDO; Name = "SetDO"; }
        [ObservableProperty] private string _outputPin = "";
        [ObservableProperty] private bool _value;
        [ObservableProperty] private bool _endOfPath;
    }

    public partial class PulseDONode : BlockNode
    {
        public PulseDONode() { NodeType = NodeType.PulseDO; Name = "PulseDO"; }
        [ObservableProperty] private string _outputPin = "";
        [ObservableProperty] private int _durationMs = 100;
        [ObservableProperty] private bool _activeHigh = true;
    }

    public partial class WaitDINode : BlockNode
    {
        public WaitDINode() { NodeType = NodeType.WaitDI; Name = "WaitDI"; }
        [ObservableProperty] private string _inputPin = "";
        [ObservableProperty] private bool _targetState = true;
        [ObservableProperty] private int _timeoutMs = 5000;
        [ObservableProperty] private string _onTimeout = "Error";
    }

    public partial class WaitNode : BlockNode
    {
        public WaitNode() { NodeType = NodeType.Wait; Name = "Wait"; }
        [ObservableProperty] private int _durationMs = 1000;
    }

    public partial class IfNode : BlockNode
    {
        public IfNode() { NodeType = NodeType.If; Name = "If"; }
        [ObservableProperty] private string _condition = "";
        public AstNode IfBranch { get; set; } = new SequenceNode();
        public AstNode ElseBranch { get; set; } = new SequenceNode();
    }

    public partial class WhileNode : BlockNode
    {
        public WhileNode() { NodeType = NodeType.While; Name = "While"; }
        [ObservableProperty] private string _condition = "";
        [ObservableProperty] private int _maxIterations = 10000;
    }

    public partial class ForNode : BlockNode
    {
        public ForNode() { NodeType = NodeType.For; Name = "For"; }
        [ObservableProperty] private string _counterVar = "i";
        [ObservableProperty] private int _start = 0;
        [ObservableProperty] private int _end = 10;
        [ObservableProperty] private int _step = 1;
    }

    public partial class BreakNode : BlockNode
    {
        public BreakNode() { NodeType = NodeType.Break; Name = "Break"; }
    }

    public partial class GripperOpenNode : BlockNode
    {
        public GripperOpenNode() { NodeType = NodeType.GripperOpen; Name = "GripperOpen"; }
        [ObservableProperty] private double _targetWidth = 80;
        [ObservableProperty] private double _speed = 50;
        [ObservableProperty] private double _forceLimit = 50;
    }

    public partial class GripperCloseNode : BlockNode
    {
        public GripperCloseNode() { NodeType = NodeType.GripperClose; Name = "GripperClose"; }
        [ObservableProperty] private double _targetWidth = 0;
        [ObservableProperty] private double _speed = 50;
        [ObservableProperty] private double _force = 30;
        [ObservableProperty] private bool _detectObject;
    }

    public partial class StopNode : BlockNode
    {
        public StopNode() { NodeType = NodeType.Stop; Name = "Stop"; }
        [ObservableProperty] private string _stopType = "Smooth";
        [ObservableProperty] private string _message = "";
    }

    public partial class SequenceNode : AstNode
    {
        public SequenceNode() { NodeType = NodeType.Sequence; }
    }
}
