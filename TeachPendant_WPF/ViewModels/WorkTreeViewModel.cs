using System;
using System.Collections.ObjectModel;
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;

namespace TeachPendant_WPF.ViewModels
{
    /// <summary>
    /// Hierarchical WorkTree for the project browser (like ABB RobotStudio).
    /// Structure: Robot > Joints | Workpieces | Points/Targets | Program
    /// </summary>
    public partial class WorkTreeViewModel : ObservableObject
    {
        public ObservableCollection<WorkTreeNode> RootNodes { get; } = new();

        // ── Selected Node ───────────────────────────────────────────
        [ObservableProperty] private WorkTreeNode? _selectedNode;

        public WorkTreeViewModel()
        {
            BuildDefaultTree();
        }

        private void BuildDefaultTree()
        {
            RootNodes.Clear();

            // Robot Node
            var robotNode = new WorkTreeNode("🤖", "Robot FR-6DOF", WorkTreeNodeType.Robot);
            robotNode.Children.Add(new WorkTreeNode("⚙", "J1 — Base Yaw", WorkTreeNodeType.Joint) { Value = "0.00°", Tag = "J1" });
            robotNode.Children.Add(new WorkTreeNode("⚙", "J2 — Shoulder", WorkTreeNodeType.Joint) { Value = "0.00°", Tag = "J2" });
            robotNode.Children.Add(new WorkTreeNode("⚙", "J3 — Elbow", WorkTreeNodeType.Joint) { Value = "0.00°", Tag = "J3" });
            robotNode.Children.Add(new WorkTreeNode("⚙", "J4 — Wrist 1", WorkTreeNodeType.Joint) { Value = "0.00°", Tag = "J4" });
            robotNode.Children.Add(new WorkTreeNode("⚙", "J5 — Wrist 2", WorkTreeNodeType.Joint) { Value = "0.00°", Tag = "J5" });
            robotNode.Children.Add(new WorkTreeNode("⚙", "J6 — Tool Roll", WorkTreeNodeType.Joint) { Value = "0.00°", Tag = "J6" });
            robotNode.IsExpanded = true;

            // Tools
            var toolsNode = new WorkTreeNode("🔧", "Tools", WorkTreeNodeType.Folder);
            toolsNode.Children.Add(new WorkTreeNode("📌", "Tool0 (default)", WorkTreeNodeType.Tool) { Value = "0, 0, 0" });
            toolsNode.IsExpanded = true;

            // Work Objects
            var wobjNode = new WorkTreeNode("📐", "Work Objects", WorkTreeNodeType.Folder);
            wobjNode.Children.Add(new WorkTreeNode("📏", "Wobj0 (world)", WorkTreeNodeType.WorkObject) { Value = "0, 0, 0" });
            wobjNode.IsExpanded = true;

            // Points/Targets
            var pointsNode = new WorkTreeNode("🎯", "Points / Targets", WorkTreeNodeType.Folder);
            pointsNode.Children.Add(new WorkTreeNode("📍", "p1", WorkTreeNodeType.Point) { Value = "100, 0, 300" });
            pointsNode.Children.Add(new WorkTreeNode("📍", "p2", WorkTreeNodeType.Point) { Value = "200, 50, 280" });
            pointsNode.Children.Add(new WorkTreeNode("📍", "p3", WorkTreeNodeType.Point) { Value = "150, -30, 350" });
            pointsNode.IsExpanded = true;

            // Program
            var programNode = new WorkTreeNode("📄", "Program: main.lua", WorkTreeNodeType.Program);
            programNode.Children.Add(new WorkTreeNode("🏠", "Initialize()", WorkTreeNodeType.ProgramInstr));
            programNode.Children.Add(new WorkTreeNode("📏", "Lin(p1, 100, -1, 0, 0)", WorkTreeNodeType.ProgramInstr));
            programNode.Children.Add(new WorkTreeNode("📏", "Lin(p2, 80, -1, 0, 0)", WorkTreeNodeType.ProgramInstr));
            programNode.Children.Add(new WorkTreeNode("⏱", "WaitMs(500)", WorkTreeNodeType.ProgramInstr));
            programNode.Children.Add(new WorkTreeNode("📏", "Lin(p3, 100, -1, 0, 0)", WorkTreeNodeType.ProgramInstr));
            programNode.Children.Add(new WorkTreeNode("🔌", "SetDO(1, 1)", WorkTreeNodeType.ProgramInstr));
            programNode.IsExpanded = true;

            RootNodes.Add(robotNode);
            RootNodes.Add(toolsNode);
            RootNodes.Add(wobjNode);
            RootNodes.Add(pointsNode);
            RootNodes.Add(programNode);
        }

        // ── Commands ────────────────────────────────────────────────

        [RelayCommand]
        private void AddPoint()
        {
            var pointsFolder = FindNodeByType(WorkTreeNodeType.Folder, "Points");
            if (pointsFolder != null)
            {
                int num = pointsFolder.Children.Count + 1;
                pointsFolder.Children.Add(new WorkTreeNode("📍", $"p{num}", WorkTreeNodeType.Point) { Value = "0, 0, 0" });
            }
        }

        [RelayCommand]
        private void AddProgramInstruction(string type)
        {
            var programFolder = FindNodeByType(WorkTreeNodeType.Program, null);
            if (programFolder == null) return;

            var node = type?.ToUpper() switch
            {
                "LIN" => new WorkTreeNode("📏", "Lin(pN, 100, -1, 0, 0)", WorkTreeNodeType.ProgramInstr),
                "PTP" => new WorkTreeNode("⤵", "MoveToPoint(pN)", WorkTreeNodeType.ProgramInstr),
                "WAIT" => new WorkTreeNode("⏱", "WaitMs(1000)", WorkTreeNodeType.ProgramInstr),
                "SETDO" => new WorkTreeNode("🔌", "SetDO(1, 1)", WorkTreeNodeType.ProgramInstr),
                "WHILE" => new WorkTreeNode("🔃", "While(True)", WorkTreeNodeType.ProgramInstr),
                "ARC" => new WorkTreeNode("◜", "Arc(p1, p2, 50)", WorkTreeNodeType.ProgramInstr),
                "CIRCLE" => new WorkTreeNode("⭕", "Circle(p1, p2)", WorkTreeNodeType.ProgramInstr),
                "DOFILE" => new WorkTreeNode("📄", "DoFile(sub.lua)", WorkTreeNodeType.ProgramInstr),
                "INIT" => new WorkTreeNode("🏠", "Initialize()", WorkTreeNodeType.ProgramInstr),
                _ => null
            };

            if (node != null)
                programFolder.Children.Add(node);
        }

        [RelayCommand]
        private void DeleteSelected()
        {
            if (_selectedNode == null) return;
            RemoveNodeRecursive(RootNodes, _selectedNode);
            SelectedNode = null;
        }

        [RelayCommand]
        private void EditSelected()
        {
            if (_selectedNode == null) return;
        }

        [RelayCommand]
        private void DuplicateSelected()
        {
            if (_selectedNode == null) return;
        }

        [RelayCommand]
        private void SetSpeed()
        {
            if (_selectedNode == null) return;
        }

        [RelayCommand]
        private void SetBlendRadius()
        {
            if (_selectedNode == null) return;
        }

        [RelayCommand]
        private void ConvertMotionType(string newType)
        {
            if (_selectedNode == null || _selectedNode.NodeType != WorkTreeNodeType.ProgramInstr) return;
        }

        private bool RemoveNodeRecursive(ObservableCollection<WorkTreeNode> nodes, WorkTreeNode target)
        {
            if (nodes.Remove(target)) return true;
            foreach (var node in nodes)
            {
                if (RemoveNodeRecursive(node.Children, target)) return true;
            }
            return false;
        }

        private WorkTreeNode? FindNodeByType(WorkTreeNodeType type, string? nameContains)
        {
            return FindRecursive(RootNodes, type, nameContains);
        }

        private WorkTreeNode? FindRecursive(ObservableCollection<WorkTreeNode> nodes, WorkTreeNodeType type, string? nameContains)
        {
            foreach (var n in nodes)
            {
                if (n.NodeType == type && (nameContains == null || n.Name.Contains(nameContains, StringComparison.OrdinalIgnoreCase)))
                    return n;
                var found = FindRecursive(n.Children, type, nameContains);
                if (found != null) return found;
            }
            return null;
        }

        /// <summary>
        /// Update joint angle display values in the tree
        /// </summary>
        public void UpdateJointValues(double[] angles)
        {
            if (RootNodes.Count == 0) return;
            var robot = RootNodes[0];
            for (int i = 0; i < Math.Min(angles.Length, robot.Children.Count); i++)
            {
                robot.Children[i].Value = $"{angles[i]:F2}°";
            }
        }
    }

    // ── WorkTree Node Types ──────────────────────────────────────

    public enum WorkTreeNodeType
    {
        Robot,
        Joint,
        Tool,
        WorkObject,
        Point,
        Folder,
        Program,
        ProgramInstr,
        Workpiece
    }

    /// <summary>
    /// A single node in the hierarchical work tree.
    /// </summary>
    public partial class WorkTreeNode : ObservableObject
    {
        [ObservableProperty] private string _icon;
        [ObservableProperty] private string _name;
        [ObservableProperty] private string _value = string.Empty;
        [ObservableProperty] private bool _isExpanded;
        [ObservableProperty] private bool _isSelected;
        [ObservableProperty] private WorkTreeNodeType _nodeType;
        [ObservableProperty] private string _tag = string.Empty;

        public ObservableCollection<WorkTreeNode> Children { get; } = new();

        public WorkTreeNode(string icon, string name, WorkTreeNodeType nodeType)
        {
            _icon = icon;
            _name = name;
            _nodeType = nodeType;
        }
    }
}
