using System.Collections.ObjectModel;
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using TeachPendant_WPF.SceneGraph;

namespace TeachPendant_WPF.ViewModels
{
    /// <summary>
    /// Manages the world scene: workpieces, coordinate frames, constraints,
    /// and the Work Area Configuration mode.
    /// </summary>
    public partial class WorldViewModel : ObservableObject
    {
        private readonly SceneGraphManager _sceneGraph;

        // ── Work Area Config Mode ───────────────────────────────────

        [ObservableProperty] private bool _isWorkAreaConfigActive;

        // ── Collections (MVVM-bound) ────────────────────────────────

        public ObservableCollection<WorkpieceNode> Workpieces { get; } = new();
        public ObservableCollection<FrameNode> Frames { get; } = new();
        public ObservableCollection<ConstraintNode> Constraints { get; } = new();

        // ── Selection ───────────────────────────────────────────────

        [ObservableProperty] private SceneNode? _selectedEntity;

        // ── Construction ────────────────────────────────────────────

        public WorldViewModel(SceneGraphManager sceneGraph)
        {
            _sceneGraph = sceneGraph;

            // Sync initial frames
            foreach (var f in sceneGraph.Frames)
                Frames.Add(f);
        }

        // ── Commands ────────────────────────────────────────────────

        [RelayCommand]
        private void ToggleWorkAreaConfig()
        {
            IsWorkAreaConfigActive = !IsWorkAreaConfigActive;
        }

        [RelayCommand]
        private void AddWorkpiece()
        {
            var wp = new WorkpieceNode
            {
                Name = $"Workpiece_{Workpieces.Count + 1}"
            };
            _sceneGraph.AddWorkpiece(wp);
            Workpieces.Add(wp);
        }

        [RelayCommand]
        private void RemoveWorkpiece(WorkpieceNode? wp)
        {
            if (wp == null) return;
            _sceneGraph.RemoveWorkpiece(wp);
            Workpieces.Remove(wp);
        }

        [RelayCommand]
        private void AddFrame(string type)
        {
            var frameKind = type?.ToUpper() switch
            {
                "TOOL" => FrameNode.FrameType.ToolTCP,
                "WORKOBJECT" or "WOBJ" => FrameNode.FrameType.WorkObject,
                "REFERENCE" => FrameNode.FrameType.ReferencePlane,
                _ => FrameNode.FrameType.UserDefined
            };

            var frame = new FrameNode
            {
                Name = $"{type}_{Frames.Count + 1}",
                FrameKind = frameKind
            };
            _sceneGraph.AddFrame(frame);
            Frames.Add(frame);
        }

        [RelayCommand]
        private void RemoveFrame(FrameNode? frame)
        {
            if (frame == null) return;
            _sceneGraph.RemoveFrame(frame);
            Frames.Remove(frame);
        }

        [RelayCommand]
        private void AddConstraint(string type)
        {
            var kind = type?.ToUpper() switch
            {
                "DISTANCE" => ConstraintNode.ConstraintType.Distance,
                "COINCIDENT" => ConstraintNode.ConstraintType.Coincident,
                "PARALLEL" => ConstraintNode.ConstraintType.Parallel,
                "TANGENT" => ConstraintNode.ConstraintType.Tangent,
                "PERPENDICULAR" => ConstraintNode.ConstraintType.Perpendicular,
                _ => ConstraintNode.ConstraintType.Distance
            };

            var constraint = new ConstraintNode
            {
                Name = $"Constraint_{Constraints.Count + 1}",
                ConstraintKind = kind
            };
            _sceneGraph.AddConstraint(constraint);
            Constraints.Add(constraint);
        }
    }
}
