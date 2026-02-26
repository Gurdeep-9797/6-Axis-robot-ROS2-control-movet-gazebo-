using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.Linq;
using System.Runtime.CompilerServices;

namespace TeachPendant_WPF.SceneGraph
{
    /// <summary>
    /// Central controller for the deterministic world model.
    /// Owns the root node, manages traversal, constraint solving,
    /// and emits change notifications for MVVM binding.
    /// 
    /// This is the single source of truth for:
    ///   - Robot position & kinematics
    ///   - Workpiece positions
    ///   - Reference frames
    ///   - Constraints
    /// </summary>
    public class SceneGraphManager : INotifyPropertyChanged
    {
        // ── Root Node ───────────────────────────────────────────────

        /// <summary>
        /// Top-level world root. All entities are descendants of this node.
        /// </summary>
        public SceneNode Root { get; private set; }

        // ── Named Subtrees ──────────────────────────────────────────

        /// <summary>
        /// The robot kinematic chain.
        /// </summary>
        public RobotNode? Robot { get; private set; }

        /// <summary>
        /// Container for all workpieces in the work area.
        /// </summary>
        public SceneNode WorkArea { get; private set; }

        /// <summary>
        /// Global coordinate frames (tool, work objects, user frames).
        /// </summary>
        public ObservableCollection<FrameNode> Frames { get; } = new();

        /// <summary>
        /// All geometric constraints in the scene.
        /// </summary>
        public ObservableCollection<ConstraintNode> Constraints { get; } = new();

        // ── Events ──────────────────────────────────────────────────

        /// <summary>
        /// Fired after every UpdateAll() completes.
        /// ViewModels subscribe to this to refresh 3D visuals.
        /// </summary>
        public event Action? Changed;

        // ── Construction ────────────────────────────────────────────

        public SceneGraphManager()
        {
            // Create the world root
            Root = new WorldRootNode { Name = "WorldRoot" };

            // Create work area container
            WorkArea = new WorkAreaNode { Name = "WorkArea" };
            Root.AddChild(WorkArea);
        }

        // ── Robot Setup ─────────────────────────────────────────────

        /// <summary>
        /// Set the robot kinematic chain. Called once during initialization
        /// after loading the URDF or configuration.
        /// </summary>
        public void SetRobot(RobotNode robot)
        {
            if (Robot != null)
                Root.RemoveChild(Robot);

            Robot = robot;
            Root.AddChild(robot);
            OnPropertyChanged(nameof(Robot));
        }

        // ── Frame Management ────────────────────────────────────────

        public void AddFrame(FrameNode frame)
        {
            Root.AddChild(frame);
            Frames.Add(frame);
        }

        public void RemoveFrame(FrameNode frame)
        {
            Root.RemoveChild(frame);
            Frames.Remove(frame);
        }

        // ── Workpiece Management ────────────────────────────────────

        public void AddWorkpiece(WorkpieceNode workpiece)
        {
            WorkArea.AddChild(workpiece);
        }

        public void RemoveWorkpiece(WorkpieceNode workpiece)
        {
            WorkArea.RemoveChild(workpiece);
        }

        // ── Constraint Management ───────────────────────────────────

        public void AddConstraint(ConstraintNode constraint)
        {
            Root.AddChild(constraint);
            Constraints.Add(constraint);
        }

        public void RemoveConstraint(ConstraintNode constraint)
        {
            Root.RemoveChild(constraint);
            Constraints.Remove(constraint);
        }

        // ── Update Cycle ────────────────────────────────────────────

        /// <summary>
        /// Traverse the entire scene graph, calling Update() on each node.
        /// Constraints are solved after all positional updates.
        /// Fire Changed event to notify UI.
        /// 
        /// Call this when:
        ///   - Robot joint angles change
        ///   - User edits work area
        ///   - Geometry is modified
        /// </summary>
        public void UpdateAll()
        {
            Traverse(Root);

            // Solve constraints after positional update
            foreach (var constraint in Constraints)
            {
                constraint.Update();
            }

            Changed?.Invoke();
        }

        private void Traverse(SceneNode node)
        {
            node.Update();
            foreach (var child in node.Children)
            {
                Traverse(child);
            }
        }

        // ── Query Helpers ───────────────────────────────────────────

        /// <summary>
        /// Find a node by ID anywhere in the scene graph.
        /// </summary>
        public SceneNode? FindById(Guid id)
        {
            return FindByIdRecursive(Root, id);
        }

        private SceneNode? FindByIdRecursive(SceneNode node, Guid id)
        {
            if (node.Id == id) return node;
            foreach (var child in node.Children)
            {
                var found = FindByIdRecursive(child, id);
                if (found != null) return found;
            }
            return null;
        }

        /// <summary>
        /// Find a node by name (first match).
        /// </summary>
        public SceneNode? FindByName(string name)
        {
            return FindByNameRecursive(Root, name);
        }

        private SceneNode? FindByNameRecursive(SceneNode node, string name)
        {
            if (node.Name == name) return node;
            foreach (var child in node.Children)
            {
                var found = FindByNameRecursive(child, name);
                if (found != null) return found;
            }
            return null;
        }

        /// <summary>
        /// Collect all nodes of a specific type from the scene graph.
        /// </summary>
        public List<T> FindAll<T>() where T : SceneNode
        {
            var results = new List<T>();
            CollectByType(Root, results);
            return results;
        }

        private void CollectByType<T>(SceneNode node, List<T> results) where T : SceneNode
        {
            if (node is T typed)
                results.Add(typed);
            foreach (var child in node.Children)
                CollectByType(child, results);
        }

        // ── INotifyPropertyChanged ──────────────────────────────────
        public event PropertyChangedEventHandler? PropertyChanged;
        private void OnPropertyChanged([CallerMemberName] string? name = null)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(name));
        }
    }

    // ── Internal Placeholder Nodes ──────────────────────────────────

    internal class WorldRootNode : SceneNode
    {
        public override void Update() { }
    }

    internal class WorkAreaNode : SceneNode
    {
        public override void Update() { }
    }
}
