using System;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.Runtime.CompilerServices;
using System.Windows.Media.Media3D;

namespace TeachPendant_WPF.SceneGraph
{
    /// <summary>
    /// Base class for every entity in the deterministic world model.
    /// Provides hierarchical transform propagation, MVVM-compatible change notification,
    /// and the foundation for robot links, workpieces, frames, and constraints.
    /// </summary>
    public abstract class SceneNode : INotifyPropertyChanged
    {
        // ── Identity ────────────────────────────────────────────────
        public Guid Id { get; set; } = Guid.NewGuid();

        private string _name = string.Empty;
        public string Name
        {
            get => _name;
            set { _name = value; OnPropertyChanged(); }
        }

        // ── Hierarchy ───────────────────────────────────────────────
        private SceneNode? _parent;
        public SceneNode? Parent
        {
            get => _parent;
            private set { _parent = value; OnPropertyChanged(); }
        }

        public ObservableCollection<SceneNode> Children { get; } = new();

        public void AddChild(SceneNode child)
        {
            child.Parent = this;
            Children.Add(child);
        }

        public void RemoveChild(SceneNode child)
        {
            child.Parent = null;
            Children.Remove(child);
        }

        // ── Transform ───────────────────────────────────────────────
        private Matrix3D _localMatrix = Matrix3D.Identity;

        /// <summary>
        /// The node's transform relative to its parent.
        /// Subclasses (e.g. JointNode) override SetLocalTransform() to
        /// recalculate this when their parameters change.
        /// </summary>
        public Matrix3D LocalMatrix
        {
            get => _localMatrix;
            set
            {
                _localMatrix = value;
                OnPropertyChanged();
                OnPropertyChanged(nameof(WorldMatrix));
                InvalidateChildWorldTransforms();
            }
        }

        /// <summary>
        /// Accumulated transform from world origin to this node.
        /// Computed lazily by walking up the parent chain.
        /// </summary>
        public Matrix3D WorldMatrix
        {
            get
            {
                if (Parent == null)
                    return LocalMatrix;

                var parentWorld = Parent.WorldMatrix;
                // Local is applied first, then the parent's world transform
                var combined = LocalMatrix;
                combined.Append(parentWorld);
                return combined;
            }
        }

        /// <summary>
        /// World-space position extracted from the world matrix.
        /// Convenience accessor for FK readout / TCP display.
        /// </summary>
        public Point3D WorldPosition => new Point3D(
            WorldMatrix.OffsetX,
            WorldMatrix.OffsetY,
            WorldMatrix.OffsetZ);

        // ── Helpers ─────────────────────────────────────────────────

        /// <summary>
        /// Build a local matrix from translation + quaternion rotation.
        /// This is the preferred factory — avoids Euler gimbal lock internally.
        /// </summary>
        public static Matrix3D MakeTransform(Vector3D translation, Quaternion rotation)
        {
            var m = Matrix3D.Identity;

            // Apply rotation
            if (!rotation.IsIdentity)
            {
                var rotMatrix = new RotateTransform3D(
                    new QuaternionRotation3D(rotation)).Value;
                m = rotMatrix;
            }

            // Apply translation
            m.OffsetX = translation.X;
            m.OffsetY = translation.Y;
            m.OffsetZ = translation.Z;

            return m;
        }

        /// <summary>
        /// Build a local matrix from translation + axis-angle rotation.
        /// Useful for revolute joints.
        /// </summary>
        public static Matrix3D MakeTransform(Vector3D translation, Vector3D axis, double angleDeg)
        {
            var q = new Quaternion(axis, angleDeg);
            return MakeTransform(translation, q);
        }

        // ── Update Hook ─────────────────────────────────────────────

        /// <summary>
        /// Called by SceneGraphManager.Traverse() on every frame / tick.
        /// Override in subclasses to recalculate local transform from
        /// domain-specific data (e.g. joint angle, constraint solver output).
        /// </summary>
        public virtual void Update() { }

        // ── Dirty Propagation ───────────────────────────────────────

        private void InvalidateChildWorldTransforms()
        {
            foreach (var child in Children)
            {
                child.OnPropertyChanged(nameof(WorldMatrix));
                child.OnPropertyChanged(nameof(WorldPosition));
                child.InvalidateChildWorldTransforms();
            }
        }

        // ── INotifyPropertyChanged ──────────────────────────────────
        public event PropertyChangedEventHandler? PropertyChanged;
        protected void OnPropertyChanged([CallerMemberName] string? name = null)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(name));
        }
    }
}
