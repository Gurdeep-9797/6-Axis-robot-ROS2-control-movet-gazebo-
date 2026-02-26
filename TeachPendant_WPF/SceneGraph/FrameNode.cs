using System.Windows.Media.Media3D;

namespace TeachPendant_WPF.SceneGraph
{
    /// <summary>
    /// A coordinate frame in the world model.
    /// Used for Tool TCP, Work Object, Reference Planes, and custom user frames.
    /// </summary>
    public class FrameNode : SceneNode
    {
        public enum FrameType
        {
            ToolTCP,
            WorkObject,
            ReferencePlane,
            UserDefined
        }

        private FrameType _frameKind = FrameType.UserDefined;
        public FrameType FrameKind
        {
            get => _frameKind;
            set { _frameKind = value; OnPropertyChanged(); }
        }

        /// <summary>
        /// Whether this frame is currently the active tool or work object.
        /// </summary>
        private bool _isActive;
        public bool IsActive
        {
            get => _isActive;
            set { _isActive = value; OnPropertyChanged(); }
        }

        /// <summary>
        /// Description text displayed in the UI.
        /// </summary>
        private string _description = string.Empty;
        public string Description
        {
            get => _description;
            set { _description = value; OnPropertyChanged(); }
        }

        /// <summary>
        /// Convenience: Get the origin position of this frame in world space.
        /// </summary>
        public Point3D Origin => WorldPosition;

        /// <summary>
        /// Convenience: Get the Z-axis direction of this frame in world space.
        /// </summary>
        public Vector3D ZAxis
        {
            get
            {
                var m = WorldMatrix;
                return new Vector3D(m.M31, m.M32, m.M33);
            }
        }

        /// <summary>
        /// Set this frame's pose from translation and quaternion rotation.
        /// </summary>
        public void SetPose(Vector3D translation, Quaternion rotation)
        {
            LocalMatrix = MakeTransform(translation, rotation);
        }

        public override void Update() { }
    }
}
