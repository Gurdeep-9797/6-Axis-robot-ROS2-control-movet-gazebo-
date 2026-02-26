using System.Windows.Media.Media3D;

namespace TeachPendant_WPF.SceneGraph
{
    /// <summary>
    /// Represents a single revolute joint (DOF) in the robot kinematic chain.
    /// When CurrentAngle changes, the LocalMatrix is recalculated, which
    /// cascades through the parent-child hierarchy so all downstream links move.
    /// </summary>
    public class JointNode : SceneNode
    {
        // ── Joint Parameters ────────────────────────────────────────
        private double _currentAngle;
        public double CurrentAngle
        {
            get => _currentAngle;
            set
            {
                // Clamp to limits
                _currentAngle = System.Math.Clamp(value, MinLimit, MaxLimit);
                OnPropertyChanged();
                RecalculateTransform();
            }
        }

        private double _minLimit = -180.0;
        public double MinLimit
        {
            get => _minLimit;
            set { _minLimit = value; OnPropertyChanged(); }
        }

        private double _maxLimit = 180.0;
        public double MaxLimit
        {
            get => _maxLimit;
            set { _maxLimit = value; OnPropertyChanged(); }
        }

        /// <summary>
        /// The axis of rotation in the joint's local frame.
        /// Typically (0,0,1) for Z-axis revolute joints in URDF.
        /// </summary>
        private Vector3D _axis = new Vector3D(0, 0, 1);
        public Vector3D Axis
        {
            get => _axis;
            set { _axis = value; OnPropertyChanged(); RecalculateTransform(); }
        }

        /// <summary>
        /// Fixed offset from parent joint origin to this joint origin
        /// (the URDF joint origin xyz). Applied BEFORE rotation.
        /// </summary>
        private Vector3D _originOffset = new Vector3D(0, 0, 0);
        public Vector3D OriginOffset
        {
            get => _originOffset;
            set { _originOffset = value; OnPropertyChanged(); RecalculateTransform(); }
        }

        /// <summary>
        /// Fixed orientation offset from URDF joint origin rpy.
        /// Applied as a pre-rotation before the variable joint angle.
        /// </summary>
        private Quaternion _originRotation = Quaternion.Identity;
        public Quaternion OriginRotation
        {
            get => _originRotation;
            set { _originRotation = value; OnPropertyChanged(); RecalculateTransform(); }
        }

        // ── Joint Type ──────────────────────────────────────────────
        public enum JointType { Revolute, Prismatic, Fixed }

        private JointType _type = JointType.Revolute;
        public JointType Type
        {
            get => _type;
            set { _type = value; OnPropertyChanged(); }
        }

        // ── Transform Calculation ──────────────────────────────────

        private void RecalculateTransform()
        {
            // 1. Start with the fixed origin offset (translation)
            var m = Matrix3D.Identity;
            m.OffsetX = _originOffset.X;
            m.OffsetY = _originOffset.Y;
            m.OffsetZ = _originOffset.Z;

            // 2. Apply fixed orientation (URDF rpy)
            if (!_originRotation.IsIdentity)
            {
                var fixedRot = new RotateTransform3D(
                    new QuaternionRotation3D(_originRotation)).Value;
                m = Matrix3D.Multiply(fixedRot, m);
            }

            // 3. Apply variable joint rotation
            if (_type == JointType.Revolute && System.Math.Abs(_currentAngle) > 1e-9)
            {
                var jointQ = new Quaternion(_axis, _currentAngle);
                var jointRot = new RotateTransform3D(
                    new QuaternionRotation3D(jointQ)).Value;
                m = Matrix3D.Multiply(jointRot, m);
            }
            else if (_type == JointType.Prismatic)
            {
                // Translate along axis
                var offset = Vector3D.Multiply(_currentAngle, _axis);
                m.OffsetX += offset.X;
                m.OffsetY += offset.Y;
                m.OffsetZ += offset.Z;
            }

            LocalMatrix = m;
        }

        public override void Update()
        {
            // RecalculateTransform is called on demand when angle changes.
            // Update() is available for constraint-driven recalculations.
        }
    }
}
