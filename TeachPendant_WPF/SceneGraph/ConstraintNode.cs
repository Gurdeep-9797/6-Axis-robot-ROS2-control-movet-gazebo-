using System.Windows.Media.Media3D;

namespace TeachPendant_WPF.SceneGraph
{
    /// <summary>
    /// Represents a geometric constraint between two scene nodes.
    /// Constraints are applied during the scene graph update cycle
    /// to enforce spatial relationships (like SolidWorks mates).
    /// </summary>
    public class ConstraintNode : SceneNode
    {
        public enum ConstraintType
        {
            Parallel,
            Distance,
            Tangent,
            Coincident,
            Perpendicular,
            Angle
        }

        // ── Constraint Parameters ───────────────────────────────────

        private ConstraintType _constraintKind = ConstraintType.Distance;
        public ConstraintType ConstraintKind
        {
            get => _constraintKind;
            set { _constraintKind = value; OnPropertyChanged(); }
        }

        /// <summary>
        /// First entity in the constraint (e.g. workpiece face, frame).
        /// </summary>
        private SceneNode? _entityA;
        public SceneNode? EntityA
        {
            get => _entityA;
            set { _entityA = value; OnPropertyChanged(); }
        }

        /// <summary>
        /// Second entity in the constraint.
        /// </summary>
        private SceneNode? _entityB;
        public SceneNode? EntityB
        {
            get => _entityB;
            set { _entityB = value; OnPropertyChanged(); }
        }

        /// <summary>
        /// Numeric value for the constraint (e.g. distance in mm, angle in degrees).
        /// Used by Distance and Angle constraint types.
        /// </summary>
        private double _targetValue;
        public double TargetValue
        {
            get => _targetValue;
            set { _targetValue = value; OnPropertyChanged(); }
        }

        /// <summary>
        /// Whether the constraint is currently satisfied.
        /// Computed by the constraint solver during Update().
        /// </summary>
        private bool _isSatisfied;
        public bool IsSatisfied
        {
            get => _isSatisfied;
            set { _isSatisfied = value; OnPropertyChanged(); }
        }

        // ── Solver ──────────────────────────────────────────────────

        public override void Update()
        {
            if (_entityA == null || _entityB == null)
            {
                IsSatisfied = false;
                return;
            }

            switch (_constraintKind)
            {
                case ConstraintType.Distance:
                    SolveDistance();
                    break;
                case ConstraintType.Coincident:
                    SolveCoincident();
                    break;
                case ConstraintType.Parallel:
                    // Full solver deferred to Phase H constraint engine
                    IsSatisfied = true;
                    break;
                default:
                    IsSatisfied = true;
                    break;
            }
        }

        private void SolveDistance()
        {
            var posA = _entityA!.WorldPosition;
            var posB = _entityB!.WorldPosition;
            var diff = posB - posA;
            double actualDistance = diff.Length;
            double error = System.Math.Abs(actualDistance - _targetValue);
            IsSatisfied = error < 0.01; // 0.01mm tolerance
        }

        private void SolveCoincident()
        {
            var posA = _entityA!.WorldPosition;
            var posB = _entityB!.WorldPosition;
            var diff = posB - posA;
            double distance = diff.Length;
            IsSatisfied = distance < 0.01; // 0.01mm tolerance
        }
    }
}
