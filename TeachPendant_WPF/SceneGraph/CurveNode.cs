using System;
using System.Windows.Media.Media3D;

namespace TeachPendant_WPF.SceneGraph
{
    /// <summary>
    /// Represents a parametric curve extracted from a workpiece
    /// or defined by the user. Used for curve-following toolpaths.
    /// 
    /// The curve is parameterized on [0, 1] where 0 = start and 1 = end.
    /// </summary>
    public class CurveNode : SceneNode
    {
        /// <summary>
        /// Evaluate the curve at parameter t ∈ [0, 1].
        /// Returns the 3D point in the curve's local coordinate frame.
        /// To get world coords: apply WorkpieceNode.WorldMatrix.
        /// </summary>
        private Func<double, Point3D>? _evaluateFunc;
        public Func<double, Point3D>? EvaluateFunc
        {
            get => _evaluateFunc;
            set { _evaluateFunc = value; OnPropertyChanged(); }
        }

        /// <summary>
        /// Evaluate the tangent direction at parameter t ∈ [0, 1].
        /// Used for orientation-along-curve computation.
        /// </summary>
        private Func<double, Vector3D>? _tangentFunc;
        public Func<double, Vector3D>? TangentFunc
        {
            get => _tangentFunc;
            set { _tangentFunc = value; OnPropertyChanged(); }
        }

        /// <summary>
        /// Total arc length of the curve in local units.
        /// </summary>
        private double _length;
        public double Length
        {
            get => _length;
            set { _length = value; OnPropertyChanged(); }
        }

        /// <summary>
        /// Whether this curve is currently selected by the user
        /// for toolpath generation.
        /// </summary>
        private bool _isSelected;
        public bool IsSelected
        {
            get => _isSelected;
            set { _isSelected = value; OnPropertyChanged(); }
        }

        // ── Evaluation Helpers ──────────────────────────────────────

        /// <summary>
        /// Evaluate position in world coordinates.
        /// </summary>
        public Point3D EvaluateWorld(double t)
        {
            if (_evaluateFunc == null) return new Point3D();
            var local = _evaluateFunc(Math.Clamp(t, 0.0, 1.0));
            return WorldMatrix.Transform(local);
        }

        /// <summary>
        /// Sample the curve into N evenly-spaced points (world coords).
        /// Used for toolpath discretization.
        /// </summary>
        public Point3D[] SampleWorld(int numPoints)
        {
            if (numPoints < 2) numPoints = 2;
            var points = new Point3D[numPoints];
            for (int i = 0; i < numPoints; i++)
            {
                double t = (double)i / (numPoints - 1);
                points[i] = EvaluateWorld(t);
            }
            return points;
        }

        public override void Update() { }
    }
}
