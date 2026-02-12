using System;
using System.Collections.Generic;
using System.Windows.Media.Media3D;
using RobotSimulator.Core.Models;
using RobotSimulator.Core.Import;

namespace RobotSimulator.Core.Supervisor
{
    /// <summary>
    /// Validation supervisor for robot program safety and physics consistency.
    /// Checks joint limits, collisions, singularities, and workspace boundaries.
    /// </summary>
    public class ValidationSupervisor
    {
        private readonly List<ValidationError> _errors = new();
        private MeshGeometry3D? _workpieceMesh;
        private Rect3D _workspaceBounds;

        public IReadOnlyList<ValidationError> Errors => _errors;
        public bool HasErrors => _errors.Count > 0;

        public ValidationSupervisor()
        {
            // Default workspace bounds (meters) - typical welding cell
            _workspaceBounds = new Rect3D(-2, -2, 0, 4, 4, 2.5);
        }

        /// <summary>
        /// Set workpiece mesh for collision detection
        /// </summary>
        public void SetWorkpiece(MeshGeometry3D mesh)
        {
            _workpieceMesh = mesh;
        }

        /// <summary>
        /// Set custom workspace boundaries
        /// </summary>
        public void SetWorkspaceBounds(Rect3D bounds)
        {
            _workspaceBounds = bounds;
        }

        /// <summary>
        /// Validate entire robot program
        /// </summary>
        public List<ValidationError> ValidateProgram(RobotProgram program)
        {
            _errors.Clear();

            for (int i = 0; i < program.Points.Count; i++)
            {
                var point = program.Points[i];
                ValidatePoint(point, i);

                // Check path between points
                if (i > 0)
                {
                    ValidatePath(program.Points[i - 1], point, i);
                }
            }

            // Check welding parameters
            ValidateWeldingSequence(program);

            return new List<ValidationError>(_errors);
        }

        /// <summary>
        /// Validate single teach point
        /// </summary>
        public List<ValidationError> ValidatePoint(TeachPoint point, int index = -1)
        {
            var pointErrors = new List<ValidationError>();
            string prefix = index >= 0 ? $"Point {index + 1}" : $"P{point.Id}";

            // Check joint limits
            for (int j = 0; j < 6; j++)
            {
                double angleDeg = point.JointAngles[j] * 180.0 / Math.PI;
                var limits = FanucArcMate120iC.JointLimits[j];

                if (angleDeg < limits.Min || angleDeg > limits.Max)
                {
                    var error = new ValidationError
                    {
                        Type = ErrorType.JointLimit,
                        Severity = ErrorSeverity.Error,
                        PointIndex = index,
                        Message = $"{prefix}: J{j + 1} = {angleDeg:F1}° exceeds limits [{limits.Min}°, {limits.Max}°]"
                    };
                    _errors.Add(error);
                    pointErrors.Add(error);
                }

                // Warning for near-limit joints
                double margin = 5.0; // degrees
                if (angleDeg < limits.Min + margin || angleDeg > limits.Max - margin)
                {
                    var warning = new ValidationError
                    {
                        Type = ErrorType.JointLimit,
                        Severity = ErrorSeverity.Warning,
                        PointIndex = index,
                        Message = $"{prefix}: J{j + 1} = {angleDeg:F1}° is near joint limit"
                    };
                    _errors.Add(warning);
                    pointErrors.Add(warning);
                }
            }

            // Check workspace bounds
            var pos = point.CartesianPosition;
            if (!_workspaceBounds.Contains(pos))
            {
                var error = new ValidationError
                {
                    Type = ErrorType.WorkspaceBoundary,
                    Severity = ErrorSeverity.Error,
                    PointIndex = index,
                    Message = $"{prefix}: Position ({pos.X * 1000:F0}, {pos.Y * 1000:F0}, {pos.Z * 1000:F0}) mm is outside workspace"
                };
                _errors.Add(error);
                pointErrors.Add(error);
            }

            // Check for singularity (J5 near zero)
            double j5Deg = Math.Abs(point.JointAngles[4] * 180.0 / Math.PI);
            if (j5Deg < 5.0)
            {
                var warning = new ValidationError
                {
                    Type = ErrorType.Singularity,
                    Severity = ErrorSeverity.Warning,
                    PointIndex = index,
                    Message = $"{prefix}: Near wrist singularity (J5 = {point.JointAngles[4] * 180 / Math.PI:F1}°)"
                };
                _errors.Add(warning);
                pointErrors.Add(warning);
            }

            // Check speed limits
            if (point.Motion == MotionType.Linear && point.Speed > 100)
            {
                var warning = new ValidationError
                {
                    Type = ErrorType.SpeedLimit,
                    Severity = ErrorSeverity.Warning,
                    PointIndex = index,
                    Message = $"{prefix}: Linear speed {point.Speed:F0} mm/s is high for welding (typical < 30 mm/s)"
                };
                _errors.Add(warning);
                pointErrors.Add(warning);
            }

            return pointErrors;
        }

        /// <summary>
        /// Validate path between two points
        /// </summary>
        private void ValidatePath(TeachPoint from, TeachPoint to, int toIndex)
        {
            // Check for large joint movements
            for (int j = 0; j < 6; j++)
            {
                double delta = Math.Abs(to.JointAngles[j] - from.JointAngles[j]) * 180.0 / Math.PI;
                if (delta > 90)
                {
                    _errors.Add(new ValidationError
                    {
                        Type = ErrorType.LargeMotion,
                        Severity = ErrorSeverity.Warning,
                        PointIndex = toIndex,
                        Message = $"Point {toIndex}: J{j + 1} moves {delta:F0}° (large motion)"
                    });
                }
            }

            // Check for rapid motion during welding
            if (to.WeldingEnabled && to.Motion == MotionType.Joint)
            {
                _errors.Add(new ValidationError
                {
                    Type = ErrorType.WeldingParameter,
                    Severity = ErrorSeverity.Warning,
                    PointIndex = toIndex,
                    Message = $"Point {toIndex}: Using JOINT motion during welding (LINEAR recommended)"
                });
            }
        }

        /// <summary>
        /// Validate welding sequence logic
        /// </summary>
        private void ValidateWeldingSequence(RobotProgram program)
        {
            bool isWelding = false;
            int weldStartIndex = -1;

            for (int i = 0; i < program.Points.Count; i++)
            {
                var point = program.Points[i];

                if (point.WeldingEnabled && !isWelding)
                {
                    // Weld start
                    isWelding = true;
                    weldStartIndex = i;

                    // Check for valid weld parameters
                    var weld = point.WeldParams ?? program.DefaultWeldParams;
                    if (weld.ArcCurrent < 50 || weld.ArcCurrent > 500)
                    {
                        _errors.Add(new ValidationError
                        {
                            Type = ErrorType.WeldingParameter,
                            Severity = ErrorSeverity.Warning,
                            PointIndex = i,
                            Message = $"Point {i + 1}: Arc current {weld.ArcCurrent}A outside typical range (50-500A)"
                        });
                    }

                    if (weld.WireFeedRate < 1 || weld.WireFeedRate > 25)
                    {
                        _errors.Add(new ValidationError
                        {
                            Type = ErrorType.WeldingParameter,
                            Severity = ErrorSeverity.Warning,
                            PointIndex = i,
                            Message = $"Point {i + 1}: Wire feed {weld.WireFeedRate} m/min outside typical range (1-25)"
                        });
                    }
                }
                else if (!point.WeldingEnabled && isWelding)
                {
                    // Weld end
                    isWelding = false;
                }
            }

            // Check for unclosed weld
            if (isWelding)
            {
                _errors.Add(new ValidationError
                {
                    Type = ErrorType.WeldingParameter,
                    Severity = ErrorSeverity.Error,
                    PointIndex = program.Points.Count - 1,
                    Message = $"Welding started at point {weldStartIndex + 1} but never ended"
                });
            }
        }

        /// <summary>
        /// Basic collision check (bounding box)
        /// </summary>
        public bool CheckCollision(Point3D robotTcp, double safetyRadius = 0.05)
        {
            if (_workpieceMesh == null) return false;

            var bounds = STLLoader.GetBoundingBox(_workpieceMesh);
            
            // Inflate bounds by safety radius
            var expanded = new Rect3D(
                bounds.X - safetyRadius,
                bounds.Y - safetyRadius,
                bounds.Z - safetyRadius,
                bounds.SizeX + 2 * safetyRadius,
                bounds.SizeY + 2 * safetyRadius,
                bounds.SizeZ + 2 * safetyRadius);

            return expanded.Contains(robotTcp);
        }

        /// <summary>
        /// Get validation summary
        /// </summary>
        public string GetSummary()
        {
            int errors = 0, warnings = 0;
            foreach (var e in _errors)
            {
                if (e.Severity == ErrorSeverity.Error) errors++;
                else warnings++;
            }
            return $"Validation: {errors} errors, {warnings} warnings";
        }
    }

    /// <summary>
    /// Validation error details
    /// </summary>
    public class ValidationError
    {
        public ErrorType Type { get; set; }
        public ErrorSeverity Severity { get; set; }
        public int PointIndex { get; set; }
        public string Message { get; set; } = "";
    }

    public enum ErrorType
    {
        JointLimit,
        WorkspaceBoundary,
        Singularity,
        Collision,
        SpeedLimit,
        LargeMotion,
        WeldingParameter
    }

    public enum ErrorSeverity
    {
        Warning,
        Error
    }
}
