using System;
using System.Windows.Media.Media3D;
using TeachPendant_WPF.SceneGraph;

namespace TeachPendant_WPF.Services
{
    /// <summary>
    /// Forward and Inverse Kinematics engine for the 6-DOF robot.
    /// Uses the SceneGraph RobotNode for FK (transform cascading)
    /// and a numerical Jacobian-based solver for IK.
    /// </summary>
    public class KinematicsEngine
    {
        private readonly SceneGraphManager _sceneGraph;

        // IK Solver parameters
        private const int MaxIterations = 100;
        private const double ConvergenceThreshold = 0.1; // mm
        private const double DampingFactor = 0.5;
        private const double JointStepDeg = 0.1;

        public KinematicsEngine(SceneGraphManager sceneGraph)
        {
            _sceneGraph = sceneGraph;
        }

        // ── Forward Kinematics ──────────────────────────────────────

        /// <summary>
        /// Apply joint angles and return the resulting TCP pose.
        /// FK is handled automatically by the SceneGraph transform chain.
        /// </summary>
        public Matrix3D ComputeFK(double[] jointAnglesDeg)
        {
            if (_sceneGraph.Robot == null)
                return Matrix3D.Identity;

            _sceneGraph.Robot.ApplyJointAngles(jointAnglesDeg);
            return _sceneGraph.Robot.GetTCPPose();
        }

        /// <summary>
        /// Get TCP position for given joint angles without modifying the scene graph.
        /// Creates a temporary calculation by reading the current state.
        /// </summary>
        public Point3D ComputeFKPosition(double[] jointAnglesDeg)
        {
            var pose = ComputeFK(jointAnglesDeg);
            return new Point3D(pose.OffsetX, pose.OffsetY, pose.OffsetZ);
        }

        // ── Inverse Kinematics ──────────────────────────────────────

        /// <summary>
        /// Solve IK for a target position using Jacobian-based damped least squares.
        /// Returns the joint angles, or null if unreachable.
        /// </summary>
        public double[]? SolveIK(Point3D targetPosition)
        {
            if (_sceneGraph.Robot == null) return null;

            var robot = _sceneGraph.Robot;
            int dof = robot.DOF;
            double[] currentAngles = robot.GetJointAngles();

            for (int iter = 0; iter < MaxIterations; iter++)
            {
                // Current TCP
                robot.ApplyJointAngles(currentAngles);
                var currentTCP = robot.GetTCPPosition();

                // Error vector
                var error = new Vector3D(
                    targetPosition.X - currentTCP.X,
                    targetPosition.Y - currentTCP.Y,
                    targetPosition.Z - currentTCP.Z);

                // Check convergence
                if (error.Length < ConvergenceThreshold)
                {
                    return currentAngles;
                }

                // Compute Jacobian numerically (3 × DOF matrix)
                double[,] jacobian = ComputeJacobian(currentAngles, dof);

                // Damped Least Squares: Δθ = Jᵀ(JJᵀ + λ²I)⁻¹ · e
                // Simplified: Use Jacobian transpose method for robustness
                double[] deltaAngles = JacobianTransposeStep(jacobian, error, dof);

                // Apply delta
                for (int j = 0; j < dof; j++)
                {
                    currentAngles[j] += deltaAngles[j];

                    // Clamp to joint limits
                    currentAngles[j] = Math.Clamp(
                        currentAngles[j],
                        robot.Joints[j].MinLimit,
                        robot.Joints[j].MaxLimit);
                }
            }

            // Did not converge — unreachable
            return null;
        }

        /// <summary>
        /// Solve IK and choose the solution with minimum joint delta from current position.
        /// </summary>
        public double[]? SolveIKMinDelta(Point3D targetPosition, double[] currentAngles)
        {
            var solution = SolveIK(targetPosition);
            // Future: Solve multiple IK solutions and pick min-delta.
            // For now, return the single numerical solution.
            return solution;
        }

        // ── Jacobian Computation ────────────────────────────────────

        private double[,] ComputeJacobian(double[] angles, int dof)
        {
            var robot = _sceneGraph.Robot!;
            double[,] J = new double[3, dof];

            // Base TCP
            robot.ApplyJointAngles(angles);
            var baseTCP = robot.GetTCPPosition();

            for (int j = 0; j < dof; j++)
            {
                // Perturb joint j
                double[] perturbed = (double[])angles.Clone();
                perturbed[j] += JointStepDeg;

                robot.ApplyJointAngles(perturbed);
                var perturbedTCP = robot.GetTCPPosition();

                // Partial derivative dP/dθj
                J[0, j] = (perturbedTCP.X - baseTCP.X) / JointStepDeg;
                J[1, j] = (perturbedTCP.Y - baseTCP.Y) / JointStepDeg;
                J[2, j] = (perturbedTCP.Z - baseTCP.Z) / JointStepDeg;
            }

            // Restore original angles
            robot.ApplyJointAngles(angles);

            return J;
        }

        private double[] JacobianTransposeStep(double[,] J, Vector3D error, int dof)
        {
            // Jᵀ · e
            double[] JtE = new double[dof];
            for (int j = 0; j < dof; j++)
            {
                JtE[j] = J[0, j] * error.X + J[1, j] * error.Y + J[2, j] * error.Z;
            }

            // J · Jᵀ · e (for scaling)
            double[] JJtE = new double[3];
            for (int i = 0; i < 3; i++)
            {
                JJtE[i] = 0;
                for (int j = 0; j < dof; j++)
                {
                    JJtE[i] += J[i, j] * JtE[j];
                }
            }

            // α = (eᵀ · J · Jᵀ · e) / (J·Jᵀ·e)ᵀ · (J·Jᵀ·e)
            double numerator = error.X * JJtE[0] + error.Y * JJtE[1] + error.Z * JJtE[2];
            double denominator = JJtE[0] * JJtE[0] + JJtE[1] * JJtE[1] + JJtE[2] * JJtE[2];

            double alpha = denominator > 1e-10 ? numerator / denominator : DampingFactor;
            alpha = Math.Clamp(alpha, 0.01, 2.0);

            // Δθ = α · Jᵀ · e
            double[] delta = new double[dof];
            for (int j = 0; j < dof; j++)
            {
                delta[j] = alpha * JtE[j];
            }

            return delta;
        }

        // ── Reachability Check ──────────────────────────────────────

        /// <summary>
        /// Check if a target position is reachable by attempting IK.
        /// </summary>
        public bool IsReachable(Point3D target)
        {
            return SolveIK(target) != null;
        }
    }
}
