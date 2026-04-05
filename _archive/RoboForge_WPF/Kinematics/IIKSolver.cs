using System.Collections.Generic;

namespace RoboForge_WPF.Kinematics
{
    /// <summary>
    /// Represents a single IK solution with quality metrics.
    /// </summary>
    public class JointSolution
    {
        public double[] JointAngles { get; set; }
        public double Cost { get; set; }        // Lower is better
        public bool IsValid { get; set; }
        public string SolverUsed { get; set; } = "Unknown";

        public JointSolution(double[] angles, double cost, bool isValid, string solver)
        {
            JointAngles = (double[])angles.Clone();
            Cost = cost;
            IsValid = isValid;
            SolverUsed = solver;
        }
    }

    /// <summary>
    /// Interface for all IK solvers (Analytical, Numerical, Hybrid).
    /// </summary>
    public interface IIKSolver
    {
        string Name { get; }

        /// <summary>
        /// Solve IK for a target end-effector pose.
        /// Returns list of valid solutions sorted by cost (best first).
        /// </summary>
        List<JointSolution> Solve(EndEffectorPose target, RobotModel model, double[] currentJoints);
    }

    /// <summary>IK solver mode selection.</summary>
    public enum IKSolverMode
    {
        Analytical,
        Numerical,
        Hybrid
    }
}
