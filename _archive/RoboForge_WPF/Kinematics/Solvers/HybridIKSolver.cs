using System.Collections.Generic;

namespace RoboForge_WPF.Kinematics.Solvers
{
    public class HybridIKSolver : IIKSolver
    {
        public string Name => "Hybrid Solver (Analytical + DLS)";

        private AnalyticalIKSolver _analytical = new AnalyticalIKSolver();
        private NumericalIKSolver _numerical = new NumericalIKSolver();

        public List<JointSolution> Solve(EndEffectorPose target, RobotModel model, double[] currentJoints)
        {
            // 1. Try Analytical First (Fastest, Global)
            var analyticalSolutions = _analytical.Solve(target, model, currentJoints);
            
            if (analyticalSolutions.Count > 0 && analyticalSolutions[0].IsValid)
            {
                return analyticalSolutions;
            }

            // 2. Fallback to Numerical if Analytical fails (Singularity, non-intersecting wrist)
            return _numerical.Solve(target, model, currentJoints);
        }
    }
}
