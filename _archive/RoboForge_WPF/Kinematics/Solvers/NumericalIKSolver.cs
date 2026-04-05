using System;
using System.Collections.Generic;

namespace RoboForge_WPF.Kinematics.Solvers
{
    public class NumericalIKSolver : IIKSolver
    {
        public string Name => "Numerical Jacobian DLS";

        public List<JointSolution> Solve(EndEffectorPose target, RobotModel model, double[] currentJoints)
        {
            var solutions = new List<JointSolution>();
            double[] joints = (double[])currentJoints.Clone();

            int maxIterations = 100;
            double tolerance = 1e-4;
            double damping = 0.05; // Damped Least Squares factor
            bool converged = false;

            for (int iter = 0; iter < maxIterations; iter++)
            {
                var fkPose = model.ComputeFK(joints);
                
                // Calculate Cartesian Error (Position)
                double ex = target.X - fkPose.X;
                double ey = target.Y - fkPose.Y;
                double ez = target.Z - fkPose.Z;

                // Simple position-only error magnitude for convergence (Rotation convergence added later)
                double errorMag = Math.Sqrt(ex*ex + ey*ey + ez*ez);

                if (errorMag < tolerance)
                {
                    converged = true;
                    break;
                }

                // Compute Numerical Jacobian (Position only for this minimal fallback implementation)
                double[,] jacobian = ComputeJacobian(model, joints);

                // pseudo-inverse with DLS: J_pinv = J^T * (J * J^T + lambda^2 * I)^-1
                // For simplicity in this base stub, we apply a direct transpose with scalar damping
                // DeltaTheta = J^T * Error * damping
                for (int i = 0; i < model.DOF; i++)
                {
                    double deltaT = (jacobian[0, i] * ex + jacobian[1, i] * ey + jacobian[2, i] * ez) * damping;
                    joints[i] += deltaT;
                    joints[i] = model.Limits[i].Clamp(joints[i]);
                }
            }

            // Calculate cost (distance from original joints)
            double cost = 0;
            for (int i = 0; i < model.DOF; i++)
            {
                cost += Math.Abs(joints[i] - currentJoints[i]);
            }

            solutions.Add(new JointSolution(joints, cost, converged, Name));
            return solutions;
        }

        private double[,] ComputeJacobian(RobotModel model, double[] joints)
        {
            double delta = 1e-4;
            double[,] J = new double[3, model.DOF]; // 3x6 Jacobian (Position only for stub)
            var basePose = model.ComputeFK(joints);

            for(int i = 0; i < model.DOF; i++)
            {
                double originalAxis = joints[i];
                joints[i] += delta;
                var testPose = model.ComputeFK(joints);
                
                J[0, i] = (testPose.X - basePose.X) / delta;
                J[1, i] = (testPose.Y - basePose.Y) / delta;
                J[2, i] = (testPose.Z - basePose.Z) / delta;
                
                joints[i] = originalAxis; // revert
            }
            return J;
        }
    }
}
