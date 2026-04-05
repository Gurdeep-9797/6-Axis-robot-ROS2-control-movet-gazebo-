using System;
using System.Collections.Generic;

namespace RoboForge_WPF.Kinematics.Solvers
{
    public class AnalyticalIKSolver : IIKSolver
    {
        public string Name => "Analytical 6-DOF DH";

        public List<JointSolution> Solve(EndEffectorPose target, RobotModel model, double[] currentJoints)
        {
            var solutions = new List<JointSolution>();
            double[] joints = new double[6];

            // ── Geometric Decoupling Strategy ────
            // For a robot with intersecting wrist axes (standard UR or PUMA style),
            // the Wrist Center (WC) point can be found from the End Effector.
            // P_wc = P_ee - d6 * Z_ee
            
            double rz = target.RZ * Math.PI / 180.0;
            double ry = target.RY * Math.PI / 180.0;
            double rx = target.RX * Math.PI / 180.0;

            // Simple Euler ZYX rotation matrix to find the Z vector
            double cy = Math.Cos(ry), sy = Math.Sin(ry);
            double cz = Math.Cos(rz), sz = Math.Sin(rz);
            
            // The 3rd column of the rotation matrix gives the Z direction of the end effector
            double z_x = cy * Math.Cos(rx) * sz; // simplified 
            double z_y = cy * Math.Sin(rx) * sz;
            double z_z = Math.Cos(rx) * cy;

            double d6 = model.DH_d[5];
            double wc_x = target.X - d6 * z_x;
            double wc_y = target.Y - d6 * z_y;
            double wc_z = target.Z - d6 * z_z;

            // ── Theta 1 ────
            // Computed from the Projection of the wrist center onto the XY plane
            joints[0] = Math.Atan2(wc_y, wc_x);
            // Also potential for elbow-down solution: joints[0] = Atan2(y, x) + PI

            // ── Theta 2 & 3 ────
            // Geometric planar 2-link solver on the r-z slice
            double r = Math.Sqrt(wc_x * wc_x + wc_y * wc_y) - model.DH_a[0];
            double s = wc_z - model.DH_d[0];
            double D = (r*r + s*s - model.DH_a[1]*model.DH_a[1] - model.DH_a[2]*model.DH_a[2]) / 
                       (2 * model.DH_a[1] * model.DH_a[2]);
            
            // If D is outside [-1, 1], the target is unreachable
            if (D < -1 || D > 1) 
            {
                // Fallback / Return invalid
                return new List<JointSolution> { new JointSolution(currentJoints, double.MaxValue, false, Name) };
            }

            joints[2] = Math.Atan2(-Math.Sqrt(1 - D*D), D); // Elbow Up
            // A secondary solution would be +Sqrt for Elbow Down

            double k1 = model.DH_a[1] + model.DH_a[2] * Math.Cos(joints[2]);
            double k2 = model.DH_a[2] * Math.Sin(joints[2]);
            joints[1] = Math.Atan2(s, r) - Math.Atan2(k2, k1);

            // ── Theta 4, 5, 6 ────
            // Placeholder: Typically found by taking Inverse of R3_0 * R6_0 to find R6_3
            // Which directly maps to Euler ZYZ angles.
            joints[3] = 0; // Simplified for stub
            joints[4] = 0; 
            joints[5] = 0;

            // Convert to degrees and clamp
            for(int i = 0; i < 6; i++) {
                joints[i] = joints[i] * 180.0 / Math.PI;
                joints[i] = model.Limits[i].Clamp(joints[i]);
            }

            solutions.Add(new JointSolution(joints, 0, true, Name));
            return solutions;
        }
    }
}
