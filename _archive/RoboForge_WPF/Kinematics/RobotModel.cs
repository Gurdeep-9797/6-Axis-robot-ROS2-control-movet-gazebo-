using System;
using System.Collections.Generic;

namespace RoboForge_WPF.Kinematics
{
    public class JointLimit
    {
        public double Min { get; set; }
        public double Max { get; set; }
        
        public JointLimit(double min, double max)
        {
            Min = min;
            Max = max;
        }

        public double Clamp(double value) => Math.Clamp(value, Min, Max);
    }

    /// <summary>
    /// Represents the kinematic and dynamic model of the robot.
    /// Used for IK/FK calculations and collision detection.
    /// </summary>
    public class RobotModel
    {
        public int DOF { get; set; } = 6;
        public List<JointLimit> Limits { get; set; } = new List<JointLimit>();
        
        // DH Parameters (a, alpha, d, theta_offset) - specifically for an industrial 6-DOF
        public double[] DH_a { get; set; } = new double[6];
        public double[] DH_alpha { get; set; } = new double[6];
        public double[] DH_d { get; set; } = new double[6];
        public double[] DH_theta_offset { get; set; } = new double[6];

        public RobotModel()
        {
            // Default limits for 6-DOF
            Limits.Add(new JointLimit(-170, 170));
            Limits.Add(new JointLimit(-135, 135));
            Limits.Add(new JointLimit(-150, 150));
            Limits.Add(new JointLimit(-180, 180));
            Limits.Add(new JointLimit(-120, 120));
            Limits.Add(new JointLimit(-360, 360));
            
            // Standard UR/industrial style DH params as placeholders
            DH_d[0] = 150; DH_a[0] = 0;   DH_alpha[0] = Math.PI / 2;
            DH_d[1] = 0;   DH_a[1] = -400; DH_alpha[1] = 0;
            DH_d[2] = 0;   DH_a[2] = -350; DH_alpha[2] = 0;
            DH_d[3] = 100; DH_a[3] = 0;   DH_alpha[3] = Math.PI / 2;
            DH_d[4] = 100; DH_a[4] = 0;   DH_alpha[4] = -Math.PI / 2;
            DH_d[5] = 100; DH_a[5] = 0;   DH_alpha[5] = 0;
        }

        public EndEffectorPose ComputeFK(double[] joints)
        {
            // Placeholder: true FK via matrix multiplication of DH params would go here.
            // Returning a dummy pose for now to satisfy interface
            return new EndEffectorPose(0, 0, 0, 0, 0, 0);
        }
    }
}
