using System;
using System.Windows.Media.Media3D;

namespace RobotSimulator.Core.Models
{
    /// <summary>
    /// Represents a taught waypoint in the robot program.
    /// Each point stores position, motion type, and optional welding parameters.
    /// </summary>
    public class TeachPoint
    {
        /// <summary>Unique point identifier (1-based)</summary>
        public int Id { get; set; }
        
        /// <summary>Point name/label</summary>
        public string Name { get; set; } = "";
        
        /// <summary>Joint angles in radians [J1-J6]</summary>
        public double[] JointAngles { get; set; } = new double[6];
        
        /// <summary>Cartesian position (meters)</summary>
        public Point3D CartesianPosition { get; set; }
        
        /// <summary>Orientation as RPY in radians</summary>
        public double Roll { get; set; }
        public double Pitch { get; set; }
        public double Yaw { get; set; }
        
        /// <summary>Motion type for reaching this point</summary>
        public MotionType Motion { get; set; } = MotionType.Joint;
        
        /// <summary>Speed (% of max for JOINT, mm/s for LINEAR/CIRCULAR)</summary>
        public double Speed { get; set; } = 50.0;
        
        /// <summary>Acceleration (% of max)</summary>
        public double Acceleration { get; set; } = 100.0;
        
        /// <summary>Fine positioning or CNT (continuous)</summary>
        public TerminationType Termination { get; set; } = TerminationType.Fine;
        
        /// <summary>CNT value (0-100, only used when Termination is CNT)</summary>
        public int CntValue { get; set; } = 50;
        
        /// <summary>Welding on at this point</summary>
        public bool WeldingEnabled { get; set; }
        
        /// <summary>Welding parameters (null if not welding)</summary>
        public WeldingParameters? WeldParams { get; set; }
        
        /// <summary>User comment/notes</summary>
        public string Comment { get; set; } = "";
        
        /// <summary>Timestamp when point was taught</summary>
        public DateTime TaughtAt { get; set; } = DateTime.Now;
        
        /// <summary>Get joint angles in degrees</summary>
        public double[] GetJointAnglesDegrees()
        {
            var degrees = new double[6];
            for (int i = 0; i < 6; i++)
            {
                degrees[i] = JointAngles[i] * 180.0 / Math.PI;
            }
            return degrees;
        }
        
        /// <summary>Set joint angles from degrees</summary>
        public void SetJointAnglesDegrees(double[] degrees)
        {
            for (int i = 0; i < 6; i++)
            {
                JointAngles[i] = degrees[i] * Math.PI / 180.0;
            }
        }
        
        /// <summary>Clone this teach point</summary>
        public TeachPoint Clone()
        {
            return new TeachPoint
            {
                Id = Id,
                Name = Name,
                JointAngles = (double[])JointAngles.Clone(),
                CartesianPosition = CartesianPosition,
                Roll = Roll,
                Pitch = Pitch,
                Yaw = Yaw,
                Motion = Motion,
                Speed = Speed,
                Acceleration = Acceleration,
                Termination = Termination,
                CntValue = CntValue,
                WeldingEnabled = WeldingEnabled,
                WeldParams = WeldParams?.Clone(),
                Comment = Comment,
                TaughtAt = TaughtAt
            };
        }
        
        public override string ToString()
        {
            var pos = CartesianPosition;
            return $"P{Id}: ({pos.X * 1000:F1}, {pos.Y * 1000:F1}, {pos.Z * 1000:F1}) mm [{Motion}]";
        }
    }

    /// <summary>Motion interpolation type</summary>
    public enum MotionType
    {
        /// <summary>Joint interpolation - fastest, may curve in Cartesian space</summary>
        Joint,
        
        /// <summary>Linear interpolation - straight line in Cartesian space</summary>
        Linear,
        
        /// <summary>Circular arc interpolation - requires via point</summary>
        Circular
    }

    /// <summary>Point termination type</summary>
    public enum TerminationType
    {
        /// <summary>Fine - robot stops exactly at point</summary>
        Fine,
        
        /// <summary>CNT - continuous, robot blends through point</summary>
        Continuous
    }
}
