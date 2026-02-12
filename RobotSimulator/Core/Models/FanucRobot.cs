using System;
using System.Collections.Generic;
using System.IO;
using System.Text.Json;
using System.Windows.Media.Media3D;

namespace RobotSimulator.Core.Models
{
    /// <summary>
    /// FANUC ARC Mate 120iC welding robot specification.
    /// Industrial 6-axis robot with integrated wire feeder and welding torch.
    /// </summary>
    public class FanucArcMate120iC
    {
        // Robot specifications based on FANUC official data
        public const string ModelName = "ARC Mate 120iC";
        public const double Reach = 1.811;           // meters
        public const double Payload = 20.0;          // kg
        public const double Repeatability = 0.00003; // meters (±0.03mm)
        public const double RobotWeight = 250.0;     // kg
        
        // Joint limits (degrees)
        public static readonly (double Min, double Max)[] JointLimits = new[]
        {
            (-170.0, 170.0),   // J1
            (-125.0, 135.0),   // J2 (asymmetric for welding)
            (-145.0, 220.0),   // J3
            (-190.0, 190.0),   // J4
            (-140.0, 140.0),   // J5
            (-360.0, 360.0)    // J6 (full rotation)
        };
        
        // Joint max speeds (degrees/sec)
        public static readonly double[] JointMaxSpeeds = new[]
        {
            260.0,  // J1
            195.0,  // J2
            200.0,  // J3
            280.0,  // J4
            315.0,  // J5
            550.0   // J6
        };
        
        // DH Parameters for FANUC ARC Mate 120iC (approximated)
        // Format: (a, alpha, d, theta_offset) in meters and radians
        public static readonly (double a, double alpha, double d, double offset)[] DhParams = 
            new (double, double, double, double)[]
        {
            (0.150,  Math.PI / 2,  0.525, 0.0),         // J1
            (0.790,  0.0,          0.0,   Math.PI / 2), // J2
            (0.150,  Math.PI / 2,  0.0,   0.0),         // J3
            (0.0,    -Math.PI / 2, 0.860, 0.0),         // J4
            (0.0,    Math.PI / 2,  0.0,   0.0),         // J5
            (0.0,    0.0,          0.100, 0.0)          // J6 (tool flange)
        };
        
        // Link lengths for visualization (meters)
        public static readonly double[] LinkLengths = new[]
        {
            0.525,  // Base height
            0.790,  // Upper arm
            0.150,  // Elbow offset
            0.860,  // Forearm
            0.100,  // Wrist 1
            0.080,  // Wrist 2
            0.050   // Tool flange
        };
        
        // Link radii for visualization (meters)
        public static readonly double[] LinkRadii = new[]
        {
            0.150,  // Base
            0.120,  // Shoulder
            0.100,  // Upper arm
            0.080,  // Elbow
            0.060,  // Forearm
            0.050,  // Wrist
            0.030   // Tool
        };
        
        // Welding torch offset from tool flange (meters)
        public const double TorchLength = 0.200;
        public const double TorchAngle = 35.0 * Math.PI / 180.0; // typical torch angle
        
        // Wire feeder mount position (behind J4)
        public static readonly Point3D WireFeederOffset = new Point3D(-0.15, 0, -0.20);
    }

    /// <summary>
    /// Complete robot program containing teach points and global settings.
    /// </summary>
    public class RobotProgram
    {
        public string Name { get; set; } = "MAIN";
        public string Comment { get; set; } = "";
        public DateTime CreatedAt { get; set; } = DateTime.Now;
        public DateTime ModifiedAt { get; set; } = DateTime.Now;
        
        public List<TeachPoint> Points { get; } = new();
        
        /// <summary>Default welding parameters for new weld sections</summary>
        public WeldingParameters DefaultWeldParams { get; set; } = new();
        
        /// <summary>Tool center point offset (meters)</summary>
        public Point3D TcpOffset { get; set; } = new Point3D(0, 0, 0.20);
        
        /// <summary>User frame offset (meters)</summary>
        public Point3D UserFrameOffset { get; set; } = new Point3D(0, 0, 0);
        
        /// <summary>Add a new teach point</summary>
        public TeachPoint AddPoint(double[] jointAngles, Point3D cartesianPos, 
            double roll, double pitch, double yaw)
        {
            var point = new TeachPoint
            {
                Id = Points.Count + 1,
                Name = $"P{Points.Count + 1}",
                JointAngles = (double[])jointAngles.Clone(),
                CartesianPosition = cartesianPos,
                Roll = roll,
                Pitch = pitch,
                Yaw = yaw
            };
            Points.Add(point);
            ModifiedAt = DateTime.Now;
            return point;
        }
        
        /// <summary>Remove a point and renumber remaining</summary>
        public void RemovePoint(int index)
        {
            if (index >= 0 && index < Points.Count)
            {
                Points.RemoveAt(index);
                // Renumber
                for (int i = 0; i < Points.Count; i++)
                {
                    Points[i].Id = i + 1;
                    if (Points[i].Name.StartsWith("P"))
                        Points[i].Name = $"P{i + 1}";
                }
                ModifiedAt = DateTime.Now;
            }
        }
        
        /// <summary>Move point up in sequence</summary>
        public void MovePointUp(int index)
        {
            if (index > 0 && index < Points.Count)
            {
                (Points[index - 1], Points[index]) = (Points[index], Points[index - 1]);
                RenumberPoints();
            }
        }
        
        /// <summary>Move point down in sequence</summary>
        public void MovePointDown(int index)
        {
            if (index >= 0 && index < Points.Count - 1)
            {
                (Points[index], Points[index + 1]) = (Points[index + 1], Points[index]);
                RenumberPoints();
            }
        }
        
        private void RenumberPoints()
        {
            for (int i = 0; i < Points.Count; i++)
            {
                Points[i].Id = i + 1;
            }
            ModifiedAt = DateTime.Now;
        }
        
        /// <summary>Clear all points</summary>
        public void Clear()
        {
            Points.Clear();
            ModifiedAt = DateTime.Now;
        }
        
        /// <summary>Save program to JSON file</summary>
        public void SaveToFile(string path)
        {
            var options = new JsonSerializerOptions { WriteIndented = true };
            var json = JsonSerializer.Serialize(this, options);
            File.WriteAllText(path, json);
        }
        
        /// <summary>Load program from JSON file</summary>
        public static RobotProgram LoadFromFile(string path)
        {
            var json = File.ReadAllText(path);
            return JsonSerializer.Deserialize<RobotProgram>(json) ?? new RobotProgram();
        }
        
        /// <summary>Calculate estimated cycle time in seconds</summary>
        public double EstimateCycleTime()
        {
            double totalTime = 0;
            for (int i = 1; i < Points.Count; i++)
            {
                var prev = Points[i - 1];
                var curr = Points[i];
                
                // Estimate distance
                var dx = curr.CartesianPosition.X - prev.CartesianPosition.X;
                var dy = curr.CartesianPosition.Y - prev.CartesianPosition.Y;
                var dz = curr.CartesianPosition.Z - prev.CartesianPosition.Z;
                var distance = Math.Sqrt(dx * dx + dy * dy + dz * dz) * 1000; // mm
                
                // Time based on motion type
                double speed = curr.Motion == MotionType.Joint 
                    ? 500 * (curr.Speed / 100.0)  // Approx mm/s for joint
                    : curr.Speed;                  // Direct mm/s for linear
                
                if (speed > 0)
                    totalTime += distance / speed;
            }
            return totalTime;
        }
    }
}
