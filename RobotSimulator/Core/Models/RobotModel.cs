using System;
using System.Collections.Generic;
using System.Xml.Linq;

namespace RobotSimulator.Core.Models
{
    /// <summary>
    /// Represents a joint in a robot arm.
    /// Based on URDF joint specification.
    /// </summary>
    public class Joint
    {
        public string Name { get; set; } = "";
        public JointType Type { get; set; } = JointType.Revolute;
        public string ParentLink { get; set; } = "";
        public string ChildLink { get; set; } = "";
        
        // Origin transform
        public double OriginX { get; set; }
        public double OriginY { get; set; }
        public double OriginZ { get; set; }
        public double OriginRoll { get; set; }
        public double OriginPitch { get; set; }
        public double OriginYaw { get; set; }
        
        // Axis of rotation/translation
        public double AxisX { get; set; }
        public double AxisY { get; set; }
        public double AxisZ { get; set; } = 1.0;
        
        // Limits
        public double LowerLimit { get; set; } = -Math.PI;
        public double UpperLimit { get; set; } = Math.PI;
        public double VelocityLimit { get; set; } = 1.0;
        public double EffortLimit { get; set; } = 100.0;
        
        // Current state
        public double Position { get; set; }
        public double Velocity { get; set; }
        
        public double PositionDegrees
        {
            get => Position * 180.0 / Math.PI;
            set => Position = value * Math.PI / 180.0;
        }
    }

    public enum JointType
    {
        Fixed,
        Revolute,
        Continuous,
        Prismatic,
        Floating,
        Planar
    }

    /// <summary>
    /// Represents a link (rigid body) in a robot arm.
    /// </summary>
    public class Link
    {
        public string Name { get; set; } = "";
        
        // Visual geometry
        public GeometryType VisualType { get; set; } = GeometryType.Box;
        public double VisualSizeX { get; set; } = 0.1;
        public double VisualSizeY { get; set; } = 0.1;
        public double VisualSizeZ { get; set; } = 0.1;
        public double VisualRadius { get; set; } = 0.05;
        public double VisualLength { get; set; } = 0.1;
        public string? MeshFile { get; set; }
        
        // Visual origin
        public double VisualOriginX { get; set; }
        public double VisualOriginY { get; set; }
        public double VisualOriginZ { get; set; }
        public double VisualOriginRoll { get; set; }
        public double VisualOriginPitch { get; set; }
        public double VisualOriginYaw { get; set; }
        
        // Color
        public double ColorR { get; set; } = 0.8;
        public double ColorG { get; set; } = 0.5;
        public double ColorB { get; set; } = 0.2;
        public double ColorA { get; set; } = 1.0;
        
        // Mass properties
        public double Mass { get; set; } = 1.0;
    }

    public enum GeometryType
    {
        Box,
        Cylinder,
        Sphere,
        Mesh
    }

    /// <summary>
    /// Complete robot model parsed from URDF.
    /// </summary>
    public class RobotModel
    {
        public string Name { get; set; } = "Robot";
        public List<Link> Links { get; } = new();
        public List<Joint> Joints { get; } = new();
        public string BaseLink { get; set; } = "base_link";

        /// <summary>
        /// Get all actuated (non-fixed) joints in order.
        /// </summary>
        public List<Joint> GetActuatedJoints()
        {
            return Joints.FindAll(j => j.Type != JointType.Fixed);
        }

        /// <summary>
        /// Set joint positions from array.
        /// </summary>
        public void SetJointPositions(double[] positions)
        {
            var actuated = GetActuatedJoints();
            for (int i = 0; i < Math.Min(positions.Length, actuated.Count); i++)
            {
                actuated[i].Position = Math.Clamp(positions[i], 
                    actuated[i].LowerLimit, actuated[i].UpperLimit);
            }
        }

        /// <summary>
        /// Get current joint positions as array.
        /// </summary>
        public double[] GetJointPositions()
        {
            var actuated = GetActuatedJoints();
            var positions = new double[actuated.Count];
            for (int i = 0; i < actuated.Count; i++)
            {
                positions[i] = actuated[i].Position;
            }
            return positions;
        }
    }
}
