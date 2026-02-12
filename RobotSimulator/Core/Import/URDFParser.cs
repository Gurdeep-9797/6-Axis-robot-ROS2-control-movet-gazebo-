using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Xml.Linq;
using RobotSimulator.Core.Models;

namespace RobotSimulator.Core.Import
{
    /// <summary>
    /// Parses URDF/XACRO files to extract robot dimensions for kinematics.
    /// </summary>
    public class URDFParser
    {
        private Dictionary<string, double> _properties = new();
        
        /// <summary>
        /// Load robot model from URDF file.
        /// </summary>
        public RobotModel LoadURDF(string filePath)
        {
            var doc = XDocument.Load(filePath);
            var robot = new RobotModel();
            
            var root = doc.Root;
            if (root == null || root.Name.LocalName != "robot")
                throw new InvalidDataException("Invalid URDF: root must be <robot>");
            
            robot.Name = root.Attribute("name")?.Value ?? "Robot";
            
            // Parse xacro properties if present
            foreach (var prop in root.Descendants())
            {
                if (prop.Name.LocalName == "property")
                {
                    var name = prop.Attribute("name")?.Value;
                    var value = prop.Attribute("value")?.Value;
                    if (name != null && value != null)
                    {
                        if (double.TryParse(value, NumberStyles.Float, CultureInfo.InvariantCulture, out var d))
                            _properties[name] = d;
                    }
                }
            }
            
            // Parse links
            foreach (var linkElement in root.Elements())
            {
                if (linkElement.Name.LocalName == "link")
                    robot.Links.Add(ParseLink(linkElement));
            }
            
            // Parse joints
            foreach (var jointElement in root.Elements())
            {
                if (jointElement.Name.LocalName == "joint")
                    robot.Joints.Add(ParseJoint(jointElement));
            }
            
            return robot;
        }

        private Link ParseLink(XElement element)
        {
            var link = new Link
            {
                Name = element.Attribute("name")?.Value ?? ""
            };
            
            var visual = element.Element("visual");
            if (visual != null)
            {
                var geometry = visual.Element("geometry");
                if (geometry != null)
                {
                    var box = geometry.Element("box");
                    var cyl = geometry.Element("cylinder");
                    var sphere = geometry.Element("sphere");
                    var mesh = geometry.Element("mesh");
                    
                    if (box != null)
                    {
                        link.VisualType = GeometryType.Box;
                        var size = box.Attribute("size")?.Value?.Split(' ');
                        if (size?.Length >= 3)
                        {
                            link.VisualSizeX = ParseValue(size[0]);
                            link.VisualSizeY = ParseValue(size[1]);
                            link.VisualSizeZ = ParseValue(size[2]);
                        }
                    }
                    else if (cyl != null)
                    {
                        link.VisualType = GeometryType.Cylinder;
                        link.VisualRadius = ParseValue(cyl.Attribute("radius")?.Value ?? "0.05");
                        link.VisualLength = ParseValue(cyl.Attribute("length")?.Value ?? "0.1");
                    }
                    else if (sphere != null)
                    {
                        link.VisualType = GeometryType.Sphere;
                        link.VisualRadius = ParseValue(sphere.Attribute("radius")?.Value ?? "0.05");
                    }
                    else if (mesh != null)
                    {
                        link.VisualType = GeometryType.Mesh;
                        link.MeshFile = mesh.Attribute("filename")?.Value;
                    }
                }
                
                var origin = visual.Element("origin");
                if (origin != null)
                {
                    var (ox, oy, oz, oroll, opitch, oyaw) = ParseOrigin(origin);
                    link.VisualOriginX = ox;
                    link.VisualOriginY = oy;
                    link.VisualOriginZ = oz;
                    link.VisualOriginRoll = oroll;
                    link.VisualOriginPitch = opitch;
                    link.VisualOriginYaw = oyaw;
                }
                
                var material = visual.Element("material");
                if (material != null)
                {
                    var color = material.Element("color");
                    if (color != null)
                    {
                        var rgba = color.Attribute("rgba")?.Value?.Split(' ');
                        if (rgba?.Length >= 4)
                        {
                            link.ColorR = ParseValue(rgba[0]);
                            link.ColorG = ParseValue(rgba[1]);
                            link.ColorB = ParseValue(rgba[2]);
                            link.ColorA = ParseValue(rgba[3]);
                        }
                    }
                }
            }
            
            var inertial = element.Element("inertial");
            if (inertial != null)
            {
                var mass = inertial.Element("mass");
                if (mass != null)
                    link.Mass = ParseValue(mass.Attribute("value")?.Value ?? "1.0");
            }
            
            return link;
        }

        private Joint ParseJoint(XElement element)
        {
            var joint = new Joint
            {
                Name = element.Attribute("name")?.Value ?? "",
                Type = ParseJointType(element.Attribute("type")?.Value ?? "fixed")
            };
            
            var parent = element.Element("parent");
            if (parent != null)
                joint.ParentLink = parent.Attribute("link")?.Value ?? "";
            
            var child = element.Element("child");
            if (child != null)
                joint.ChildLink = child.Attribute("link")?.Value ?? "";
            
            var origin = element.Element("origin");
            if (origin != null)
            {
                var (jx, jy, jz, jroll, jpitch, jyaw) = ParseOrigin(origin);
                joint.OriginX = jx;
                joint.OriginY = jy;
                joint.OriginZ = jz;
                joint.OriginRoll = jroll;
                joint.OriginPitch = jpitch;
                joint.OriginYaw = jyaw;
            }
            
            var axis = element.Element("axis");
            if (axis != null)
            {
                var xyz = axis.Attribute("xyz")?.Value?.Split(' ');
                if (xyz?.Length >= 3)
                {
                    joint.AxisX = ParseValue(xyz[0]);
                    joint.AxisY = ParseValue(xyz[1]);
                    joint.AxisZ = ParseValue(xyz[2]);
                }
            }
            
            var limit = element.Element("limit");
            if (limit != null)
            {
                joint.LowerLimit = ParseValue(limit.Attribute("lower")?.Value ?? "-3.14159");
                joint.UpperLimit = ParseValue(limit.Attribute("upper")?.Value ?? "3.14159");
                joint.VelocityLimit = ParseValue(limit.Attribute("velocity")?.Value ?? "1.0");
                joint.EffortLimit = ParseValue(limit.Attribute("effort")?.Value ?? "100.0");
            }
            
            return joint;
        }

        private (double x, double y, double z, double roll, double pitch, double yaw) ParseOrigin(XElement origin)
        {
            double x = 0, y = 0, z = 0, roll = 0, pitch = 0, yaw = 0;
            
            var xyz = origin.Attribute("xyz")?.Value?.Split(' ');
            if (xyz?.Length >= 3)
            {
                x = ParseValue(xyz[0]);
                y = ParseValue(xyz[1]);
                z = ParseValue(xyz[2]);
            }
            
            var rpy = origin.Attribute("rpy")?.Value?.Split(' ');
            if (rpy?.Length >= 3)
            {
                roll = ParseValue(rpy[0]);
                pitch = ParseValue(rpy[1]);
                yaw = ParseValue(rpy[2]);
            }
            
            return (x, y, z, roll, pitch, yaw);
        }

        private JointType ParseJointType(string type) => type.ToLower() switch
        {
            "revolute" => JointType.Revolute,
            "continuous" => JointType.Continuous,
            "prismatic" => JointType.Prismatic,
            "floating" => JointType.Floating,
            "planar" => JointType.Planar,
            _ => JointType.Fixed
        };

        private double ParseValue(string value)
        {
            if (string.IsNullOrWhiteSpace(value))
                return 0;
            
            value = value.Trim();
            
            // Handle xacro expressions like ${-165*DEG2RAD}
            if (value.StartsWith("${") && value.EndsWith("}"))
            {
                var expr = value.Substring(2, value.Length - 3);
                return EvaluateExpression(expr);
            }
            
            if (double.TryParse(value, NumberStyles.Float, CultureInfo.InvariantCulture, out var result))
                return result;
            
            return 0;
        }

        private double EvaluateExpression(string expr)
        {
            // Simple expression parser for xacro
            expr = expr.Trim();
            
            // Replace property references
            foreach (var prop in _properties)
                expr = expr.Replace(prop.Key, prop.Value.ToString(CultureInfo.InvariantCulture));
            
            // Handle negation
            if (expr.StartsWith("-"))
                return -EvaluateExpression(expr.Substring(1));
            
            // Handle multiplication
            var mulIndex = expr.IndexOf('*');
            if (mulIndex > 0)
            {
                var left = EvaluateExpression(expr.Substring(0, mulIndex));
                var right = EvaluateExpression(expr.Substring(mulIndex + 1));
                return left * right;
            }
            
            // Handle division
            var divIndex = expr.IndexOf('/');
            if (divIndex > 0)
            {
                var left = EvaluateExpression(expr.Substring(0, divIndex));
                var right = EvaluateExpression(expr.Substring(divIndex + 1));
                return right != 0 ? left / right : 0;
            }
            
            // Handle addition
            var addIndex = expr.LastIndexOf('+');
            if (addIndex > 0)
            {
                var left = EvaluateExpression(expr.Substring(0, addIndex));
                var right = EvaluateExpression(expr.Substring(addIndex + 1));
                return left + right;
            }
            
            // Try parse as number
            if (double.TryParse(expr, NumberStyles.Float, CultureInfo.InvariantCulture, out var val))
                return val;
            
            return 0;
        }

        /// <summary>
        /// Create default ABB IRB 120 model without URDF file.
        /// Uses exact dimensions from datasheet.
        /// </summary>
        public static RobotModel CreateABBIRB120()
        {
            var robot = new RobotModel { Name = "ABB IRB 120" };
            
            // Base link
            robot.Links.Add(new Link { Name = "base_link", VisualType = GeometryType.Box, 
                VisualSizeX = 0.2, VisualSizeY = 0.2, VisualSizeZ = 0.2,
                ColorR = 1.0, ColorG = 0.5, ColorB = 0.0 });
            
            // Link 1 - Base rotation column (290mm)
            robot.Links.Add(new Link { Name = "link_1", VisualType = GeometryType.Cylinder,
                VisualRadius = 0.07, VisualLength = 0.29, VisualOriginZ = 0.145,
                ColorR = 1.0, ColorG = 0.5, ColorB = 0.0 });
            
            // Link 2 - Upper arm (270mm)
            robot.Links.Add(new Link { Name = "link_2", VisualType = GeometryType.Box,
                VisualSizeX = 0.05, VisualSizeY = 0.05, VisualSizeZ = 0.27, VisualOriginZ = 0.135,
                ColorR = 1.0, ColorG = 0.5, ColorB = 0.0 });
            
            // Link 3 - Forearm (302mm)
            robot.Links.Add(new Link { Name = "link_3", VisualType = GeometryType.Box,
                VisualSizeX = 0.05, VisualSizeY = 0.05, VisualSizeZ = 0.302, VisualOriginZ = 0.151,
                ColorR = 1.0, ColorG = 0.5, ColorB = 0.0 });
            
            // Link 4 - Wrist 1 (72mm)
            robot.Links.Add(new Link { Name = "link_4", VisualType = GeometryType.Cylinder,
                VisualRadius = 0.035, VisualLength = 0.072, VisualOriginX = 0.036,
                VisualOriginPitch = Math.PI / 2,
                ColorR = 0.9, ColorG = 0.9, ColorB = 0.9 });
            
            // Link 5 - Wrist 2 (50mm)
            robot.Links.Add(new Link { Name = "link_5", VisualType = GeometryType.Cylinder,
                VisualRadius = 0.03, VisualLength = 0.05, VisualOriginX = 0.025,
                VisualOriginPitch = Math.PI / 2,
                ColorR = 0.9, ColorG = 0.9, ColorB = 0.9 });
            
            // Link 6 - Flange (10mm)
            robot.Links.Add(new Link { Name = "link_6", VisualType = GeometryType.Cylinder,
                VisualRadius = 0.02, VisualLength = 0.01, VisualOriginX = 0.005,
                VisualOriginPitch = Math.PI / 2,
                ColorR = 0.5, ColorG = 0.5, ColorB = 0.5 });
            
            // Tool frame
            robot.Links.Add(new Link { Name = "tool0" });
            
            // Joints with exact ABB IRB 120 dimensions
            double DEG2RAD = Math.PI / 180.0;
            
            robot.Joints.Add(new Joint { 
                Name = "joint_1", Type = JointType.Revolute,
                ParentLink = "base_link", ChildLink = "link_1",
                OriginX = 0, OriginY = 0, OriginZ = 0,
                AxisX = 0, AxisY = 0, AxisZ = 1,
                LowerLimit = -165 * DEG2RAD, UpperLimit = 165 * DEG2RAD
            });
            
            robot.Joints.Add(new Joint { 
                Name = "joint_2", Type = JointType.Revolute,
                ParentLink = "link_1", ChildLink = "link_2",
                OriginX = 0, OriginY = 0, OriginZ = 0.290,
                AxisX = 0, AxisY = 1, AxisZ = 0,
                LowerLimit = -110 * DEG2RAD, UpperLimit = 110 * DEG2RAD
            });
            
            robot.Joints.Add(new Joint { 
                Name = "joint_3", Type = JointType.Revolute,
                ParentLink = "link_2", ChildLink = "link_3",
                OriginX = 0, OriginY = 0, OriginZ = 0.270,
                AxisX = 0, AxisY = 1, AxisZ = 0,
                LowerLimit = -110 * DEG2RAD, UpperLimit = 70 * DEG2RAD
            });
            
            robot.Joints.Add(new Joint { 
                Name = "joint_4", Type = JointType.Revolute,
                ParentLink = "link_3", ChildLink = "link_4",
                OriginX = 0, OriginY = 0, OriginZ = 0.302,
                AxisX = 1, AxisY = 0, AxisZ = 0,
                LowerLimit = -160 * DEG2RAD, UpperLimit = 160 * DEG2RAD
            });
            
            robot.Joints.Add(new Joint { 
                Name = "joint_5", Type = JointType.Revolute,
                ParentLink = "link_4", ChildLink = "link_5",
                OriginX = 0.072, OriginY = 0, OriginZ = 0,
                AxisX = 0, AxisY = 1, AxisZ = 0,
                LowerLimit = -120 * DEG2RAD, UpperLimit = 120 * DEG2RAD
            });
            
            robot.Joints.Add(new Joint { 
                Name = "joint_6", Type = JointType.Revolute,
                ParentLink = "link_5", ChildLink = "link_6",
                OriginX = 0.050, OriginY = 0, OriginZ = 0,
                AxisX = 1, AxisY = 0, AxisZ = 0,
                LowerLimit = -400 * DEG2RAD, UpperLimit = 400 * DEG2RAD
            });
            
            robot.Joints.Add(new Joint { 
                Name = "tool_joint", Type = JointType.Fixed,
                ParentLink = "link_6", ChildLink = "tool0",
                OriginX = 0.010, OriginY = 0, OriginZ = 0
            });
            
            return robot;
        }
    }
}
