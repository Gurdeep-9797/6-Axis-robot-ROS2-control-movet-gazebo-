using System;
using System.Collections.Generic;
using System.Xml.Linq;
using RobotSimulator.Core.Models;

namespace RobotSimulator.Core.Urdf
{
    /// <summary>
    /// Parser for URDF (Unified Robot Description Format) files.
    /// Converts XML robot description to RobotModel.
    /// </summary>
    public static class UrdfParser
    {
        public static RobotModel Parse(string urdfXml)
        {
            var doc = XDocument.Parse(urdfXml);
            var robot = new RobotModel();

            var robotElement = doc.Element("robot");
            if (robotElement == null)
                throw new ArgumentException("Invalid URDF: no robot element");

            robot.Name = robotElement.Attribute("name")?.Value ?? "Robot";

            // Parse links
            foreach (var linkElement in robotElement.Elements("link"))
            {
                robot.Links.Add(ParseLink(linkElement));
            }

            // Parse joints
            foreach (var jointElement in robotElement.Elements("joint"))
            {
                robot.Joints.Add(ParseJoint(jointElement));
            }

            // Find base link (link with no parent joint)
            var childLinks = new HashSet<string>();
            foreach (var j in robot.Joints)
            {
                childLinks.Add(j.ChildLink);
            }
            foreach (var l in robot.Links)
            {
                if (!childLinks.Contains(l.Name))
                {
                    robot.BaseLink = l.Name;
                    break;
                }
            }

            return robot;
        }

        private static Link ParseLink(XElement element)
        {
            var link = new Link
            {
                Name = element.Attribute("name")?.Value ?? ""
            };

            var visual = element.Element("visual");
            if (visual != null)
            {
                // Origin
                var origin = visual.Element("origin");
                if (origin != null)
                {
                    ParseOrigin(origin, out double x, out double y, out double z,
                        out double roll, out double pitch, out double yaw);
                    link.VisualOriginX = x;
                    link.VisualOriginY = y;
                    link.VisualOriginZ = z;
                    link.VisualOriginRoll = roll;
                    link.VisualOriginPitch = pitch;
                    link.VisualOriginYaw = yaw;
                }

                // Geometry
                var geometry = visual.Element("geometry");
                if (geometry != null)
                {
                    var box = geometry.Element("box");
                    var cylinder = geometry.Element("cylinder");
                    var sphere = geometry.Element("sphere");
                    var mesh = geometry.Element("mesh");

                    if (box != null)
                    {
                        link.VisualType = GeometryType.Box;
                        var size = box.Attribute("size")?.Value ?? "0.1 0.1 0.1";
                        var parts = size.Split(' ');
                        if (parts.Length >= 3)
                        {
                            link.VisualSizeX = ParseDouble(parts[0]);
                            link.VisualSizeY = ParseDouble(parts[1]);
                            link.VisualSizeZ = ParseDouble(parts[2]);
                        }
                    }
                    else if (cylinder != null)
                    {
                        link.VisualType = GeometryType.Cylinder;
                        link.VisualRadius = ParseDouble(cylinder.Attribute("radius")?.Value);
                        link.VisualLength = ParseDouble(cylinder.Attribute("length")?.Value);
                    }
                    else if (sphere != null)
                    {
                        link.VisualType = GeometryType.Sphere;
                        link.VisualRadius = ParseDouble(sphere.Attribute("radius")?.Value);
                    }
                    else if (mesh != null)
                    {
                        link.VisualType = GeometryType.Mesh;
                        link.MeshFile = mesh.Attribute("filename")?.Value;
                    }
                }

                // Material/Color
                var material = visual.Element("material");
                if (material != null)
                {
                    var color = material.Element("color");
                    if (color != null)
                    {
                        var rgba = color.Attribute("rgba")?.Value ?? "0.8 0.5 0.2 1.0";
                        var parts = rgba.Split(' ');
                        if (parts.Length >= 4)
                        {
                            link.ColorR = ParseDouble(parts[0]);
                            link.ColorG = ParseDouble(parts[1]);
                            link.ColorB = ParseDouble(parts[2]);
                            link.ColorA = ParseDouble(parts[3]);
                        }
                    }
                }
            }

            return link;
        }

        private static Joint ParseJoint(XElement element)
        {
            var joint = new Joint
            {
                Name = element.Attribute("name")?.Value ?? ""
            };

            // Type
            var typeStr = element.Attribute("type")?.Value ?? "fixed";
            joint.Type = typeStr switch
            {
                "revolute" => JointType.Revolute,
                "continuous" => JointType.Continuous,
                "prismatic" => JointType.Prismatic,
                "floating" => JointType.Floating,
                "planar" => JointType.Planar,
                _ => JointType.Fixed
            };

            // Parent/Child
            joint.ParentLink = element.Element("parent")?.Attribute("link")?.Value ?? "";
            joint.ChildLink = element.Element("child")?.Attribute("link")?.Value ?? "";

            // Origin
            var origin = element.Element("origin");
            if (origin != null)
            {
                ParseOrigin(origin, out double x, out double y, out double z,
                    out double roll, out double pitch, out double yaw);
                joint.OriginX = x;
                joint.OriginY = y;
                joint.OriginZ = z;
                joint.OriginRoll = roll;
                joint.OriginPitch = pitch;
                joint.OriginYaw = yaw;
            }

            // Axis
            var axis = element.Element("axis");
            if (axis != null)
            {
                var xyz = axis.Attribute("xyz")?.Value ?? "0 0 1";
                var parts = xyz.Split(' ');
                if (parts.Length >= 3)
                {
                    joint.AxisX = ParseDouble(parts[0]);
                    joint.AxisY = ParseDouble(parts[1]);
                    joint.AxisZ = ParseDouble(parts[2]);
                }
            }

            // Limits
            var limit = element.Element("limit");
            if (limit != null)
            {
                joint.LowerLimit = ParseDouble(limit.Attribute("lower")?.Value);
                joint.UpperLimit = ParseDouble(limit.Attribute("upper")?.Value);
                joint.VelocityLimit = ParseDouble(limit.Attribute("velocity")?.Value, 1.0);
                joint.EffortLimit = ParseDouble(limit.Attribute("effort")?.Value, 100.0);
            }

            return joint;
        }

        private static void ParseOrigin(XElement origin,
            out double x, out double y, out double z,
            out double roll, out double pitch, out double yaw)
        {
            x = y = z = roll = pitch = yaw = 0;

            var xyz = origin.Attribute("xyz")?.Value;
            if (xyz != null)
            {
                var parts = xyz.Split(' ');
                if (parts.Length >= 3)
                {
                    x = ParseDouble(parts[0]);
                    y = ParseDouble(parts[1]);
                    z = ParseDouble(parts[2]);
                }
            }

            var rpy = origin.Attribute("rpy")?.Value;
            if (rpy != null)
            {
                var parts = rpy.Split(' ');
                if (parts.Length >= 3)
                {
                    roll = ParseDouble(parts[0]);
                    pitch = ParseDouble(parts[1]);
                    yaw = ParseDouble(parts[2]);
                }
            }
        }

        private static double ParseDouble(string? value, double defaultValue = 0.0)
        {
            if (string.IsNullOrEmpty(value)) return defaultValue;
            return double.TryParse(value, out double result) ? result : defaultValue;
        }

        /// <summary>
        /// Built-in industrial robot URDF for fallback.
        /// </summary>
        public static string GetDefaultUrdf()
        {
            return @"<?xml version=""1.0""?>
<robot name=""industrial_arm"">
  <material name=""orange""><color rgba=""1.0 0.45 0.0 1.0""/></material>
  <material name=""grey""><color rgba=""0.4 0.4 0.4 1.0""/></material>
  
  <link name=""base_link"">
    <visual>
      <geometry><cylinder radius=""0.12"" length=""0.15""/></geometry>
      <origin xyz=""0 0 0.075""/>
      <material name=""orange""/>
    </visual>
  </link>

  <link name=""link_1"">
    <visual>
      <geometry><cylinder radius=""0.1"" length=""0.25""/></geometry>
      <origin xyz=""0 0 0.125""/>
      <material name=""orange""/>
    </visual>
  </link>
  <joint name=""joint_1"" type=""revolute"">
    <parent link=""base_link""/><child link=""link_1""/>
    <origin xyz=""0 0 0.15""/>
    <axis xyz=""0 0 1""/>
    <limit lower=""-2.97"" upper=""2.97"" effort=""150"" velocity=""4.36""/>
  </joint>

  <link name=""link_2"">
    <visual>
      <geometry><box size=""0.08 0.1 0.3""/></geometry>
      <origin xyz=""0 0 0.15""/>
      <material name=""orange""/>
    </visual>
  </link>
  <joint name=""joint_2"" type=""revolute"">
    <parent link=""link_1""/><child link=""link_2""/>
    <origin xyz=""0 0 0.25""/>
    <axis xyz=""0 1 0""/>
    <limit lower=""-1.92"" upper=""1.92"" effort=""150"" velocity=""4.36""/>
  </joint>

  <link name=""link_3"">
    <visual>
      <geometry><cylinder radius=""0.05"" length=""0.25""/></geometry>
      <origin xyz=""0 0 0.125""/>
      <material name=""orange""/>
    </visual>
  </link>
  <joint name=""joint_3"" type=""revolute"">
    <parent link=""link_2""/><child link=""link_3""/>
    <origin xyz=""0 0 0.3""/>
    <axis xyz=""0 1 0""/>
    <limit lower=""-1.22"" upper=""1.92"" effort=""100"" velocity=""4.36""/>
  </joint>

  <link name=""link_4"">
    <visual>
      <geometry><cylinder radius=""0.04"" length=""0.1""/></geometry>
      <origin xyz=""0 0 0.05""/>
      <material name=""grey""/>
    </visual>
  </link>
  <joint name=""joint_4"" type=""revolute"">
    <parent link=""link_3""/><child link=""link_4""/>
    <origin xyz=""0 0 0.25""/>
    <axis xyz=""0 0 1""/>
    <limit lower=""-2.79"" upper=""2.79"" effort=""50"" velocity=""5.58""/>
  </joint>

  <link name=""link_5"">
    <visual>
      <geometry><cylinder radius=""0.035"" length=""0.08""/></geometry>
      <origin xyz=""0 0 0.04""/>
      <material name=""grey""/>
    </visual>
  </link>
  <joint name=""joint_5"" type=""revolute"">
    <parent link=""link_4""/><child link=""link_5""/>
    <origin xyz=""0 0 0.1""/>
    <axis xyz=""0 1 0""/>
    <limit lower=""-2.09"" upper=""2.09"" effort=""50"" velocity=""5.58""/>
  </joint>

  <link name=""link_6"">
    <visual>
      <geometry><cylinder radius=""0.02"" length=""0.03""/></geometry>
      <origin xyz=""0 0 0.015""/>
      <material name=""grey""/>
    </visual>
  </link>
  <joint name=""joint_6"" type=""revolute"">
    <parent link=""link_5""/><child link=""link_6""/>
    <origin xyz=""0 0 0.08""/>
    <axis xyz=""0 0 1""/>
    <limit lower=""-6.98"" upper=""6.98"" effort=""20"" velocity=""7.33""/>
  </joint>
</robot>";
        }
    }
}
