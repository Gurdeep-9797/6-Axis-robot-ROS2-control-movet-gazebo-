using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Windows.Media.Media3D;
using System.Xml.Linq;
using TeachPendant_WPF.SceneGraph;

namespace TeachPendant_WPF.Services
{
    public static class URDFParser
    {
        public static RobotNode? ParseLogicalRobot(string path)
        {
            try
            {
                if (!File.Exists(path)) return null;

                var doc = XDocument.Load(path);
                var robotEl = doc.Element("robot");
                if (robotEl == null) return null;

                var robot = new RobotNode { Name = robotEl.Attribute("name")?.Value ?? "LoadedRobot" };
                var joints = robotEl.Elements("joint").Where(j => j.Attribute("type")?.Value == "revolute" || j.Attribute("type")?.Value == "continuous").ToList();

                var childToJoint = new Dictionary<string, XElement>();
                foreach (var j in robotEl.Elements("joint")) // Track hierarchy for all joints
                {
                    var child = j.Element("child")?.Attribute("link")?.Value;
                    if (child != null)
                    {
                        childToJoint[child] = j;
                    }
                }

                // Find root-most link
                var allLinks = robotEl.Elements("link").Select(l => l.Attribute("name")?.Value).Where(n => n != null).ToList();
                string currentLinkName = allLinks.FirstOrDefault(l => !childToJoint.ContainsKey(l!)) ?? allLinks.FirstOrDefault() ?? "base_link";

                // Traverse the chain
                while (true)
                {
                    // Find the next moving joint
                    var nextJoint = joints.FirstOrDefault(j => j.Element("parent")?.Attribute("link")?.Value == currentLinkName);
                    
                    if (nextJoint == null)
                    {
                        // Maybe this is a fixed joint on the path to the next revolute joint
                        var fixedJoint = robotEl.Elements("joint").FirstOrDefault(j => j.Element("parent")?.Attribute("link")?.Value == currentLinkName);
                        if (fixedJoint == null) break;
                        currentLinkName = fixedJoint.Element("child")?.Attribute("link")?.Value!;
                        continue;
                    }

                    string jointName = nextJoint.Attribute("name")?.Value ?? "UnnamedJoint";
                    
                    var axisStr = nextJoint.Element("axis")?.Attribute("xyz")?.Value ?? "0 0 1";
                    var limitEl = nextJoint.Element("limit");
                    double minLimit = limitEl != null ? ParseDouble(limitEl.Attribute("lower")?.Value) * 180 / Math.PI : -180.0;
                    double maxLimit = limitEl != null ? ParseDouble(limitEl.Attribute("upper")?.Value) * 180 / Math.PI : 180.0;

                    var originEl = nextJoint.Element("origin");
                    Vector3D offset = ParseVector(originEl?.Attribute("xyz")?.Value);
                    
                    var jointNode = new JointNode
                    {
                        Name = jointName,
                        Axis = ParseVector(axisStr),
                        MinLimit = minLimit,
                        MaxLimit = maxLimit,
                        OriginOffset = offset
                    };

                    robot.AddJoint(jointNode);

                    currentLinkName = nextJoint.Element("child")?.Attribute("link")?.Value!;
                    if (currentLinkName == null) break;
                }

                var tool = new FrameNode { Name = "Tool0", FrameKind = FrameNode.FrameType.ToolTCP };
                robot.SetToolFrame(tool);

                return robot;
            }
            catch (Exception)
            {
                return null;
            }
        }

        private static Vector3D ParseVector(string? vecStr)
        {
            if (string.IsNullOrWhiteSpace(vecStr)) return new Vector3D(0, 0, 0);
            var parts = vecStr.Split(new[] { ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries);
            if (parts.Length >= 3 && double.TryParse(parts[0], out double x) && double.TryParse(parts[1], out double y) && double.TryParse(parts[2], out double z))
            {
                return new Vector3D(x, y, z);
            }
            return new Vector3D(0, 0, 0);
        }

        private static double ParseDouble(string? val)
        {
            return double.TryParse(val, out double d) ? d : 0.0;
        }
    }
}
