using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using System.Xml.Linq;
using HelixToolkit.Wpf;

namespace TeachPendant_WPF.Views
{
    public class URDFVisualizer
    {
        public ModelVisual3D RootVisual { get; private set; }
        private Dictionary<int, AxisAngleRotation3D> _jointRotations;

        public URDFVisualizer(string urdfPath)
        {
            RootVisual = new ModelVisual3D();
            _jointRotations = new Dictionary<int, AxisAngleRotation3D>();

            if (File.Exists(urdfPath))
            {
                ParseSimpleUrdf(urdfPath);
            }
            else
            {
                var fallback = BuildGeometry(Brushes.Red, CreateBoxMesh(0.1, 0.1, 0.1));
                RootVisual.Children.Add(fallback);
            }
        }

        private void ParseSimpleUrdf(string path)
        {
            var doc = XDocument.Load(path);
            var robot = doc.Element("robot");
            if (robot == null) return;

            var links = robot.Elements("link").ToDictionary(l => l.Attribute("name").Value);
            var joints = robot.Elements("joint").ToList();

            string currentLinkName = "base_link";
            ModelVisual3D currentGroup = RootVisual;

            if (links.TryGetValue(currentLinkName, out var baseLink))
            {
                var visual = BuildLinkVisual(baseLink, Brushes.DarkSlateGray);
                if (visual != null) currentGroup.Children.Add(visual);
            }

            int jointIndex = 1;
            Brush[] brushes = { 
                Brushes.Goldenrod,     // Orange
                Brushes.LightGray,     // Light Gray
                Brushes.Goldenrod, 
                Brushes.LightGray, 
                Brushes.Goldenrod, 
                Brushes.DarkSlateGray  // Dark Gray
            };

            while (true)
            {
                var nextJoint = joints.FirstOrDefault(j => j.Element("parent").Attribute("link").Value == currentLinkName);
                if (nextJoint == null) break;

                string childLinkName = nextJoint.Element("child").Attribute("link").Value;
                
                var originEl = nextJoint.Element("origin");
                Vector3D origin = ParseVector(originEl?.Attribute("xyz")?.Value);
                Transform3D rpyTransform = ParseRpyToTransform(originEl?.Attribute("rpy")?.Value);

                Vector3D axis = ParseVector(nextJoint?.Element("axis")?.Attribute("xyz")?.Value ?? "0 0 1");

                var jointContainer = new ModelVisual3D();
                var transform = new Transform3DGroup();
                
                // 1. Dynamic joint rotation around joint axis
                var rotation = new AxisAngleRotation3D(axis, 0);
                _jointRotations[jointIndex] = rotation;
                transform.Children.Add(new RotateTransform3D(rotation));
                
                // 2. Fixed orientation of joint frame in parent
                transform.Children.Add(rpyTransform);
                
                // 3. Fixed translation of joint frame in parent
                transform.Children.Add(new TranslateTransform3D(origin));
                
                jointContainer.Transform = transform;

                if (links.TryGetValue(childLinkName, out var childLink))
                {
                    Brush brush = brushes[(jointIndex - 1) % brushes.Length];
                    var visual = BuildLinkVisual(childLink, brush);
                    if (visual != null) jointContainer.Children.Add(visual);
                }

                currentGroup.Children.Add(jointContainer);
                currentGroup = jointContainer;
                currentLinkName = childLinkName;
                jointIndex++;
            }
        }

        private ModelVisual3D? BuildLinkVisual(XElement linkEl, Brush brush)
        {
            var visualEl = linkEl.Element("visual");
            if (visualEl == null) return null;

            var originEl = visualEl.Element("origin");
            var origin = ParseVector(originEl?.Attribute("xyz")?.Value);
            var rpyTransform = ParseRpyToTransform(originEl?.Attribute("rpy")?.Value);
            
            var geom = visualEl.Element("geometry");

            if (geom.Element("cylinder") != null)
            {
                double radius = double.Parse(geom.Element("cylinder").Attribute("radius").Value, CultureInfo.InvariantCulture);
                double length = double.Parse(geom.Element("cylinder").Attribute("length").Value, CultureInfo.InvariantCulture);
                
                var visual = BuildGeometry(brush, CreateCylinderMesh(radius, length, 36));
                
                var tg = new Transform3DGroup();
                tg.Children.Add(rpyTransform);
                tg.Children.Add(new TranslateTransform3D(origin.X, origin.Y, origin.Z));
                visual.Transform = tg;
                
                return visual;
            }
            else if (geom.Element("box") != null)
            {
                var size = ParseVector(geom.Element("box").Attribute("size").Value);
                
                var visual = BuildGeometry(brush, CreateBoxMesh(size.X, size.Y, size.Z));
                
                var tg = new Transform3DGroup();
                tg.Children.Add(rpyTransform);
                tg.Children.Add(new TranslateTransform3D(origin.X, origin.Y, origin.Z));
                visual.Transform = tg;
                
                return visual;
            }

            return null;
        }

        private Transform3D ParseRpyToTransform(string? val)
        {
            var rpy = ParseVector(val);
            if (rpy.X == 0 && rpy.Y == 0 && rpy.Z == 0) return Transform3D.Identity;

            var tg = new Transform3DGroup();
            // In WPF, applying X then Y then Z to Children means row vector multiplying v * Rx * Ry * Rz
            tg.Children.Add(new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(1, 0, 0), rpy.X * 180.0 / Math.PI)));
            tg.Children.Add(new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(0, 1, 0), rpy.Y * 180.0 / Math.PI)));
            tg.Children.Add(new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(0, 0, 1), rpy.Z * 180.0 / Math.PI)));
            return tg;
        }
        
        private ModelVisual3D BuildGeometry(Brush brush, MeshGeometry3D mesh)
        {
            var material = MaterialHelper.CreateMaterial(brush.Clone());
            var geomModel = new GeometryModel3D(mesh, material);
            return new ModelVisual3D { Content = geomModel };
        }

        private MeshGeometry3D CreateBoxMesh(double x, double y, double z)
        {
            var mesh = new MeshGeometry3D();
            double dx = x / 2.0; double dy = y / 2.0; double dz = z / 2.0;

            Point3D[] p = {
                new Point3D(-dx, -dy,  dz), new Point3D( dx, -dy,  dz), new Point3D( dx,  dy,  dz), new Point3D(-dx,  dy,  dz),
                new Point3D(-dx, -dy, -dz), new Point3D( dx, -dy, -dz), new Point3D( dx,  dy, -dz), new Point3D(-dx,  dy, -dz)
            };

            int[] indices = {
                0,1,2, 0,2,3, // Front
                1,5,6, 1,6,2, // Right
                5,4,7, 5,7,6, // Back
                4,0,3, 4,3,7, // Left
                3,2,6, 3,6,7, // Top
                4,5,1, 4,1,0  // Bottom
            };

            foreach(var pt in p) mesh.Positions.Add(pt);
            foreach(var idx in indices) mesh.TriangleIndices.Add(idx);

            return mesh;
        }

        private MeshGeometry3D CreateCylinderMesh(double radius, double length, int thetaDiv)
        {
            var mesh = new MeshGeometry3D();
            double dz = length / 2.0;

            for (int i = 0; i < thetaDiv; i++)
            {
                double theta = (2.0 * Math.PI * i) / thetaDiv;
                double x = radius * Math.Cos(theta);
                double y = radius * Math.Sin(theta);

                mesh.Positions.Add(new Point3D(x, y, dz));
                mesh.Positions.Add(new Point3D(x, y, -dz));
            }

            mesh.Positions.Add(new Point3D(0, 0, dz));
            int topCenter = mesh.Positions.Count - 1;
            mesh.Positions.Add(new Point3D(0, 0, -dz));
            int bottomCenter = mesh.Positions.Count - 1;

            for (int i = 0; i < thetaDiv; i++)
            {
                int next = (i + 1) % thetaDiv;
                int topCur = i * 2; int botCur = i * 2 + 1;
                int topNext = next * 2; int botNext = next * 2 + 1;

                // Sides
                mesh.TriangleIndices.Add(topCur); mesh.TriangleIndices.Add(botCur); mesh.TriangleIndices.Add(botNext);
                mesh.TriangleIndices.Add(topCur); mesh.TriangleIndices.Add(botNext); mesh.TriangleIndices.Add(topNext);

                // Caps
                mesh.TriangleIndices.Add(topCenter); mesh.TriangleIndices.Add(topNext); mesh.TriangleIndices.Add(topCur);
                mesh.TriangleIndices.Add(bottomCenter); mesh.TriangleIndices.Add(botCur); mesh.TriangleIndices.Add(botNext);
            }

            return mesh;
        }

        private Vector3D ParseVector(string vecStr)
        {
            var parts = vecStr.Split(new[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
            return new Vector3D(
                double.Parse(parts[0], CultureInfo.InvariantCulture),
                double.Parse(parts[1], CultureInfo.InvariantCulture),
                double.Parse(parts[2], CultureInfo.InvariantCulture));
        }

        public void UpdateJoints(double j1, double j2, double j3, double j4, double j5, double j6)
        {
            if (_jointRotations.ContainsKey(1)) _jointRotations[1].Angle = j1;
            if (_jointRotations.ContainsKey(2)) _jointRotations[2].Angle = j2;
            if (_jointRotations.ContainsKey(3)) _jointRotations[3].Angle = j3;
            if (_jointRotations.ContainsKey(4)) _jointRotations[4].Angle = j4;
            if (_jointRotations.ContainsKey(5)) _jointRotations[5].Angle = j5;
            if (_jointRotations.ContainsKey(6)) _jointRotations[6].Angle = j6;
        }
    }
}
