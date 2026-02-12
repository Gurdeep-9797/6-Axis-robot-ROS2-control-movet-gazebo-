using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using System.Windows.Media;
using System.Windows.Media.Media3D;

namespace RobotSimulator.Core.Import
{
    /// <summary>
    /// Loads STL files (binary and ASCII format) for workpiece visualization.
    /// </summary>
    public class STLLoader
    {
        /// <summary>
        /// Load an STL file and return mesh geometry.
        /// Automatically detects binary vs ASCII format.
        /// </summary>
        public static MeshGeometry3D LoadSTL(string filePath)
        {
            if (!File.Exists(filePath))
                throw new FileNotFoundException($"STL file not found: {filePath}");

            // Check if binary or ASCII
            var bytes = File.ReadAllBytes(filePath);
            
            // ASCII STL starts with "solid "
            if (bytes.Length > 6 && Encoding.ASCII.GetString(bytes, 0, 6) == "solid ")
            {
                // Could be ASCII, but binary files might also start with "solid"
                // Check for "facet" keyword which only appears in ASCII
                var text = Encoding.ASCII.GetString(bytes);
                if (text.Contains("facet normal"))
                    return LoadAsciiSTL(filePath);
            }

            return LoadBinarySTL(bytes);
        }

        /// <summary>
        /// Load binary STL file
        /// </summary>
        private static MeshGeometry3D LoadBinarySTL(byte[] data)
        {
            var mesh = new MeshGeometry3D();
            var positions = new Point3DCollection();
            var normals = new Vector3DCollection();
            var indices = new Int32Collection();

            // Binary format:
            // 80 bytes header
            // 4 bytes triangle count (uint32)
            // For each triangle:
            //   12 bytes normal (3 floats)
            //   36 bytes vertices (9 floats, 3 vertices × 3 coords)
            //   2 bytes attribute byte count

            if (data.Length < 84)
                throw new InvalidDataException("STL file too small to be valid");

            int triangleCount = BitConverter.ToInt32(data, 80);
            int expectedSize = 84 + triangleCount * 50;

            if (data.Length < expectedSize)
                throw new InvalidDataException($"STL file truncated: expected {expectedSize} bytes, got {data.Length}");

            int offset = 84;
            int vertexIndex = 0;

            for (int i = 0; i < triangleCount; i++)
            {
                // Normal
                float nx = BitConverter.ToSingle(data, offset);
                float ny = BitConverter.ToSingle(data, offset + 4);
                float nz = BitConverter.ToSingle(data, offset + 8);
                var normal = new Vector3D(nx, ny, nz);
                offset += 12;

                // Three vertices
                for (int v = 0; v < 3; v++)
                {
                    float x = BitConverter.ToSingle(data, offset);
                    float y = BitConverter.ToSingle(data, offset + 4);
                    float z = BitConverter.ToSingle(data, offset + 8);
                    offset += 12;

                    // Convert mm to meters (STL typically in mm)
                    positions.Add(new Point3D(x / 1000.0, y / 1000.0, z / 1000.0));
                    normals.Add(normal);
                    indices.Add(vertexIndex++);
                }

                // Skip attribute byte count
                offset += 2;
            }

            mesh.Positions = positions;
            mesh.Normals = normals;
            mesh.TriangleIndices = indices;

            return mesh;
        }

        /// <summary>
        /// Load ASCII STL file
        /// </summary>
        private static MeshGeometry3D LoadAsciiSTL(string filePath)
        {
            var mesh = new MeshGeometry3D();
            var positions = new Point3DCollection();
            var normals = new Vector3DCollection();
            var indices = new Int32Collection();

            var lines = File.ReadAllLines(filePath);
            Vector3D currentNormal = new Vector3D();
            int vertexIndex = 0;

            foreach (var rawLine in lines)
            {
                var line = rawLine.Trim();

                if (line.StartsWith("facet normal"))
                {
                    var parts = line.Split(new[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
                    if (parts.Length >= 5)
                    {
                        double nx = ParseDouble(parts[2]);
                        double ny = ParseDouble(parts[3]);
                        double nz = ParseDouble(parts[4]);
                        currentNormal = new Vector3D(nx, ny, nz);
                    }
                }
                else if (line.StartsWith("vertex"))
                {
                    var parts = line.Split(new[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
                    if (parts.Length >= 4)
                    {
                        double x = ParseDouble(parts[1]);
                        double y = ParseDouble(parts[2]);
                        double z = ParseDouble(parts[3]);

                        // Convert mm to meters
                        positions.Add(new Point3D(x / 1000.0, y / 1000.0, z / 1000.0));
                        normals.Add(currentNormal);
                        indices.Add(vertexIndex++);
                    }
                }
            }

            mesh.Positions = positions;
            mesh.Normals = normals;
            mesh.TriangleIndices = indices;

            return mesh;
        }

        private static double ParseDouble(string s)
        {
            return double.Parse(s, CultureInfo.InvariantCulture);
        }

        /// <summary>
        /// Get bounding box of loaded mesh
        /// </summary>
        public static Rect3D GetBoundingBox(MeshGeometry3D mesh)
        {
            if (mesh.Positions.Count == 0)
                return Rect3D.Empty;

            double minX = double.MaxValue, minY = double.MaxValue, minZ = double.MaxValue;
            double maxX = double.MinValue, maxY = double.MinValue, maxZ = double.MinValue;

            foreach (var pos in mesh.Positions)
            {
                minX = Math.Min(minX, pos.X);
                minY = Math.Min(minY, pos.Y);
                minZ = Math.Min(minZ, pos.Z);
                maxX = Math.Max(maxX, pos.X);
                maxY = Math.Max(maxY, pos.Y);
                maxZ = Math.Max(maxZ, pos.Z);
            }

            return new Rect3D(minX, minY, minZ, maxX - minX, maxY - minY, maxZ - minZ);
        }

        /// <summary>
        /// Scale mesh to fit within specified size
        /// </summary>
        public static void ScaleMesh(MeshGeometry3D mesh, double maxDimension)
        {
            var bounds = GetBoundingBox(mesh);
            if (bounds.IsEmpty) return;

            double currentMax = Math.Max(bounds.SizeX, Math.Max(bounds.SizeY, bounds.SizeZ));
            if (currentMax <= 0) return;

            double scale = maxDimension / currentMax;
            var newPositions = new Point3DCollection();

            foreach (var pos in mesh.Positions)
            {
                newPositions.Add(new Point3D(pos.X * scale, pos.Y * scale, pos.Z * scale));
            }

            mesh.Positions = newPositions;
        }

        /// <summary>
        /// Translate mesh to new origin
        /// </summary>
        public static void TranslateMesh(MeshGeometry3D mesh, Vector3D offset)
        {
            var newPositions = new Point3DCollection();

            foreach (var pos in mesh.Positions)
            {
                newPositions.Add(new Point3D(pos.X + offset.X, pos.Y + offset.Y, pos.Z + offset.Z));
            }

            mesh.Positions = newPositions;
        }
    }
}
