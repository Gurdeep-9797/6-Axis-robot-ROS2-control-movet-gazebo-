using System;
using System.Collections.Generic;
using System.Windows.Media;
using System.Windows.Media.Media3D;

namespace TeachPendant_WPF.SceneGraph
{
    /// <summary>
    /// Represents a loaded workpiece (STL/STEP) in the work area.
    /// Contains the mesh geometry plus extracted curves and edges
    /// that can be used for toolpath generation.
    /// </summary>
    public class WorkpieceNode : SceneNode
    {
        // ── Mesh Geometry ───────────────────────────────────────────

        private MeshGeometry3D? _mesh;
        public MeshGeometry3D? Mesh
        {
            get => _mesh;
            set { _mesh = value; OnPropertyChanged(); }
        }

        private Material? _material;
        public Material? Material
        {
            get => _material;
            set { _material = value; OnPropertyChanged(); }
        }

        /// <summary>
        /// File path from which this workpiece was loaded.
        /// </summary>
        private string _filePath = string.Empty;
        public string FilePath
        {
            get => _filePath;
            set { _filePath = value; OnPropertyChanged(); }
        }

        // ── Extracted Features ──────────────────────────────────────

        /// <summary>
        /// Parametric curves extracted from the workpiece geometry.
        /// These can be selected by the user for curve-following toolpaths.
        /// </summary>
        public List<CurveNode> Curves { get; } = new();

        /// <summary>
        /// Edges extracted from the mesh.
        /// </summary>
        public List<EdgeNode> Edges { get; } = new();

        /// <summary>
        /// Reference planes defined on this workpiece.
        /// </summary>
        public List<FrameNode> ReferencePlanes { get; } = new();

        // ── Visibility ──────────────────────────────────────────────

        private bool _isVisible = true;
        public bool IsVisible
        {
            get => _isVisible;
            set { _isVisible = value; OnPropertyChanged(); }
        }

        private double _opacity = 1.0;
        public double Opacity
        {
            get => _opacity;
            set { _opacity = Math.Clamp(value, 0.0, 1.0); OnPropertyChanged(); }
        }

        public override void Update() { }
    }

    /// <summary>
    /// Represents an edge on a workpiece mesh.
    /// An edge is defined by a sequence of 3D points.
    /// </summary>
    public class EdgeNode : SceneNode
    {
        /// <summary>
        /// Ordered points defining this edge.
        /// </summary>
        public List<Point3D> Points { get; } = new();

        /// <summary>
        /// Approximate total length of the edge.
        /// </summary>
        public double Length
        {
            get
            {
                double len = 0;
                for (int i = 1; i < Points.Count; i++)
                    len += (Points[i] - Points[i - 1]).Length;
                return len;
            }
        }

        public override void Update() { }
    }
}
