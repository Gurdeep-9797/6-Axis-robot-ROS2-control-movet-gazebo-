using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Media.Media3D;

namespace TeachPendant_WPF.SceneGraph
{
    /// <summary>
    /// Represents the entire robot kinematic chain in the scene graph.
    /// Manages a sequential list of JointNodes and provides convenience
    /// methods for applying joint angles and reading TCP.
    /// </summary>
    public class RobotNode : SceneNode
    {
        // ── Joint Chain ─────────────────────────────────────────────
        private readonly List<JointNode> _joints = new();

        /// <summary>
        /// All joints in kinematic order (J1 → J6).
        /// Read-only view; add via AddJoint().
        /// </summary>
        public IReadOnlyList<JointNode> Joints => _joints;

        /// <summary>
        /// Number of degrees of freedom.
        /// </summary>
        public int DOF => _joints.Count;

        // ── Tool Frame ──────────────────────────────────────────────

        /// <summary>
        /// The tool center point frame, attached as the last child
        /// of the last joint. Its WorldMatrix gives TCP pose.
        /// </summary>
        public FrameNode? ToolFrame { get; private set; }

        // ── Construction ────────────────────────────────────────────

        /// <summary>
        /// Add a joint to the chain. Joints are linked sequentially:
        /// Robot → J1 → J2 → … → J6 → ToolFrame
        /// </summary>
        public void AddJoint(JointNode joint)
        {
            SceneNode parentNode = _joints.Count > 0
                ? _joints[^1]   // last joint
                : (SceneNode)this;   // robot base

            parentNode.AddChild(joint);
            _joints.Add(joint);

            // If a tool frame exists, re-parent it under the new last joint
            if (ToolFrame != null)
            {
                var oldParent = _joints.Count > 1 ? _joints[^2] : (SceneNode)this;
                oldParent.RemoveChild(ToolFrame);
                joint.AddChild(ToolFrame);
            }
        }

        /// <summary>
        /// Attach or replace the tool center point frame.
        /// </summary>
        public void SetToolFrame(FrameNode frame)
        {
            // Remove previous
            if (ToolFrame != null)
            {
                var lastParent = _joints.Count > 0 ? (SceneNode)_joints[^1] : this;
                lastParent.RemoveChild(ToolFrame);
            }

            ToolFrame = frame;

            if (_joints.Count > 0)
                _joints[^1].AddChild(frame);
            else
                AddChild(frame);
        }

        // ── Kinematics API ──────────────────────────────────────────

        /// <summary>
        /// Apply an array of joint angles (degrees) to the kinematic chain.
        /// The 3D model will update automatically through transform cascading.
        /// </summary>
        public void ApplyJointAngles(double[] anglesDeg)
        {
            if (anglesDeg == null) return;

            int count = Math.Min(anglesDeg.Length, _joints.Count);
            for (int i = 0; i < count; i++)
            {
                _joints[i].CurrentAngle = anglesDeg[i];
            }
        }

        /// <summary>
        /// Read current joint angles as an array.
        /// </summary>
        public double[] GetJointAngles()
        {
            return _joints.Select(j => j.CurrentAngle).ToArray();
        }

        /// <summary>
        /// Get the world-space pose of the tool center point.
        /// Returns the 4×4 matrix of the ToolFrame (or last joint if no tool).
        /// </summary>
        public Matrix3D GetTCPPose()
        {
            if (ToolFrame != null)
                return ToolFrame.WorldMatrix;

            if (_joints.Count > 0)
                return _joints[^1].WorldMatrix;

            return WorldMatrix;
        }

        /// <summary>
        /// Get the world-space position of the tool center point.
        /// </summary>
        public Point3D GetTCPPosition()
        {
            var m = GetTCPPose();
            return new Point3D(m.OffsetX, m.OffsetY, m.OffsetZ);
        }

        /// <summary>
        /// Check if given angles are all within joint limits.
        /// </summary>
        public bool AreAnglesReachable(double[] anglesDeg)
        {
            if (anglesDeg == null || anglesDeg.Length != _joints.Count)
                return false;

            for (int i = 0; i < _joints.Count; i++)
            {
                if (anglesDeg[i] < _joints[i].MinLimit || anglesDeg[i] > _joints[i].MaxLimit)
                    return false;
            }
            return true;
        }

        public override void Update()
        {
            // Joint angles are applied on-demand. 
            // This hook is available for constraint-driven updates.
        }
    }
}
