using System;

namespace RoboForge.Domain
{
    public class RobotState
    {
        public double[] JointAngles { get; set; } = Array.Empty<double>();
        public double[] JointVelocities { get; set; } = Array.Empty<double>();
        public DateTime Timestamp { get; set; }
    }

    public class IKResult
    {
        public bool Success { get; set; }
        public double[] JointValues { get; set; } = Array.Empty<double>();
    }

    public class MoveResult
    {
        public bool Success { get; set; }
        public string ErrorCode { get; set; } = string.Empty;
    }

    public enum JogMode { Joint, Cartesian }

    public class JogRequest
    {
        public JogMode Mode { get; set; }
        public double[] Values { get; set; } = Array.Empty<double>();
    }
}
