using System;

namespace RoboForge_WPF.Kinematics
{
    /// <summary>
    /// Represents a 6-DOF end-effector pose in Cartesian space.
    /// </summary>
    public class EndEffectorPose
    {
        public double X { get; set; }
        public double Y { get; set; }
        public double Z { get; set; }
        public double RX { get; set; } // Roll
        public double RY { get; set; } // Pitch
        public double RZ { get; set; } // Yaw

        public EndEffectorPose() { }

        public EndEffectorPose(double x, double y, double z, double rx = 0, double ry = 0, double rz = 0)
        {
            X = x; Y = y; Z = z;
            RX = rx; RY = ry; RZ = rz;
        }

        /// <summary>Euclidean position distance to another pose.</summary>
        public double PositionDistanceTo(EndEffectorPose other)
        {
            double dx = X - other.X, dy = Y - other.Y, dz = Z - other.Z;
            return Math.Sqrt(dx * dx + dy * dy + dz * dz);
        }

        /// <summary>Combined position + orientation error norm.</summary>
        public double FullErrorTo(EndEffectorPose other)
        {
            double posDist = PositionDistanceTo(other);
            double drx = RX - other.RX, dry = RY - other.RY, drz = RZ - other.RZ;
            double oriDist = Math.Sqrt(drx * drx + dry * dry + drz * drz);
            return posDist + oriDist * 0.1; // Weight orientation less than position
        }

        public double[] ToArray() => new[] { X, Y, Z, RX, RY, RZ };

        public static EndEffectorPose FromArray(double[] arr)
        {
            return new EndEffectorPose(
                arr.Length > 0 ? arr[0] : 0,
                arr.Length > 1 ? arr[1] : 0,
                arr.Length > 2 ? arr[2] : 0,
                arr.Length > 3 ? arr[3] : 0,
                arr.Length > 4 ? arr[4] : 0,
                arr.Length > 5 ? arr[5] : 0);
        }

        public override string ToString() => $"({X:F1}, {Y:F1}, {Z:F1}, {RX:F1}°, {RY:F1}°, {RZ:F1}°)";
    }
}
