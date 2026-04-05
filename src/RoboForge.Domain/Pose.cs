using System;
using System.Numerics;

namespace RoboForge.Domain
{
    public record Pose
    {
        public double X { get; set; }
        public double Y { get; set; }
        public double Z { get; set; }
        public double Rx { get; set; }
        public double Ry { get; set; }
        public double Rz { get; set; }

        public Quaternion ToQuaternion()
        {
            float pitch = (float)(Ry * Math.PI / 180.0);
            float roll = (float)(Rx * Math.PI / 180.0);
            float yaw = (float)(Rz * Math.PI / 180.0);
            return Quaternion.CreateFromYawPitchRoll(yaw, pitch, roll);
        }

        public Matrix4x4 ToTransform()
        {
            var q = ToQuaternion();
            var m = Matrix4x4.CreateFromQuaternion(q);
            m.Translation = new Vector3((float)X, (float)Y, (float)Z);
            return m;
        }
    }

    public enum ZoneType
    {
        Fine,
        Z1,
        Z5,
        Z10,
        Z25,
        Z50,
        Z100
    }
}
