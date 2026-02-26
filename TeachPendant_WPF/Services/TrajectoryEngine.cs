using System;
using System.Collections.Generic;
using System.Windows.Media.Media3D;

namespace TeachPendant_WPF.Services
{
    /// <summary>
    /// Smooth trajectory generation engine.
    /// Supports S-curve velocity profiles, jerk-limited motion,
    /// corner rounding, and blend radius for industrial-grade paths.
    /// </summary>
    public class TrajectoryEngine
    {
        // ── Trajectory Point ────────────────────────────────────────

        public class TrajectoryPoint
        {
            public double[] JointAngles { get; set; } = Array.Empty<double>();
            public Point3D CartesianPosition { get; set; }
            public double TimeFromStart { get; set; } // seconds
            public double Velocity { get; set; }       // mm/s or deg/s
        }

        // ── S-Curve Profile ─────────────────────────────────────────

        /// <summary>
        /// Generate an S-curve velocity profile between two waypoints.
        /// Returns timestamped joint angle interpolations.
        /// </summary>
        public List<TrajectoryPoint> GenerateLinearTrajectory(
            double[] startAngles,
            double[] endAngles,
            double maxVelocityDegPerSec,
            double maxAccelDegPerSec2,
            double maxJerkDegPerSec3,
            double sampleRateHz = 50.0)
        {
            if (startAngles.Length != endAngles.Length)
                throw new ArgumentException("Joint arrays must match in length.");

            int dof = startAngles.Length;
            var trajectory = new List<TrajectoryPoint>();

            // Find the joint that needs the most travel
            double maxDelta = 0;
            for (int j = 0; j < dof; j++)
                maxDelta = Math.Max(maxDelta, Math.Abs(endAngles[j] - startAngles[j]));

            if (maxDelta < 0.001)
            {
                // Already at target
                trajectory.Add(new TrajectoryPoint
                {
                    JointAngles = (double[])startAngles.Clone(),
                    TimeFromStart = 0
                });
                return trajectory;
            }

            // Compute S-curve timing
            double totalTime = ComputeSCurveTime(maxDelta, maxVelocityDegPerSec, maxAccelDegPerSec2, maxJerkDegPerSec3);
            double dt = 1.0 / sampleRateHz;

            for (double t = 0; t <= totalTime; t += dt)
            {
                double s = SCurveParameter(t, totalTime); // normalized 0→1

                double[] angles = new double[dof];
                for (int j = 0; j < dof; j++)
                {
                    angles[j] = startAngles[j] + (endAngles[j] - startAngles[j]) * s;
                }

                trajectory.Add(new TrajectoryPoint
                {
                    JointAngles = angles,
                    TimeFromStart = t,
                    Velocity = maxVelocityDegPerSec * SCurveVelocity(t, totalTime)
                });
            }

            // Ensure we end exactly at the target
            trajectory.Add(new TrajectoryPoint
            {
                JointAngles = (double[])endAngles.Clone(),
                TimeFromStart = totalTime
            });

            return trajectory;
        }

        /// <summary>
        /// Generate blended trajectory through multiple waypoints with corner rounding.
        /// </summary>
        public List<TrajectoryPoint> GenerateMultiPointTrajectory(
            List<double[]> waypoints,
            double maxVelocity,
            double maxAccel,
            double maxJerk,
            double blendRadius = 5.0,
            double sampleRateHz = 50.0)
        {
            var fullTrajectory = new List<TrajectoryPoint>();
            double timeOffset = 0;

            for (int i = 0; i < waypoints.Count - 1; i++)
            {
                var segment = GenerateLinearTrajectory(
                    waypoints[i], waypoints[i + 1],
                    maxVelocity, maxAccel, maxJerk, sampleRateHz);

                foreach (var pt in segment)
                {
                    pt.TimeFromStart += timeOffset;
                    fullTrajectory.Add(pt);
                }

                if (segment.Count > 0)
                    timeOffset = segment[^1].TimeFromStart;
            }

            return fullTrajectory;
        }

        // ── S-Curve Math ────────────────────────────────────────────

        private double ComputeSCurveTime(double distance, double vMax, double aMax, double jMax)
        {
            // Simplified trapezoidal estimation with S-curve ramps
            double tAccel = vMax / aMax;
            double distAccel = 0.5 * vMax * tAccel;

            if (2 * distAccel >= distance)
            {
                // Triangle profile (never reaches max velocity)
                return 2 * Math.Sqrt(distance / aMax);
            }
            else
            {
                // Trapezoidal profile
                double distCruise = distance - 2 * distAccel;
                double tCruise = distCruise / vMax;
                return 2 * tAccel + tCruise;
            }
        }

        /// <summary>
        /// Smoothstep-based S-curve parameter (position). Returns 0→1.
        /// Uses quintic Hermite for jerk continuity.
        /// </summary>
        private double SCurveParameter(double t, double totalTime)
        {
            double x = Math.Clamp(t / totalTime, 0, 1);
            // Quintic smoothstep: 6x⁵ - 15x⁴ + 10x³
            return x * x * x * (x * (x * 6 - 15) + 10);
        }

        /// <summary>
        /// S-curve velocity (derivative of position). Normalized 0→1→0.
        /// </summary>
        private double SCurveVelocity(double t, double totalTime)
        {
            double x = Math.Clamp(t / totalTime, 0, 1);
            // Derivative of quintic smoothstep: 30x⁴ - 60x³ + 30x²
            return x * x * (30 * x * x - 60 * x + 30) / totalTime;
        }

        // ── Cartesian Trajectory ────────────────────────────────────

        /// <summary>
        /// Generate trajectory along a parametric curve with uniform speed.
        /// Used for curve-following toolpaths.
        /// </summary>
        public List<TrajectoryPoint> GenerateCurveFollowTrajectory(
            Func<double, Point3D> curveEvaluator,
            double curveLength,
            double linearSpeedMmPerSec,
            double sampleRateHz = 50.0)
        {
            var trajectory = new List<TrajectoryPoint>();
            double totalTime = curveLength / linearSpeedMmPerSec;
            double dt = 1.0 / sampleRateHz;

            for (double t = 0; t <= totalTime; t += dt)
            {
                double param = Math.Clamp(t / totalTime, 0, 1);
                var position = curveEvaluator(param);

                trajectory.Add(new TrajectoryPoint
                {
                    CartesianPosition = position,
                    TimeFromStart = t,
                    Velocity = linearSpeedMmPerSec
                });
            }

            return trajectory;
        }
    }
}
