using System;
using System.Collections.Generic;
using System.Linq;
using RobotSimulator.Core.Models;

namespace RobotSimulator.Core.Motion
{
    public enum InterpolationType
    {
        Linear,
        SCurve
    }

    public class TrajectoryPlanner
    {
        private readonly double _maxVelocity = 2.0; // rad/s
        private readonly double _maxAcceleration = 1.0; // rad/s^2
        private readonly double _controlFrequency = 50.0; // Hz

        public TrajectoryPlanner(double maxVel = 2.0, double maxAcc = 1.0, double freq = 50.0)
        {
            _maxVelocity = maxVel;
            _maxAcceleration = maxAcc;
            _controlFrequency = freq;
        }

        public List<double[]> PlanTrajectory(List<double[]> waypoints, InterpolationType type = InterpolationType.SCurve)
        {
            if (waypoints == null || waypoints.Count < 2)
                return waypoints;

            var fullTrajectory = new List<double[]>();
            
            for (int i = 0; i < waypoints.Count - 1; i++)
            {
                var segment = GenerateSegment(waypoints[i], waypoints[i + 1], type);
                // Avoid duplicating points at segment boundaries
                if (i > 0) segment.RemoveAt(0);
                fullTrajectory.AddRange(segment);
            }

            return fullTrajectory;
        }

        private List<double[]> GenerateSegment(double[] start, double[] end, InterpolationType type)
        {
            // Calculate max displacement to determine duration
            double maxDisp = 0;
            for (int i = 0; i < start.Length; i++)
            {
                maxDisp = Math.Max(maxDisp, Math.Abs(end[i] - start[i]));
            }

            // Simple trapezoidal time calculation
            double duration = Math.Max(maxDisp / _maxVelocity, Math.Sqrt(4 * maxDisp / _maxAcceleration));
            // Minimum duration check
            duration = Math.Max(duration, 0.1);

            int steps = (int)(duration * _controlFrequency);
            var segmentPoints = new List<double[]>();

            for (int step = 0; step <= steps; step++)
            {
                double t = (double)step / steps;
                double progress = type == InterpolationType.SCurve ? SCurve(t) : t;
                
                var point = new double[start.Length];
                for (int j = 0; j < start.Length; j++)
                {
                    point[j] = start[j] + (end[j] - start[j]) * progress;
                }
                segmentPoints.Add(point);
            }

            return segmentPoints;
        }

        private double SCurve(double t)
        {
            // Cubic Hermite spline (SmoothStep)
            // 3t^2 - 2t^3
            return t * t * (3 - 2 * t);
            
            // For smoother S-Curve (Quintic):
            // 6t^5 - 15t^4 + 10t^3
            // return t * t * t * (t * (t * 6 - 15) + 10);
        }
    }
}
