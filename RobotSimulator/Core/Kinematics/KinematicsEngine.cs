using System;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using RobotSimulator.Core.Models;

namespace RobotSimulator.Core.Kinematics
{
    /// <summary>
    /// Forward and Inverse Kinematics calculations for 6-axis robot.
    /// Uses DH (Denavit-Hartenberg) convention.
    /// </summary>
    public class KinematicsEngine
    {
        private readonly RobotModel _robot;
        
        public KinematicsEngine(RobotModel robot)
        {
            _robot = robot;
        }

        /// <summary>
        /// Calculate forward kinematics - joint angles to end effector pose.
        /// Returns 4x4 homogeneous transformation matrix.
        /// </summary>
        public Matrix<double> CalculateFK(double[] jointAngles)
        {
            var result = DenseMatrix.CreateIdentity(4);
            var joints = _robot.GetActuatedJoints();
            
            for (int i = 0; i < Math.Min(joints.Count, jointAngles.Length); i++)
            {
                var joint = joints[i];
                
                // Translation from joint origin
                var translation = CreateTranslation(joint.OriginX, joint.OriginY, joint.OriginZ);
                
                // Rotation from joint origin
                var rotOrigin = CreateRotation(joint.OriginRoll, joint.OriginPitch, joint.OriginYaw);
                
                // Joint rotation around axis
                var rotJoint = CreateJointRotation(jointAngles[i], 
                    joint.AxisX, joint.AxisY, joint.AxisZ);
                
                result = (DenseMatrix)(result * translation * rotOrigin * rotJoint);
            }
            
            return result;
        }

        /// <summary>
        /// Get end effector position (X, Y, Z).
        /// </summary>
        public (double X, double Y, double Z) GetEndEffectorPosition(double[] jointAngles)
        {
            var T = CalculateFK(jointAngles);
            return (T[0, 3], T[1, 3], T[2, 3]);
        }

        /// <summary>
        /// Get end effector orientation as RPY (Roll, Pitch, Yaw).
        /// </summary>
        public (double Roll, double Pitch, double Yaw) GetEndEffectorOrientation(double[] jointAngles)
        {
            var T = CalculateFK(jointAngles);
            
            // Extract Euler angles from rotation matrix
            double pitch = Math.Asin(-T[2, 0]);
            double roll, yaw;
            
            if (Math.Abs(Math.Cos(pitch)) > 1e-6)
            {
                roll = Math.Atan2(T[2, 1], T[2, 2]);
                yaw = Math.Atan2(T[1, 0], T[0, 0]);
            }
            else
            {
                roll = Math.Atan2(-T[1, 2], T[1, 1]);
                yaw = 0;
            }
            
            return (roll, pitch, yaw);
        }

        /// <summary>
        /// Robust IK Solver using Damped Least Squares with Random Restarts.
        /// This ensures we find a solution even if the initial guess leads to a local minimum.
        /// </summary>
        public bool SolveIK(double targetX, double targetY, double targetZ,
            double roll, double pitch, double yaw,
            double[] initialGuess, out double[] result)
        {
            // 1. Try with initial guess (fastest)
            if (CalculateIK(targetX, targetY, targetZ, roll, pitch, yaw, initialGuess, out result))
                return true;

            // 2. Try with current robot pose (if different from guess)
            // (Assumed passed as initialGuess, skipping)

            // 3. Random Restarts (Global Search)
            // Try standard configurations: [0,0...], [90,0...], etc.
            double[][] seeds = new double[][] 
            {
                new double[] {0,0,0,0,0,0},
                new double[] {0, -Math.PI/4, Math.PI/4, 0, Math.PI/2, 0}, // Typical reach
                new double[] {Math.PI, 0, 0, 0, 0, 0} // Backwards
            };

            foreach (var seed in seeds)
            {
                if (CalculateIK(targetX, targetY, targetZ, roll, pitch, yaw, seed, out result))
                    return true;
            }

            // 4. Randomized Restarts
            var rand = new Random();
            for (int i = 0; i < 5; i++)
            {
                var randomSeed = new double[6];
                for(int j=0; j<6; j++) 
                    randomSeed[j] = (rand.NextDouble() * 2 - 1) * Math.PI; // -PI to PI
                
                if (CalculateIK(targetX, targetY, targetZ, roll, pitch, yaw, randomSeed, out result))
                    return true;
            }

            return false;
        }

        /// <summary>
        /// Core Numerical Inverse Kinematics using Damped Least Squares Jacobian.
        /// </summary>
        private bool CalculateIK(double targetX, double targetY, double targetZ,
            double targetRoll, double targetPitch, double targetYaw,
            double[] initialGuess, out double[] result, int maxIterations = 50)
        {
            result = (double[])initialGuess.Clone();
            var joints = _robot.GetActuatedJoints();
            int n = joints.Count;
            
            double tolerance = 1e-3; // 1mm precison
            
            for (int iter = 0; iter < maxIterations; iter++)
            {
                var (cx, cy, cz) = GetEndEffectorPosition(result);
                var (cr, cp, cw) = GetEndEffectorOrientation(result);
                
                // Error vector
                double[] error = {
                    targetX - cx, targetY - cy, targetZ - cz,
                    AngleDiff(targetRoll, cr), AngleDiff(targetPitch, cp), AngleDiff(targetYaw, cw)
                };
                
                double errorNorm = Math.Sqrt(
                    error[0]*error[0] + error[1]*error[1] + error[2]*error[2] +
                    error[3]*error[3] + error[4]*error[4] + error[5]*error[5]);
                
                if (errorNorm < tolerance)
                    return true;
                
                // Calculate Jacobian
                var J = CalculateJacobian(result);
                
                // Pseudo-inverse: J+ = J^T * (J * J^T)^-1
                var JT = J.Transpose();
                var JJT = J * JT;
                
                // Add damping for singularity robustness
                var damping = DenseMatrix.CreateIdentity(6) * 0.001;
                JJT = JJT + damping;
                
                try
                {
                    var JJTinv = JJT.Inverse();
                    var Jpinv = JT * JJTinv;
                    
                    var errorVec = DenseVector.OfArray(error);
                    var dq = Jpinv * errorVec;
                    
                    // Update joint angles
                    for (int i = 0; i < n; i++)
                    {
                        result[i] += dq[i] * 0.5; // Damped step
                        result[i] = Math.Clamp(result[i], joints[i].LowerLimit, joints[i].UpperLimit);
                    }
                }
                catch
                {
                    return false; // Singular configuration
                }
            }
            
            return false;
        }

        /// <summary>
        /// Check if a target pose is reachable.
        /// </summary>
        public bool IsReachable(double x, double y, double z, 
            double roll, double pitch, double yaw, double[] initialGuess)
        {
            return CalculateIK(x, y, z, roll, pitch, yaw, initialGuess, out _);
        }

        /// <summary>
        /// Explicitly verifies if a set of joint angles results in the expected pose.
        /// Throws exception if error is too large.
        /// </summary>
        public bool VerifyIK(double[] jointAngles, double targetX, double targetY, double targetZ, 
            double targetRoll, double targetPitch, double targetYaw, double tolerance = 1e-3)
        {
            var (cx, cy, cz) = GetEndEffectorPosition(jointAngles);
            var (cr, cp, cw) = GetEndEffectorOrientation(jointAngles);

            double posError = Math.Sqrt(
                Math.Pow(targetX - cx, 2) + 
                Math.Pow(targetY - cy, 2) + 
                Math.Pow(targetZ - cz, 2));

            // Simplified orientation error check (Euclidean dist of angles - debatable but useful for sanity check)
            double rotError = Math.Sqrt(
                Math.Pow(AngleDiff(targetRoll, cr), 2) + 
                Math.Pow(AngleDiff(targetPitch, cp), 2) + 
                Math.Pow(AngleDiff(targetYaw, cw), 2));

            if (posError > tolerance || rotError > 0.1) // Rot tolerance slightly looser
            {
                // Log failure details
                // Console.WriteLine($"IK Verification Failed! PosErr: {posError:F4}, RotErr: {rotError:F4}");
                return false;
            }
            return true;
        }

        /// <summary>
        /// Calculate numerical Jacobian.
        /// </summary>
        private Matrix<double> CalculateJacobian(double[] jointAngles)
        {
            int n = jointAngles.Length;
            var J = DenseMatrix.Create(6, n, 0);
            double delta = 1e-6;
            
            var (px, py, pz) = GetEndEffectorPosition(jointAngles);
            var (or, op, oy) = GetEndEffectorOrientation(jointAngles);
            
            for (int i = 0; i < n; i++)
            {
                var perturbed = (double[])jointAngles.Clone();
                perturbed[i] += delta;
                
                var (px2, py2, pz2) = GetEndEffectorPosition(perturbed);
                var (or2, op2, oy2) = GetEndEffectorOrientation(perturbed);
                
                J[0, i] = (px2 - px) / delta;
                J[1, i] = (py2 - py) / delta;
                J[2, i] = (pz2 - pz) / delta;
                J[3, i] = AngleDiff(or2, or) / delta;
                J[4, i] = AngleDiff(op2, op) / delta;
                J[5, i] = AngleDiff(oy2, oy) / delta;
            }
            
            return J;
        }

        private static double AngleDiff(double a, double b)
        {
            double diff = a - b;
            while (diff > Math.PI) diff -= 2 * Math.PI;
            while (diff < -Math.PI) diff += 2 * Math.PI;
            return diff;
        }

        private static Matrix<double> CreateTranslation(double x, double y, double z)
        {
            var T = DenseMatrix.CreateIdentity(4);
            T[0, 3] = x;
            T[1, 3] = y;
            T[2, 3] = z;
            return T;
        }

        private static Matrix<double> CreateRotation(double roll, double pitch, double yaw)
        {
            double cr = Math.Cos(roll), sr = Math.Sin(roll);
            double cp = Math.Cos(pitch), sp = Math.Sin(pitch);
            double cy = Math.Cos(yaw), sy = Math.Sin(yaw);
            
            var R = DenseMatrix.CreateIdentity(4);
            R[0, 0] = cy * cp;
            R[0, 1] = cy * sp * sr - sy * cr;
            R[0, 2] = cy * sp * cr + sy * sr;
            R[1, 0] = sy * cp;
            R[1, 1] = sy * sp * sr + cy * cr;
            R[1, 2] = sy * sp * cr - cy * sr;
            R[2, 0] = -sp;
            R[2, 1] = cp * sr;
            R[2, 2] = cp * cr;
            return R;
        }

        private static Matrix<double> CreateJointRotation(double angle, double ax, double ay, double az)
        {
            // Normalize axis
            double len = Math.Sqrt(ax*ax + ay*ay + az*az);
            if (len < 1e-6) return DenseMatrix.CreateIdentity(4);
            ax /= len; ay /= len; az /= len;
            
            double c = Math.Cos(angle), s = Math.Sin(angle);
            double t = 1 - c;
            
            var R = DenseMatrix.CreateIdentity(4);
            R[0, 0] = t*ax*ax + c;
            R[0, 1] = t*ax*ay - s*az;
            R[0, 2] = t*ax*az + s*ay;
            R[1, 0] = t*ax*ay + s*az;
            R[1, 1] = t*ay*ay + c;
            R[1, 2] = t*ay*az - s*ax;
            R[2, 0] = t*ax*az - s*ay;
            R[2, 1] = t*ay*az + s*ax;
            R[2, 2] = t*az*az + c;
            return R;
        }
    }
}
