import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time
import math

class PipelineVerifier(Node):
    def __init__(self):
        super().__init__('pipeline_verifier')
        self.pub = self.create_publisher(JointTrajectory, '/joint_trajectory_command', 10)
        self.sub = self.create_subscription(JointState, '/joint_states', self.state_callback, 10)
        
        self.initial_state = None
        self.final_state = None
        self.target = [0.1, -0.2, 0.3, -0.1, 0.2, -0.3]
        self.got_message = False

    def state_callback(self, msg):
        self.got_message = True
        if not self.initial_state:
            self.initial_state = list(msg.position[:6])
            self.get_logger().info(f"Got initial state: {self.initial_state}")
        self.final_state = list(msg.position[:6])

    def run_test(self):
        self.get_logger().info("Waiting for initial /joint_states...")
        
        start = time.time()
        while time.time() - start < 3.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.initial_state: break
            
        if not self.got_message:
            self.get_logger().error("FAIL: Did not receive any /joint_states from pseudo_hardware.")
            return False

        self.get_logger().info("Sending trajectory command...")
        
        # Build trajectory
        traj = JointTrajectory()
        traj.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        
        p = JointTrajectoryPoint()
        p.positions = self.target
        p.velocities = [0.0]*6
        p.time_from_start.sec = 2
        
        traj.points.append(p)
        self.pub.publish(traj)
        
        self.get_logger().info("Monitoring pipeline execution for 6 seconds...")
        
        start = time.time()
        while time.time() - start < 6.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            
        # Verify
        error = sum(abs(a - b) for a, b in zip(self.final_state, self.target))
        if error < 0.05:
            self.get_logger().info(f"[SUCCESS] Pipeline Verified! Pseudo-hardware reached target. Error: {error:.4f}")
            return True
        else:
            self.get_logger().error(f"[FAIL] Backend pipeline failed to execute interpolation. Final Error: {error:.4f}")
            return False

def main():
    rclpy.init()
    node = PipelineVerifier()
    success = node.run_test()
    node.destroy_node()
    rclpy.shutdown()
    exit(0 if success else 1)

if __name__ == '__main__':
    main()
