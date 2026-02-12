import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
import math
import time

class AccuracyAnalysisNode(Node):
    def __init__(self):
        super().__init__('accuracy_analysis_node')
        
        # Subscriptions
        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)
        self.create_subscription(JointTrajectory, '/planned_trajectory', self.trajectory_cb, 10)
        
        # State
        self.latest_state = {}
        self.latest_plan = None
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        
        self.get_logger().info("Accuracy Analysis Node Started - Monitoring /planned_trajectory vs /joint_states")

    def joint_state_cb(self, msg):
        for i, name in enumerate(msg.name):
            if name in self.joint_names:
                try:
                    self.latest_state[name] = msg.position[i]
                except IndexError:
                    pass
        
        self.compute_error()

    def trajectory_cb(self, msg):
        self.latest_plan = msg
        self.get_logger().info(f"Received new plan with {len(msg.points)} points")

    def compute_error(self):
        if not self.latest_plan or not self.latest_state:
            return

        # Simple logic: Compare current state to the LAST point of the plan (Goal)
        # Ideally, we would track time-based error, but for static accuracy check, goal deviation is key.
        
        last_point = self.latest_plan.points[-1]
        
        error_sum = 0.0
        details = []
        
        for i, name in enumerate(self.latest_plan.joint_names):
            if name in self.latest_state:
                target = last_point.positions[i]
                actual = self.latest_state[name]
                diff = target - actual
                error_sum += diff * diff
                # details.append(f"{name}: err={diff:.4f}")
        
        rmse = math.sqrt(error_sum / len(self.latest_plan.joint_names))
        
        # Log if error is significant (e.g. > 1 degree ~ 0.017 rad)
        if rmse > 0.01:
             self.get_logger().info(f"TRACKING ERROR | RMSE: {rmse:.4f} rad")
        else:
             # self.get_logger().debug(f"Tracking Good | RMSE: {rmse:.4f}")
             pass

def main(args=None):
    rclpy.init(args=args)
    node = AccuracyAnalysisNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
