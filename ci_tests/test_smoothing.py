#!/usr/bin/env python3
"""
CI Test: Trajectory Smoothing Validation
Exits non-zero on failure.
"""
import sys
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import Constraints, JointConstraint

class SmoothingTest(Node):
    def __init__(self):
        super().__init__('test_smoothing')
        self.client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        
    def run(self):
        if not self.client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("FAIL: Planning service not available")
            return 1
            
        req = GetMotionPlan.Request()
        req.motion_plan_request.group_name = "robot_arm"
        req.motion_plan_request.num_planning_attempts = 5
        req.motion_plan_request.allowed_planning_time = 5.0
        
        goal = Constraints()
        for i in range(1, 7):
            jc = JointConstraint()
            jc.joint_name = f"joint_{i}"
            jc.position = 0.5
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            goal.joint_constraints.append(jc)
        req.motion_plan_request.goal_constraints.append(goal)
        
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
        
        if future.result() is None:
            self.get_logger().error("FAIL: Planning call failed")
            return 1
            
        response = future.result()
        if response.motion_plan_response.error_code.val != 1:
            self.get_logger().error("FAIL: Planner failed")
            return 1
            
        traj = response.motion_plan_response.trajectory.joint_trajectory
        
        # TEST: time_from_start strictly increasing
        prev_time = 0.0
        for i, pt in enumerate(traj.points):
            current_time = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            if current_time <= prev_time and i > 0:
                self.get_logger().error(f"FAIL: time_from_start not monotonic at point {i}")
                return 1
            prev_time = current_time
        self.get_logger().info("PASS: time_from_start is monotonic")
        
        # TEST: No zero-duration points
        for i, pt in enumerate(traj.points):
            if i > 0:
                t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
                if t == 0.0:
                    self.get_logger().error(f"FAIL: Zero-duration point at {i}")
                    return 1
        self.get_logger().info("PASS: No zero-duration trajectory points")
        
        # TEST: Velocity bounds (if velocities present)
        if traj.points and traj.points[0].velocities:
            max_vel = 0.0
            for pt in traj.points:
                for v in pt.velocities:
                    max_vel = max(max_vel, abs(v))
            if max_vel > 10.0:  # Sanity check
                self.get_logger().error(f"FAIL: Excessive velocity {max_vel}")
                return 1
            self.get_logger().info(f"PASS: Velocities bounded (max: {max_vel:.2f})")
        else:
            self.get_logger().info("INFO: No velocity data in trajectory")
            
        return 0

def main():
    rclpy.init()
    node = SmoothingTest()
    result = node.run()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(result)

if __name__ == '__main__':
    main()
