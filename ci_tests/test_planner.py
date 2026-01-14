#!/usr/bin/env python3
"""
CI Test: Planner Validation
Exits non-zero on failure.
"""
import sys
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint

class PlannerTest(Node):
    def __init__(self):
        super().__init__('test_planner')
        self.client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        
    def run(self):
        if not self.client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("FAIL: /plan_kinematic_path service not available")
            return 1
        self.get_logger().info("PASS: Planning service exists")
        
        req = GetMotionPlan.Request()
        req.motion_plan_request.group_name = "robot_arm"
        req.motion_plan_request.num_planning_attempts = 5
        req.motion_plan_request.allowed_planning_time = 5.0
        
        # Goal: move to 0.5 rad on all joints
        goal = Constraints()
        for i in range(1, 7):
            jc = JointConstraint()
            jc.joint_name = f"joint_{i}"
            jc.position = 0.3
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
        if response.motion_plan_response.error_code.val == 1:
            traj = response.motion_plan_response.trajectory.joint_trajectory
            if len(traj.points) >= 2:
                self.get_logger().info(f"PASS: Plan returned {len(traj.points)} trajectory points")
            else:
                self.get_logger().error("FAIL: Plan returned <2 points")
                return 1
        else:
            self.get_logger().error(f"FAIL: Planner failed (code: {response.motion_plan_response.error_code.val})")
            return 1
            
        return 0

def main():
    rclpy.init()
    node = PlannerTest()
    result = node.run()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(result)

if __name__ == '__main__':
    main()
