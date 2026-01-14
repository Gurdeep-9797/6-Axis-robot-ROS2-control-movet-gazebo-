#!/usr/bin/env python3
"""
CI Test: IK Service Validation
Exits non-zero on failure.
"""
import sys
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped

class IKTest(Node):
    def __init__(self):
        super().__init__('test_ik')
        self.client = self.create_client(GetPositionIK, '/compute_ik')
        
    def run(self):
        # Wait for service
        if not self.client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("FAIL: /compute_ik service not available")
            return 1
        self.get_logger().info("PASS: /compute_ik service exists")
        
        # Test reachable pose
        req = GetPositionIK.Request()
        req.ik_request.group_name = "robot_arm"
        req.ik_request.pose_stamped.header.frame_id = "base_link"
        req.ik_request.pose_stamped.pose.position.x = 0.3
        req.ik_request.pose_stamped.pose.position.y = 0.0
        req.ik_request.pose_stamped.pose.position.z = 0.4
        req.ik_request.pose_stamped.pose.orientation.w = 1.0
        
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is None:
            self.get_logger().error("FAIL: IK call failed")
            return 1
            
        if future.result().error_code.val == 1:  # SUCCESS
            self.get_logger().info("PASS: IK solved reachable pose")
        else:
            self.get_logger().error(f"FAIL: IK failed (code: {future.result().error_code.val})")
            return 1
            
        # Test unreachable pose
        req.ik_request.pose_stamped.pose.position.x = 10.0
        req.ik_request.pose_stamped.pose.position.y = 10.0
        req.ik_request.pose_stamped.pose.position.z = 10.0
        
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() and future.result().error_code.val != 1:
            self.get_logger().info("PASS: IK correctly rejected unreachable pose")
        else:
            self.get_logger().error("FAIL: IK accepted unreachable pose")
            return 1
            
        return 0

def main():
    rclpy.init()
    node = IKTest()
    result = node.run()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(result)

if __name__ == '__main__':
    main()
