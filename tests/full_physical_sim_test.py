#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, Constraints, JointConstraint, RobotState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import time
import sys
import math

LOG_FILE = "/ros_ws/logs/final_system_validation.log"

def log(component, status, detail):
    timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
    entry = f"[{timestamp}] [{component}] [{status}] {detail}"
    print(entry)
    with open(LOG_FILE, "a") as f:
        f.write(entry + "\n")

class FullPhysicalSimTest(Node):
    def __init__(self):
        super().__init__('full_physical_sim_test')
        
        # Clients
        self.move_group_client = ActionClient(self, MoveGroup, 'move_action')
        self.compute_ik_client = self.create_client(GetPositionIK, 'compute_ik')
        
        # Subscriptions
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)
        self.gazebo_joint_sub = self.create_subscription(JointState, '/gazebo/joint_states', self.repo_gazebo_cb, 10)
        
        self.latest_joint_state = None
        self.latest_gazebo_state = None
        
        self.group_name = "robot_arm" # Must match SRDF
        
        log("INIT", "PASS", "Test Node Initialized. Log file: " + LOG_FILE)

    def joint_state_cb(self, msg):
        self.latest_joint_state = msg

    def repo_gazebo_cb(self, msg):
        self.latest_gazebo_state = msg

    def wait_for_services(self):
        log("SETUP", "INFO", "Waiting for MoveGroup and IK services...")
        if not self.move_group_client.wait_for_server(timeout_sec=10.0):
            log("SETUP", "FAIL", "MoveGroup action server timeout")
            return False
        if not self.compute_ik_client.wait_for_service(timeout_sec=10.0):
            log("SETUP", "FAIL", "Compute IK service timeout")
            return False
        return True

    def test_ik(self):
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.group_name
        req.ik_request.robot_state.is_diff = True
        
        # Target: Forward and up slightly
        ps = PoseStamped()
        ps.header.frame_id = "base_link"
        ps.pose.position.x = 0.4
        ps.pose.position.y = 0.0
        ps.pose.position.z = 0.6
        ps.pose.orientation.w = 1.0
        
        req.ik_request.pose_stamped = ps
        req.ik_request.avoid_collisions = True
        
        future = self.compute_ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        
        if response.error_code.val == 1:
            log("IK", "PASS", "IK Solution Found for target pose")
            return True
        else:
            log("IK", "FAIL", f"IK Failed with error code: {response.error_code.val}")
            return False

    def test_planning_and_execution(self):
        # Plan to a joint target (simulating Pick approach)
        goal = MoveGroup.Goal()
        goal.request.group_name = self.group_name
        
        # Target: Joint 2 @ 45 deg, Joint 3 @ -45 deg
        c = Constraints()
        
        jc1 = JointConstraint()
        jc1.joint_name = "joint_2"
        jc1.position = 0.785
        jc1.tolerance_above = 0.01; jc1.tolerance_below = 0.01
        c.joint_constraints.append(jc1)
        
        jc2 = JointConstraint()
        jc2.joint_name = "joint_3"
        jc2.position = -0.785
        jc2.tolerance_above = 0.01; jc2.tolerance_below = 0.01
        c.joint_constraints.append(jc2)
        
        goal.request.goal_constraints.append(c)
        goal.planning_options.plan_only = False # EXECUTE!
        
        log("PLANNER", "INFO", "Sending Plan+Execute Goal...")
        
        # Capture state before
        start_js = self.latest_joint_state
        
        future = self.move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        
        if not handle.accepted:
            log("PLANNER", "FAIL", "Goal Rejected by MoveGroup")
            return False
            
        res_future = handle.get_result_async()
        
        # Wait for execution (monitor feedback loop)
        log("EXECUTION", "INFO", "Waiting for completion...")
        for i in range(10): # Wait up to 10 seconds checking motion
            rclpy.spin_once(self, timeout_sec=1.0)
            if self.latest_joint_state:
                # Check if moving
                 pass
        
        rclpy.spin_until_future_complete(self, res_future)
        result = res_future.result().result
        
        if result.error_code.val == 1:
            log("PLANNER", "PASS", "Plan Execution Reported Success")
            
            # Verify Physical Motion via Feedback
            # Check if Joint 2 makes it close to 0.785
            current_j2 = 0.0
            found = False
            if self.latest_joint_state:
                 try:
                    idx = self.latest_joint_state.name.index("joint_2")
                    current_j2 = self.latest_joint_state.position[idx]
                    found = True
                 except ValueError:
                    pass
            
            if found and abs(current_j2 - 0.785) < 0.1:
                log("EXECUTION", "PASS", f"Robot physically reached target (Joint 2 ~ {current_j2:.2f})")
                return True
            else:
                log("EXECUTION", "WARNING", f"Robot reported success but joint state {current_j2:.2f} != 0.785. Simulation might be lagging or disconnected.")
                return False # Strict fail? strict.
        else:
            log("EXECUTION", "FAIL", f"Execution failed code: {result.error_code.val}")
            return False

    def test_authority_separation(self):
        # Verify /gazebo/joint_states exists (from Gazebo)
        # Verify /joint_states exists (from Bridge)
        # In this simple node we just check if we received messages on both
        
        if self.latest_gazebo_state is not None:
             log("AUTHORITY", "PASS", "/gazebo/joint_states detected (Gazebo active)")
        else:
             log("AUTHORITY", "FAIL", "No data on /gazebo/joint_states")
             return False

        if self.latest_joint_state is not None:
             log("AUTHORITY", "PASS", "/joint_states detected (Bridge active)")
        else:
             log("AUTHORITY", "FAIL", "No data on /joint_states")
             return False
             
        # Ideally check that /joint_states has 'name' fields matching our URDF and /gazebo/joint_states might have extra info or different order
        return True

def main(args=None):
    rclpy.init(args=args)
    node = FullPhysicalSimTest()
    
    try:
        if not node.wait_for_services():
            sys.exit(1)
            
        if not node.test_ik():
            sys.exit(1)
            
        if not node.test_authority_separation():
             sys.exit(1)

        if not node.test_planning_and_execution():
             sys.exit(1)
             
        log("FINAL", "PASS", "All Systems Verified")
        
    except Exception as e:
        log("FATAL", "FAIL", str(e))
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
