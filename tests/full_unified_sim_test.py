#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
import time
import sys
import threading

LOG_FILE = "/ros_ws/logs/unified_system_validation.log"

def log(stage, status, detail):
    timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
    entry = f"[{timestamp}] [{stage}] [{status}] {detail}"
    print(entry)
    with open(LOG_FILE, "a") as f:
        f.write(entry + "\n")

class UnifiedSimTest(Node):
    def __init__(self):
        super().__init__('unified_sim_test')
        
        # 1. User Input Emulation (MoveGroup Client)
        self.move_group_client = ActionClient(self, MoveGroup, 'move_action')
        
        # 2. Feedback Monitoring
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)
        self.gazebo_joint_sub = self.create_subscription(JointState, '/gazebo/joint_states', self.gazebo_state_cb, 10)
        self.bridge_traj_sub = self.create_subscription(JointTrajectory, '/planned_trajectory', self.bridge_traj_cb, 10)
        
        self.latest_joint_state = None
        self.latest_gazebo_state = None
        self.bridge_traj_count = 0
        
        # Lock for thread safety
        self.lock = threading.Lock()
        
        log("INIT", "PASS", "Test Node Initialized. Log file: " + LOG_FILE)

    def joint_state_cb(self, msg):
        with self.lock:
            self.latest_joint_state = msg

    def gazebo_state_cb(self, msg):
        with self.lock:
            self.latest_gazebo_state = msg

    def bridge_traj_cb(self, msg):
        with self.lock:
            self.bridge_traj_count += 1

    def wait_for_services(self):
        log("SETUP", "INFO", "Waiting for MoveGroup action server...")
        if not self.move_group_client.wait_for_server(timeout_sec=20.0):
            log("SETUP", "FAIL", "MoveGroup action server timeout")
            return False
        return True

    def test_authority_separation(self):
        """
        Verify:
        - /gazebo/joint_states exists
        - /joint_states exists
        - They are NOT the same publisher (implicit check via topic names)
        """
        # Wait for data
        time.sleep(2.0)
        
        with self.lock:
            if self.latest_gazebo_state is None:
                log("AUTHORITY", "FAIL", "No data on /gazebo/joint_states (Gazebo not running or not remapped?)")
                return False
            else:
                log("AUTHORITY", "PASS", "/gazebo/joint_states detected")

            if self.latest_joint_state is None:
                # In Sim mode, Bridge should republish Gazebo state.
                log("AUTHORITY", "FAIL", "No data on /joint_states (Bridge not relaying?)")
                return False
            else:
                log("AUTHORITY", "PASS", "/joint_states detected")
                
        return True

    def test_end_to_end_control(self):
        """
        Send Goal -> MoveIt -> Adapter -> Bridge -> Gazebo -> Bridge -> Feedback
        """
        # Define Goal: Move Joint 2 to 0.5 rad
        goal = MoveGroup.Goal()
        goal.request.group_name = "robot_arm"
        
        c = Constraints()
        jc = JointConstraint()
        jc.joint_name = "joint_2"
        jc.position = 0.5
        jc.tolerance_above = 0.01
        jc.tolerance_below = 0.01
        jc.weight = 1.0
        c.joint_constraints.append(jc)
        goal.request.goal_constraints.append(c)
        
        goal.planning_options.plan_only = False
        
        log("CONTROL", "INFO", "Sending MoveGroup Goal (Joint 2 -> 0.5)...")
        
        future = self.move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        
        if not handle.accepted:
            log("CONTROL", "FAIL", "Goal Rejected by MoveGroup")
            return False
            
        log("CONTROL", "INFO", "Goal Accepted. Waiting for Result...")
        
        res_future = handle.get_result_async()
        
        # Monitor Loop
        start_time = time.time()
        traj_seen = False
        
        while (time.time() - start_time) < 15.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            
            with self.lock:
                if self.bridge_traj_count > 0 and not traj_seen:
                    log("FLOW", "PASS", "Control Flow Verified: Trajectory passed through Bridge")
                    traj_seen = True
            
            if res_future.done():
                break
                
        if not res_future.done():
            log("CONTROL", "FAIL", "Execution Timeout")
            return False
            
        result = res_future.result().result
        if result.error_code.val == 1: # SUCCESS
            log("CONTROL", "PASS", "Execution Succeeded")
            
            # Verify Physical State in Gazebo
            if self.latest_gazebo_state:
                try:
                    idx = self.latest_gazebo_state.name.index("joint_2")
                    pos = self.latest_gazebo_state.position[idx]
                    if abs(pos - 0.5) < 0.1:
                        log("PHYSICS", "PASS", f"Gazebo Robot Actually Moved (J2={pos:.2f})")
                        return True
                    else:
                        log("PHYSICS", "FAIL", f"Gazebo Robot Position Mismatch (J2={pos:.2f} != 0.5)")
                        return False
                except ValueError:
                    log("PHYSICS", "FAIL", "Joint 2 not found in Gazebo state")
                    return False
            return True
        else:
            log("CONTROL", "FAIL", f"Execution Failed: {result.error_code.val}")
            return False

def main(args=None):
    rclpy.init(args=args)
    node = UnifiedSimTest()
    
    try:
        if not node.wait_for_services():
            sys.exit(1)
            
        if not node.test_authority_separation():
            sys.exit(1)
            
        if not node.test_end_to_end_control():
            sys.exit(1)
            
        log("FINAL", "PASS", "Unified System Validation Complete")
        
    except Exception as e:
        log("FATAL", "FAIL", str(e))
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
