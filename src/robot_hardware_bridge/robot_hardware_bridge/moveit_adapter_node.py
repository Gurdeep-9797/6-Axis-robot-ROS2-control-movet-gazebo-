#!/usr/bin/env python3
"""
MoveIt Adapter Node
Translates MoveIt Action Goals (FollowJointTrajectory) to Bridge Topics.

AUTHORITY: INTERFACE ADAPTER (No Control Authority)
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from robot_msgs.msg import TrajectoryAck, ExecutionState

class MoveItAdapterNode(Node):
    def __init__(self):
        super().__init__('moveit_adapter')
        
        self.get_logger().info("MoveIt Adapter Starting...")
        
        # State
        self.last_ack = None
        self.current_execution_state = ExecutionState.STATE_IDLE
        self.current_joint_state = None
        
        # QoS
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers / Subscribers
        self.traj_pub = self.create_publisher(JointTrajectory, '/planned_trajectory', reliable_qos)
        
        self.create_subscription(TrajectoryAck, '/trajectory_ack', self.ack_callback, reliable_qos)
        self.create_subscription(ExecutionState, '/execution_state', self.execution_state_callback, reliable_qos)
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, reliable_qos)
        
        # Action Server
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/robot_arm_controller/follow_joint_trajectory',
            self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        
        self.get_logger().info("MoveIt Adapter Ready (Action Server Active)")

    def goal_callback(self, goal_request):
        self.get_logger().info("Received new Action Goal")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Received Cancel Request")
        return CancelResponse.ACCEPT

    def ack_callback(self, msg):
        self.last_ack = msg

    def execution_state_callback(self, msg):
        self.current_execution_state = msg.state

    def joint_state_callback(self, msg):
        self.current_joint_state = msg

    async def execute_callback(self, goal_handle):
        self.get_logger().info("Executing Goal...")
        goal = goal_handle.request
        traj = goal.trajectory
        
        # 1. Publish to Bridge
        # Ensure header stamp is now (ID for ACK)
        traj.header.stamp = self.get_clock().now().to_msg()
        target_timestamp = traj.header.stamp
        
        self.last_ack = None # Reset ACK
        self.traj_pub.publish(traj)
        self.get_logger().info(f"Forwarded Trajectory with {len(traj.points)} points")
        
        # 2. Wait for ACK
        # Timeout 2 seconds
        start_wait = time.time()
        accepted = False
        while (time.time() - start_wait) < 2.0:
            if self.last_ack and self.last_ack.trajectory_id.sec == target_timestamp.sec:
                # Basic check, nanosec might differ slightly if float conversion issues?
                # Ideally check closer. For now, assume fresh ack is ours.
                if self.last_ack.status >= TrajectoryAck.STATUS_ACCEPTED:
                    accepted = True
                    break
                else:
                    self.get_logger().error(f"Goal REJECTED: {self.last_ack.reject_reason}")
                    goal_handle.abort()
                    result = FollowJointTrajectory.Result()
                    result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                    result.error_string = self.last_ack.reject_reason
                    return result
            time.sleep(0.05)
            
        if not accepted:
            self.get_logger().error("Timed out waiting for ACK")
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        self.get_logger().info("Goal ACCEPTED by Bridge. Monitoring Execution...")
        
        # 3. Monitor Execution
        # We wait until state goes IDLE or FAULT
        # Also send feedback
        loop_rate = self.create_rate(10.0) # 10Hz
        
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Goal Canceled")
                goal_handle.canceled()
                return FollowJointTrajectory.Result()
            
            # Check Faults
            if self.current_execution_state == ExecutionState.STATE_FAULT:
                self.get_logger().error("Bridge reported FAULT")
                goal_handle.abort()
                result = FollowJointTrajectory.Result()
                result.error_code = -1 # System Error
                result.error_string = "Bridge reported FAULT"
                return result
                
            # Check Success
            # Logic: If we are IDLE and we were previously Executing? 
            # Simplified: If Adapter sees IDLE, and we just started, wait for EXECUTING?
            # State machine: Just Sent -> Wait for EXECUTING -> Wait for IDLE.
            # But Bridge might complete very fast.
            # Better check: ExecutionState progress.
            
            # Send Feedback
            if self.current_joint_state:
                feedback = FollowJointTrajectory.Feedback()
                feedback.header.stamp = self.get_clock().now().to_msg()
                feedback.joint_names = self.current_joint_state.name
                feedback.actual.positions = self.current_joint_state.position
                goal_handle.publish_feedback(feedback)
            
            # Done Condition
            if self.current_execution_state == ExecutionState.STATE_IDLE:
                # If we were just accepted, maybe we haven't started EXECUTING yet?
                # The Sim Backend sets EXECUTING immediately on receive. 
                # So if we are IDLE, we are done (or haven't started, but we waited for ACK).
                # To be safe, maybe check progress?
                # Assuming Progress = 1.0 means done.
                # Just succeed.
                self.get_logger().info("Execution Complete (IDLE)")
                goal_handle.succeed()
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                return result
                
            loop_rate.sleep()

def main(args=None):
    rclpy.init(args=args)
    node = MoveItAdapterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
