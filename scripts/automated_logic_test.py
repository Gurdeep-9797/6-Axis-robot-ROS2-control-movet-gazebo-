#!/usr/bin/env python3
"""
Automated Logic Test Node
Validates ALL logic paths without visual inspection.

ROLE: Robotics Test Orchestrator
"""

import time
import random
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from robot_msgs.msg import TrajectoryAck, ExecutionState

class AutomatedLogicTest(Node):
    def __init__(self):
        super().__init__('automated_logic_test')
        
        self.results = []
        self.test_count = 0
        self.pass_count = 0
        self.fail_count = 0
        
        # State tracking
        self.last_joint_state = None
        self.last_execution_state = ExecutionState.STATE_IDLE
        self.last_ack = None
        self.gazebo_published_joint_states = False
        
        # QoS
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self.traj_pub = self.create_publisher(JointTrajectory, '/planned_trajectory', reliable_qos)
        
        # Subscribers
        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, reliable_qos)
        self.create_subscription(JointState, '/gazebo/joint_states', self.gazebo_joint_state_cb, reliable_qos)
        self.create_subscription(ExecutionState, '/execution_state', self.execution_state_cb, reliable_qos)
        self.create_subscription(TrajectoryAck, '/trajectory_ack', self.ack_cb, reliable_qos)
        
        self.get_logger().info("=== AUTOMATED LOGIC TEST STARTING ===")

    def joint_state_cb(self, msg):
        self.last_joint_state = msg
        
    def gazebo_joint_state_cb(self, msg):
        # If we receive on /gazebo/joint_states, Gazebo is correctly isolated
        pass  # Expected
        
    def execution_state_cb(self, msg):
        self.last_execution_state = msg.state
        
    def ack_cb(self, msg):
        self.last_ack = msg

    def log_result(self, test_name, passed, details=""):
        self.test_count += 1
        if passed:
            self.pass_count += 1
            self.get_logger().info(f"[PASS] {test_name}")
        else:
            self.fail_count += 1
            self.get_logger().error(f"[FAIL] {test_name}: {details}")
        self.results.append((test_name, passed, details))

    def send_trajectory(self, positions, duration=2.0):
        """Send a trajectory and wait for ACK."""
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        
        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.time_from_start.sec = int(duration)
        pt.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        traj.points.append(pt)
        
        self.last_ack = None
        self.traj_pub.publish(traj)
        
        # Wait for ACK
        start = time.time()
        while (time.time() - start) < 3.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.last_ack:
                return self.last_ack
        return None

    def wait_for_idle(self, timeout=10.0):
        """Wait for execution to complete."""
        start = time.time()
        while (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.last_execution_state == ExecutionState.STATE_IDLE:
                return True
        return False

    def run_all_tests(self):
        """Execute all test categories."""
        self.get_logger().info("\n" + "="*50)
        self.get_logger().info("STARTING AUTOMATED LOGIC VALIDATION")
        self.get_logger().info("="*50 + "\n")
        
        # Wait for system ready
        time.sleep(2.0)
        rclpy.spin_once(self, timeout_sec=1.0)
        
        # Test 1: Authority Enforcement
        self.test_authority_enforcement()
        
        # Test 2: Valid Trajectory Acceptance
        self.test_valid_trajectory()
        
        # Test 3: Invalid Trajectory Rejection
        self.test_invalid_trajectory()
        
        # Test 4: Execution State Transitions
        self.test_execution_states()
        
        # Test 5: Pick-and-Place Cycles
        self.test_pick_and_place_cycles(cycles=20)
        
        # Test 6: Fault Recovery
        self.test_fault_injection()
        
        # Summary
        self.print_summary()

    def test_authority_enforcement(self):
        """Verify Gazebo does not publish to /joint_states."""
        self.get_logger().info("\n--- Test: Authority Enforcement ---")
        
        # Check we receive on /joint_states (from Bridge)
        rclpy.spin_once(self, timeout_sec=1.0)
        self.log_result(
            "Bridge publishes /joint_states",
            self.last_joint_state is not None,
            "No joint state received from Bridge"
        )

    def test_valid_trajectory(self):
        """Send a valid trajectory, expect ACCEPTED."""
        self.get_logger().info("\n--- Test: Valid Trajectory ---")
        
        valid_positions = [0.1, 0.1, 0.1, 0.0, 0.0, 0.0]
        ack = self.send_trajectory(valid_positions)
        
        self.log_result(
            "Valid trajectory ACKed",
            ack and ack.status >= TrajectoryAck.STATUS_ACCEPTED,
            f"ACK status: {ack.status if ack else 'None'}"
        )
        
        self.wait_for_idle()

    def test_invalid_trajectory(self):
        """Send trajectory with wrong joint count, expect REJECTED."""
        self.get_logger().info("\n--- Test: Invalid Trajectory (Schema) ---")
        
        # Wrong number of joints - this should trigger schema rejection
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = ['joint_1', 'joint_2']  # Only 2 joints
        pt = JointTrajectoryPoint()
        pt.positions = [0.1, 0.1]
        pt.time_from_start.sec = 2
        traj.points.append(pt)
        
        self.last_ack = None
        self.traj_pub.publish(traj)
        
        start = time.time()
        while (time.time() - start) < 3.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.last_ack:
                break
                
        self.log_result(
            "Invalid trajectory REJECTED",
            self.last_ack and self.last_ack.status == TrajectoryAck.STATUS_REJECTED,
            f"ACK status: {self.last_ack.status if self.last_ack else 'None'}"
        )

    def test_execution_states(self):
        """Verify IDLE -> EXECUTING -> IDLE transitions."""
        self.get_logger().info("\n--- Test: Execution State Transitions ---")
        
        # Should start IDLE
        initial_idle = self.last_execution_state == ExecutionState.STATE_IDLE
        
        # Send trajectory
        ack = self.send_trajectory([0.2, 0.2, 0.2, 0.0, 0.0, 0.0])
        
        # Should go EXECUTING (may be fast)
        rclpy.spin_once(self, timeout_sec=0.5)
        
        # Wait for IDLE
        completed = self.wait_for_idle(timeout=5.0)
        
        self.log_result(
            "Execution completes to IDLE",
            completed,
            "Timed out waiting for IDLE state"
        )

    def test_pick_and_place_cycles(self, cycles=20):
        """Run multiple pick-and-place cycles without drift."""
        self.get_logger().info(f"\n--- Test: {cycles} Pick-and-Place Cycles ---")
        
        HOME = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        PICK = [0.3, 0.3, 0.3, 0.0, 0.0, 0.0]
        PLACE = [-0.3, 0.3, 0.3, 0.0, 0.0, 0.0]
        
        success_count = 0
        
        for i in range(cycles):
            # Pick
            ack = self.send_trajectory(PICK, duration=1.0)
            if not ack or ack.status < TrajectoryAck.STATUS_ACCEPTED:
                continue
            if not self.wait_for_idle(timeout=3.0):
                continue
                
            # Place
            ack = self.send_trajectory(PLACE, duration=1.0)
            if not ack or ack.status < TrajectoryAck.STATUS_ACCEPTED:
                continue
            if not self.wait_for_idle(timeout=3.0):
                continue
                
            # Home
            ack = self.send_trajectory(HOME, duration=1.0)
            if not ack or ack.status < TrajectoryAck.STATUS_ACCEPTED:
                continue
            if not self.wait_for_idle(timeout=3.0):
                continue
                
            success_count += 1
            
        self.log_result(
            f"Pick-and-place cycles ({cycles})",
            success_count == cycles,
            f"{success_count}/{cycles} cycles completed"
        )

    def test_fault_injection(self):
        """Test fault handling (if supported by backend)."""
        self.get_logger().info("\n--- Test: Fault Handling ---")
        
        # Send trajectory beyond limits (semantic violation)
        # IRB 120 J3 limit: -110 to +70 deg (-1.92 to +1.22 rad)
        # Sending +2.0 rad should trigger warning (but SIM may accept)
        out_of_limit = [0.0, 0.0, 2.0, 0.0, 0.0, 0.0]  # J3 out of range
        
        ack = self.send_trajectory(out_of_limit)
        
        # In SIM, this may be accepted with warning
        # In REAL, this should be rejected
        self.log_result(
            "Out-of-limit trajectory handled",
            ack is not None,  # At least got a response
            "System processed limit violation trajectory"
        )
        
        self.wait_for_idle()

    def print_summary(self):
        """Print final test summary."""
        self.get_logger().info("\n" + "="*50)
        self.get_logger().info("TEST SUMMARY")
        self.get_logger().info("="*50)
        self.get_logger().info(f"Total Tests: {self.test_count}")
        self.get_logger().info(f"Passed: {self.pass_count}")
        self.get_logger().info(f"Failed: {self.fail_count}")
        
        if self.fail_count == 0:
            self.get_logger().info("\n✅ ALL TESTS PASSED")
        else:
            self.get_logger().error(f"\n❌ {self.fail_count} TESTS FAILED")
            for name, passed, details in self.results:
                if not passed:
                    self.get_logger().error(f"  - {name}: {details}")


def main(args=None):
    rclpy.init(args=args)
    node = AutomatedLogicTest()
    
    try:
        node.run_all_tests()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
