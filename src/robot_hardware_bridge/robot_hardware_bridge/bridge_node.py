"""
Hardware Bridge Node

AUTHORITY: RELAY ONLY - NO CONTROL AUTHORITY

This node acts as a protocol gateway between ROS 2 and the RT Controller.
It performs SCHEMA validation only; all semantic validation is done by Controller.

Responsibilities:
- Subscribe to trajectory commands from MoveIt
- Validate message schema (types, field presence, array lengths)
- Serialize and transmit to Controller (or Gazebo backend in SIM)
- Receive joint states from Controller and publish to ROS
- Receive TrajectoryAck and ExecutionState from Controller

Explicit Non-Responsibilities:
- Joint limit checking (Controller does this)
- Velocity bound checking (Controller does this)
- Safety decisions (Controller does this)
- Motor control (Controller does this)
"""

import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from robot_msgs.msg import TrajectoryAck, ExecutionState

# Import backends
from .sim_backend import SimulationBackend
from .real_backend import RealControllerBackend


class HardwareBridgeNode(Node):
    """
    Hardware Bridge Node - RELAY ONLY
    
    This node relays commands between ROS and the Controller.
    It has NO authority over motor control or safety.
    """
    
    # Expected joint names (must match URDF)
    JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
    
    def __init__(self):
        super().__init__('hardware_bridge')
        
        # Declare parameters
        self.declare_parameter('robot_mode', 'SIM')
        self.declare_parameter('controller_type', 'FAKE')
        self.declare_parameter('controller_ip', '192.168.1.100')
        self.declare_parameter('controller_port', 5000)
        self.declare_parameter('controller_proto', 'TCP')
        
        # Get parameters
        self.robot_mode = self.get_parameter('robot_mode').get_parameter_value().string_value
        self.controller_type = self.get_parameter('controller_type').get_parameter_value().string_value
        self.controller_ip = self.get_parameter('controller_ip').get_parameter_value().string_value
        self.controller_port = self.get_parameter('controller_port').get_parameter_value().integer_value
        self.controller_proto = self.get_parameter('controller_proto').get_parameter_value().string_value
        
        # Override from environment if present
        self.robot_mode = os.environ.get('ROBOT_MODE', self.robot_mode)
        self.controller_type = os.environ.get('CONTROLLER_TYPE', self.controller_type)
        self.controller_ip = os.environ.get('CONTROLLER_IP', self.controller_ip)
        self.controller_port = int(os.environ.get('CONTROLLER_PORT', str(self.controller_port)))
        self.controller_proto = os.environ.get('CONTROLLER_PROTO', self.controller_proto)
        
        self.get_logger().info(f'Hardware Bridge starting in {self.robot_mode} mode')
        self.get_logger().info(f'Controller type: {self.controller_type}')
        
        # Initialize appropriate backend
        if self.robot_mode == 'SIM' or self.controller_type == 'FAKE' or self.controller_type == 'GAZEBO':
            self.get_logger().info(f'Using Simulation Backend ({self.controller_type} Mode)')
            self.backend = SimulationBackend(self, self.controller_type)
        else:
            self.get_logger().info(f'Using Real Controller Backend at {self.controller_ip}:{self.controller_port}')
            self.backend = RealControllerBackend(
                self,
                ip=self.controller_ip,
                port=self.controller_port,
                protocol=self.controller_proto
            )
        
        # QoS for reliable trajectory delivery
        trajectory_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # QoS for best-effort joint state streaming
        joint_state_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # === SUBSCRIBERS (Commands from ROS) ===
        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            '/planned_trajectory',
            self.trajectory_callback,
            trajectory_qos
        )
        
        # === PUBLISHERS (State to ROS) ===
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            joint_state_qos
        )
        
        self.trajectory_ack_pub = self.create_publisher(
            TrajectoryAck,
            '/trajectory_ack',
            trajectory_qos
        )
        
        self.execution_state_pub = self.create_publisher(
            ExecutionState,
            '/execution_state',
            trajectory_qos
        )
        
        # Start backend communication
        self.backend.start()
        
        # Timer for state publishing
        self.create_timer(0.01, self.publish_state_callback)  # 100 Hz
        
        self.get_logger().info('Hardware Bridge initialized')
    
    def trajectory_callback(self, msg: JointTrajectory):
        """
        Handle incoming trajectory from MoveIt.
        
        Performs SCHEMA validation only:
        - Check message type
        - Check joint names match expected
        - Check points array is not empty
        - Check array lengths are consistent
        
        Does NOT perform semantic validation (Controller does that):
        - Joint limits
        - Velocity bounds
        - Physics constraints
        """
        self.get_logger().info(f'Received trajectory with {len(msg.points)} points')
        
        # === SCHEMA VALIDATION ONLY ===
        
        # Validate joint names match expected
        if len(msg.joint_names) != len(self.JOINT_NAMES):
            self.get_logger().error(f'Schema error: Expected {len(self.JOINT_NAMES)} joints, got {len(msg.joint_names)}')
            self._publish_rejection(msg, 'SCHEMA_ERROR_JOINT_COUNT')
            return
        
        for i, name in enumerate(msg.joint_names):
            if name != self.JOINT_NAMES[i]:
                self.get_logger().error(f'Schema error: Expected joint {self.JOINT_NAMES[i]}, got {name}')
                self._publish_rejection(msg, f'SCHEMA_ERROR_JOINT_NAME_{i}')
                return
        
        # Validate points array is not empty
        if len(msg.points) == 0:
            self.get_logger().error('Schema error: Empty points array')
            self._publish_rejection(msg, 'SCHEMA_ERROR_EMPTY_POINTS')
            return
        
        # Validate each point has correct array lengths
        for i, point in enumerate(msg.points):
            if len(point.positions) != len(self.JOINT_NAMES):
                self.get_logger().error(f'Schema error: Point {i} has {len(point.positions)} positions, expected {len(self.JOINT_NAMES)}')
                self._publish_rejection(msg, f'SCHEMA_ERROR_POSITIONS_LENGTH_{i}')
                return
        
        # Schema validation passed - relay to backend
        # NOTE: Semantic validation (limits, velocities) is done by Controller
        self.get_logger().debug('Schema validation passed, relaying to backend')
        self.backend.send_trajectory(msg)
    
    def _publish_rejection(self, msg: JointTrajectory, reason: str):
        """Publish trajectory rejection due to schema error."""
        ack = TrajectoryAck()
        ack.trajectory_id = msg.header.stamp
        ack.status = TrajectoryAck.STATUS_REJECTED
        ack.reject_reason = reason
        self.trajectory_ack_pub.publish(ack)
    
    def publish_state_callback(self):
        """Publish current state from backend to ROS."""
        # Get joint state from backend
        joint_state = self.backend.get_joint_state()
        if joint_state is not None:
            self.joint_state_pub.publish(joint_state)
        
        # Get execution state from backend
        exec_state = self.backend.get_execution_state()
        if exec_state is not None:
            self.execution_state_pub.publish(exec_state)
        
        # Get any pending trajectory acks
        ack = self.backend.get_trajectory_ack()
        if ack is not None:
            self.trajectory_ack_pub.publish(ack)
    
    def destroy_node(self):
        """Clean shutdown."""
        self.get_logger().info('Shutting down Hardware Bridge')
        self.backend.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HardwareBridgeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
