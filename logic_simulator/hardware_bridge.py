"""
Hardware Bridge (Logic Mirror)

Mirrors the logic of src/robot_hardware_bridge/bridge_node.py
but uses the local MessageBus instead of ROS 2.

Role:
- Subscribe to 'planned_trajectory'
- Perform Schema Validation
- Relay to 'controller_command' (simulating the wire)
- Relay feedback topics
"""

import messages
from bus import default_bus

class HardwareBridge:
    JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
    
    def __init__(self):
        self.bus = default_bus
        
        # Subscribe to Task (ROS side)
        self.bus.subscribe('planned_trajectory', self.handle_planned_trajectory)
        
        print("[BRIDGE] Initialized (Relay-Only)")
        
    def handle_planned_trajectory(self, trajectory):
        """Receive sequence (ROS side). Verify Schema. Relay to Wire."""
        print(f"[BRIDGE] Received trajectory. Validating Schema...")
        
        # 1. Schema Validation
        if not self._validate_schema(trajectory):
            # Reject locally if schema fails
            ack = messages.TrajectoryAck(
                trajectory_id=trajectory.header.stamp,
                status=messages.TrajectoryAck.STATUS_REJECTED,
                reject_reason="SCHEMA_VALIDATION_FAILED"
            )
            self.bus.publish('trajectory_ack', ack)
            print("[BRIDGE] Rejected (Schema Failure)")
            return
            
        # 2. Relay to Controller (Wire side)
        # Note: We do NOT check limits here. That is Controller's job.
        print("[BRIDGE] Schema OK. Relaying to Controller...")
        self.bus.publish('controller_command', trajectory)
        
    def _validate_schema(self, trajectory):
        """Check field types and array lengths."""
        # Check joint count
        if len(trajectory.joint_names) != 6:
            return False
            
        # Check names match
        for name in trajectory.joint_names:
            if name not in self.JOINT_NAMES:
                return False
                
        # Check points exist
        if not trajectory.points:
            return False
            
        # Check point structure
        for pt in trajectory.points:
            if len(pt.positions) != 6:
                return False
                
        return True
