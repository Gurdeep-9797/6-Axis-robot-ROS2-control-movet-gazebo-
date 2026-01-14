# TrajectoryAck - Controller acknowledgment of trajectory command
# Direction: Controller → ROS (via Hardware Bridge)
# 
# This message is sent immediately upon trajectory receipt and validation.
# The Controller performs ALL semantic validation (joint limits, velocities, physics).
# The Hardware Bridge performs schema validation only.

# Echoed from command (header.stamp or sequence number)
builtin_interfaces/Time trajectory_id

# Acknowledgment status
# ACCEPTED = 0: Trajectory passed validation, will execute
# REJECTED = 1: Trajectory failed validation, discarded
# QUEUED = 2: Trajectory accepted but waiting for current execution
uint8 status
uint8 STATUS_ACCEPTED = 0
uint8 STATUS_REJECTED = 1
uint8 STATUS_QUEUED = 2

# Rejection reason (if status == REJECTED)
# Examples: "JOINT_LIMIT_EXCEEDED", "VELOCITY_TOO_HIGH", "BUFFER_FULL"
string reject_reason
