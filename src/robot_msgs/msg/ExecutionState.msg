# ExecutionState - Controller execution status
# Direction: Controller → ROS (via Hardware Bridge)
# Authority: Controller is the ONLY source of this truth
#
# When state == FAULT, upstream nodes (MoveIt, UI) MUST:
#   - Suspend trajectory planning
#   - Suspend command generation
#   - New trajectories will be REJECTED until fault is cleared

# Current execution state
uint8 state
uint8 STATE_IDLE = 0
uint8 STATE_EXECUTING = 1
uint8 STATE_PAUSED = 2
uint8 STATE_FAULT = 3
uint8 STATE_COMM_LOSS = 4

# Trajectory completion progress (0.0 - 1.0)
float32 progress

# Fault information (valid when state == FAULT)
# 0 = OK, non-zero = specific fault code
uint32 fault_code
string fault_message

# Fault code definitions (for reference)
# 1xxx = Critical faults (immediate stop)
#   1001 = E-STOP triggered
#   1002 = Overcurrent detected
#   1003 = Watchdog timeout
# 2xxx = Major faults (controlled stop)
#   2001 = Joint limit exceeded
#   2002 = Encoder error
#   2003 = Overtemperature
# 3xxx = Minor faults (logged only)
#   3001 = Communication warning
#   3002 = Temperature warning
#   3003 = Voltage warning
