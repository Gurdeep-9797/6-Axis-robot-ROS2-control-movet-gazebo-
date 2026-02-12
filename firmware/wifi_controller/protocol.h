#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

// Message Types
#define MSG_TRAJECTORY          0x01
#define MSG_STOP                0x02
#define MSG_JOINT_STATE         0x10
#define MSG_TRAJECTORY_ACK      0x11
#define MSG_EXECUTION_STATE     0x12

// Trajectory Status
#define STATUS_ACCEPTED         0
#define STATUS_REJECTED         1
#define STATUS_QUEUED           2

// Execution States
#define STATE_IDLE              0
#define STATE_EXECUTING         1
#define STATE_PAUSED            2
#define STATE_FAULT             3
#define STATE_COMM_LOSS         4

// Structures (Packed for binary compatibility)
#pragma pack(push, 1)

struct MsgHeader {
    uint8_t type;
    uint16_t length;
};

struct JointStateMsg {
    uint64_t timestamp_us;
    double position[6];
    double velocity[6];
    double effort[6];
};

struct ExecutionStateMsg {
    uint8_t state;
    float progress;
    uint32_t fault_code;
    uint16_t msg_len;
    // Followed by char msg[]
};

struct TrajectoryAckMsg {
    uint64_t trajectory_id;
    uint8_t status;
    uint16_t reason_len;
    // Followed by char reason[]
};

#pragma pack(pop)

#endif
