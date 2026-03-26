#pragma once
#include <cstdint>

#define MULTICAST_GROUP "239.0.0.1"
#define MULTICAST_PORT 8888
#define VCU_REPORT_PORT 9000

#pragma pack(push, 1)

// Server -> VCU: Broadcasted Physics State 
struct WorldUpdatePacket {
    uint32_t header;       // 0x55AA55AA
    uint32_t seq;          // Sequence number
    double timestamp;      // Global precise timestamp for latency calculation
    uint32_t train_id;     // ID of the train
    double pos;            // Absolute position (m)
    double vel;            // Velocity (m/s)
    double accel;          // Acceleration (m/s^2)
    uint32_t status_flag;  // 0 = OK, 1 = DEGRADED
    uint32_t crc;          // Checksum
};

// VCU -> Server: Force Command Report
struct ForceReportPacket {
    uint32_t train_id;     // Which VCU is reporting
    float cmd_force;       // The calculated force (N)
    uint32_t status_flag;  // 0 = OK, 1 = DEGRADED
};

#pragma pack(pop)