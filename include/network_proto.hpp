#pragma once
#include <cstdint>

#define MULTICAST_GROUP "239.0.0.1"
#define MULTICAST_PORT 8888
#define VCU_REPORT_PORT 9000 // Server listens here for VCU force reports

// 1. Server -> VCU: Broadcasted Physics State (Replaces RawT2TPacket)
struct WorldUpdatePacket {
    uint32_t header; // 0x55AA55AA
    uint32_t seq;
    uint32_t train_id;
    double pos;
    double vel;
    double accel;
    uint32_t crc; // We can keep your CRC logic!
};

// 2. VCU -> Server: Force Command Report
struct ForceReportPacket {
    uint32_t train_id;
    float cmd_force;
};