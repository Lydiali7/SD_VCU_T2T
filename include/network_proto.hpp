#pragma once
#include <cstdint>

#define MULTICAST_GROUP "239.0.0.1"
#define MULTICAST_PORT 8888
#define VCU_REPORT_PORT 9000

#pragma pack(push, 1)
struct RawT2TPacket {
    uint32_t sender_id; 
    uint32_t header;    
    uint32_t seq;       
    uint64_t payload;   
    uint32_t crc;       
};

struct WorldUpdatePacket {
    uint32_t header; 
    double timestamp; 
    uint32_t seq; 
    uint32_t train_id;
    double pos; 
    double vel; 
    double accel; 
    uint32_t status_flag;
};

struct ForceReportPacket {
    uint32_t train_id; 
    float cmd_force; 
    uint32_t status_flag;
};
#pragma pack(pop)