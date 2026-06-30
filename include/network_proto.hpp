#pragma once
#include <cstdint>
#include <cstring>

#define MULTICAST_GROUP "239.0.0.1"
#define MULTICAST_PORT 8888
#define VCU_REPORT_PORT 9000

enum class PlatoonState {
    STARTING,        
    CRUISING,        
    DECOUPLING,      
    LEADER_NEW       
};

// =================================================================
// UDP (V2V and World Server Communication)
// =================================================================
#pragma pack(push, 1)

struct RawT2TPacket {
    uint32_t sender_id; 
    uint32_t header;    // 0x55AA55AA
    uint32_t seq;       
    uint64_t global_timestamp_us; // [PTP SYNC] IEEE 1588 Microsecond Timestamp
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
    float current_vel;
    float current_pos;
};

// =================================================================
// Hardware Interface Protocols (MVB / LoRa)
// =================================================================

struct MVB_Hardware_Frame {
    uint8_t head[2];      
    uint8_t cmd_type;     
    uint8_t port_addr;    
    
    uint16_t raw_speed;    
    uint16_t brake_press;  
    uint32_t io_status;    
    
    uint8_t reserved[240]; 
    uint8_t checksum;      
};
/*
struct LoRa_Physical_Packet {
    uint16_t target_addr;  
    uint8_t channel;       
    uint8_t payload[128];  
};
*/

#pragma pack(pop)

// =================================================================
// Sensor Redundancy Protocols (CAN / AVX-512 aligned)
// =================================================================

struct alignas(64) CANPacket {
    uint32_t header;       
    uint16_t source_id;    
    uint16_t seq_num;      
    uint64_t gptp_timestamp_us; // [ADDED] TSN Time Sync for onboard sensors
    float payload[10];          // [MODIFIED] Reduced to 10 to keep exact 64-byte alignment
    uint32_t crc32;        
    uint32_t padding;      
};
struct DualSensorBroadcast {
    uint16_t target_train_id; 
    CANPacket path_a;         
    CANPacket path_b;         
};

class ProtocolConverter {
public:
    static void encode_mvb(MVB_Hardware_Frame& frame, double speed, double pressure) {
        frame.head[0] = 0xFE;
        frame.head[1] = 0xFA;
        frame.cmd_type = 0x05;
        frame.raw_speed = static_cast<uint16_t>(speed * 100.0);
        frame.brake_press = static_cast<uint16_t>(pressure * 10.0);
    }

    static bool validate_mvb(const uint8_t* buffer, size_t len) {
        if (len < sizeof(MVB_Hardware_Frame)) return false;
        return (buffer[0] == 0xFE && buffer[1] == 0xFA);
    }
};