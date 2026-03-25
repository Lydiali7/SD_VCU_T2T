#pragma once
#include <immintrin.h>
#include <iostream>
#include <cstdint>
#include <vector>

// T2T Protocol Packet Structure
struct RawT2TPacket {
    uint32_t sender_id; 
    uint32_t header;    
    uint32_t seq;       
    uint64_t payload;   // Bit 0-15: Speed, Bit 16-47: Absolute Position,48-63: Acceleration
    uint32_t crc;       
};

// Physical State of a Train Node
struct TrainState {
    int id;
    double pos;         // Absolute position (m)
    double vel;         // Velocity (m/s)
    double accel;       // Acceleration (m/s^2)
    float cmd_force;    // VCU commanded force (N)
    float mass = 450000.0f; 
};

enum class TrainType { LOCO_HAULED, EMU_DISTRIBUTED };

// SIMD aligned sensor data array
struct alignas(64) SensorData {
    float distances[16]; //0pos, 1vel, 2accel
};

class PerceptionEngine {
public:
    // Unpack bits into physical values
    static bool fast_unpack(const RawT2TPacket& raw, SensorData& out, uint32_t& last_seq);
    
    // Dynamic safety envelope calculation
    static float calculate_safe_dist(float v_self, float v_front, TrainType type);
    
    // Legacy 2oo2 hardware redundancy check
    static bool is_system_safe(const SensorData& pathA, const SensorData& pathB);
};