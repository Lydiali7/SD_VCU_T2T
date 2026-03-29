#pragma once
#include <immintrin.h>
#include <nmmintrin.h>
#include <cstdint>
#include <vector>
#include <algorithm>
#include "network_proto.hpp"

struct TrainState {
    int id;
    double pos;         
    double vel;         
    double accel;       
    float cmd_force;    
    float mass = 5000000.0f; 
};

enum class TrainType { LOCO_HAULED, EMU_DISTRIBUTED, HEAVY_HAUL };

struct alignas(64) SensorData {
    float distances[16]; 
};

class PerceptionEngine {
public:
    static bool fast_unpack(const RawT2TPacket& raw, SensorData& out, uint32_t& last_seq);
    static float calculate_safe_dist(float v_s, float v_f, TrainType type);
    static bool is_system_safe(const SensorData& pathA, const SensorData& pathB);
};