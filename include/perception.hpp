#pragma once
#include <immintrin.h>
#include <nmmintrin.h>
#include <cstdint>
#include <vector>
#include <algorithm>
#include <cstring>
#include "network_proto.hpp"

struct TrainState {
    int id;
    double pos;         
    double vel;         
    double accel;       
    float cmd_force;    
    float mass = 5000000.0f; 
    float smooth_brake_pressure = 0.0f; 
    uint16_t lora_target_addr = 0xFFFF;
};

enum class TrainType { LOCO_HAULED, EMU_DISTRIBUTED, HEAVY_HAUL };

struct alignas(64) SensorData {
    float distances[16]; 
};

class SDVCU_Core {
private:
    int consecutive_errors;
    const int ERROR_THRESHOLD = 3; 
    uint16_t last_seq;
    bool is_eb_triggered;

    // 底层硬件指令封装
    bool avx512_payload_match(const CANPacket& a, const CANPacket& b);

public:
    float current_safe_gap;
    float current_vel;

    SDVCU_Core();
    
    // 返回 true 表示数据健康，返回 false 表示触发了制动或外推
    bool process_sensors(const CANPacket& path_a, const CANPacket& path_b, double mass);
    
    bool is_eb() const;
    bool is_atp_braking(float gap) const;
};
class PerceptionEngine {
public:
    static bool fast_unpack(const RawT2TPacket& raw, SensorData& out, uint32_t& last_seq);
    
    static bool hardware_unpack(const MVB_Hardware_Frame& hw, SensorData& out) {
        if (hw.head[0] != 0xFE || hw.head[1] != 0xFA) return false;
        std::memset(&out, 0, sizeof(SensorData));
        
        // [BUG FIX] Map speed to index 1 to match vcu_node's kf_my_vel expectation
        out.distances[0] = 0.0f;                       // Absolute position (not provided by MVB)
        out.distances[1] = hw.raw_speed / 100.0f;      // Velocity
        out.distances[2] = hw.brake_press / 10.0f;     // Brake Cylinder Pressure
        return true;
    }

    static float calculate_safe_dist(float v_s, float v_f, TrainType type);
    static bool is_system_safe(const SensorData& pathA, const SensorData& pathB);
};