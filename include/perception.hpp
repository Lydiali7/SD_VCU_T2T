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

// 统一的物理传感数据结构
struct alignas(64) SensorData {
    float distances[16]; 
};

class SDVCU_Core {
private:
    int consecutive_errors;
    const int ERROR_THRESHOLD = 3; 
    uint16_t last_seq;
    bool is_eb_triggered;

    // 硬件冗余校验 直接对比纯物理内存块
    bool avx512_payload_match(const SensorData& a, const SensorData& b);

public:
    float current_safe_gap;
    float current_vel;

    SDVCU_Core();
    
    // 大脑核心只接收干净的物理量 SensorData
    bool process_sensors(uint16_t seq_num, const SensorData& path_a, const SensorData& path_b);
    
    bool is_eb() const;
    bool is_atp_braking(float gap) const;
};

class PerceptionEngine {
public:
    static bool fast_unpack(const RawT2TPacket& raw, SensorData& out, uint32_t& last_seq);
    
    // 将 MVB 报文翻译为标准 SensorData
    static bool hardware_unpack(const MVB_Hardware_Frame& hw, SensorData& out) {
        if (hw.head[0] != 0xFE || hw.head[1] != 0xFA) return false;
        std::memset(&out, 0, sizeof(SensorData));
        
        out.distances[0] = 0.0f;                       // Gap (MVB暂无)
        out.distances[1] = hw.raw_speed / 100.0f;      // 本车速度
        out.distances[2] = -1.0f;                      // 【修正】前车速度 (MVB无法提供，打上 -1 标记)
        out.distances[3] = hw.brake_press / 10.0f;     // 【修正】管压挪到下标 3
        return true;
    }

    // 将 CAN 报文翻译为标准 SensorData
    static bool can_unpack(const CANPacket& can_pkt, SensorData& out) {
        if (can_pkt.header != 0xAA55) return false; 
        std::memset(&out, 0, sizeof(SensorData));
        
        out.distances[0] = can_pkt.payload[0]; // 雷达测得的间距 Gap
        out.distances[1] = can_pkt.payload[1]; // 本车速度
        out.distances[2] = can_pkt.payload[2]; // 前车速度/其他数据
        return true;
    }

    static float calculate_safe_dist(float v_s, float v_f, TrainType type);
    static bool is_system_safe(const SensorData& pathA, const SensorData& pathB);
};