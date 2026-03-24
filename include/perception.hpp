#pragma once
#include <immintrin.h>
#include <iostream>
#include <cstdint>

// 模拟 RSSP-2 风格的二进制报文
struct RawT2TPacket {
    uint32_t header;    // 帧头：0x55AA55AA
    uint32_t seq;       // 序列号（防止重放攻击）
    uint64_t payload;   // 压缩数据（Bit 0-15:速度, Bit 16-47:距离）
    uint32_t crc;       // 硬件校验码
};
// Defined train composition types
enum class TrainType {
    LOCO_HAULED,    // Locomotive-hauled (Single power car at front, slower response)
    EMU_DISTRIBUTED // Electric Multiple Unit (Distributed power/brake, fast response)
};

struct alignas(64) SensorData {
    float distances[16]; 
    //[0] = distance to front obstacle (m)
    //[1] = current speed (m/s)
    //[2] = former speed (m/s) 
    //[3] = former acceleration (m/s^2)
};

class PerceptionEngine {
public:
    static bool fast_unpack(const RawT2TPacket& raw, SensorData& out, uint32_t & last_seq);
    // Calculate safety distance based on train dynamics
    static float calculate_safe_envelope(float v, TrainType type);
    
    // Core safety check combining 2oo2 and dynamics
    static bool is_system_safe(const SensorData& pathA, const SensorData& pathB, TrainType type,bool cooperative);
};