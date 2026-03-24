#include "perception.hpp"
#include <nmmintrin.h> // 包含硬件 CRC32 指令

bool PerceptionEngine::fast_unpack(const RawT2TPacket& raw, SensorData& out, uint32_t & last_seq) {
    // 帧头
    if (raw.header != 0x55AA55AA) return false;

    //防止重放攻击,如果新收到的这个包的编号还没上一个处理过的编号大，那说明这个包要么是重发的，要么是迟到的
    if (raw.seq <= last_seq) return false;

    //硬件 CRC32 校验 (利用 x86 原生指令，1 个周期完成)
    uint32_t computed_crc = _mm_crc32_u64(0, raw.payload);
    if (computed_crc != raw.crc) return false;

    // 4. 位字段提取 (Bit-field Extraction)
    // 假设速度放大100倍后存入前16位
    uint16_t v_raw = raw.payload & 0xFFFF;
    // 假设距离放大100倍后存入接下来的32位
    uint32_t d_raw = (raw.payload >> 16) & 0xFFFFFFFF;
    
    uint16_t vf_raw = (raw.payload >> 48) & 0xFFFF; // 提取前车速度

    out.distances[1] = v_raw / 100.0f;
    out.distances[0] = d_raw / 100.0f;
    out.distances[2] = vf_raw / 100.0f; // 关键：把前车速度传给算法
    out.distances[3] = 0.8f;             // 假设前车是 LOCO 性能
    //out.distances[3] = 1.2f;             // 假设前车是emu

    return true;
}

float PerceptionEngine::calculate_safe_envelope(float v, TrainType type) {
    float a, t;
    
    if (type == TrainType::EMU_DISTRIBUTED) {
        a = 1.2f;  // m/s^2, efficient distributed braking
        t = 0.2f;  // seconds, fast electronic signal synchronization
    } else {
        a = 0.8f;  // m/s^2, mechanical friction focus
        t = 1.5f;  // seconds, considering pneumatic wave propagation delay
    }

    float reaction_dist = v * t;
    float braking_dist = (v * v) / (2.0f * a);
    float margin = 5.0f; // 5 meters buffer

    return reaction_dist + braking_dist + margin;
}

bool PerceptionEngine::is_system_safe(const SensorData& pathA, const SensorData& pathB, TrainType type, bool cooperative) {
    // 2oo2
    __m512 vecA = _mm512_loadu_ps(pathA.distances);
    __m512 vecB = _mm512_loadu_ps(pathB.distances);
    if (_mm512_cmp_ps_mask(vecA, vecB, _CMP_NEQ_OQ) != 0) return false;

    float d_actual = pathA.distances[0];
    float v_self   = pathA.distances[1];
    float v_front  = pathA.distances[2];
    float a_front = (pathA.distances[3] > 0) ? pathA.distances[3] : 0.8f;

    // type?
    float a_self = (type == TrainType::EMU_DISTRIBUTED) ? 1.2f : 0.8f;
    float t_delay = (type == TrainType::EMU_DISTRIBUTED) ? 0.2f : 1.5f;

    float d_safe;
    if (cooperative) {
        // 协同模式：考虑前车也在制动
        float relative_speed_dist = (v_self - v_front) * t_delay;
        // 核心差异：减去前车的预期制动距离
        float braking_diff = (v_self * v_self / (2.0f * a_self)) - (v_front * v_front / (2.0f * a_front));
        if (braking_diff < 0) braking_diff = 0; // 防止前车性能太强导致计算出负距离
        
        d_safe = relative_speed_dist + braking_diff + 5.0f;
    } else {
        // traditional mode, only consider self braking(front--0)
        d_safe = v_self * t_delay + (v_self * v_self / (2.0f * a_self)) + 5.0f;
    }

    
    if (d_actual < d_safe) {
    std::cout << (cooperative ? "[COOP] " : "[TRAD] ") 
              << "!!! BRAKE !!! Dist: " << d_actual << std::endl;
    return false;
} else {
    // 注意：在正式 SIL4 环境中要删掉(打印很耗时)
    static int count = 0;
    if (count++ % 100 == 0) { // 每 100 次循环打印一次，防止刷屏太快
        std::cout << ">>> [SYSTEM SAFE] Mode: " << (cooperative ? "COOP" : "TRAD") 
                  <<" | Actual Dist: " << d_actual << " | Safe Req: " << d_safe << std::endl;
    }
}
return true;
    return true;
}