#include "perception.hpp"
#include <algorithm>

bool PerceptionEngine::fast_unpack(const RawT2TPacket& raw, SensorData& out, uint32_t& last_seq) {
    if (raw.header != 0x55AA55AA || raw.seq <= last_seq) return false;
    
    // Hardware CRC32 verification
    if (_mm_crc32_u64(0, raw.payload) != raw.crc) return false;

    last_seq = raw.seq;

    uint16_t v_raw = raw.payload & 0xFFFF;
    uint32_t p_raw = (raw.payload >> 16) & 0xFFFFFFFF;
    int16_t a_raw = (int16_t)(raw.payload >> 48);

    out.distances[0] = p_raw / 100.0f; 
    out.distances[1] = v_raw / 100.0f; 
    out.distances[2] = a_raw / 100.0f; 
    return true;
}

float PerceptionEngine::calculate_safe_dist(float v_s, float v_f, TrainType type) {
    if (type == TrainType::HEAVY_HAUL) {
        // Coordinated braking cap for 5000t tight formation
        float a_limit = 0.25f;  
        float t_ecp = 0.4f;    
        float d_buffer = 15.0f; 
        
        float braking_diff = (v_s * v_s - v_f * v_f) / (2.0f * a_limit);
        float reaction = v_s * t_ecp;
        
        return std::max(d_buffer, braking_diff + reaction + d_buffer);
    }
    return 15.0f;
}

bool PerceptionEngine::is_system_safe(const SensorData& pathA, const SensorData& pathB) {
    // 2oo2 Hardware Redundancy Check utilizing AVX-512
    __m512 vA = _mm512_loadu_ps(pathA.distances);
    __m512 vB = _mm512_loadu_ps(pathB.distances);
    return (_mm512_cmp_ps_mask(vA, vB, _CMP_NEQ_OQ) == 0);
}