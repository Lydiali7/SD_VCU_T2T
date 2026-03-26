#include "perception.hpp"
#include <nmmintrin.h>
#include <algorithm>

bool PerceptionEngine::fast_unpack(const RawT2TPacket& raw, SensorData& out, uint32_t& last_seq) {
    // Protocol integrity checks
    if (raw.header != 0x55AA55AA || raw.seq <= last_seq) return false;
    if (_mm_crc32_u64(0, raw.payload) != raw.crc) return false;

    last_seq = raw.seq;

    // Bit-field extraction
    uint16_t v_raw = raw.payload & 0xFFFF;
    uint32_t p_raw = (raw.payload >> 16) & 0xFFFFFFFF;
    int16_t a_raw = (int16_t)(raw.payload >> 48);
    out.distances[2] = a_raw / 100.0f;

    out.distances[1] = v_raw / 100.0f; // Front vehicle speed
    out.distances[0] = p_raw / 100.0f; // Front vehicle absolute position
    return true;
}

float PerceptionEngine::calculate_safe_dist(float v_s, float v_f, TrainType type) {
    float a_self = 1.0f;    // Current train braking rate (EMU)
    float a_front = 1.2f;   // Worst-case assumption for front train (LOCO)
    float t_delay = 0.5f;   // System reaction delay
    float d_buffer = 15.0f;  // Absolute minimum safety buffer

    // Relative distance covered during reaction time
    float rel_dist = std::max(0.0f, (v_s - v_f) * t_delay);
    
    // Braking distance differential
    float brake_diff = (v_s * v_s / (2.0f * a_self)) - (v_f * v_f / (2.0f * a_front));
    
    return rel_dist + std::max(0.0f, brake_diff) + d_buffer;
}

bool PerceptionEngine::is_system_safe(const SensorData& pathA, const SensorData& pathB) {
    __m512 vA = _mm512_loadu_ps(pathA.distances);
    __m512 vB = _mm512_loadu_ps(pathB.distances);
    if (_mm512_cmp_ps_mask(vA, vB, _CMP_NEQ_OQ) != 0) return false;
    return true; 
}