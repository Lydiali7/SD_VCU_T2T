#include "perception.hpp"
#include <iostream>
#include <algorithm>

bool PerceptionEngine::fast_unpack(const RawT2TPacket& raw, SensorData& out, uint32_t& last_seq) {
    if (raw.header != 0x55AA55AA || raw.seq <= last_seq) return false;
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
    switch (type) {
        case TrainType::HEAVY_HAUL: {
            float a_limit = 0.25f;  
            float t_ecp = 0.4f;    
            float d_buffer = 150.0f; 
            
            float braking_diff = std::max(0.0f, (v_s * v_s - v_f * v_f) / (2.0f * a_limit));
            float reaction = v_s * t_ecp;
            return d_buffer + reaction + braking_diff;
        }
        case TrainType::EMU_DISTRIBUTED: {
            float a_limit = 1.0f;
            float t_resp = 0.1f;
            float d_buffer = 15.0f; 
            float braking_diff = std::max(0.0f, (v_s * v_s - v_f * v_f) / (2.0f * a_limit));
            float reaction = v_s * t_resp;
            return d_buffer + reaction + braking_diff;
        }
        case TrainType::LOCO_HAULED: 
            return 50.0f;
        default: 
            return 150.0f;
    }
}

bool PerceptionEngine::is_system_safe(const SensorData& pathA, const SensorData& pathB) {
    __m512 vA = _mm512_loadu_ps(pathA.distances);
    __m512 vB = _mm512_loadu_ps(pathB.distances);
    return (_mm512_cmp_ps_mask(vA, vB, _CMP_NEQ_OQ) == 0);
}

SDVCU_Core::SDVCU_Core() : 
    consecutive_errors(0), 
    last_seq(0), 
    is_eb_triggered(false), 
    current_safe_gap(150.0f), 
    current_vel(0.0f) {}

// AVX-512 现在直接对比 64 字节的纯物理 SensorData
bool SDVCU_Core::avx512_payload_match(const SensorData& a, const SensorData& b) {
    __m512i vec_a = _mm512_loadu_si512((const void*)&a);
    __m512i vec_b = _mm512_loadu_si512((const void*)&b);
    __mmask16 mismatch_mask = _mm512_cmpneq_epi32_mask(vec_a, vec_b);
    return (mismatch_mask == 0);
}

bool SDVCU_Core::process_sensors(uint16_t seq_num, const SensorData& path_a, const SensorData& path_b) {
    if (is_eb_triggered) return false;

    if (seq_num != 0 && seq_num <= last_seq) {
        consecutive_errors++;
    } 
    else if (!avx512_payload_match(path_a, path_b)) {
        consecutive_errors++;
        std::cout << "[PERCEPTION] AVX-512 Check Failed! Bit-flip detected. Entering dead reckoning.\n";
    } 
    else {
        consecutive_errors = 0; 
        if (seq_num != 0) last_seq = seq_num;
        
        float distance_to_obstacle = path_a.distances[0]; 
        current_vel = path_a.distances[1]; 
        float front_vel = path_a.distances[2]; 

        if (front_vel < 0.0f) {
            front_vel = current_vel; // 假设前车与本车同速，不产生额外制动差
        }
        float a_limit = 0.25f; 
        float t_ecp = 0.4f;
        float d_buffer = 150.0f;
        
        float braking_diff = std::max(0.0f, (current_vel * current_vel - front_vel * front_vel) / (2.0f * a_limit));
        current_safe_gap = d_buffer + (current_vel * t_ecp) + braking_diff; 
        
        // 只有当提供了有效的雷达距离(>0)时才触发硬件 ATP
        if (distance_to_obstacle > 0.1f && distance_to_obstacle < current_safe_gap) {
            std::cout << "\n[PERCEPTION_FATAL] Hardware ATP Trip! Radar detected Gap < Safe. Triggering EB Lock!\n";
            is_eb_triggered = true; 
            return false; 
        }
    }

    if (consecutive_errors >= ERROR_THRESHOLD) {
        std::cout << "[PERCEPTION_FATAL] FTTI exceeded! Permanent hardware fault confirmed. Triggering EB!\n";
        is_eb_triggered = true;
        return false; 
    }

    return true; 
}

bool SDVCU_Core::is_eb() const { return is_eb_triggered; }
bool SDVCU_Core::is_atp_braking(float gap) const { return gap < current_safe_gap; }