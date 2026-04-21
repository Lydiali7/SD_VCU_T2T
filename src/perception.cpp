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
            
            // 【修复点】：制动动能差如果为负，直接归 0。绝不能减损反应时间！
            float braking_diff = std::max(0.0f, (v_s * v_s - v_f * v_f) / (2.0f * a_limit));
            float reaction = v_s * t_ecp;
            
            // 安全距离 = 绝对裕量 + 空走反应距离 + 相对制动差
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

bool SDVCU_Core::avx512_payload_match(const CANPacket& a, const CANPacket& b) {
    __m512i vec_a = _mm512_loadu_si512((const void*)&a);
    __m512i vec_b = _mm512_loadu_si512((const void*)&b);
    __mmask16 mismatch_mask = _mm512_cmpneq_epi32_mask(vec_a, vec_b);
    return (mismatch_mask == 0);
}

bool SDVCU_Core::process_sensors(const CANPacket& path_a, const CANPacket& path_b, double mass) {
    if (is_eb_triggered) return false;

    if (path_a.header != 0xAA55 || path_a.seq_num <= last_seq) {
        consecutive_errors++;
    } 
    else if (!avx512_payload_match(path_a, path_b)) {
        consecutive_errors++;
        std::cout << "[PERCEPTION] AVX-512 Check Failed! Bit-flip detected. Entering dead reckoning.\n";
    } 
    else {
        consecutive_errors = 0; 
        last_seq = path_a.seq_num;
        
        float distance_to_obstacle = path_a.payload[0]; 
        current_vel = path_a.payload[1]; 
        float front_vel = path_a.payload[2]; // 读取雷达测得的前车速度
        
        // 底层的硬件级 ATP 也全面采用相对安全动能包络
        float a_limit = 0.25f; 
        float t_ecp = 0.4f;
        float d_buffer = 150.0f;
        
        float braking_diff = std::max(0.0f, (current_vel * current_vel - front_vel * front_vel) / (2.0f * a_limit));
        current_safe_gap = d_buffer + (current_vel * t_ecp) + braking_diff; 
        
        // ==========================================
        // 【核心修复】：硬件级 ATP 越界必须强行锁死！
        // ==========================================
        if (distance_to_obstacle < current_safe_gap) {
            std::cout << "\n[PERCEPTION_FATAL] Hardware ATP Trip! Radar detected Gap < Safe. Triggering EB Lock!\n";
            is_eb_triggered = true; // 真正拉下物理紧急制动的关键一行
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