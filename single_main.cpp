#include <iostream>
#include <thread>
#include <chrono>
#include <immintrin.h>
#include <algorithm>
#include <cstdint>
#include <iomanip>

struct alignas(64) CANPacket {
    uint32_t header;       
    uint16_t source_id;    
    uint16_t seq_num;      
    float payload[12];     
    uint32_t crc32;        
    uint32_t padding;      
};

struct TrainState {
    double mass;
    double pos;
    double vel;
    double accel;
};

class SDVCU {
private:
    TrainState ego_train;
    int consecutive_errors;
    const int ERROR_THRESHOLD = 3; 
    uint16_t last_seq;
    bool is_eb_triggered;

    bool verify_crc(const CANPacket& pkt) {
        return true; 
    }

    bool avx512_payload_match(const CANPacket& a, const CANPacket& b) {
        __m512 vec_a = _mm512_load_ps(a.payload);
        __m512 vec_b = _mm512_load_ps(b.payload);
        __mmask16 mismatch_mask = _mm512_cmp_ps_mask(vec_a, vec_b, _CMP_NEQ_OQ);
        return (mismatch_mask == 0);
    }

    void trigger_eb() {
        std::cout << "[FATAL] FTTI Exceeded. Hardware degradation confirmed. Emergency Brake applied!\n";
        is_eb_triggered = true;
        // Heavy Haul EB deceleration is severely limited by longitudinal dynamics
        ego_train.accel = -0.25; 
    }

public:
    SDVCU() : consecutive_errors(0), last_seq(0), is_eb_triggered(false) {
        ego_train.mass = 5000000.0; // 5000tons
        ego_train.pos = 0.0;
        ego_train.vel = 22.2; // ~80 km/h
        ego_train.accel = 0.0;
    }

    void process_cycle(const CANPacket& path_a, const CANPacket& path_b, double dt) {
        if (is_eb_triggered) {
            ego_train.vel = std::max(0.0, ego_train.vel + ego_train.accel * dt);
            ego_train.pos += ego_train.vel * dt;
            return;
        }

        if (path_a.header != 0xAA55 || path_a.seq_num <= last_seq || !verify_crc(path_a)) {
            consecutive_errors++;
            std::cout << "[WARN] Protocol frame dropped. Seq: " << path_a.seq_num << "\n";
        } 
        else if (!avx512_payload_match(path_a, path_b)) {
            consecutive_errors++;
            std::cout << "[WARN] AVX-512 2oo2 Check Failed. Bit-flip detected. Dropping cycle.\n";
        } 
        else {
            consecutive_errors = 0; 
            last_seq = path_a.seq_num;
            
            float distance_to_obstacle = path_a.payload[0]; 
            
            double kinetic_energy = 0.5 * ego_train.mass * (ego_train.vel * ego_train.vel);
            // Limit max braking force to avoid coupler failure in heavy haul
            double max_braking_force = ego_train.mass * 0.25; 
            // 150m margin accounts for the immense propagation delay of pneumatic brakes across 1-2km train
            double min_safe_distance = kinetic_energy / max_braking_force + 150.0; 

            if (distance_to_obstacle < min_safe_distance) {
                std::cout << "[ATP] Obstacle entering safety envelope. Applying service brake.\n";
                // Gentle service brake for freight
                ego_train.accel = -0.15; 
            } else {
                ego_train.accel = 0.0; 
            }
        }

        if (consecutive_errors >= ERROR_THRESHOLD) {
            trigger_eb();
            return; 
        }

        if (consecutive_errors > 0) {
            ego_train.vel += ego_train.accel * dt;
            ego_train.pos += ego_train.vel * dt;
        } else {
            ego_train.vel = path_a.payload[1]; 
            ego_train.vel += ego_train.accel * dt;
            ego_train.pos += ego_train.vel * dt;
        }
    }

    void print_status(int cycle) {
        std::cout << "Cycle: " << std::setw(3) << cycle 
                  << " | Pos: " << std::fixed << std::setprecision(1) << std::setw(6) << ego_train.pos 
                  << " m | Vel: " << std::setw(4) << ego_train.vel * 3.6 
                  << " km/h | Accel: " << std::setw(5) << ego_train.accel 
                  << " | ErrCnt: " << consecutive_errors 
                  << " | Status: " << (is_eb_triggered ? "EB_LOCKED" : "NOMINAL") << "\n";
    }
};

int main() {
    SDVCU vcu;
    double dt = 0.01; 
    uint16_t seq = 0;
    // Massive initial distance required for 5000t train stopping distance
    float current_dist = 4000.0f; 
    float current_vel = 22.2f;

    std::cout << "--- SD-VCU Single Node Simulation Started (Heavy Haul 5000t) ---\n";

    for (int cycle = 1; cycle <= 100; ++cycle) {
        seq++;
        current_dist -= current_vel * dt; 

        CANPacket path_a = {0xAA55, 0x01, seq, {0}, 0xFFFF, 0};
        CANPacket path_b = {0xAA55, 0x01, seq, {0}, 0xFFFF, 0};
        
        path_a.payload[0] = current_dist; path_a.payload[1] = current_vel;
        path_b.payload[0] = current_dist; path_b.payload[1] = current_vel;

        if (cycle == 30) {
            path_b.payload[0] += 0.5f; 
        }

        if (cycle >= 70 && cycle <= 75) {
            path_b.payload[1] = 0.0f; 
        }

        vcu.process_cycle(path_a, path_b, dt);

        if (cycle % 10 == 0 || cycle == 30 || cycle == 31 || (cycle >= 70 && cycle <= 74)) {
            vcu.print_status(cycle);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "--- Simulation Finished ---\n";
    return 0;
}