#include <iostream>
#include <vector>
#include <thread>
#include <iomanip>
#include <algorithm>
#include <sys/mman.h>
#include <pthread.h>
#include "perception.hpp"
#include "spsc_queue.hpp"

// Global T2T Mailboxes and Fleet State
SPSCQueue<RawT2TPacket>* mailboxes[5];
std::vector<TrainState> fleet(5);

void pin_thread_to_core(int core_id) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core_id, &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
}

// Module A & B: Physics Engine & Virtual T2T Bus (Core 0)
void world_simulator_thread() {
    pin_thread_to_core(0);
    const double dt = 0.01; 
    uint32_t global_seq = 0;

    while (true) {
        for (int i = 0; i < 5; ++i) {
            // F = ma -> Physics Integration
            double force = fleet[i].cmd_force;
            double accel = force / fleet[i].mass;
            fleet[i].accel = accel;
            fleet[i].vel += accel * dt;
            if (fleet[i].vel < 0) fleet[i].vel = 0; 
            fleet[i].pos += fleet[i].vel * dt;

            // T2T Broadcast logic
            if (i < 4) {
                RawT2TPacket pkt = { (uint32_t)i, 0x55AA55AA, ++global_seq, 0, 0 };
                uint64_t v_enc = (uint64_t)(fleet[i].vel * 100);
                uint64_t p_enc = (uint64_t)(fleet[i].pos * 100);
                pkt.payload = v_enc | (p_enc << 16);
                pkt.crc = _mm_crc32_u64(0, pkt.payload);
                mailboxes[i+1]->push(pkt);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

// Module C: Distributed VCU Controllers (Cores 1-4)
void follower_control_thread(int id, int target_core) {
    pin_thread_to_core(target_core);
    uint32_t last_seq = 0;
    RawT2TPacket pkt;
    SensorData sensor;

    while (true) {
        if (mailboxes[id]->pop(pkt)) {
            if (PerceptionEngine::fast_unpack(pkt, sensor, last_seq)) {
                float v_f = sensor.distances[1];
                float p_f = sensor.distances[0];
                float d_actual = p_f - fleet[id].pos;
                float d_safe = PerceptionEngine::calculate_safe_dist(fleet[id].vel, v_f, TrainType::EMU_DISTRIBUTED);

                // --- IMPROVED PD CONTROL LAW (Mass-Normalized) ---
                
                // 1. Add a 2.0m Comfort Buffer to the target gap
                float target_gap = d_safe + 2.0f;
                float error = d_actual - target_gap;
                float v_diff = v_f - fleet[id].vel;

                // 2. Calculate Base Acceleration Command (m/s^2)
                // Kp = 0.5, Kd = 0.8 (Stable for large inertia)
                float accel_cmd = (error * 0.5f) + (v_diff * 0.8f);

                // 3. Convert to Force based on Mass (F = m * a)
                fleet[id].cmd_force = fleet[id].mass * accel_cmd;

                // 4. Actuator Saturation Limits (Dynamic based on Mass)
                // Max Traction: 1.0 m/s^2 | Max Braking: -1.3 m/s^2
                float max_traction = fleet[id].mass * 1.0f;
                float max_braking  = fleet[id].mass * -1.3f;

                fleet[id].cmd_force = std::clamp(fleet[id].cmd_force, max_braking, max_traction);
                
                // --- EMERGENCY SAFETY OVERRIDE ---
                if (d_actual < d_safe) {
                    fleet[id].cmd_force = max_braking; // Force maximum emergency brake
                }
            }
        }
        asm("pause");
    }
}

// Module D: Real-time Dashboard (Core 0)
void dashboard_thread() {
    pin_thread_to_core(0);
    while (true) {
        std::cout << "\033[2J\033[H"; 
        std::cout << "=== SD-VCU EMU-PLATOON (450t) SIMULATION ===\n";
        std::cout << "Config: Mass-Normalized PD Control | Gap Buffer: 2.0m\n";
        std::cout << "------------------------------------------------------------\n";
        std::cout << "ID | Pos(m) | Vel(km/h) | Gap(m) | Safe(m) | Force(kN)\n";
        std::cout << "------------------------------------------------------------\n";
        for (int i = 0; i < 5; ++i) {
            float gap = (i == 0) ? 0 : (fleet[i-1].pos - fleet[i].pos);
            float d_s = (i == 0) ? 0 : PerceptionEngine::calculate_safe_dist(fleet[i].vel, fleet[i-1].vel, TrainType::EMU_DISTRIBUTED);
            std::printf("[%d] | %7.1f | %8.1f | %6.1f | %7.1f | %6.0f\n", 
                        i, fleet[i].pos, fleet[i].vel * 3.6, gap, d_s, fleet[i].cmd_force / 1000.0f);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

int main() {
    mlockall(MCL_CURRENT | MCL_FUTURE);
    
    // Initialize Fleet state (Standard 450t EMU Trainsets)
    const float EMU_MASS = 450000.0f; 
    
    fleet[0].mass = EMU_MASS;
    fleet[0].pos = 1000.0; fleet[0].vel = 30.0; fleet[0].cmd_force = 0;

    for(int i=1; i<5; i++) {
        mailboxes[i] = new SPSCQueue<RawT2TPacket>(128);
        fleet[i].mass = EMU_MASS;
        fleet[i].pos = 1000.0 - i * 60.0; // Initial 60m spacing
        fleet[i].vel = 30.0;
    }

    // Launch threads
    std::thread world(world_simulator_thread);
    std::thread vcu1(follower_control_thread, 1, 1);
    std::thread vcu2(follower_control_thread, 2, 2);
    std::thread vcu3(follower_control_thread, 3, 3);
    std::thread vcu4(follower_control_thread, 4, 4);
    std::thread dash(dashboard_thread);

    // Scenario: Leader emergency brake (1.2 m/s^2) after 10 seconds
    std::this_thread::sleep_for(std::chrono::seconds(10));
    fleet[0].cmd_force = fleet[0].mass * -1.2f; 

    world.join();
    return 0;
}