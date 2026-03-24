#include <iostream>
#include <vector>
#include <thread>
#include <iomanip>
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
            // Newton's Second Law
            double force = fleet[i].cmd_force;
            double accel = force / fleet[i].mass;
            fleet[i].accel = accel;
            fleet[i].vel += accel * dt;
            if (fleet[i].vel < 0) fleet[i].vel = 0; // Prevent reversing
            fleet[i].pos += fleet[i].vel * dt;

            // Broadcast state to the following train
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

                // PD Control Law to maintain dynamic safety gap
                float error = d_actual - d_safe;
                float v_diff = v_f - fleet[id].vel;
                fleet[id].cmd_force = (error * 25000.0f) + (v_diff * 35000.0f);

                // Actuator saturation limits
                if (fleet[id].cmd_force > 150000.0f) fleet[id].cmd_force = 150000.0f;
                if (fleet[id].cmd_force < -250000.0f) fleet[id].cmd_force = -250000.0f;
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
        std::cout << "=== SD-VCU 5-TRAIN DISTRIBUTED PLATOON SIM ===\n";
        std::cout << "Event: Cruising at 108km/h, Leader brakes at T+10s\n";
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
    
    // Initialize Fleet state
    fleet[0].pos = 500.0; fleet[0].vel = 30.0; fleet[0].cmd_force = 0;
    for(int i=1; i<5; i++) {
        mailboxes[i] = new SPSCQueue<RawT2TPacket>(128);
        fleet[i].pos = 500.0 - i * 50.0; 
        fleet[i].vel = 30.0;
    }

    // Launch threads with specific CPU core affinities
    std::thread world(world_simulator_thread);
    std::thread vcu1(follower_control_thread, 1, 1);
    std::thread vcu2(follower_control_thread, 2, 2);
    std::thread vcu3(follower_control_thread, 3, 3);
    std::thread vcu4(follower_control_thread, 4, 4);
    std::thread dash(dashboard_thread);

    // Scenario Injection: Leader emergency brake after 10 seconds
    std::this_thread::sleep_for(std::chrono::seconds(10));
    fleet[0].cmd_force = -180000.0f; 

    world.join();
    return 0;
}