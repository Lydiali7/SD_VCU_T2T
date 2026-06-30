#include <iostream>
#include <vector>
#include <thread>
#include <sys/mman.h>
#include <random>
#include "perception.hpp"
#include "spsc_queue.hpp"
#include <algorithm>

SPSCQueue<RawT2TPacket>* mailboxes[5];
std::vector<TrainState> fleet(5);
bool chaos_mode = true; // Enable Fault Injection

void pin_thread_to_core(int core_id) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core_id, &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
}

// Module A & B: World Sim & T2T Bus
void world_simulator_thread() {
    pin_thread_to_core(0);
    const double dt = 0.01;
    uint32_t global_seq = 0;
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(0.0, 1.0);

    while (true) {
        for (int i = 0; i < 5; ++i) {
            fleet[i].accel = fleet[i].cmd_force / fleet[i].mass;
            fleet[i].vel = std::max(0.0, fleet[i].vel + fleet[i].accel * dt);
            fleet[i].pos += fleet[i].vel * dt;

            if (i < 4) {
                // Fault Injection: 5% packet loss simulation
                if (chaos_mode && distribution(generator) < 0.05) continue; 

                RawT2TPacket pkt = { (uint32_t)i, 0x55AA55AA, ++global_seq, 0, 0 };
                uint64_t v_enc = (uint64_t)(fleet[i].vel * 100);
                uint64_t p_enc = (uint64_t)(fleet[i].pos * 100);
                uint64_t a_enc = (uint16_t)(fleet[i].accel * 100);
                
                pkt.payload = v_enc | (p_enc << 16) | (a_enc << 48);
                pkt.crc = _mm_crc32_u64(0, pkt.payload);
                mailboxes[i+1]->push(pkt);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

// Module C: VCU Control with String Stability & Degraded Mode
void follower_control_thread(int id, int core) {
    pin_thread_to_core(core);
    uint32_t last_seq = 0;
    uint32_t timeout_counter = 0;
    RawT2TPacket pkt;
    SensorData sensor;

    while (true) {
        if (mailboxes[id]->pop(pkt)) {
            timeout_counter = 0; // Reset on success
            if (PerceptionEngine::fast_unpack(pkt, sensor, last_seq)) {
                float d_safe = PerceptionEngine::calculate_safe_dist(fleet[id].vel, sensor.distances[1], TrainType::EMU_DISTRIBUTED);
                float d_actual = sensor.distances[0] - fleet[id].pos;
                
                
                // a_cmd = a_front + Kp * error + Kd * v_diff
                float a_front = sensor.distances[2];// add Feed-forward
                float target_gap = d_safe + 2.0f;
                float accel_cmd = a_front + (d_actual - target_gap) * 0.5f + (sensor.distances[1] - fleet[id].vel) * 0.8f;

                fleet[id].cmd_force = fleet[id].mass * accel_cmd;
                fleet[id].cmd_force = std::clamp(fleet[id].cmd_force, fleet[id].mass * -1.3f, fleet[id].mass * 1.0f);
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(2)); // 给一点点缓冲
            timeout_counter++;
            // If more than 5 cycles (~50ms) without data, enter Safe Coast
            if (timeout_counter > 10) {
                fleet[id].cmd_force = fleet[id].mass * -0.5f; // Constant gentle deceleration
            }
        }
        //asm("pause");
    }
}

void dashboard_thread() {
    pin_thread_to_core(0);
    while (true) {
        std::cout << "\033[2J\033[H"; 
        std::cout << "--- SD-VCU String with 5%% Loss Sim ---\n";
        std::cout << "ID | Position(m) | Speed(km/h) | Gap(m) | Safe(m) | CmdForce(kN) | Status\n";
        for (int i = 0; i < 5; ++i) {
            float gap = (i == 0) ? 0 : fleet[i-1].pos - fleet[i].pos;
            float d_safe = PerceptionEngine::calculate_safe_dist(fleet[i].vel, fleet[i-1].vel, TrainType::EMU_DISTRIBUTED);
            std::printf("[%d] | %8.1f | %6.1f | %6.0f | %6.0f | %8.1f | %s\n", 
                i, fleet[i].pos, fleet[i].vel*3.6, gap, d_safe, fleet[i].cmd_force/1000, 
                (i > 0 && fleet[i].accel < -0.49 && fleet[i].accel > -0.51) ? "DEGRADED" : "NOMINAL");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

int main() {
    mlockall(MCL_CURRENT | MCL_FUTURE);
    for(int i=0; i<5; i++) {
        fleet[i].mass = 450000.0f;
        fleet[i].pos = 1000 - i * 60; fleet[i].vel = 30;
        if(i > 0) mailboxes[i] = new SPSCQueue<RawT2TPacket>(128);
    }
    std::thread world(world_simulator_thread), dash(dashboard_thread);
    std::thread v1(follower_control_thread, 1, 1), v2(follower_control_thread, 2, 2);
    std::thread v3(follower_control_thread, 3, 3), v4(follower_control_thread, 4, 4);

    std::this_thread::sleep_for(std::chrono::seconds(10));
    fleet[0].cmd_force = fleet[0].mass * -1.2f; // Leader Brakes

    world.join();
    return 0;
}