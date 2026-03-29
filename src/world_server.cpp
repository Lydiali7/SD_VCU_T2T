#include <iostream>
#include <vector>
#include <thread>
#include <fstream>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <chrono>
#include <random>
#include "perception.hpp"

std::vector<TrainState> fleet(5);
std::vector<uint32_t> fleet_status(5, 0); 
float forces[5] = {0}; 

void receive_forces_thread(int sd) {
    ForceReportPacket report;
    while (recv(sd, &report, sizeof(report), 0) > 0) {
        if (report.train_id > 0 && report.train_id < 5) {
            forces[report.train_id] = report.cmd_force; 
            fleet_status[report.train_id] = report.status_flag;
        }
    }
}

int main() {
    std::ofstream log_file("fleet_log.csv");
    log_file << "Time,ID,Pos,Vel,Gap,Safe,Force\n";

    for(int i = 0; i < 5; i++) {
        fleet[i].mass = 5000000.0f;
        fleet[i].pos = 10000.0 - i * 60.0; 
        fleet[i].vel = 0.0;
        fleet[i].cmd_force = 0.0;
    }

    int send_sd = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in group_addr;
    memset(&group_addr, 0, sizeof(group_addr));
    group_addr.sin_family = AF_INET;
    group_addr.sin_addr.s_addr = inet_addr(MULTICAST_GROUP);
    group_addr.sin_port = htons(MULTICAST_PORT);

    int recv_sd = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in srv_addr;
    memset(&srv_addr, 0, sizeof(srv_addr));
    srv_addr.sin_family = AF_INET;
    srv_addr.sin_addr.s_addr = INADDR_ANY;
    srv_addr.sin_port = htons(VCU_REPORT_PORT);
    bind(recv_sd, (struct sockaddr*)&srv_addr, sizeof(srv_addr));
    std::thread(receive_forces_thread, recv_sd).detach();

    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> pos_noise(0.0, 1.5);   
    std::normal_distribution<double> vel_noise(0.0, 0.8);   
    std::normal_distribution<double> accel_noise(0.0, 0.2); 
    std::uniform_real_distribution<double> pkt_loss(0.0, 1.0);

    const double dt = 0.01;
    uint32_t step = 0;
    const float DAVIS_A = 18000.0f;  
    const float DAVIS_B = 450.0f;    
    const float DAVIS_C = 15.5f;     

    auto start_time = std::chrono::steady_clock::now();

    while (true) {
        double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();

        bool collision = false;
        for (int i = 1; i < 5; ++i) {
            if (fleet[i-1].pos - fleet[i].pos < -1.0) collision = true;
        }

        if (collision) {
            fleet[0].cmd_force = -1250000.0f; 
        } else {
            if (elapsed < 5.0) fleet[0].cmd_force = 0.0f; 
            else if (elapsed < 350.0) { // Extended acceleration window for heavy haul
                if (fleet[0].vel < 22.22) 
                    fleet[0].cmd_force = 400000.0f; // [CRITICAL] 400kN smooth acceleration. Leaves 600kN headroom!
                else if (fleet[0].vel > 22.3) 
                    fleet[0].cmd_force = -100000.0f;
                else 
                    fleet[0].cmd_force = 60000.0f;
            } else fleet[0].cmd_force = -1250000.0f;
        }

        for (int i = 0; i < 5; ++i) {
            if (i > 0) fleet[i].cmd_force = forces[i];

            float v = fleet[i].vel;
            float res = (v > 0.1f) ? (DAVIS_A + DAVIS_B * v + DAVIS_C * v * v) : 0.0f;
            fleet[i].accel = (fleet[i].cmd_force - res) / fleet[i].mass;
            fleet[i].vel = std::max(0.0, fleet[i].vel + fleet[i].accel * dt);
            fleet[i].pos += fleet[i].vel * dt;

            float gap = (i == 0) ? 0.0f : fleet[i-1].pos - fleet[i].pos;
            float safe_dist = (i == 0) ? 0.0f : PerceptionEngine::calculate_safe_dist(fleet[i].vel, fleet[i-1].vel, TrainType::HEAVY_HAUL);
            log_file << elapsed << "," << i << "," << fleet[i].pos << "," << fleet[i].vel << "," << gap << "," << safe_dist << "," << fleet[i].cmd_force << "\n";

            if (pkt_loss(gen) < 0.05) continue; 

            double noisy_pos = fleet[i].pos + pos_noise(gen);
            double noisy_vel = std::max(0.0, fleet[i].vel + vel_noise(gen));
            double noisy_accel = fleet[i].accel + accel_noise(gen);

            uint64_t v_enc = (uint64_t)(noisy_vel * 100.0);
            uint64_t p_enc = (uint64_t)(noisy_pos * 100.0);
            uint64_t a_enc = (uint16_t)(int16_t)(noisy_accel * 100.0);

            RawT2TPacket pkt = {(uint32_t)i, 0x55AA55AA, step, v_enc | (p_enc << 16) | (a_enc << 48), 0};
            pkt.crc = _mm_crc32_u64(0, pkt.payload);
            sendto(send_sd, &pkt, sizeof(pkt), 0, (struct sockaddr*)&group_addr, sizeof(group_addr));
        }

        if (step % 20 == 0) {
            std::printf("\033[2J\033[H"); 
            std::printf("=== SD-VCU UDP DISTRIBUTED NETWORK MONITOR ===\n");
            std::printf("Sim Time: %7.2fs | Target Speed: 80km/h | Target Gap: 50m\n", elapsed);
            if (collision) std::printf(">>> CRITICAL ALARM: COLLISION DETECTED. FLEET HALTED. <<<\n");
            std::printf("--------------------------------------------------------------------------------\n");
            std::printf("ID | TruePos(m)| TrueVel(km/h)|  Gap(m) | Safe(m) | Force(kN) | NetAccel | Status\n");
            for (int i = 0; i < 5; ++i) {
                float gap = (i == 0) ? 0.0f : fleet[i-1].pos - fleet[i].pos;
                float s_dist = (i == 0) ? 0.0f : PerceptionEngine::calculate_safe_dist(fleet[i].vel, fleet[i-1].vel, TrainType::HEAVY_HAUL);
                std::printf("[%d] |  %7.1f |    %9.1f | %7.1f | %7.1f | %9.1f | %8.3f | %s\n", 
                            i, fleet[i].pos, fleet[i].vel*3.6, gap, s_dist, fleet[i].cmd_force/1000.0, fleet[i].accel, fleet_status[i] ? "DEGRADED" : "OK");
            }
        }
        step++;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return 0;
}