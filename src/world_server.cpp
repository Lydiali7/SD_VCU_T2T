#include <iostream>
#include <vector>
#include <thread>
#include <fstream>
#include <iomanip>
#include <algorithm>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <chrono>
#include <random>
#include "perception.hpp"
#include "network_proto.hpp"

std::vector<TrainState> fleet(5);
std::vector<uint32_t> fleet_status(5, 0); 
std::ofstream log_file;

int init_multicast_sender(struct sockaddr_in& group_addr) {
    int sd = socket(AF_INET, SOCK_DGRAM, 0);
    memset(&group_addr, 0, sizeof(group_addr));
    group_addr.sin_family = AF_INET;
    group_addr.sin_addr.s_addr = inet_addr(MULTICAST_GROUP);
    group_addr.sin_port = htons(MULTICAST_PORT);
    return sd;
}

int init_force_receiver() {
    int sd = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(VCU_REPORT_PORT);
    bind(sd, (struct sockaddr*)&server_addr, sizeof(server_addr));
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 1000; 
    setsockopt(sd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    return sd;
}

void receive_forces_thread(int sd) {
    ForceReportPacket report;
    while (true) {
        if (recv(sd, &report, sizeof(report), 0) > 0) {
            if (report.train_id > 0 && report.train_id < 5) {
                fleet[report.train_id].cmd_force = report.cmd_force;
                fleet_status[report.train_id] = report.status_flag;
            }
        }
    }
}

double get_global_time() {
    auto now = std::chrono::system_clock::now();
    return std::chrono::duration<double>(now.time_since_epoch()).count();
}

int main() {
    log_file.open("fleet_log.csv");
    log_file << "Time,ID,Pos,Vel,Gap,Safe,Force\n";

    for(int i = 0; i < 5; i++) {
        fleet[i].mass = 450000.0f;
        fleet[i].pos = 5000.0 - i * 150.0; 
        fleet[i].vel = 0.0;
        fleet[i].cmd_force = 0.0;
    }

    struct sockaddr_in group_addr;
    int send_sd = init_multicast_sender(group_addr);
    int recv_sd = init_force_receiver();
    std::thread recv_thread(receive_forces_thread, recv_sd);

    const double dt = 0.01;
    uint32_t step = 0;
    
    const float DAVIS_A = 2500.0f;  
    const float DAVIS_B = 30.0f;    
    const float DAVIS_C = 4.5f;     

    // Noise Generators
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> pos_noise(0.0, 1.5);   // 1.5m standard deviation
    std::normal_distribution<double> vel_noise(0.0, 0.8);   // 0.8m/s standard deviation
    std::normal_distribution<double> accel_noise(0.0, 0.2); // 0.2m/s^2 standard deviation

    auto start_time = std::chrono::steady_clock::now();

    while (true) {
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - start_time).count();
        double current_timestamp = get_global_time();

        if (elapsed < 5.0) fleet[0].cmd_force = 0.0f; 
        else if (elapsed >= 5.0 && elapsed < 25.0) {
            if (fleet[0].vel < 30.0) fleet[0].cmd_force = fleet[0].mass * 0.8f;
            else fleet[0].cmd_force = 0.0f;
        } else if (elapsed >= 25.0 && elapsed < 40.0) {
            if (fleet[0].vel < 29.8) fleet[0].cmd_force = fleet[0].mass * 0.5f;
            else if (fleet[0].vel > 30.2) fleet[0].cmd_force = fleet[0].mass * -0.5f;
            else fleet[0].cmd_force = 0.0f;
        } else if (elapsed >= 40.0) {
            fleet[0].cmd_force = fleet[0].mass * -1.2f;
        }

        for (int i = 0; i < 5; ++i) {
            float v = fleet[i].vel;
            float resistance = (v > 0.01f) ? (DAVIS_A + DAVIS_B * v + DAVIS_C * v * v) : 0.0f;
            float net_force = fleet[i].cmd_force;
            
            if (v > 0.01f) net_force -= resistance;
            else {
                if (net_force < DAVIS_A && net_force > -DAVIS_A) net_force = 0.0f;
                else if (net_force >= DAVIS_A) net_force -= DAVIS_A;
            }

            fleet[i].accel = net_force / fleet[i].mass;
            fleet[i].vel = std::max(0.0, fleet[i].vel + fleet[i].accel * dt);
            fleet[i].pos += fleet[i].vel * dt;

            // Log GROUND TRUTH to CSV
            float gap = (i == 0) ? 0.0f : fleet[i-1].pos - fleet[i].pos;
            float safe_dist = (i == 0) ? 0.0f : PerceptionEngine::calculate_safe_dist(fleet[i].vel, fleet[i-1].vel, TrainType::EMU_DISTRIBUTED);
            log_file << elapsed << "," << i << "," << fleet[i].pos << ","
                     << fleet[i].vel << "," << gap << "," << safe_dist << ","
                     << fleet[i].cmd_force << "\n";

            // Broadcast NOISY data
            double noisy_pos = fleet[i].pos + pos_noise(gen);
            double noisy_vel = std::max(0.0, fleet[i].vel + vel_noise(gen));
            double noisy_accel = fleet[i].accel + accel_noise(gen);

            WorldUpdatePacket pkt = {
                0x55AA55AA, step, current_timestamp, (uint32_t)i, 
                noisy_pos, noisy_vel, noisy_accel, fleet_status[i], 0
            };
            sendto(send_sd, &pkt, sizeof(pkt), 0, (struct sockaddr*)&group_addr, sizeof(group_addr));
        }

        if (step % 10 == 0) {
            std::printf("\033[2J\033[H"); 
            std::printf("=== SD-VCU DISTRIBUTED NETWORK MONITOR ===\n");
            std::printf("Sim Time: %.2fs | Network: UDP Multicast 239.0.0.1\n", elapsed);
            std::printf("--------------------------------------------------------------------------------\n");
            std::printf("ID | TruePos(m)| TrueVel(km/h)|  Gap(m) | Safe(m) | Force(kN) | NetAccel | Status\n");
            std::printf("--------------------------------------------------------------------------------\n");
            for (int i = 0; i < 5; ++i) {
                float gap = (i == 0) ? 0.0f : fleet[i-1].pos - fleet[i].pos;
                float safe_dist = (i == 0) ? 0.0f : PerceptionEngine::calculate_safe_dist(fleet[i].vel, fleet[i-1].vel, TrainType::EMU_DISTRIBUTED);
                const char* status = (fleet_status[i] == 1) ? "DEGRADED" : "OK";
                std::printf("[%d] |  %7.1f |    %9.1f | %7.1f | %7.1f | %9.1f | %8.2f | %s\n", 
                            i, fleet[i].pos, fleet[i].vel*3.6, gap, safe_dist, fleet[i].cmd_force/1000.0, fleet[i].accel, status);
            }
        }
        step++;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return 0;
}