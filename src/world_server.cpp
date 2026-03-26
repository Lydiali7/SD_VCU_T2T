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
#include "perception.hpp"
#include "network_proto.hpp"

std::vector<TrainState> fleet(5);
std::ofstream log_file;

// Setup UDP Multicast Sender
int init_multicast_sender(struct sockaddr_in& group_addr) {
    int sd = socket(AF_INET, SOCK_DGRAM, 0);
    memset(&group_addr, 0, sizeof(group_addr));
    group_addr.sin_family = AF_INET;
    group_addr.sin_addr.s_addr = inet_addr(MULTICAST_GROUP);
    group_addr.sin_port = htons(MULTICAST_PORT);
    return sd;
}

// Setup UDP Unicast Receiver for Force Reports
int init_force_receiver() {
    int sd = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(VCU_REPORT_PORT);
    bind(sd, (struct sockaddr*)&server_addr, sizeof(server_addr));

    // Set non-blocking so physics loop doesn't freeze
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 1000; // 1ms timeout
    setsockopt(sd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    return sd;
}

// Thread to receive forces from external VCUs
void receive_forces_thread(int sd) {
    ForceReportPacket report;
    while (true) {
        if (recv(sd, &report, sizeof(report), 0) > 0) {
            if (report.train_id > 0 && report.train_id < 5) {
                fleet[report.train_id].cmd_force = report.cmd_force;
            }
        }
    }
}

int main() {
    // Initialize CSV log file with Gap and Safe columns
    log_file.open("fleet_log.csv");
    log_file << "Time,ID,Pos,Vel,Gap,Safe,Force\n";

    // Initialize Fleet (450t EMU)
    for(int i = 0; i < 5; i++) {
        fleet[i].mass = 450000.0f;
        fleet[i].pos = 2000.0 - i * 60.0; 
        fleet[i].vel = 30.0;
    }

    struct sockaddr_in group_addr;
    int send_sd = init_multicast_sender(group_addr);
    int recv_sd = init_force_receiver();
    std::thread recv_thread(receive_forces_thread, recv_sd);

    const double dt = 0.01;
    uint32_t step = 0;
    auto start_time = std::chrono::steady_clock::now();

    while (true) {
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - start_time).count();

        // Scenario Injection: Leader emergency brake after 10s
        if (elapsed > 10.0) {
            fleet[0].cmd_force = fleet[0].mass * -1.2f;
        }

        // Physics Update and Logging
        for (int i = 0; i < 5; ++i) {
            fleet[i].accel = fleet[i].cmd_force / fleet[i].mass;
            fleet[i].vel = std::max(0.0, fleet[i].vel + fleet[i].accel * dt);
            fleet[i].pos += fleet[i].vel * dt;

            // Calculate Gap and Safe Distance for logging
            float gap = (i == 0) ? 0.0f : fleet[i-1].pos - fleet[i].pos;
            float safe_dist = (i == 0) ? 0.0f : PerceptionEngine::calculate_safe_dist(fleet[i].vel, fleet[i-1].vel, TrainType::EMU_DISTRIBUTED);

            log_file << elapsed << "," << i << "," << fleet[i].pos << ","
                     << fleet[i].vel << "," << gap << "," << safe_dist << ","
                     << fleet[i].cmd_force << "\n";

            // Broadcast state via UDP
            WorldUpdatePacket pkt = {0x55AA55AA, step, (uint32_t)i, fleet[i].pos, fleet[i].vel, fleet[i].accel, 0};
            sendto(send_sd, &pkt, sizeof(pkt), 0, (struct sockaddr*)&group_addr, sizeof(group_addr));
        }

        // Dashboard Display (Refresh every 10 steps / 100ms)
        if (step % 10 == 0) {
            std::printf("\033[2J\033[H"); 
            std::printf("=== SD-VCU DISTRIBUTED NETWORK MONITOR ===\n");
            std::printf("Sim Time: %.2fs | Network: UDP Multicast 239.0.0.1\n", elapsed);
            std::printf("------------------------------------------------------------------------\n");
            std::printf("ID |  Pos(m) | Vel(km/h) |  Gap(m) | Safe(m) | Force(kN) | Status\n");
            std::printf("------------------------------------------------------------------------\n");
            for (int i = 0; i < 5; ++i) {
                float gap = (i == 0) ? 0.0f : fleet[i-1].pos - fleet[i].pos;
                float safe_dist = (i == 0) ? 0.0f : PerceptionEngine::calculate_safe_dist(fleet[i].vel, fleet[i-1].vel, TrainType::EMU_DISTRIBUTED);

                const char* status = (i > 0 && fleet[i].cmd_force == fleet[i].mass * -0.5f) ? "\033[1;31mLOST\033[0m" : "\033[1;32mOK\033[0m";
                std::printf("[%d] | %7.1f | %9.1f | %7.1f | %7.1f | %9.1f | %s\n", 
                            i, fleet[i].pos, fleet[i].vel*3.6, gap, safe_dist, fleet[i].cmd_force/1000.0, status);
            }
        }

        step++;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return 0;
}