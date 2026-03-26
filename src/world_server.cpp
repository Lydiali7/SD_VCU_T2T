#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <random>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include "perception.hpp"
#include "network_proto.hpp"

std::vector<TrainState> fleet(5);
bool chaos_mode = true; 

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
    // Initialize Fleet (450t EMU)
    for(int i=0; i<5; i++) {
        fleet[i].mass = 450000.0f;
        fleet[i].pos = 1000 - i * 60; 
        fleet[i].vel = 30;
    }

    struct sockaddr_in group_addr;
    int send_sd = init_multicast_sender(group_addr);
    int recv_sd = init_force_receiver();

    std::thread recv_thread(receive_forces_thread, recv_sd);

    const double dt = 0.01;
    uint32_t global_seq = 0;
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(0.0, 1.0);

    std::cout << "World Physics Server Started on UDP 239.0.0.1:8888\n";
    auto start_time = std::chrono::steady_clock::now();

    while (true) {
        // Trigger Leader Brake after 10s
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count() == 10) {
            fleet[0].cmd_force = fleet[0].mass * -1.2f; 
        }

        // Physics Update
        for (int i = 0; i < 5; ++i) {
            fleet[i].accel = fleet[i].cmd_force / fleet[i].mass;
            fleet[i].vel = std::max(0.0, fleet[i].vel + fleet[i].accel * dt);
            fleet[i].pos += fleet[i].vel * dt;

            // Broadcast state
            if (chaos_mode && distribution(generator) < 0.05) continue; // 5% Loss

            WorldUpdatePacket pkt;
            pkt.header = 0x55AA55AA;
            pkt.seq = ++global_seq;
            pkt.train_id = i;
            pkt.pos = fleet[i].pos;
            pkt.vel = fleet[i].vel;
            pkt.accel = fleet[i].accel;
            
            sendto(send_sd, &pkt, sizeof(pkt), 0, (struct sockaddr*)&group_addr, sizeof(group_addr));
        }

        // Console Dashboard
        std::printf("\rWorld Time: %u | L_Pos: %.1f | L_Vel: %.1f | L_Force: %.0f    ", 
                    global_seq, fleet[0].pos, fleet[0].vel*3.6, fleet[0].cmd_force/1000);
        std::fflush(stdout);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    return 0;
}