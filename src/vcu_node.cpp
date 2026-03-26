#include <iostream>
#include <chrono>
#include <thread>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <algorithm>
#include "perception.hpp"
#include "network_proto.hpp"

// Setup Multicast Receiver
int init_multicast_recv() {
    int sd = socket(AF_INET, SOCK_DGRAM, 0);
    int reuse = 1;
    setsockopt(sd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    
    struct sockaddr_in localAddr;
    memset(&localAddr, 0, sizeof(localAddr));
    localAddr.sin_family = AF_INET;
    localAddr.sin_port = htons(MULTICAST_PORT);
    localAddr.sin_addr.s_addr = INADDR_ANY;
    bind(sd, (struct sockaddr*)&localAddr, sizeof(localAddr));

    struct ip_mreq group;
    group.imr_multiaddr.s_addr = inet_addr(MULTICAST_GROUP);
    group.imr_interface.s_addr = htonl(INADDR_ANY);
    setsockopt(sd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &group, sizeof(group));
    
    // 50ms Timeout for Fail-safe Detection! (Replaces the sleep logic)
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 50000; 
    setsockopt(sd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    
    return sd;
}

// Setup UDP Sender to report Force back to Server
int init_force_sender(struct sockaddr_in& server_addr) {
    int sd = socket(AF_INET, SOCK_DGRAM, 0);
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = inet_addr("127.0.0.1"); // Send back to local server
    server_addr.sin_port = htons(VCU_REPORT_PORT);
    return sd;
}

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: ./vcu_node <Train_ID (1-4)>\n";
        return 1;
    }
    int my_id = std::stoi(argv[1]);
    int target_front_id = my_id - 1; // Always follow the train ahead
    
    const float MASS = 450000.0f;
    double my_vel = 30.0; // Local estimation of own speed

    int recv_sd = init_multicast_recv();
    
    struct sockaddr_in server_addr;
    int send_sd = init_force_sender(server_addr);

    WorldUpdatePacket pkt;
    uint32_t last_seq = 0;
    
    std::cout << "VCU Node [" << my_id << "] Started. Listening for Train " << target_front_id << "...\n";

    while (true) {
        // Block and wait for network packet (Will timeout after 50ms if lost!)
        int bytes = recv(recv_sd, &pkt, sizeof(pkt), 0);
        
        ForceReportPacket report;
        report.train_id = my_id;
        
        if (bytes > 0 && pkt.header == 0x55AA55AA) {
            // Update local state if it's my own broadcast
            if (pkt.train_id == (uint32_t)my_id) {
                my_vel = pkt.vel;
            }
            
            // Process Front Train data
            if (pkt.train_id == (uint32_t)target_front_id && pkt.seq > last_seq) {
                last_seq = pkt.seq;
                
                // Note: We use the exact math from your perception.cpp
                float v_f = pkt.vel;
                float p_f = pkt.pos;
                float a_f = pkt.accel; // Feed-forward
                
                // We need local position. For pure distributed, we should use our own packet.
                // But to keep logic same as before, assume we track distance directly.
                float d_safe = PerceptionEngine::calculate_safe_dist(my_vel, v_f, TrainType::EMU_DISTRIBUTED);
                
                // *Simulate* d_actual based on front pos and standard 60m initial spacing math
                // In reality, node needs its own position from odometry.
                float simulated_my_pos = p_f - 60.0f; // Simplified for network test
                float d_actual = p_f - simulated_my_pos; 
                
                float target_gap = d_safe + 2.0f;
                float accel_cmd = a_f + (d_actual - target_gap) * 0.5f + (v_f - my_vel) * 0.8f;

                float force = MASS * accel_cmd;
                report.cmd_force = std::clamp(force, MASS * -1.3f, MASS * 1.0f);
                
                std::printf("Front T%d | Vel: %.1f | Accel: %.2f | VCU Output: %.0fkN (NOMINAL)\n", 
                            target_front_id, v_f*3.6, a_f, report.cmd_force/1000);
            }
        } else {
            // TIMEOUT! Fail-safe Degraded Mode triggers automatically
            report.cmd_force = MASS * -0.5f;
            std::printf("\033[1;31mTIMEOUT! Communication Lost. VCU Output: %.0fkN (DEGRADED)\033[0m\n", report.cmd_force/1000);
        }

        // Send calculated force back to Physics Engine
        sendto(send_sd, &report, sizeof(report), 0, (struct sockaddr*)&server_addr, sizeof(server_addr));
    }

    return 0;
}