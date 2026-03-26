#include <iostream>
#include <chrono>
#include <thread>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <algorithm>
#include "perception.hpp"
#include "network_proto.hpp"

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
    struct timeval tv; tv.tv_sec = 0; tv.tv_usec = 50000; 
    setsockopt(sd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    return sd;
}

int init_force_sender(struct sockaddr_in& server_addr) {
    int sd = socket(AF_INET, SOCK_DGRAM, 0);
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = inet_addr("127.0.0.1"); 
    server_addr.sin_port = htons(VCU_REPORT_PORT);
    return sd;
}

int main(int argc, char* argv[]) {
    if (argc != 2) return 1;
    int my_id = std::stoi(argv[1]);
    int target_front_id = my_id - 1; 
    
    const float MASS = 450000.0f;
    const float DT = 0.01f;          
    const float MAX_JERK = 0.75f;    
    
    double my_vel = 0.0; // Phase 1: Start from standstill
    double my_pos = 5000.0 - my_id * 150.0; // Phase 1: 150m initial gap
    float current_actual_force = 0.0f; 

    int recv_sd = init_multicast_recv();
    struct sockaddr_in server_addr;
    int send_sd = init_force_sender(server_addr);
    WorldUpdatePacket pkt;
    uint32_t last_seq = 0;

    std::printf("VCU Node [%d] Started. Listening for Train %d...\n", my_id, target_front_id);

    while (true) {
        int bytes = recv(recv_sd, &pkt, sizeof(pkt), 0);
        
        if (bytes > 0 && pkt.header == 0x55AA55AA) {
            if (pkt.train_id == (uint32_t)my_id) {
                my_vel = pkt.vel;
                my_pos = pkt.pos; 
            }
            
            if (pkt.train_id == (uint32_t)target_front_id && pkt.seq > last_seq) {
                last_seq = pkt.seq;
                
                float d_safe = PerceptionEngine::calculate_safe_dist(my_vel, pkt.vel, TrainType::EMU_DISTRIBUTED);
                float d_actual = pkt.pos - my_pos; 
                float target_gap = d_safe + 5.0f; // Maintain a 5m buffer above safe line
                float target_force = 0.0f;

                if (d_actual < d_safe) {
                    target_force = MASS * -1.3f; 
                } else {
                    float accel_cmd = pkt.accel + (d_actual - target_gap) * 0.5f + (pkt.vel - my_vel) * 0.8f;
                    target_force = std::clamp(MASS * accel_cmd, MASS * -1.3f, MASS * 1.0f);
                }

                float max_force_delta = MASS * MAX_JERK * DT; 
                if (target_force > current_actual_force + max_force_delta) current_actual_force += max_force_delta;
                else if (target_force < current_actual_force - max_force_delta) current_actual_force -= max_force_delta;
                else current_actual_force = target_force;     

                ForceReportPacket report = {(uint32_t)my_id, current_actual_force};
                sendto(send_sd, &report, sizeof(report), 0, (struct sockaddr*)&server_addr, sizeof(server_addr));

                if (pkt.seq % 50 == 0) {
                    std::printf("Target: T%d | Vel: %5.1f | Gap: %5.1f | Safe: %5.1f | Output: %5.0f\n", 
                                target_front_id, pkt.vel*3.6, d_actual, d_safe, current_actual_force/1000);
                }
            }
        } else if (bytes < 0) {
            float target_force = MASS * -0.5f;
            float max_force_delta = MASS * MAX_JERK * DT; 
            if (target_force > current_actual_force + max_force_delta) current_actual_force += max_force_delta;
            else if (target_force < current_actual_force - max_force_delta) current_actual_force -= max_force_delta;
            else current_actual_force = target_force;
            
            ForceReportPacket report = {(uint32_t)my_id, current_actual_force};
            sendto(send_sd, &report, sizeof(report), 0, (struct sockaddr*)&server_addr, sizeof(server_addr));
            std::printf("Warning: Timeout detected. Entering degraded mode. Output: %5.0f\n", current_actual_force/1000);
        }
    }
    return 0;
}