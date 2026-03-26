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

double get_global_time() {
    auto now = std::chrono::system_clock::now();
    return std::chrono::duration<double>(now.time_since_epoch()).count();
}

int main(int argc, char* argv[]) {
    if (argc != 2) return 1;
    int my_id = std::stoi(argv[1]);
    int target_front_id = my_id - 1; 
    
    const float MASS = 450000.0f;
    const float DT = 0.01f;          
    const float MAX_JERK = 0.75f;    
    
    double my_vel = 0.0; 
    double my_pos = 5000.0 - my_id * 150.0; 
    float current_actual_force = 0.0f; 
    
    // PID Controller variables
    float integral_error = 0.0f;
    const float KP = 0.5f;
    const float KI = 0.02f; // Small integral gain to fight air resistance
    const float KD = 0.8f;

    int recv_sd = init_multicast_recv();
    struct sockaddr_in server_addr;
    int send_sd = init_force_sender(server_addr);
    WorldUpdatePacket pkt;
    uint32_t last_seq = 0;

    std::printf("VCU Node [%d] Started. Listening for Train %d...\n", my_id, target_front_id);

    while (true) {
        int bytes = recv(recv_sd, &pkt, sizeof(pkt), 0);
        uint32_t my_status = 0; 
        
        if (bytes > 0 && pkt.header == 0x55AA55AA) {
            double latency_ms = (get_global_time() - pkt.timestamp) * 1000.0;

            if (pkt.train_id == (uint32_t)my_id) {
                my_vel = pkt.vel;
                my_pos = pkt.pos; 
            }
            
            if (pkt.train_id == (uint32_t)target_front_id && pkt.seq > last_seq) {
                last_seq = pkt.seq;
                
                float d_safe = PerceptionEngine::calculate_safe_dist(my_vel, pkt.vel, TrainType::EMU_DISTRIBUTED);
                float d_actual = pkt.pos - my_pos; 
                float target_gap = d_safe + 5.0f; 
                float target_force = 0.0f;

                // Cooperative Degradation: If front train is degraded, I must be careful
                if (pkt.status_flag == 1) {
                    target_force = MASS * -0.8f;
                    my_status = 1; 
                }
                else if (d_actual < d_safe) {
                    target_force = MASS * -1.3f; 
                } 
                else {
                    float error = d_actual - target_gap;
                    
                    // Anti-windup for Integral term
                    integral_error += error * DT;
                    integral_error = std::clamp(integral_error, -20.0f, 20.0f); 

                    float accel_cmd = pkt.accel + (error * KP) + (integral_error * KI) + ((pkt.vel - my_vel) * KD);
                    target_force = std::clamp(MASS * accel_cmd, MASS * -1.3f, MASS * 1.0f);
                }

                float max_force_delta = MASS * MAX_JERK * DT; 
                if (target_force > current_actual_force + max_force_delta) current_actual_force += max_force_delta;
                else if (target_force < current_actual_force - max_force_delta) current_actual_force -= max_force_delta;
                else current_actual_force = target_force;     

                ForceReportPacket report = {(uint32_t)my_id, current_actual_force, my_status};
                sendto(send_sd, &report, sizeof(report), 0, (struct sockaddr*)&server_addr, sizeof(server_addr));

                if (pkt.seq % 50 == 0) {
                    std::printf("Target: T%d | Latency: %4.1fms | Gap: %5.1f | Safe: %5.1f | Output: %5.0f\n", 
                                target_front_id, latency_ms, d_actual, d_safe, current_actual_force/1000);
                }
            }
        } else if (bytes < 0) {
            float target_force = MASS * -0.5f;
            my_status = 1; // Mark myself as DEGRADED due to timeout
            
            float max_force_delta = MASS * MAX_JERK * DT; 
            if (target_force > current_actual_force + max_force_delta) current_actual_force += max_force_delta;
            else if (target_force < current_actual_force - max_force_delta) current_actual_force -= max_force_delta;
            else current_actual_force = target_force;
            
            ForceReportPacket report = {(uint32_t)my_id, current_actual_force, my_status};
            sendto(send_sd, &report, sizeof(report), 0, (struct sockaddr*)&server_addr, sizeof(server_addr));
            std::printf("[WARNING] Timeout detected. Entering degraded mode. Output: %5.0f\n", current_actual_force/1000);
        }
    }
    return 0;
}