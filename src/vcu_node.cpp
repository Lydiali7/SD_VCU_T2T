#include <iostream>
#include <chrono>
#include <thread>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <algorithm>
#include "perception.hpp"
#include "network_proto.hpp"

class KalmanFilter1D {
private:
    double x; // State estimate
    double p; // Estimate uncertainty
    double q; // Process noise covariance (system trust)
    double r; // Measurement noise covariance (sensor trust)

public:
    KalmanFilter1D(double init_x, double process_noise, double meas_noise, double est_error) {
        x = init_x;
        q = process_noise;
        r = meas_noise;
        p = est_error;
    }

    double update(double measurement, double dt) {
        // Predict step
        p = p + q * dt;
        // Update step
        double k = p / (p + r); // Kalman Gain
        x = x + k * (measurement - x);
        p = (1.0 - k) * p;
        return x;
    }
    
    void set_state(double new_x) { x = new_x; }
};

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
    
    float integral_error = 0.0f;
    const float KP = 0.5f;
    const float KI = 0.02f; 
    const float KD = 0.8f;

    // Initialize Kalman Filters for the front train's telemetry
    // Parameters: init_val, process_noise (Q), meas_noise (R), est_error (P)
    KalmanFilter1D kf_front_pos(5000.0 - target_front_id * 150.0, 0.1, 1.5, 1.0);
    KalmanFilter1D kf_front_vel(0.0, 0.5, 0.8, 1.0);
    KalmanFilter1D kf_front_acc(0.0, 1.0, 0.2, 1.0);

    int recv_sd = init_multicast_recv();
    struct sockaddr_in server_addr;
    int send_sd = init_force_sender(server_addr);
    WorldUpdatePacket pkt;
    uint32_t last_seq = 0;

    std::printf("VCU Node [%d] Started. Filtering active.\n", my_id);

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
                
                // Apply Kalman Filter to dirty incoming data
                double clean_pos = kf_front_pos.update(pkt.pos, DT);
                double clean_vel = kf_front_vel.update(pkt.vel, DT);
                double clean_acc = kf_front_acc.update(pkt.accel, DT);
                
                float d_safe = PerceptionEngine::calculate_safe_dist(my_vel, clean_vel, TrainType::EMU_DISTRIBUTED);
                float d_actual = clean_pos - my_pos; 
                float target_gap = d_safe + 5.0f; 
                float target_force = 0.0f;

                if (pkt.status_flag == 1) {
                    target_force = MASS * -0.8f;
                    my_status = 1; 
                } else if (d_actual < d_safe) {
                    target_force = MASS * -1.3f; 
                } else {
                    float error = d_actual - target_gap;
                    integral_error += error * DT;
                    integral_error = std::clamp(integral_error, -20.0f, 20.0f); 

                    // Use CLEANED data for control
                    float accel_cmd = clean_acc + (error * KP) + (integral_error * KI) + ((clean_vel - my_vel) * KD);
                    target_force = std::clamp(MASS * accel_cmd, MASS * -1.3f, MASS * 1.0f);
                }

                float max_force_delta = MASS * MAX_JERK * DT; 
                if (target_force > current_actual_force + max_force_delta) current_actual_force += max_force_delta;
                else if (target_force < current_actual_force - max_force_delta) current_actual_force -= max_force_delta;
                else current_actual_force = target_force;     

                ForceReportPacket report = {(uint32_t)my_id, current_actual_force, my_status};
                sendto(send_sd, &report, sizeof(report), 0, (struct sockaddr*)&server_addr, sizeof(server_addr));

                if (pkt.seq % 50 == 0) {
                    std::printf("T%d | RawVel: %5.1f | CleanVel: %5.1f | Gap: %5.1f | Out: %5.0fkN\n", 
                                target_front_id, pkt.vel*3.6, clean_vel*3.6, d_actual, current_actual_force/1000);
                }
            }
        } else if (bytes < 0) {
            float target_force = MASS * -0.5f;
            my_status = 1; 
            
            float max_force_delta = MASS * MAX_JERK * DT; 
            if (target_force > current_actual_force + max_force_delta) current_actual_force += max_force_delta;
            else if (target_force < current_actual_force - max_force_delta) current_actual_force -= max_force_delta;
            else current_actual_force = target_force;
            
            ForceReportPacket report = {(uint32_t)my_id, current_actual_force, my_status};
            sendto(send_sd, &report, sizeof(report), 0, (struct sockaddr*)&server_addr, sizeof(server_addr));
            std::printf("WARNING: Timeout. Degraded mode. Out: %5.0fkN\n", current_actual_force/1000);
        }
    }
    return 0;
}