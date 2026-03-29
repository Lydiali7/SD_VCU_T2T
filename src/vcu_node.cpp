#include <iostream>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <algorithm>
#include <chrono>
#include "perception.hpp"

class KalmanFilter1D {
private:
    double x, p, q, r; 
public:
    KalmanFilter1D(double init_x, double process_noise, double meas_noise, double est_error) {
        x = init_x; q = process_noise; r = meas_noise; p = est_error;
    }
    double update(double measurement, double dt) {
        p = p + q * dt;
        double k = p / (p + r); 
        x = x + k * (measurement - x);
        p = (1.0 - k) * p;
        return x;
    }
};

int main(int argc, char* argv[]) {
    if(argc != 2) return 1;
    int id = std::atoi(argv[1]);
    const float MASS = 5000000.0f;

    int sd = socket(AF_INET, SOCK_DGRAM, 0);
    int reuse = 1;
    setsockopt(sd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET; 
    addr.sin_port = htons(MULTICAST_PORT);
    addr.sin_addr.s_addr = INADDR_ANY;
    bind(sd, (struct sockaddr*)&addr, sizeof(addr));
    
    struct ip_mreq mreq;
    mreq.imr_multiaddr.s_addr = inet_addr(MULTICAST_GROUP);
    mreq.imr_interface.s_addr = htonl(INADDR_ANY);
    setsockopt(sd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq));
    
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 10000; 
    setsockopt(sd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    int srv_sd = socket(AF_INET, SOCK_DGRAM, 0); 
    struct sockaddr_in srv;
    memset(&srv, 0, sizeof(srv));
    srv.sin_family = AF_INET; 
    srv.sin_port = htons(VCU_REPORT_PORT);
    srv.sin_addr.s_addr = inet_addr("127.0.0.1");

    KalmanFilter1D kf_my_pos(10000.0 - id*60.0, 0.01, 1.5, 1.0);
    KalmanFilter1D kf_my_vel(0.0, 0.05, 0.8, 1.0);
    KalmanFilter1D kf_front_pos(10000.0 - (id-1)*60.0, 0.01, 1.5, 1.0);
    KalmanFilter1D kf_front_vel(0.0, 0.05, 0.8, 1.0);
    KalmanFilter1D kf_front_acc(0.0, 0.1, 1.0, 1.0);

    RawT2TPacket pkt;
    SensorData sensor;
    uint32_t last_seq_self = 0, last_seq_front = 0;
    
    double my_p = 10000.0 - id*60.0, my_v = 0.0;
    double c_pos = 10000.0 - (id-1)*60.0, c_vel = 0.0, c_acc = 0.0;
    float cur_f = 0, i_err = 0;
    auto last_recv_front = std::chrono::steady_clock::now();

    while(true) {
        int bytes = recv(sd, &pkt, sizeof(pkt), 0);
        auto now = std::chrono::steady_clock::now();
        
        if (bytes > 0 && pkt.header == 0x55AA55AA) {
            if (pkt.sender_id == (uint32_t)id) {
                if (PerceptionEngine::fast_unpack(pkt, sensor, last_seq_self)) {
                    my_p = kf_my_pos.update(sensor.distances[0], 0.01);
                    my_v = kf_my_vel.update(sensor.distances[1], 0.01);
                }
            } else if (pkt.sender_id == (uint32_t)(id-1)) {
                last_recv_front = now;
                if (PerceptionEngine::fast_unpack(pkt, sensor, last_seq_front)) {
                    SensorData pathB = sensor; 
                    if (PerceptionEngine::is_system_safe(sensor, pathB)) {
                        c_pos = kf_front_pos.update(sensor.distances[0], 0.01);
                        c_vel = kf_front_vel.update(sensor.distances[1], 0.01);
                        c_acc = kf_front_acc.update(sensor.distances[2], 0.01);
                    }
                }
            }
        }

        bool state_updated = (bytes > 0 && pkt.header == 0x55AA55AA && pkt.sender_id == (uint32_t)id);
        bool is_timeout = (bytes < 0);

        if (state_updated || is_timeout) {
            double elapsed_since_front = std::chrono::duration<double>(now - last_recv_front).count();
            
            if (elapsed_since_front > 0.5) { 
                cur_f = MASS * -0.05f; 
                ForceReportPacket rep = {(uint32_t)id, cur_f, 1};
                sendto(srv_sd, &rep, sizeof(rep), 0, (struct sockaddr*)&srv, sizeof(srv));
                continue;
            }

            if (state_updated) {
                float ds = PerceptionEngine::calculate_safe_dist(my_v, c_vel, TrainType::HEAVY_HAUL);
                float da = c_pos - my_p;
                
                float dynamic_target = ds + c_vel * 0.35f + 10.0f;
                float target = std::max(50.0f, dynamic_target); 

                float err = da - target;
                if(std::abs(err) < 0.5f) err = 0; 
                
                // Anti-Windup Logic
                if (std::abs(err) < 10.0f) i_err = std::clamp(i_err + err * 0.005f, -15.0f, 15.0f);
                else i_err = 0.0f; 
                
                float raw_f = 0.0f;
                
                // [CRITICAL FIX] Avoid "Ghost Braking Cascade" when closing gap rapidly
                if(da < ds * 0.8f) { 
                    raw_f = -1250000.0f;
                } else {
                    // [CRITICAL FIX] Heavy-haul Soft PID parameters
                    float accel_cmd = c_acc + err * 0.005f + i_err * 0.005f + (c_vel - my_v) * 0.05f;
                    
                    if (my_v > 25.0f && accel_cmd > 0.0f) accel_cmd = 0.0f; 
                    
                    raw_f = std::clamp(MASS * accel_cmd, -1250000.0f, 1000000.0f);
                }

                // Smooth out the force outputs
                cur_f = 0.02f * raw_f + 0.98f * cur_f;
                float max_d = MASS * 0.25f * 0.01f; 
                cur_f = std::clamp(cur_f, cur_f - max_d, cur_f + max_d);

                ForceReportPacket rep = {(uint32_t)id, cur_f, 0};
                sendto(srv_sd, &rep, sizeof(rep), 0, (struct sockaddr*)&srv, sizeof(srv));
            }
        }
    }
    return 0;
}