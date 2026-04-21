#include <iostream>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <algorithm>
#include <chrono>
#include <nmmintrin.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include "perception.hpp"
#include "network_proto.hpp"
#include <thread>

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

int init_hardware_serial(const char* port) {
    int fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd == -1) return -1;
    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    options.c_cflag |= (CLOCAL | CREAD | CS8);
    options.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
    tcsetattr(fd, TCSANOW, &options);
    return fd;
}

int init_can_fd(const char* ifname) {
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;
    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) return -1;
    int enable_canfd = 1;
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd));
    strcpy(ifr.ifr_name, ifname);
    ioctl(s, SIOCGIFINDEX, &ifr);
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    fcntl(s, F_SETFL, fcntl(s, F_GETFL, 0) | O_NONBLOCK);
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) return -1;
    return s;
}

int main(int argc, char* argv[]) {
    if(argc != 2) return 1;
    int id = std::atoi(argv[1]);
    const float MASS = 5000000.0f;

    int hw_fd = init_hardware_serial("/tmp/v-bus-v");
    int vcan0_fd = init_can_fd("vcan0");
    int vcan1_fd = init_can_fd("vcan1");
    
    if (vcan0_fd < 0 || vcan1_fd < 0) {
        std::cerr << "[VCU " << id << "] Error: SocketCAN missing!\n";
        return 1;
    }

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

    int srv_sd = socket(AF_INET, SOCK_DGRAM, 0); 
    struct sockaddr_in srv;
    memset(&srv, 0, sizeof(srv));
    srv.sin_family = AF_INET; 
    srv.sin_port = htons(VCU_REPORT_PORT);
    srv.sin_addr.s_addr = inet_addr("127.0.0.1");

    KalmanFilter1D kf_my_pos(10000.0 - id*200.0, 0.01, 1.5, 1.0);
    KalmanFilter1D kf_my_vel(0.0, 0.05, 0.8, 1.0);
    KalmanFilter1D kf_front_pos(10000.0 - (id-1)*200.0, 0.01, 1.5, 1.0);
    KalmanFilter1D kf_front_vel(0.0, 0.05, 0.8, 1.0);
    KalmanFilter1D kf_front_acc(0.0, 0.1, 1.0, 1.0);

    SDVCU_Core sdvcu;
    alignas(64) uint8_t recv_buf[512]; 

    SensorData udp_sensor, hw_sensor;
    uint32_t last_seq_self = 0, last_seq_front = 0;
    
    double my_p = 10000.0 - id*200.0, my_v = 0.0;
    double c_pos = 10000.0 - (id-1)*200.0, c_vel = 0.0, c_acc = 0.0;
    float cur_f = 0, i_err = 0;
    
    auto last_recv_front = std::chrono::steady_clock::now();
    bool is_network_linked = false;
    MVB_Hardware_Frame hw_buffer;

    CANPacket path_a, path_b;
    bool got_a = false, got_b = false;
    uint32_t desync_counter = 0; 

    while(true) {
        auto now = std::chrono::steady_clock::now();
        bool state_updated = false;

        if (hw_fd >= 0) {
            int hw_bytes = read(hw_fd, &hw_buffer, sizeof(hw_buffer));
            if (hw_bytes == sizeof(MVB_Hardware_Frame)) {
                if (hw_buffer.port_addr == id && PerceptionEngine::hardware_unpack(hw_buffer, hw_sensor)) {
                    my_v = kf_my_vel.update(hw_sensor.distances[1], 0.01);
                    state_updated = true;
                }
            } else if (hw_bytes > 0) tcflush(hw_fd, TCIFLUSH);
        }

        struct canfd_frame frame_a, frame_b;

        while (read(vcan0_fd, &frame_a, sizeof(frame_a)) > 0) {
            if (frame_a.can_id == (uint32_t)id) {
                std::memcpy(&path_a, frame_a.data, 64);
                got_a = true;
            }
        }
        while (read(vcan1_fd, &frame_b, sizeof(frame_b)) > 0) {
            if (frame_b.can_id == (uint32_t)id) {
                std::memcpy(&path_b, frame_b.data, 64);
                got_b = true;
            }
        }

        if (got_a && got_b) {
            if (path_a.seq_num == path_b.seq_num) {
                if (path_a.seq_num % 50 == 0) {
                    std::cout << "[CAN-FD OBSERVE] VCU " << id 
                              << " | Seq: " << path_a.seq_num 
                              << " | Lidar Gap: " << path_a.payload[0] << "m"
                              << " | SelfVel: " << path_a.payload[1] << "m/s"
                              << " | FrontVel: " << path_a.payload[2] << "m/s"
                              << " | AVX-512 Verify: OK\n" << std::flush;
                }
                sdvcu.process_sensors(path_a, path_b, MASS);
                got_a = false;
                got_b = false;
                desync_counter = 0;
            } else {
                if (path_a.seq_num < path_b.seq_num) got_a = false;
                else got_b = false;
            }
        } else if (got_a || got_b) {
            desync_counter++;
        }

        if (desync_counter > 10) {
            std::cout << "[VCU " << id << "] Warning: CAN bus persistent desync! Forcing fault check.\n";
            CANPacket dummy = {0}; 
            sdvcu.process_sensors(got_a ? path_a : dummy, got_b ? path_b : dummy, MASS);
            desync_counter = 0; 
        }

        int bytes = recv(sd, recv_buf, sizeof(recv_buf), 0);
        if (bytes == sizeof(RawT2TPacket)) {
            RawT2TPacket* pkt = reinterpret_cast<RawT2TPacket*>(recv_buf);
            if (pkt->header == 0x55AA55AA) {
                if (pkt->sender_id == (uint32_t)id && PerceptionEngine::fast_unpack(*pkt, udp_sensor, last_seq_self)) {
                    my_p = kf_my_pos.update(udp_sensor.distances[0], 0.1); 
                    if (!state_updated) my_v = kf_my_vel.update(udp_sensor.distances[1], 0.1);
                    state_updated = true;
                } else if (pkt->sender_id == (uint32_t)(id-1)) {
                    // Update heartbeat ONLY if sequence valid to prevent false network safety
                    if (PerceptionEngine::fast_unpack(*pkt, udp_sensor, last_seq_front)) {
                        last_recv_front = now;
                        is_network_linked = true;
                        c_pos = kf_front_pos.update(udp_sensor.distances[0], 0.1); 
                        c_vel = kf_front_vel.update(udp_sensor.distances[1], 0.1);
                        c_acc = kf_front_acc.update(udp_sensor.distances[2], 0.1);
                    }
                }
            }
        }

        // Dead Reckoning: Smooth out ghosting during LoRa packet drops
        // CRITICAL FIX: 推算自身位置，避免不对称的位置信息
        float dt_loop = 0.002f; 
        my_p += my_v * dt_loop;  // 推算自身位置
        
        if (is_network_linked) {
            c_pos += c_vel * dt_loop;  // 推算前车位置
            c_vel += c_acc * dt_loop;
        }

        double elapsed = std::chrono::duration<double>(now - last_recv_front).count();

        // CRITICAL FIX: ATP和制动判断不再受state_updated限制，避免控制循环休眠
        // 网络丧失超时检查：始终执行（不等待state_updated）
        if (is_network_linked && elapsed > 1.0) { 
            std::cout << "[VCU " << id << "] NETWORK TIMEOUT: Emergency brake triggered after " 
                      << elapsed << "s\n";
            is_network_linked = false;
            cur_f = -1250000.0f; 
            ForceReportPacket rep = {(uint32_t)id, cur_f, 0};
            sendto(srv_sd, &rep, sizeof(rep), 0, (struct sockaddr*)&srv, sizeof(srv));
        }
        // 紧急制动（ATP）：始终执行（不等待state_updated）
        else if (sdvcu.is_eb()) {
            std::cout << "[VCU " << id << "] ATP TRIGGERED: Emergency brake active\n";
            cur_f = -1250000.0f;  // CRITICAL: 紧急制动时不用滤波器，直接输出满制动
            ForceReportPacket rep = {(uint32_t)id, cur_f, 1}; 
            sendto(srv_sd, &rep, sizeof(rep), 0, (struct sockaddr*)&srv, sizeof(srv));
        } 
        // PID控制：仅在state_updated时执行
        else if (state_updated) {
            float ds = PerceptionEngine::calculate_safe_dist(my_v, c_vel, TrainType::HEAVY_HAUL);
            float da = c_pos - my_p;
            
            float dynamic_target = ds + c_vel * 0.35f + 30.0f; 
            float target = std::max(180.0f, dynamic_target); 

            float err = da - target;
            if(std::abs(err) < 0.5f) err = 0; 
            
            if (std::abs(err) < 10.0f) i_err = std::clamp(i_err + err * 0.005f, -15.0f, 15.0f);
            else i_err = 0.0f; 
            
            float raw_f = 0.0f;
            
            if(sdvcu.is_atp_braking(da) || (da < ds * 0.8f && is_network_linked)) { 
                // CRITICAL FIX: 紧急制动时绕过低通滤波器，瞬间排空空气管路
                raw_f = -1250000.0f;
                cur_f = raw_f;  // 直接设置，不用平滑滤波
            } else {
                float accel_cmd = c_acc + err * 0.005f + i_err * 0.005f + (c_vel - my_v) * 0.05f;
                if (my_v > 25.0f && accel_cmd > 0.0f) accel_cmd = 0.0f; 
                raw_f = std::clamp(MASS * accel_cmd, -1250000.0f, 1000000.0f);
                
                // 仅在正常加速/减速时使用低通滤波器
                cur_f = 0.02f * raw_f + 0.98f * cur_f;
                float max_d = MASS * 0.25f * 0.01f; 
                cur_f = std::clamp(cur_f, cur_f - max_d, cur_f + max_d);
            }

            ForceReportPacket rep = {(uint32_t)id, cur_f, 0};
            sendto(srv_sd, &rep, sizeof(rep), 0, (struct sockaddr*)&srv, sizeof(srv));
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    
    if (hw_fd >= 0) close(hw_fd);
    return 0;
}