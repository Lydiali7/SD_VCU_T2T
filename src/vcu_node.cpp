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
#include <thread>
#include "perception.hpp"
#include "network_proto.hpp"
#include "t2t_radio.hpp" // 引入射频 HAL 层

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

    // ==========================================
    // 通道 1: 宽带 Wi-Fi/5G (UDP Socket 模拟)
    // ==========================================
    int wifi_sd = socket(AF_INET, SOCK_DGRAM, 0);
    int reuse = 1;
    setsockopt(wifi_sd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    struct sockaddr_in wifi_addr;
    memset(&wifi_addr, 0, sizeof(wifi_addr));
    wifi_addr.sin_family = AF_INET;
    wifi_addr.sin_port = htons(MULTICAST_PORT); 
    wifi_addr.sin_addr.s_addr = INADDR_ANY;
    bind(wifi_sd, (struct sockaddr*)&wifi_addr, sizeof(wifi_addr));
    fcntl(wifi_sd, F_SETFL, fcntl(wifi_sd, F_GETFL, 0) | O_NONBLOCK);

    struct ip_mreq mreq;
    mreq.imr_multiaddr.s_addr = inet_addr(MULTICAST_GROUP);
    mreq.imr_interface.s_addr = htonl(INADDR_ANY);
    setsockopt(wifi_sd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq));

    // ==========================================
    // 通道 2: 窄带保底通信生命线 LoRa (虚拟串口)
    // ==========================================
    T2T_Radio t2t_radio;
    if (!t2t_radio.init("/tmp/lora-node", B115200, 0x17)) {
        std::cerr << "[VCU " << id << "] Error: 无法打开 /tmp/lora-node，请确认 socat 已运行！\n";
        return 1;
    }

    // UI 上报套接字
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
    
    // 状态机与仲裁机制状态
    PlatoonState current_state = (id == 0) ? PlatoonState::LEADER_NOMINAL : PlatoonState::FOLLOWER;
    auto sim_start_time = std::chrono::steady_clock::now();
    
    auto last_wifi_time = std::chrono::steady_clock::now();
    bool using_lora_fallback = false;
    uint8_t udp_buf[256];

    while(true) {
        auto now = std::chrono::steady_clock::now();
        bool state_updated = false;
        double elapsed = std::chrono::duration<double>(now - sim_start_time).count();

        // 状态机：第 60 秒强行解编
        if (id == 3 && elapsed > 60.0 && current_state == PlatoonState::FOLLOWER) {
            current_state = PlatoonState::DECOUPLING;
            std::cout << "\n[DISPATCH COMMAND] VCU 3 initiating DECOUPLING! Target safe gap: 800m.\n";
            i_err = 0.0f; 
        }

        // 读取硬件总线
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
            if (frame_a.can_id == (uint32_t)id) { std::memcpy(&path_a, frame_a.data, 64); got_a = true; }
        }
        while (read(vcan1_fd, &frame_b, sizeof(frame_b)) > 0) {
            if (frame_b.can_id == (uint32_t)id) { std::memcpy(&path_b, frame_b.data, 64); got_b = true; }
        }

        if (got_a && got_b) {
            if (path_a.seq_num == path_b.seq_num) {
                sdvcu.process_sensors(path_a, path_b, MASS);
                got_a = false; got_b = false; desync_counter = 0;
            } else {
                if (path_a.seq_num < path_b.seq_num) got_a = false; else got_b = false;
            }
        } else if (got_a || got_b) desync_counter++;

        if (desync_counter > 10) {
            CANPacket dummy = {0}; 
            sdvcu.process_sensors(got_a ? path_a : dummy, got_b ? path_b : dummy, MASS);
            desync_counter = 0; 
        }

        // ==========================================
        // 【核心异构网络仲裁】：优先 Wi-Fi，超时降级 LoRa
        // ==========================================
        
        // 提取数据处理逻辑为 Lambda 闭包，确保处理【每一个】到来的封包，杜绝覆盖丢包
        auto process_packet = [&](const RawT2TPacket& p) {
            if (p.sender_id == (uint32_t)id && PerceptionEngine::fast_unpack(p, udp_sensor, last_seq_self)) {
                my_p = kf_my_pos.update(udp_sensor.distances[0], 0.1); 
                if (!state_updated) my_v = kf_my_vel.update(udp_sensor.distances[1], 0.1);
                state_updated = true;
            } else if (p.sender_id == (uint32_t)(id-1)) {
                if (PerceptionEngine::fast_unpack(p, udp_sensor, last_seq_front)) {
                    last_recv_front = now;
                    is_network_linked = true;
                    c_pos = kf_front_pos.update(udp_sensor.distances[0], 0.1); 
                    c_vel = kf_front_vel.update(udp_sensor.distances[1], 0.1);
                    c_acc = kf_front_acc.update(udp_sensor.distances[2], 0.1);
                }
            }
        };

        // 1. 尝试监听宽带 (Wi-Fi / UDP)
        int bytes;
        while ((bytes = recv(wifi_sd, udp_buf, sizeof(udp_buf), 0)) > 0) {
            if (bytes == sizeof(RawT2TPacket)) {
                RawT2TPacket* udp_pkt = reinterpret_cast<RawT2TPacket*>(udp_buf);
                if (udp_pkt->header == 0x55AA55AA) {
                    process_packet(*udp_pkt);
                    last_wifi_time = now;
                    if (using_lora_fallback) {
                        std::cout << "\033[32m[NETWORK] Wi-Fi linked. Broadband restored.\033[0m\n";
                        using_lora_fallback = false;
                    }
                }
            }
        }

        // 2. 尝试监听窄带保底 (LoRa / Serial)
        RawT2TPacket lora_pkt;
        while (t2t_radio.receive(lora_pkt)) {
            if (lora_pkt.header == 0x55AA55AA) {
                double time_since_wifi = std::chrono::duration<double>(now - last_wifi_time).count();
                // 只有 Wi-Fi 超过 50ms 没更新，才采纳 LoRa 数据
                if (time_since_wifi > 0.05) { 
                    process_packet(lora_pkt);
                    if (!using_lora_fallback) {
                        std::cout << "\033[33m[WARNING] Wi-Fi lost. LoRa fallback activated.\033[0m\n";
                        using_lora_fallback = true;
                    }
                }
            }
        }

        // 航位推算
        float dt_loop = 0.002f; 
        my_p += my_v * dt_loop;  
        if (is_network_linked) {
            c_pos += c_vel * dt_loop; 
            c_vel += c_acc * dt_loop;
        }

        double time_since_last_pkt = std::chrono::duration<double>(now - last_recv_front).count();

        // 状态机跃迁检测
        if (current_state == PlatoonState::DECOUPLING) {
            float da = c_pos - my_p;
            if (da > 800.0f) { 
                current_state = PlatoonState::LEADER_NEW;
                std::cout << "\n[STATE MACHINE] VCU 3 DECOUPLING COMPLETE. Promoted to Independent Leader!\n";
                i_err = 0.0f; 
            }
        }

        // 核心控制逻辑
        if (is_network_linked && time_since_last_pkt > 1.0 && current_state == PlatoonState::FOLLOWER) { 
            is_network_linked = false;
            cur_f = -1250000.0f; 
        } 
        else if (sdvcu.is_eb()) {
            cur_f = -1250000.0f; // 瞬间爆发，无低通滤波
        } 
        else if (state_updated || current_state != PlatoonState::FOLLOWER) {
            float ds = PerceptionEngine::calculate_safe_dist(my_v, c_vel, TrainType::HEAVY_HAUL);
            float da = c_pos - my_p;
            float raw_f = 0.0f;
            
            if (sdvcu.is_atp_braking(da) || (da <= ds && current_state == PlatoonState::FOLLOWER)) { 
                raw_f = -1250000.0f;
                cur_f = raw_f; // 绕过低通滤波
                i_err = 0.0f;
            } else {
                if (current_state == PlatoonState::FOLLOWER) {
                    float dynamic_target = ds + c_vel * 0.35f + 30.0f; 
                    float target = std::max(180.0f, dynamic_target); 
                    float err = da - target;
                    if(std::abs(err) < 0.5f) err = 0; 
                    if (std::abs(err) < 10.0f) i_err = std::clamp(i_err + err * 0.005f, -15.0f, 15.0f);
                    else i_err = 0.0f; 
                    float accel_cmd = c_acc + err * 0.005f + i_err * 0.005f + (c_vel - my_v) * 0.05f;
                    if (my_v > 25.0f && accel_cmd > 0.0f) accel_cmd = 0.0f; 
                    raw_f = std::clamp(MASS * accel_cmd, -1250000.0f, 1000000.0f);
                } else if (current_state == PlatoonState::DECOUPLING || current_state == PlatoonState::LEADER_NEW) {
                    //pinghua减速，防止常用制动越界触发 ATP 报警
                    float target_v;
                    if (current_state == PlatoonState::DECOUPLING) {
                        target_v = std::max(0.0, my_v - 0.5); 
                    } else {
                        target_v = 22.22f; 
                    }
                    float err_v = target_v - my_v;
                    i_err = std::clamp(i_err + err_v * 0.005f, -10.0f, 10.0f);
                    float accel_cmd = err_v * 0.1f + i_err * 0.01f;
                    
                    // 如果是脱离状态，最大刹车力限制为 -80万牛 (常用制动)
                    float min_force = (current_state == PlatoonState::DECOUPLING) ? -800000.0f : -1250000.0f;
                    raw_f = std::clamp(MASS * accel_cmd, min_force, 1000000.0f);
                }

                cur_f = 0.02f * raw_f + 0.98f * cur_f;
                float max_d = MASS * 0.25f * 0.01f; 
                cur_f = std::clamp(cur_f, cur_f - max_d, cur_f + max_d);
            }
        }

        // UI 状态机标志汇报
        uint32_t report_status = 0; 
        if (sdvcu.is_eb() || cur_f <= -1200000.0f) report_status = 1; 
        else if (current_state == PlatoonState::DECOUPLING) report_status = 2;
        else if (current_state == PlatoonState::LEADER_NEW) report_status = 3;

        ForceReportPacket rep = {(uint32_t)id, cur_f, report_status};
        sendto(srv_sd, &rep, sizeof(rep), 0, (struct sockaddr*)&srv, sizeof(srv));

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    
    if (hw_fd >= 0) close(hw_fd);
    if (wifi_sd >= 0) close(wifi_sd);
    return 0;
}