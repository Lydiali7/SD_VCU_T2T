#include <iostream>
#include <vector>
#include <thread>
#include <fstream>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>
#include <chrono>
#include <random>
#include <nmmintrin.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include "perception.hpp"

std::vector<TrainState> fleet(5);
std::vector<uint32_t> fleet_status(5, 0); 
float forces[5] = {0}; 
bool is_eb_locked[5] = {false}; 

void receive_forces_thread(int sd) {
    ForceReportPacket report;
    while (recv(sd, &report, sizeof(report), 0) > 0) {
        if (report.train_id > 0 && report.train_id < 5) {
            if (report.status_flag == 1) {
                is_eb_locked[report.train_id] = true;
            }
            
            if (is_eb_locked[report.train_id]) {
                forces[report.train_id] = -1250000.0f; 
                fleet_status[report.train_id] = 1;
            } else {
                forces[report.train_id] = report.cmd_force; 
                fleet_status[report.train_id] = report.status_flag;
            }
        }
    }
}

void broadcast_hardware_signal(int fd, TrainState& state) {
    if (fd < 0) return;
    MVB_Hardware_Frame hw_frame;
    std::memset(&hw_frame, 0, sizeof(hw_frame));
    
    hw_frame.head[0] = 0xFE; 
    hw_frame.head[1] = 0xFA;
    hw_frame.cmd_type = 0x05; 
    hw_frame.port_addr = (uint8_t)state.id;
    
    float target_pressure = std::abs(state.cmd_force / 10000.0f);
    state.smooth_brake_pressure += (target_pressure - state.smooth_brake_pressure) * 0.05f; 

    hw_frame.raw_speed = static_cast<uint16_t>(state.vel * 100.0f);
    hw_frame.brake_press = static_cast<uint16_t>(state.smooth_brake_pressure * 10.0f);
    hw_frame.io_status = fleet_status[state.id];

    ssize_t ret = write(fd, &hw_frame, sizeof(hw_frame));
    if (ret < 0) {}
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

int main() {
    int mvb_fd = open("/tmp/v-bus-m", O_RDWR | O_NOCTTY | O_NONBLOCK);
    int vcan0_fd = init_can_fd("vcan0");
    int vcan1_fd = init_can_fd("vcan1");
    if(vcan0_fd < 0 || vcan1_fd < 0) std::cerr << "[SERVER] Warning: Virtual CAN interfaces not found." << std::endl;
    if (mvb_fd < 0) std::cerr << "[SERVER] Warning: Virtual MVB Bus not found." << std::endl;

    std::ofstream log_file("fleet_log.csv");
    log_file << "Time,ID,Pos,Vel,Gap,Safe,Force,BrakePress" << std::endl;

    for(int i = 0; i < 5; i++) {
        fleet[i].id = i;
        fleet[i].mass = 5000000.0f;
        fleet[i].pos = 10000.0 - i * 200.0; 
        fleet[i].vel = 0.0;
        fleet[i].cmd_force = 0.0;
        fleet[i].smooth_brake_pressure = 0.0f;
    }

    int send_sd = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in group_addr;
    memset(&group_addr, 0, sizeof(group_addr));
    group_addr.sin_family = AF_INET;
    group_addr.sin_addr.s_addr = inet_addr(MULTICAST_GROUP);
    group_addr.sin_port = htons(MULTICAST_PORT);

    int recv_sd = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in srv_addr;
    memset(&srv_addr, 0, sizeof(srv_addr));
    srv_addr.sin_family = AF_INET;
    srv_addr.sin_addr.s_addr = INADDR_ANY;
    srv_addr.sin_port = htons(VCU_REPORT_PORT);
    bind(recv_sd, (struct sockaddr*)&srv_addr, sizeof(srv_addr));
    std::thread(receive_forces_thread, recv_sd).detach();

    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> pos_noise(0.0, 1.5);   
    std::normal_distribution<double> vel_noise(0.0, 0.8);   
    std::normal_distribution<double> accel_noise(0.0, 0.2); 
    std::uniform_real_distribution<double> pkt_loss(0.0, 1.0);

    const double dt = 0.01;
    uint32_t step = 0;
    auto start_time = std::chrono::steady_clock::now();

    while (true) {
        double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
        bool collision = false;
        for (int i = 1; i < 5; ++i) {
            if (fleet[i-1].pos - fleet[i].pos < -1.0) collision = true;
        }

        if (collision) fleet[0].cmd_force = -1250000.0f; 
        else {
            if (elapsed < 5.0) fleet[0].cmd_force = 0.0f; 
            else if (elapsed < 350.0) { 
                if (fleet[0].vel < 22.22) fleet[0].cmd_force = 400000.0f; 
                else if (fleet[0].vel > 22.3) fleet[0].cmd_force = -100000.0f;
                else fleet[0].cmd_force = 60000.0f;
            } else fleet[0].cmd_force = -1250000.0f;
        }

        for (int i = 0; i < 5; ++i) {
            if (i > 0) fleet[i].cmd_force = forces[i];

            float v = fleet[i].vel;
            float res = (v > 0.1f) ? (18000.0f + 450.0f * v + 15.5f * v * v) : 0.0f;
            fleet[i].accel = (fleet[i].cmd_force - res) / fleet[i].mass;
            fleet[i].vel = std::max(0.0, fleet[i].vel + fleet[i].accel * dt);
            fleet[i].pos += fleet[i].vel * dt;

            broadcast_hardware_signal(mvb_fd, fleet[i]);

            float gap = (i == 0) ? 0.0f : fleet[i-1].pos - fleet[i].pos;
            float safe_dist = (i == 0) ? 0.0f : PerceptionEngine::calculate_safe_dist(fleet[i].vel, fleet[i-1].vel, TrainType::HEAVY_HAUL);
            
            log_file << elapsed << "," << i << "," << fleet[i].pos << "," << fleet[i].vel << "," 
                     << gap << "," << safe_dist << "," << fleet[i].cmd_force << "," 
                     << fleet[i].smooth_brake_pressure << std::endl;

            if (i > 0) {
                float front_v = fleet[i-1].vel;
                CANPacket path_a = {0xAA55, (uint16_t)i, (uint16_t)step, {gap, v, front_v, 0}, 0xFFFF, 0};
                CANPacket path_b = path_a;

                struct canfd_frame frame_a, frame_b;
                memset(&frame_a, 0, sizeof(frame_a));
                memset(&frame_b, 0, sizeof(frame_b));

                frame_a.can_id = i; 
                frame_b.can_id = i;
                frame_a.len = 64;   
                frame_b.len = 64;

                memcpy(frame_a.data, &path_a, 64);
                memcpy(frame_b.data, &path_b, 64);

                ssize_t ret_a = write(vcan0_fd, &frame_a, sizeof(struct canfd_frame));
                ssize_t ret_b = write(vcan1_fd, &frame_b, sizeof(struct canfd_frame));
                if (ret_a < 0 || ret_b < 0) {}
            }

            if (step % 10 == 0) {
                if (pkt_loss(gen) < 0.05) continue; 

                uint64_t v_enc = (uint64_t)((std::max(0.0, fleet[i].vel + vel_noise(gen))) * 100.0);
                uint64_t p_enc = (uint64_t)((fleet[i].pos + pos_noise(gen)) * 100.0);
                uint64_t a_enc = (uint64_t)((uint16_t)(int16_t)((fleet[i].accel + accel_noise(gen)) * 100.0));

                RawT2TPacket pkt = {(uint32_t)i, 0x55AA55AA, step, v_enc | (p_enc << 16) | (a_enc << 48), 0};
                pkt.crc = _mm_crc32_u64(0, pkt.payload); 
                sendto(send_sd, &pkt, sizeof(pkt), 0, (struct sockaddr*)&group_addr, sizeof(group_addr));
            }
        }

        if (step % 20 == 0) {
            std::printf("\033[2J\033[H=== SD-VCU UDP AND SERIAL HARDWARE MONITOR ===\n");
            std::printf("Sim Time: %7.2fs | Target Speed: 80km/h\n", elapsed);
            if (collision) std::printf(">>> CRITICAL ALARM: COLLISION DETECTED. FLEET HALTED. <<<\n");
            std::printf("ID | TruePos(m)| TrueVel(km/h)|  Gap(m) | Safe(m) | Force(kN) | Status\n");
            for (int i = 0; i < 5; ++i) {
                float gap = (i == 0) ? 0.0f : fleet[i-1].pos - fleet[i].pos;
                float s_dist = (i == 0) ? 0.0f : PerceptionEngine::calculate_safe_dist(fleet[i].vel, fleet[i-1].vel, TrainType::HEAVY_HAUL);
                
                // 【修复：UI界面状态颜色增强显示】
                const char* status_str = "\033[32mOK\033[0m";
                if (is_eb_locked[i]) {
                    status_str = "\033[31mHW_FAULT\033[0m"; // 硬件熔断
                } else if (fleet[i].cmd_force <= -1200000.0f) {
                    status_str = "\033[33mATP_TRIP\033[0m"; // 距离超限ATP介入
                }

                std::printf("[%d] |  %7.1f |    %9.1f | %7.1f | %7.1f | %9.1f | %s\n", 
                            i, fleet[i].pos, fleet[i].vel*3.6, gap, s_dist, fleet[i].cmd_force/1000.0, status_str);
            }
        }
        step++;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (mvb_fd >= 0) close(mvb_fd);
    return 0;
}