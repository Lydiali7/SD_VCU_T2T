#include <iostream>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>
#include <algorithm>
#include <chrono>
#include <thread>
#include <string>
#include <memory>

#include "network_proto.hpp"
#include "perception.hpp"
#include "t2t_radio.hpp" 
#include "can_hal.hpp"
#include "mvb_hal.hpp"

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
    if(argc < 2) {
        std::cerr << "Usage: ./vcu_node <id> [--bus=can | --bus=mvb | --sim]\n";
        return 1;
    }
    
    int id = std::atoi(argv[1]);
    const float MASS = 5000000.0f; 
    
    std::string bus_mode = "sim"; 
    if (argc >= 3) {
        std::string arg = argv[2];
        if (arg.find("--bus=") == 0) bus_mode = arg.substr(6);
        else if (arg == "--sim") bus_mode = "sim";
    }

    std::cout << "\n======================================================\n";
    std::cout << "[SYSTEM] Starting Heavy-Haul VCU Node ID: " << id << "\n";
    std::cout << "[SYSTEM] Intra-Car Bus Mode: " << bus_mode << "\n";
    std::cout << "======================================================\n";

    std::unique_ptr<ICanDevice> local_can = nullptr;
    std::unique_ptr<MVB_Device> local_mvb = nullptr;
    int sim_fd = -1;

    if (bus_mode == "can") {
        local_can = std::make_unique<UsbCanDevice>();
        if (!local_can->init()) return 1;
    } else if (bus_mode == "mvb") {
        local_mvb = std::make_unique<MVB_Device>();
        if (!local_mvb->init("/dev/ttyWCH0")) return 1;
    } else {
        local_can = std::make_unique<SocketCanDevice>("vcan0");
        local_can->init();
        sim_fd = open("/tmp/v-bus-v", O_RDWR | O_NOCTTY | O_NONBLOCK);
    }

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

    T2T_Radio t2t_radio;
    const char* lora_port = (bus_mode == "sim") ? "/tmp/lora-node" : "/dev/ttyS1";
    if (!t2t_radio.init(lora_port, B115200, 0x17)) {
        std::cerr << "[VCU " << id << "] Error: LoRa init failed on " << lora_port << "\n";
    }

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
    SensorData udp_sensor, local_sensor_a, local_sensor_b;
    uint32_t last_seq_self = 0, last_seq_front = 0;
    
    double my_p = 10000.0 - id*200.0, my_v = 0.0;
    double c_pos = 10000.0 - (id-1)*200.0, c_vel = 0.0, c_acc = 0.0;
    float cur_f = 0, i_err = 0;
    
    auto last_recv_front = std::chrono::steady_clock::now();
    bool is_network_linked = false;
    PlatoonState current_state = (id == 0) ? PlatoonState::LEADER_NOMINAL : PlatoonState::FOLLOWER;
    auto sim_start_time = std::chrono::steady_clock::now();
    
    auto last_wifi_time = std::chrono::steady_clock::now();
    bool using_lora_fallback = false;
    uint8_t udp_buf[256];

    // Cruise observer variables
    double cruise_start_time = 0.0;
    bool is_cruising = false;
    bool decoupling_triggered = false; 

    while(true) {
        auto now = std::chrono::steady_clock::now();
        bool state_updated = false;
        double elapsed = std::chrono::duration<double>(now - sim_start_time).count();

        // 1. Cruise Observer Logic
        if (my_v > 21.5 && c_vel > 21.5) { 
            if (!is_cruising) {
                is_cruising = true;
                cruise_start_time = elapsed;
            }
        } else {
            is_cruising = false; 
        }

        // 2. Trigger decoupling if stable for 10 seconds
        if (id == 3 && current_state == PlatoonState::FOLLOWER && !decoupling_triggered && is_cruising) {
            if (elapsed - cruise_start_time > 10.0) {
                current_state = PlatoonState::DECOUPLING;
                decoupling_triggered = true;
                std::cout << "\n[DISPATCH] Fleet stably cruised at 80km/h for 10s. VCU 3 initiating DECOUPLING! Target gap: 800m.\n";
                i_err = 0.0f; 
            }
        }

        bool local_data_ready = false;
        uint16_t local_seq = 0;

        // Drain buffer using while loop to eliminate latency
        if (bus_mode == "mvb" && local_mvb) {
            MVB_Hardware_Frame hw_buffer;
            while (local_mvb->read_frame(&hw_buffer, sizeof(hw_buffer)) == sizeof(MVB_Hardware_Frame)) {
                if (hw_buffer.port_addr == id && PerceptionEngine::hardware_unpack(hw_buffer, local_sensor_a)) {
                    local_sensor_b = local_sensor_a; 
                    local_data_ready = true;
                    local_seq++; 
                }
            }
        } 
        else if (bus_mode == "can" && local_can) {
            CANPacket can_pkt;
            while (local_can->receive(can_pkt) > 0 && can_pkt.source_id == (uint16_t)id) {
                if (PerceptionEngine::can_unpack(can_pkt, local_sensor_a)) {
                    local_sensor_b = local_sensor_a;
                    local_data_ready = true;
                    local_seq = can_pkt.seq_num;
                }
            }
        }
        else if (bus_mode == "sim") {
            MVB_Hardware_Frame hw_buffer;
            while (sim_fd >= 0 && read(sim_fd, &hw_buffer, sizeof(hw_buffer)) == sizeof(MVB_Hardware_Frame)) {
                if (hw_buffer.port_addr == id && PerceptionEngine::hardware_unpack(hw_buffer, local_sensor_a)) {
                    local_sensor_b = local_sensor_a;
                    local_data_ready = true;
                }
            }
        }

        if (local_data_ready) {
            if (sdvcu.process_sensors(local_seq, local_sensor_a, local_sensor_b)) {
                my_v = kf_my_vel.update(local_sensor_a.distances[1], 0.01);
                state_updated = true;
            }
        }

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

        int bytes;
        while ((bytes = recv(wifi_sd, udp_buf, sizeof(udp_buf), 0)) > 0) {
            if (bytes == sizeof(RawT2TPacket)) {
                RawT2TPacket* udp_pkt = reinterpret_cast<RawT2TPacket*>(udp_buf);
                if (udp_pkt->header == 0x55AA55AA) {
                    process_packet(*udp_pkt);
                    last_wifi_time = now;
                    if (using_lora_fallback) {
                        std::cout << "[NETWORK] Wi-Fi linked. Broadband restored.\n";
                        using_lora_fallback = false;
                    }
                }
            }
        }

        RawT2TPacket lora_pkt;
        while (t2t_radio.receive(lora_pkt)) {
            if (lora_pkt.header == 0x55AA55AA) {
                double time_since_wifi = std::chrono::duration<double>(now - last_wifi_time).count();
                if (time_since_wifi > 0.05) { 
                    process_packet(lora_pkt);
                    if (!using_lora_fallback) {
                        std::cout << "[WARNING] Wi-Fi lost. LoRa fallback activated.\n";
                        using_lora_fallback = true;
                    }
                }
            }
        }

        float dt_loop = 0.002f; 
        my_p += my_v * dt_loop;  
        if (is_network_linked) {
            c_pos += c_vel * dt_loop; 
            c_vel += c_acc * dt_loop;
        }

        double time_since_last_pkt = std::chrono::duration<double>(now - last_recv_front).count();

        if (current_state == PlatoonState::DECOUPLING) {
            float da = c_pos - my_p;
            if (da > 800.0f) { 
                current_state = PlatoonState::LEADER_NEW;
                std::cout << "\n[STATE MACHINE] VCU 3 DECOUPLING COMPLETE. Promoted to Independent Leader!\n";
                i_err = 0.0f; 
            }
        }

        if (is_network_linked && time_since_last_pkt > 1.0 && current_state == PlatoonState::FOLLOWER) { 
            is_network_linked = false;
            cur_f = -1250000.0f; 
        } 
        else if (sdvcu.is_eb()) {
            cur_f = -1250000.0f; 
        } 
        else if (state_updated || current_state != PlatoonState::FOLLOWER) {
            float ds = PerceptionEngine::calculate_safe_dist(my_v, c_vel, TrainType::HEAVY_HAUL);
            float da = c_pos - my_p;
            float raw_f = 0.0f;
            
            if (sdvcu.is_atp_braking(da) || (da <= ds && current_state == PlatoonState::FOLLOWER)) { 
                raw_f = -1250000.0f;
                cur_f = raw_f; 
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
                } 
                else if (current_state == PlatoonState::DECOUPLING || current_state == PlatoonState::LEADER_NEW) {
                    float target_v;
                    if (current_state == PlatoonState::DECOUPLING) {
                        target_v = 15.0f; 
                    } else {
                        target_v = 22.22f; 
                    }
                    
                    float err_v = target_v - my_v;
                    i_err = std::clamp(i_err + err_v * 0.005f, -10.0f, 10.0f);
                    float accel_cmd = err_v * 0.1f + i_err * 0.01f;
                    
                    float min_force = (current_state == PlatoonState::DECOUPLING) ? -400000.0f : -1250000.0f;
                    raw_f = std::clamp(MASS * accel_cmd, min_force, 1000000.0f);
                }

                cur_f = 0.02f * raw_f + 0.98f * cur_f;
                float max_d = MASS * 0.25f * 0.01f; 
                cur_f = std::clamp(cur_f, cur_f - max_d, cur_f + max_d);
            }
        }

        uint32_t report_status = 0; 
        if (sdvcu.is_eb()) report_status = 1; 
        else if (current_state == PlatoonState::DECOUPLING) report_status = 2;
        else if (current_state == PlatoonState::LEADER_NEW) report_status = 3;

        ForceReportPacket rep = {(uint32_t)id, cur_f, report_status};
        sendto(srv_sd, &rep, sizeof(rep), 0, (struct sockaddr*)&srv, sizeof(srv));

        if (bus_mode == "mvb" && local_mvb) {
            MVB_Hardware_Frame tx_frame;
            ProtocolConverter::encode_mvb(tx_frame, my_v, std::abs(cur_f/10000.0f));
            tx_frame.port_addr = id;
            local_mvb->write_frame(&tx_frame, sizeof(tx_frame));
        } else if (bus_mode == "can" && local_can) {
            CANPacket tx_can;
            tx_can.header = 0xAA55;
            tx_can.source_id = (uint16_t)id;
            tx_can.payload[1] = my_v;
            local_can->send(tx_can);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    
    if (sim_fd >= 0) close(sim_fd);
    if (wifi_sd >= 0) close(wifi_sd);
    return 0;
}