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
#include <algorithm>
#include <atomic>
#include <ctime>
#include <cstdlib>
#include <string>
#include <mutex>
#include <array>
#include "perception.hpp"
#include "train_dynamics.hpp"

std::vector<TrainState> fleet(5);
std::vector<uint32_t> fleet_status(5, 0); 
float forces[5] = {0}; 

std::array<struct sockaddr_in, 5> active_client_addr;
std::array<std::atomic<bool>, 5> client_connected;
std::atomic<bool> simulation_complete(false);
std::array<std::atomic<float>, 5> node_force_cmd;
std::array<std::atomic<uint32_t>, 5> node_status_cmd;
std::mutex client_addr_mutex;

// Global state trackers for legacy simulated nodes
float post_tunnel_steady[5] = {0.0f};
bool tunnel_exit[5] = {false};

enum class RuntimeMode {
    SIM,
    HIL
};

uint64_t get_steady_time_us() {
    auto now = std::chrono::steady_clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::microseconds>(now).count();
}

uint64_t get_ptp_time_us() {
    struct timespec ts;
#ifdef CLOCK_TAI
    if (clock_gettime(CLOCK_TAI, &ts) == 0) {
        return static_cast<uint64_t>(ts.tv_sec) * 1000000ULL + static_cast<uint64_t>(ts.tv_nsec / 1000ULL);
    }
#endif
    clock_gettime(CLOCK_REALTIME, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000ULL + static_cast<uint64_t>(ts.tv_nsec / 1000ULL);
}

uint64_t get_runtime_time_us(RuntimeMode mode) {
    return (mode == RuntimeMode::HIL) ? get_ptp_time_us() : get_steady_time_us();
}

RuntimeMode parse_runtime_mode(int argc, char* argv[]) {
    const char* env_mode = std::getenv("SDVCU_MODE");
    if (env_mode != nullptr && std::string(env_mode) == "HIL") return RuntimeMode::HIL;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--hil") return RuntimeMode::HIL;
        if (std::string(argv[i]) == "--sim") return RuntimeMode::SIM;
    }
    return RuntimeMode::SIM;
}

double get_env_double(const char* name, double default_value) {
    const char* value = std::getenv(name);
    return value ? std::atof(value) : default_value;
}

int get_env_int(const char* name, int default_value) {
    const char* value = std::getenv(name);
    return value ? std::atoi(value) : default_value;
}

double random_unit() {
    return static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX);
}

void receive_forces_thread(int sd) {
    ForceReportPacket report;
    struct sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);

    while (true) {
        client_len = sizeof(client_addr);
        ssize_t received = recvfrom(sd, &report, sizeof(report), 0, (struct sockaddr*)&client_addr, &client_len);
        if (received <= 0) {
            break;
        }
        if (received != static_cast<ssize_t>(sizeof(report))) {
            continue;
        }
        int idx = report.train_id; 
        if (idx >= 0 && idx < 5) { 
            float force_cmd = (report.status_flag == STATUS_ATP_TRIP) ? -5000000.0f : report.cmd_force;
            node_force_cmd[idx].store(force_cmd);
            node_status_cmd[idx].store(report.status_flag);
            {
                std::lock_guard<std::mutex> lock(client_addr_mutex);
                active_client_addr[idx] = client_addr;
            }
            client_connected[idx].store(true);
            if (report.status_flag == STATUS_SIM_COMPLETE) {
                simulation_complete.store(true);
            }
        }
    }
}

int main(int argc, char* argv[]) {
    std::srand(static_cast<unsigned int>(std::time(nullptr)));
    RuntimeMode runtime_mode = parse_runtime_mode(argc, argv);
    bool hil_mode = runtime_mode == RuntimeMode::HIL;
    for (int i = 0; i < 5; ++i) {
        client_connected[i].store(false);
        node_force_cmd[i].store(0.0f);
        node_status_cmd[i].store(0);
    }
    std::cout << "[SERVER] Booting Heterogeneous World Simulation. Mode: "
              << (hil_mode ? "HIL" : "SIM") << "\n";
    int server_sd = socket(AF_INET, SOCK_DGRAM, 0);
    if (server_sd < 0) {
        std::perror("[SERVER] socket");
        return 1;
    }
    int reuse_addr = 1;
    setsockopt(server_sd, SOL_SOCKET, SO_REUSEADDR, &reuse_addr, sizeof(reuse_addr));
    struct sockaddr_in server_addr;
    std::memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    const char* bind_ip = std::getenv("SERVER_BIND_IP");
    if (bind_ip == nullptr) {
        bind_ip = hil_mode ? "192.168.1.10" : "0.0.0.0";
    }
    if (inet_pton(AF_INET, bind_ip, &server_addr.sin_addr) != 1) {
        std::cerr << "[SERVER] Invalid bind IP: " << bind_ip << "\n";
        return 1;
    }
    server_addr.sin_port = htons(VCU_REPORT_PORT);
    if (bind(server_sd, (struct sockaddr*)&server_addr, sizeof(server_addr)) != 0) {
        std::perror("[SERVER] bind");
        return 1;
    }
    std::cout << "[SERVER] UDP bind: " << bind_ip << ":" << VCU_REPORT_PORT << "\n";

    double hil_drop_start_s = get_env_double("HIL_DROP_START_S", -1.0);
    double hil_drop_duration_s = get_env_double("HIL_DROP_DURATION_S", 0.0);
    double hil_drop_prob = std::clamp(get_env_double("HIL_DROP_PROB", 0.0), 0.0, 1.0);
    double hil_random_drop_prob = std::clamp(get_env_double("HIL_RANDOM_DROP_PROB", 0.0), 0.0, 1.0);
    double hil_delay_base_ms = std::max(0.0, get_env_double("HIL_DELAY_BASE_MS", 0.0));
    double hil_delay_jitter_ms = std::max(0.0, get_env_double("HIL_DELAY_JITTER_MS", 0.0));
    int hil_drop_target_id = get_env_int("HIL_DROP_TARGET_ID", 3);
    bool hil_drop_was_active = false;
    if (hil_mode && hil_drop_start_s >= 0.0 && hil_drop_duration_s > 0.0 && hil_drop_prob > 0.0) {
        std::cout << "[SERVER] HIL fault injection armed: target=" << hil_drop_target_id
                  << " start=" << hil_drop_start_s << "s duration=" << hil_drop_duration_s
                  << "s drop_prob=" << hil_drop_prob << "\n";
    }
    if (hil_mode && (hil_random_drop_prob > 0.0 || hil_delay_base_ms > 0.0 || hil_delay_jitter_ms > 0.0)) {
        std::cout << "[SERVER] HIL impairment armed: random_drop_prob=" << hil_random_drop_prob
                  << " delay_base=" << hil_delay_base_ms << "ms"
                  << " delay_jitter=+/-" << hil_delay_jitter_ms << "ms\n";
    }

    //int flags = fcntl(server_sd, F_GETFL, 0);
    //fcntl(server_sd, F_SETFL, flags | O_NONBLOCK);

    std::thread rx_thread(receive_forces_thread, server_sd);
    rx_thread.detach();

    std::ofstream log_file("fleet_log.csv");
    log_file << "Time(s),ID0_Pos,ID0_Vel,ID0_F,ID0_St,ID1_Pos,ID1_Vel,ID1_F,ID1_St,ID2_Pos,ID2_Vel,ID2_F,ID2_St,ID3_Pos,ID3_Vel,ID3_F,ID3_St,ID4_Pos,ID4_Vel,ID4_F,ID4_St\n";

    for (int i = 0; i < 5; ++i) {
        fleet[i].pos = (4 - i) * 1000.0f + 1000.0f; 
        fleet[i].vel = 0.0f;
    }

    auto start_time = std::chrono::steady_clock::now();
    float dt = 0.01f; 

    while (true) {
        auto current_time = std::chrono::steady_clock::now();
        float elapsed_s = std::chrono::duration<float>(current_time - start_time).count();

        for (int i = 0; i < 5; ++i) {
            if (client_connected[i].load()) {
                forces[i] = node_force_cmd[i].load();
                fleet_status[i] = node_status_cmd[i].load();
            }
        }

            std::printf("\033[2J\033[H");
            std::printf("=== HETEROGENEOUS PLATOON WORLD SERVER ===\n");
            std::printf("ID | TruePos(m)| TrueVel(km/h)|  Gap(m) | Safe(m) | Force(kN) | Current Status\n");
        
            for (int i = 0; i < 5; ++i) {
            TrainConfig cfg = get_fleet_config(i);
            LocomotiveProfile my_profile = FleetDatabase::get_loco_profile(cfg.model);
            WagonProfile my_wagon = cfg.is_empty ? HeavyHaulDynamics::C80_EMPTY_PROFILE : HeavyHaulDynamics::C80_PROFILE;
            float total_mass = my_profile.mass_kg + (cfg.num_wagons * my_wagon.gross_mass_kg);
            float train_len = 22.0f + cfg.num_wagons * my_wagon.length_m; 
            
            float s_dist = 0.0f;
            if (i > 0) {
                TrainConfig front_cfg = get_fleet_config(i-1);
                WagonProfile front_wagon = front_cfg.is_empty ? HeavyHaulDynamics::C80_EMPTY_PROFILE : HeavyHaulDynamics::C80_PROFILE;
                s_dist = PerceptionEngine::calculate_heavy_haul_safe_dist(
                                fleet[i].vel, fleet[i-1].vel, cfg.model, front_cfg.model, 
                                my_wagon, front_wagon, cfg.num_wagons, front_cfg.num_wagons, false, 0.05f);
            }

            // Real Track Topography Profile
            float gradient_permille = 0.0f;
            float curve_radius_m = 0.0f; 

            if (fleet[i].pos > 8000.0f && fleet[i].pos <= 10000.0f) {
                gradient_permille = -4.0f; 
            } else if (fleet[i].pos > 10000.0f && fleet[i].pos < 12000.0f) {
                gradient_permille = -4.0f; 
                curve_radius_m = 400.0f;   
            } else if (fleet[i].pos > 12000.0f && fleet[i].pos < 15000.0f) {
                gradient_permille = 6.0f;
            }

            float curve_resistance_permille = 0.0f;
            if (curve_radius_m > 0.1f) {
                curve_resistance_permille = 600.0f / curve_radius_m;
            }

            float effective_gradient = gradient_permille + curve_resistance_permille;
            float f_gravity = -total_mass * 9.8f * (effective_gradient / 1000.0f);

            if (!client_connected[i].load()) { 
                float gap = (i == 0) ? 9999.0f : (fleet[i-1].pos - fleet[i].pos - train_len);
                float target_net_f = 0.0f; 
                
                if (fleet[i].vel > 25.0f) { 
                    target_net_f = -total_mass * 0.4f; 
                } else {
                    if (i == 0) {
                        float err_v = (88.2f / 3.6f) - fleet[0].vel; 
                        target_net_f = err_v * 50000.0f;
                    } else {
                        if (fleet_status[i] == 4) {
                            // FIX 1: Gradient Decoupling Speeds to prevent rear-ending
                            float decoupling_target_v = 24.5f - i * 3.5f; 
                            float err_v = decoupling_target_v - fleet[i].vel; 
                            target_net_f = err_v * 50000.0f;
                            
                            // FIX 2: State Machine Completion
                            float v_diff = fleet[i-1].vel - fleet[i].vel;
                            if (gap > 900.0f && v_diff > 3.0f) {
                                fleet_status[i] = 5; 
                            }
                        } else {
                            float target_gap;
                            if (fleet_status[i] == 5) {
                                // FIX 3: Eradicate the 1100m hardcode for tracking
                                target_gap = std::max(850.0f, s_dist + 100.0f);
                            } else {
                                target_gap = s_dist + 80.0f; 
                            }
                            
                            float err_p = gap - target_gap; 
                            float err_v = fleet[i-1].vel - fleet[i].vel;
                            float cruise_force = ((88.2f / 3.6f) - fleet[i].vel) * 50000.0f;
                            float follow_force = (err_p * 15000.0f) + (err_v * 80000.0f);
                            
                            if (gap < 2500.0f) {
                                target_net_f = std::min(cruise_force, follow_force);
                            } else {
                                target_net_f = cruise_force;
                            }
                        }
                    }
                    if (fleet[i].vel > 24.0f && target_net_f > 0.0f) target_net_f = 0.0f;
                }

                // Decoupling trigger for legacy nodes
                if (fleet[i].pos > 7000.0f) {
                    tunnel_exit[i] = true;
                }
                if (tunnel_exit[i] && fleet_status[i] == 0) {
                    post_tunnel_steady[i] += fleet[i].vel * dt;
                }
                if (post_tunnel_steady[i] > 1000.0f && fleet_status[i] == 0 && i != 0) {
                    fleet_status[i] = 4; // Safely enter DECOUPLING state
                }

                if (i < 4) {
                    bool follower_is_gone = (fleet_status[i+1] >= 4 || fleet_status[i+1] == 1);
                    if (!follower_is_gone) {
                        TrainConfig back_cfg = get_fleet_config(i + 1);
                        WagonProfile back_wagon = back_cfg.is_empty ? HeavyHaulDynamics::C80_EMPTY_PROFILE : HeavyHaulDynamics::C80_PROFILE;
                        float back_len = 22.0f + back_cfg.num_wagons * back_wagon.length_m;
                        float gap_to_follower = fleet[i].pos - fleet[i+1].pos - back_len;

                        // Relaxed elastic thresholds to absorb blind-run buffering
                        if (gap_to_follower > 1300.0f) target_net_f = -total_mass * 0.05f; 
                        else if (gap_to_follower > 1150.0f && target_net_f > 0.0f) target_net_f = 0.0f; 
                    }
                }

                if (i != 0 && gap > 0.1f && gap < s_dist) fleet_status[i] = 1; 
                if (fleet_status[i] == 1) target_net_f = -total_mass * my_profile.avg_deceleration_limit; 

                float target_motor_f = target_net_f - f_gravity;
                float max_f = my_profile.max_starting_tractive_effort_n;
                float min_f = -total_mass * my_profile.avg_deceleration_limit;
                
                forces[i] = std::clamp(target_motor_f, min_f, max_f);
            }

            float net_physical_force = forces[i] + f_gravity;
            fleet[i].accel = net_physical_force / total_mass;
            fleet[i].vel += fleet[i].accel * dt;
            if (fleet[i].vel < 0.0f) fleet[i].vel = 0.0f;
            fleet[i].pos += fleet[i].vel * dt;

            float gap = (i == 0) ? 0.0f : (fleet[i-1].pos - fleet[i].pos - train_len);
            if (gap < 0.0f && i != 0) gap = 0.0f;
            
            const char* status_str = "\033[32mFOLLOWER(5G)\033[0m";
            if (i == 0) status_str = "\033[36mLEADER_A\033[0m";
            else if (fleet_status[i] == STATUS_FAIL_SAFE_LOCK) status_str = "\033[31mFAIL_SAFE_LOCK\033[0m"; 
            else if (fleet_status[i] == STATUS_ATP_TRIP) status_str = "\033[31mATP_TRIP(EB)\033[0m"; 
            else if (fleet_status[i] == STATUS_DEGRADED_UU) status_str = "\033[33mDEGRADED(Uu)\033[0m"; 
            else if (fleet_status[i] == STATUS_BLIND_RUN) status_str = "\033[35mBLIND_RUN(KF)\033[0m"; 
            else if (fleet_status[i] == STATUS_DECOUPLING) status_str = "\033[34mDECOUPLING\033[0m"; 
            else if (fleet_status[i] == STATUS_LEADER_NEW) status_str = "\033[36mTRACKING(>1km)\033[0m"; 
            else if (fleet_status[i] == STATUS_SIM_COMPLETE) status_str = "\033[32mSIM_COMPLETE\033[0m"; 
            
            std::printf("%d  | %8.1f  | %9.2f    | %7.1f | %7.1f | %9.1f | %s\n", 
                        i, fleet[i].pos, fleet[i].vel * 3.6f, gap, s_dist, forces[i]/1000.0f, status_str);
            }

            for (int client_id = 0; client_id < 5; ++client_id) {
            if (!client_connected[client_id].load()) {
                continue;
            }
            struct sockaddr_in client_addr_snapshot = {};
            {
                std::lock_guard<std::mutex> lock(client_addr_mutex);
                client_addr_snapshot = active_client_addr[client_id];
            }

            TrainConfig cfg = get_fleet_config(client_id);
            WagonProfile my_wagon = cfg.is_empty ? HeavyHaulDynamics::C80_EMPTY_PROFILE : HeavyHaulDynamics::C80_PROFILE;
            float train_len = 22.0f + cfg.num_wagons * my_wagon.length_m; 
            
            float true_gap = 9999.0f;
            float true_front_vel = fleet[client_id].vel;
            if (client_id > 0) {
                true_gap = fleet[client_id - 1].pos - fleet[client_id].pos - train_len;
                if (true_gap < 0.0f) true_gap = 0.0f;
                true_front_vel = fleet[client_id - 1].vel;
            }
            
            float my_pos = fleet[client_id].pos;
            bool in_tunnel = !hil_mode && (my_pos > 5000.0f && my_pos < 7000.0f);
            bool nlos_curve = !hil_mode && (my_pos > 10000.0f && my_pos < 11000.0f);

            bool link_active = true;
            uint64_t simulated_latency_us = 0;

            if (in_tunnel) {
                link_active = false;
            } else if (nlos_curve) {
                simulated_latency_us = 45000; 
            } else {
                simulated_latency_us = 4000; 
            }

            bool all_followers_tracking = true;
            for (int i = 1; i < 5; ++i) {
                all_followers_tracking = all_followers_tracking && (fleet_status[i] == STATUS_LEADER_NEW);
            }
            bool scenario_complete = all_followers_tracking && fleet[3].pos > 11350.0f && elapsed_s > 800.0f;

            bool hil_drop_window_active = hil_mode &&
                                          hil_drop_start_s >= 0.0 &&
                                          elapsed_s >= hil_drop_start_s &&
                                          elapsed_s <= (hil_drop_start_s + hil_drop_duration_s) &&
                                          hil_drop_target_id == client_id;
            if (hil_drop_window_active != hil_drop_was_active) {
                std::cout << "[SERVER] HIL feedback drop "
                          << (hil_drop_window_active ? "START" : "END")
                          << " at t=" << elapsed_s << "s\n";
                hil_drop_was_active = hil_drop_window_active;
            }
            bool drop_feedback = false;
            if (hil_drop_window_active && hil_drop_prob > 0.0) {
                drop_feedback = random_unit() < hil_drop_prob;
            }
            if (hil_mode && !drop_feedback && hil_random_drop_prob > 0.0) {
                drop_feedback = random_unit() < hil_random_drop_prob;
            }

            if (scenario_complete) {
                WorldFeedbackPacket feedback = {
                    true_gap,
                    true_front_vel,
                    static_cast<float>(fleet[client_id].pos),
                    static_cast<float>(fleet[client_id].vel),
                    WORLD_FEEDBACK_SIM_COMPLETE,
                    get_runtime_time_us(runtime_mode)
                };
                for (int i = 0; i < 20; ++i) {
                    feedback.tx_timestamp_us = get_runtime_time_us(runtime_mode);
                    sendto(server_sd, &feedback, sizeof(feedback), 0, (struct sockaddr*)&client_addr_snapshot, sizeof(client_addr_snapshot));
                    std::this_thread::sleep_for(std::chrono::milliseconds(2));
                }
                fleet_status[3] = STATUS_SIM_COMPLETE;
                simulation_complete.store(true);
            } else if (link_active && !drop_feedback) {
                uint64_t hil_delay_us = 0;
                if (hil_mode && (hil_delay_base_ms > 0.0 || hil_delay_jitter_ms > 0.0)) {
                    double jitter_ms = (random_unit() * 2.0 - 1.0) * hil_delay_jitter_ms;
                    double delay_ms = std::max(0.0, hil_delay_base_ms + jitter_ms);
                    hil_delay_us = static_cast<uint64_t>(delay_ms * 1000.0);
                }
                WorldFeedbackPacket feedback = {
                    true_gap,
                    true_front_vel,
                    static_cast<float>(fleet[client_id].pos),
                    static_cast<float>(fleet[client_id].vel),
                    hil_delay_us > 0 ? WORLD_FEEDBACK_HIL_IMPAIRED : 0u,
                    get_runtime_time_us(runtime_mode)
                };
                if (!hil_mode) {
                    feedback.tx_timestamp_us -= simulated_latency_us;
                    std::this_thread::sleep_for(std::chrono::microseconds(simulated_latency_us));
                } else if (hil_delay_us > 0) {
                    std::this_thread::sleep_for(std::chrono::microseconds(hil_delay_us));
                }
                sendto(server_sd, &feedback, sizeof(feedback), 0, (struct sockaddr*)&client_addr_snapshot, sizeof(client_addr_snapshot));
            }
            }

            log_file << elapsed_s << ",";
            for (int i = 0; i < 5; ++i) {
                log_file << fleet[i].pos << "," << fleet[i].vel * 3.6f << "," << forces[i] / 1000.0f << "," << fleet_status[i] << (i == 4 ? "" : ",");
            }
            log_file << "\n";
            log_file.flush(); 

            if (simulation_complete.load()) {
                std::cout << "[SERVER] Scenario complete. Final formation reached, logs flushed, server exiting cleanly.\n";
                break;
            }

        std::this_thread::sleep_until(current_time + std::chrono::milliseconds(10));
    }
    return 0;
}
