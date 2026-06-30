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
#include "perception.hpp"
#include "train_dynamics.hpp"

std::vector<TrainState> fleet(5);
std::vector<uint32_t> fleet_status(5, 0); 
float forces[5] = {0}; 

struct sockaddr_in active_client_addr;
bool client_connected = false;

// Global state trackers for legacy simulated nodes
float post_tunnel_steady[5] = {0.0f};
bool tunnel_exit[5] = {false};

void receive_forces_thread(int sd) {
    ForceReportPacket report;
    struct sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);

    while (recvfrom(sd, &report, sizeof(report), 0, (struct sockaddr*)&client_addr, &client_len) > 0) {
        int idx = report.train_id; 
        if (idx == 3) { 
            forces[idx] = (report.status_flag == 1) ? -5000000.0f : report.cmd_force; 
            fleet_status[idx] = report.status_flag; 
            active_client_addr = client_addr;
            client_connected = true;
        }
    }
}

int main() {
    std::cout << "[SERVER] Booting Heterogeneous World Simulation...\n";
    int server_sd = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(VCU_REPORT_PORT);
    bind(server_sd, (struct sockaddr*)&server_addr, sizeof(server_addr));

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

            if (i != 3) { 
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
            else if (fleet_status[i] == 1) status_str = "\033[31mATP_TRIP(EB)\033[0m"; 
            else if (fleet_status[i] == 2) status_str = "\033[33mDEGRADED(Uu)\033[0m"; 
            else if (fleet_status[i] == 3) status_str = "\033[35mBLIND_RUN(KF)\033[0m"; 
            else if (fleet_status[i] == 4) status_str = "\033[34mDECOUPLING\033[0m"; 
            else if (fleet_status[i] == 5) status_str = "\033[36mTRACKING(>1km)\033[0m"; 
            
            std::printf("%d  | %8.1f  | %9.2f    | %7.1f | %7.1f | %9.1f | %s\n", 
                        i, fleet[i].pos, fleet[i].vel * 3.6f, gap, s_dist, forces[i]/1000.0f, status_str);
        }

        if (client_connected) {
            TrainConfig cfg = get_fleet_config(3);
            WagonProfile my_wagon = cfg.is_empty ? HeavyHaulDynamics::C80_EMPTY_PROFILE : HeavyHaulDynamics::C80_PROFILE;
            float train_len = 22.0f + cfg.num_wagons * my_wagon.length_m; 
            
            float true_gap = fleet[2].pos - fleet[3].pos - train_len;
            if (true_gap < 0.0f) true_gap = 0.0f; 
            float true_front_vel = fleet[2].vel;
            
            float my_pos = fleet[3].pos;
            bool in_tunnel = (my_pos > 5000.0f && my_pos < 7000.0f);
            bool nlos_curve = (my_pos > 10000.0f && my_pos < 11000.0f);

            bool link_active = true;
            uint64_t simulated_latency_us = 0;

            if (in_tunnel) {
                link_active = false;
            } else if (nlos_curve) {
                simulated_latency_us = 45000; 
            } else {
                simulated_latency_us = 4000; 
            }

            if (link_active) {
                float env_feedback[3] = {true_gap, true_front_vel, 0.0f};
                std::this_thread::sleep_for(std::chrono::microseconds(simulated_latency_us));
                sendto(server_sd, env_feedback, sizeof(env_feedback), 0, (struct sockaddr*)&active_client_addr, sizeof(active_client_addr));
            }
        }

        log_file << elapsed_s << ",";
        for (int i = 0; i < 5; ++i) {
            log_file << fleet[i].pos << "," << fleet[i].vel * 3.6f << "," << forces[i] / 1000.0f << "," << fleet_status[i] << (i == 4 ? "" : ",");
        }
        log_file << "\n";
        log_file.flush(); 

        std::this_thread::sleep_until(current_time + std::chrono::milliseconds(10));
    }
    return 0;
}