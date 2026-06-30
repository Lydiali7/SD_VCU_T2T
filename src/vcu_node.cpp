#include <iostream>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <algorithm>
#include <chrono>
#include <thread>
#include <cstdlib>
#include <cstdio>
#include <fstream> // Required for local telemetry logging
#include "network_proto.hpp"
#include "perception.hpp"
#include "t2t_radio.hpp" 
#include "train_dynamics.hpp"

// Standardized V2V communication states reflecting 3GPP R16/R17 architecture
enum class NetworkHealth { 
    HEALTHY_PC5,      // 5G D2D Sidelink: Ultra-low latency (<5ms)
    DEGRADED_UU,      // 5G Base Station: Higher latency (~30-50ms)
    BLIND_KINEMATIC   // Link Loss: High risk KF dead reckoning
};

// Simulated High-Resolution Clock for gPTP Time Sync
uint64_t get_system_time_us() {
    auto now = std::chrono::steady_clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::microseconds>(now).count();
}

// Optimized control function with smooth boundaries and capped uncertainty
float calculate_business_force(PlatoonState state, float v, float target_v, float gap, float safe_gap, float uncertainty_buffer, float max_force, float front_v, NetworkHealth net_state) {
    if (v > 26.5f) {
        return -max_force * 0.35f; 
    }
    
    float final_net_f = 0.0f;
    float dynamic_uncertainty = std::min(uncertainty_buffer, 150.0f);

    if (state == PlatoonState::DECOUPLING) {
        float decoupling_target_v = 14.0f; 
        float err_v = decoupling_target_v - v;
        final_net_f = std::clamp(err_v * 60000.0f, -max_force, max_force);
    } else {
        float err_v_cruise = target_v - v;
        float cruise_force = err_v_cruise * 50000.0f;
        
        float target_gap;
        if (state == PlatoonState::LEADER_NEW) {
            target_gap = std::max(850.0f, safe_gap + 100.0f);
        } else {
            target_gap = safe_gap + 80.0f + dynamic_uncertainty; 
        }
        
        float err_p = gap - target_gap;
        float err_v_follow = front_v - v;
        
        float follow_force = (err_p * 12000.0f) + (err_v_follow * 75000.0f);
        
        if (gap < 2500.0f) {
            final_net_f = std::min(cruise_force, follow_force);
        } else {
            final_net_f = cruise_force;
        }
    }
    
    if (v > (target_v - 0.5f) && final_net_f > 0.0f) {
        float factor = std::max(0.0f, 1.0f - (v - (target_v - 0.5f)) / 0.5f);
        final_net_f *= factor;
    }
    
    return std::clamp(final_net_f, -max_force, max_force);
}

int main(int argc, char* argv[]) {
    srand(time(NULL));
    int id = 3; 
    TrainConfig my_cfg = get_fleet_config(id);
    TrainConfig front_cfg = get_fleet_config(id - 1);
    
    LocomotiveProfile my_profile = FleetDatabase::get_loco_profile(my_cfg.model);
    WagonProfile my_wagon = my_cfg.is_empty ? HeavyHaulDynamics::C80_EMPTY_PROFILE : HeavyHaulDynamics::C80_PROFILE;
    WagonProfile front_wagon = front_cfg.is_empty ? HeavyHaulDynamics::C80_EMPTY_PROFILE : HeavyHaulDynamics::C80_PROFILE;
    
    float TOTAL_MASS = my_profile.mass_kg + (my_cfg.num_wagons * my_wagon.gross_mass_kg);
    float train_len = 22.0f + my_cfg.num_wagons * my_wagon.length_m; 

    std::cout << "[SD-VCU] Node " << id << " Booting up. Mass: " << TOTAL_MASS / 1000.0f << " t.\n";

    SDVCU_Core sdvcu(my_cfg.model, front_cfg.model, my_cfg.num_wagons);
    PlatoonState current_state = PlatoonState::STARTING;
    NetworkHealth net_state = NetworkHealth::HEALTHY_PC5;

    // --- Control Plane Initialization (5G/UDP) ---
    // NOTE FOR HIL DEPLOYMENT: Change "127.0.0.1" to the physical IP of the World Server (e.g., 192.168.1.100)
    int srv_sd = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in srv;
    srv.sin_family = AF_INET;
    srv.sin_port = htons(VCU_REPORT_PORT);
    inet_pton(AF_INET, "127.0.0.1", &srv.sin_addr);

    // --- Safety Plane Initialization (LoRa 400MHz) ---
    T2T_Radio lora_radio;
    bool lora_active = lora_radio.init("/dev/ttyUSB0", B115200, 0x17); 
    if (lora_active) {
        std::cout << "[LORA_HAL] 400MHz Safety Plane Active.\n";
    } else {
        std::cout << "[WARN] LoRa hardware not found. Running Control Plane only.\n";
    }

    // --- Local Telemetry Logger for Cyber-Physical Metrics ---
    std::ofstream vcu_log("vcu_telemetry.csv", std::ios::trunc);
    vcu_log << "Time(s),AoI(ms),BurstLossCount,TimeJitter(us),EstimatedGap(m),SafeGap(m),State\n";

    float cur_f = 0.0f;
    float dt = 0.01f; 
    float simulated_my_pos = (4 - id) * 1000.0f + 1000.0f; 
    float simulated_my_vel = 0.0f;

    KalmanTracker kf(simulated_my_pos + 400.0f + train_len, 0.0f);

    float uncertainty_buffer = 0.0f;
    float time_since_last_tx = 0.0f;
    float lora_tx_timer = 0.0f;
    const float LORA_HEARTBEAT_INTERVAL = 2.0f; 
    float current_tx_interval = 0.01f; 

    int tick_counter = 0;
    uint64_t total_ticks = 0;
    float TARGET_CRUISE_VEL = 24.5f; 

    float current_aoi_ms = 0.0f;
    uint64_t last_rx_timestamp_us = get_system_time_us();

    // Cyber-Physical monitoring variables
    int burst_loss_count = 0;
    uint64_t last_time_diff_us = 0;

    while (true) {
        auto loop_start = std::chrono::steady_clock::now();
        kf.predict(dt);
        time_since_last_tx += dt;
        total_ticks++;

        // =======================================================
        // 1. SAFETY PLANE (LoRa Asynchronous EB Polling)
        // =======================================================
        if (lora_active) {
            uint8_t rx_buf[256];
            int n = read(lora_radio.get_fd(), rx_buf, sizeof(rx_buf));
            if (n > 0) {
                bool emergency_detected = false;
                for(int i = 0; i < n - 3; ++i) {
                    if (rx_buf[i] == 0xEE && rx_buf[i+1] == 0xEE && rx_buf[i+2] == 0xEE && rx_buf[i+3] == 0xEE) {
                        emergency_detected = true;
                        break;
                    }
                }
                
                if (emergency_detected && !sdvcu.is_eb()) {
                    std::cout << "[CRITICAL] LoRa Emergency Broadcast Received! Global EB Initiated.\n";
                    sdvcu.force_eb_trigger();
                }
            }
        }

        // =======================================================
        // 2. CONTROL PLANE (5G TSN Deterministic Sync)
        // =======================================================
        bool in_tunnel = (simulated_my_pos > 5000.0f && simulated_my_pos < 7000.0f);
        bool nlos_curve = (simulated_my_pos > 10000.0f && simulated_my_pos < 11000.0f);
        
        bool pc5_available = (!in_tunnel && !nlos_curve);
        bool uu_available = !in_tunnel; 

        float env_data[3]; 
        bool raw_recv = (recv(srv_sd, env_data, sizeof(env_data), MSG_DONTWAIT) > 0);
        bool packet_received = false;

        // NOTE FOR HIL DEPLOYMENT: Remove the probability drop logic below.
        // Rely purely on raw_recv when running on physical radio hardware.
        if (raw_recv) {
            float dist = kf.pos - simulated_my_pos;
            bool in_tunnel_now = (simulated_my_pos > 5000.0f && simulated_my_pos < 7000.0f);
            float per = PerceptionEngine::get_packet_error_rate(dist, in_tunnel_now);
            
            float dice = (float)rand() / (float)RAND_MAX;
            
            if (dice > per) { 
                packet_received = true;
                burst_loss_count = 0; 
                
                uint64_t rx_ts = get_system_time_us();
                uint64_t tx_ts = rx_ts - (pc5_available ? 5000 : (uu_available ? 35000 : 0));
                last_time_diff_us = rx_ts - tx_ts; 
            } else {
                packet_received = false;
                burst_loss_count++; 
            }
        } else {
            burst_loss_count++; 
        }

        if (packet_received) {
            uint64_t sim_tx_time = get_system_time_us() - (pc5_available ? 5000 : (uu_available ? 35000 : 0));

            if (pc5_available) {
                if (net_state != NetworkHealth::HEALTHY_PC5) {
                    std::cout << "[NETWORK] PC5 Direct Link Active. Latency < 5ms.\n";
                    net_state = NetworkHealth::HEALTHY_PC5;
                }
                last_rx_timestamp_us = sim_tx_time;
                kf.update(simulated_my_pos + env_data[0] + train_len, env_data[1]);
            } else if (uu_available) {
                if (net_state != NetworkHealth::DEGRADED_UU) {
                    std::cout << "[WARN] PC5 Blocked. Fallback to 5G Uu. Latency ~35ms.\n";
                    net_state = NetworkHealth::DEGRADED_UU;
                }
                last_rx_timestamp_us = sim_tx_time;
                kf.update(simulated_my_pos + env_data[0] + train_len, env_data[1]);
            }
            // Double kf.update() bug fixed here.
        }

        uint64_t now_us = get_system_time_us();
        current_aoi_ms = (now_us - last_rx_timestamp_us) / 1000.0f;

        if (current_aoi_ms > 2000.0f && net_state != NetworkHealth::BLIND_KINEMATIC) {
            std::cout << "[CRITICAL] Network Blackout. AoI > 2000ms. Enter BLIND RUN.\n";
            net_state = NetworkHealth::BLIND_KINEMATIC;
        }

        if (net_state == NetworkHealth::HEALTHY_PC5) {
            uncertainty_buffer = std::max(0.0f, uncertainty_buffer - 5.0f * dt); 
        } else if (net_state == NetworkHealth::DEGRADED_UU) {
            uncertainty_buffer = std::max(30.0f, uncertainty_buffer - 2.0f * dt); 
        } else if (net_state == NetworkHealth::BLIND_KINEMATIC) {
            uncertainty_buffer = std::min(150.0f, uncertainty_buffer + 2.5f * dt); 
        }

        float estimated_gap = kf.pos - simulated_my_pos - train_len;
        if (estimated_gap < 0.0f) estimated_gap = 0.0f;
        
        // =======================================================
        // 3. TOPOGRAPHY-AWARE SAFETY ENVELOPE
        // =======================================================
        float gradient_permille = 0.0f;
        float curve_radius_m = 0.0f; 
        if (simulated_my_pos > 8000.0f && simulated_my_pos <= 10000.0f) gradient_permille = -4.0f; 
        else if (simulated_my_pos > 10000.0f && simulated_my_pos < 12000.0f) { gradient_permille = -4.0f; curve_radius_m = 400.0f; } 
        else if (simulated_my_pos > 12000.0f && simulated_my_pos < 15000.0f) gradient_permille = 6.0f;

        float curve_resistance_permille = (curve_radius_m > 0.1f) ? (600.0f / curve_radius_m) : 0.0f;
        float effective_gradient = gradient_permille + curve_resistance_permille;
        float f_gravity = -TOTAL_MASS * 9.8f * (effective_gradient / 1000.0f);

        float effective_delay_s = current_aoi_ms / 1000.0f;
        if (net_state == NetworkHealth::BLIND_KINEMATIC) {
            effective_delay_s = 2.0f; 
        }

        sdvcu.current_safe_gap = PerceptionEngine::calculate_heavy_haul_safe_dist(
            simulated_my_vel, kf.vel, my_cfg.model, front_cfg.model, 
            my_wagon, front_wagon, my_cfg.num_wagons, front_cfg.num_wagons, false, effective_delay_s,
            effective_gradient); 

        if (estimated_gap > 0.1f && estimated_gap <= sdvcu.current_safe_gap) {
            if (!sdvcu.is_eb()) {
                std::cout << "[FATAL] ATP TRIP! Gap: " << estimated_gap << "m | Safe Gap: " << sdvcu.current_safe_gap << "m\n";
                sdvcu.force_eb_trigger();
            }
        }

        // =======================================================
        // 4. KINEMATICS & STATE MACHINE
        // =======================================================
        float v_difference = kf.vel - simulated_my_vel; 

        if (current_state == PlatoonState::STARTING && simulated_my_vel >= 18.0f) {
            current_state = PlatoonState::CRUISING;
        }
        else if (current_state == PlatoonState::CRUISING && simulated_my_pos > 8000.0f) {
            current_state = PlatoonState::DECOUPLING;
            std::cout << "[EVENT] Reached 8km marking. Initiating V2V Decoupling Protocol...\n";
        }
        else if (current_state == PlatoonState::DECOUPLING) {
            if (estimated_gap > 900.0f && v_difference > 3.0f) {
                current_state = PlatoonState::LEADER_NEW; 
                std::cout << "[EVENT] Conditions met. Enter Target Tracking Mode\n";
            }
        }

        float target_net_f = 0.0f;
        if (sdvcu.is_eb()) target_net_f = -TOTAL_MASS * my_profile.avg_deceleration_limit;
        else {
            target_net_f = calculate_business_force(
                current_state, simulated_my_vel, TARGET_CRUISE_VEL, estimated_gap, 
                sdvcu.current_safe_gap, uncertainty_buffer, my_profile.max_starting_tractive_effort_n, kf.vel, net_state);
        }

        float target_motor_f = target_net_f - f_gravity;
        
        float max_up = (my_profile.max_starting_tractive_effort_n / my_profile.tractive_build_up_time_s) * dt;
        float max_down = ((TOTAL_MASS * my_profile.avg_deceleration_limit) / 3.0f) * dt;
        cur_f = (target_motor_f > cur_f) ? std::min(target_motor_f, cur_f + max_up) : std::max(target_motor_f, cur_f - max_down);

        simulated_my_vel += ((cur_f + f_gravity) / TOTAL_MASS) * dt;
        if(simulated_my_vel < 0.0f) simulated_my_vel = 0.0f;
        simulated_my_pos += simulated_my_vel * dt;

        uint32_t report_status = 0;
        if (sdvcu.is_eb()) report_status = 1;
        else if (net_state == NetworkHealth::DEGRADED_UU) report_status = 2;
        else if (net_state == NetworkHealth::BLIND_KINEMATIC) report_status = 3;
        else if (current_state == PlatoonState::DECOUPLING) report_status = 4;
        else if (current_state == PlatoonState::LEADER_NEW) report_status = 5;

        // --- Local CSV Telemetry Logging (10Hz) ---
        if (tick_counter % 10 == 0) {
            float time_s = total_ticks * dt; 
            vcu_log << time_s << ","
                    << current_aoi_ms << ","
                    << burst_loss_count << ","
                    << last_time_diff_us << ","
                    << estimated_gap << ","
                    << sdvcu.current_safe_gap << ","
                    << report_status << "\n";
            vcu_log.flush();
        }

        // --- 5G TSN Telemetry Output ---
        if (time_since_last_tx >= current_tx_interval) {
            ForceReportPacket rep = {(uint32_t)id, cur_f, report_status, simulated_my_vel, simulated_my_pos};
            sendto(srv_sd, &rep, sizeof(rep), 0, (struct sockaddr*)&srv, sizeof(srv));
            time_since_last_tx = 0.0f; 
        }

        // --- LoRa Heartbeat / Alarm Broadcast ---
        if (lora_active) {
            lora_tx_timer += dt;
            if (lora_tx_timer >= LORA_HEARTBEAT_INTERVAL || (sdvcu.is_eb() && lora_tx_timer >= 0.5f)) {
                uint8_t tx_buf[12];
                uint32_t header = 0x55AA55AA;
                uint64_t payload = sdvcu.is_eb() ? 0xEEEEEEEE : (uint64_t)simulated_my_pos;
                
                std::memcpy(tx_buf, &header, 4);
                std::memcpy(tx_buf + 4, &payload, 8);
                
                ssize_t bytes_written = write(lora_radio.get_fd(), tx_buf, sizeof(tx_buf));
                (void)bytes_written;
                lora_tx_timer = 0.0f; 
            }
        }

        if (++tick_counter >= 100) {
            std::printf("[TELEMETRY] ID:%d | Vel: %5.1f | Gap: %6.1f | AoI: %6.1fms | Uncert: %5.1fm | State: %d\n", 
                id, simulated_my_vel * 3.6f, estimated_gap, current_aoi_ms, uncertainty_buffer, report_status);
            tick_counter = 0;
        }

        std::this_thread::sleep_until(loop_start + std::chrono::milliseconds(10));
    }
    return 0;
}