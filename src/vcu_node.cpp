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
#include <cmath>
#include <fstream> 
#include <ctime>
#include <string>
#include "atp_adapter.hpp"
#include "network_proto.hpp"
#include "perception.hpp"
#include "t2t_radio.hpp" 
#include "train_dynamics.hpp"
#include "infra/rt_system.hpp" 
#include "infra/watching_dog.hpp" 
#include "safety_config.hpp"

namespace Safety = VCU::SafetyConfig;

enum class NetworkHealth { 
    HEALTHY_PC5,      
    DEGRADED_UU,      
    BLIND_KINEMATIC   
};

enum class RuntimeMode {
    SIM,
    HIL
};

uint64_t get_system_time_us() {
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
    return (mode == RuntimeMode::HIL) ? get_ptp_time_us() : get_system_time_us();
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

float get_env_float(const char* name, float default_value) {
    const char* value = std::getenv(name);
    return value ? static_cast<float>(std::atof(value)) : default_value;
}

int get_env_int(const char* name, int default_value) {
    const char* value = std::getenv(name);
    return value ? std::atoi(value) : default_value;
}

struct PeerHealthState {
    bool enabled = false;
    bool has_valid = false;
    int sd = -1;
    uint16_t listen_port = static_cast<uint16_t>(VCU_PEER_PORT + 1);
    uint32_t expected_sender_id = T2T_BROADCAST_TRAIN_ID;
    uint32_t last_seq = 0;
    uint32_t received_count = 0;
    uint32_t seq_gap_count = 0;
    uint32_t invalid_count = 0;
    uint32_t wrong_source_count = 0;
    ssize_t last_invalid_size = 0;
    uint32_t last_invalid_header = 0;
    uint64_t last_rx_us = 0;
    PeerHealthPacket latest = {};
    char last_src_ip[INET_ADDRSTRLEN] = {0};
    uint16_t last_src_port = 0;
};

bool init_peer_health_receiver(PeerHealthState& state) {
    if (get_env_int("VCU_PEER_HEALTH_RX", 1) == 0) {
        std::cout << "[PEER_HEALTH_RX] Disabled by VCU_PEER_HEALTH_RX=0\n";
        return false;
    }

    int listen_port = get_env_int("VCU_PEER_HEALTH_PORT", VCU_PEER_PORT + 1);
    if (listen_port <= 0 || listen_port > 65535) {
        std::cerr << "[PEER_HEALTH_RX] Invalid VCU_PEER_HEALTH_PORT: " << listen_port << "\n";
        return false;
    }
    state.listen_port = static_cast<uint16_t>(listen_port);

    int expected_peer = get_env_int("VCU_EXPECTED_PEER_ID", -1);
    state.expected_sender_id = (expected_peer >= 0)
                                   ? static_cast<uint32_t>(expected_peer)
                                   : T2T_BROADCAST_TRAIN_ID;

    state.sd = socket(AF_INET, SOCK_DGRAM, 0);
    if (state.sd < 0) {
        std::perror("[PEER_HEALTH_RX] socket");
        return false;
    }

    int reuse_addr = 1;
    setsockopt(state.sd, SOL_SOCKET, SO_REUSEADDR, &reuse_addr, sizeof(reuse_addr));

    sockaddr_in bind_addr = {};
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_port = htons(state.listen_port);
    const char* bind_ip = std::getenv("VCU_PEER_BIND_IP");
    if (bind_ip == nullptr || bind_ip[0] == '\0') {
        bind_addr.sin_addr.s_addr = INADDR_ANY;
    } else if (inet_pton(AF_INET, bind_ip, &bind_addr.sin_addr) != 1) {
        std::cerr << "[PEER_HEALTH_RX] Invalid VCU_PEER_BIND_IP: " << bind_ip << "\n";
        close(state.sd);
        state.sd = -1;
        return false;
    }

    if (bind(state.sd, reinterpret_cast<sockaddr*>(&bind_addr), sizeof(bind_addr)) != 0) {
        std::perror("[PEER_HEALTH_RX] bind");
        close(state.sd);
        state.sd = -1;
        return false;
    }

    state.enabled = true;
    std::cout << "[PEER_HEALTH_RX] Listening on UDP port " << state.listen_port;
    if (state.expected_sender_id != T2T_BROADCAST_TRAIN_ID) {
        std::cout << " expected_peer=" << state.expected_sender_id;
    }
    std::cout << "\n";
    return true;
}

void poll_peer_health(PeerHealthState& state, RuntimeMode runtime_mode) {
    if (!state.enabled || state.sd < 0) return;

    while (true) {
        PeerHealthPacket pkt = {};
        sockaddr_in src_addr = {};
        socklen_t src_len = sizeof(src_addr);
        ssize_t n = recvfrom(state.sd, &pkt, sizeof(pkt), MSG_DONTWAIT,
                             reinterpret_cast<sockaddr*>(&src_addr), &src_len);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) return;
            std::perror("[PEER_HEALTH_RX] recvfrom");
            return;
        }

        if (n != static_cast<ssize_t>(sizeof(pkt)) || pkt.header != PEER_HEALTH_HEADER) {
            state.invalid_count++;
            state.last_invalid_size = n;
            state.last_invalid_header = pkt.header;
            continue;
        }
        if (state.expected_sender_id != T2T_BROADCAST_TRAIN_ID &&
            pkt.sender_id != state.expected_sender_id) {
            state.wrong_source_count++;
            continue;
        }

        if (state.has_valid) {
            uint32_t expected_next = state.last_seq + 1u;
            if (pkt.seq != expected_next) {
                state.seq_gap_count++;
            }
        }

        state.has_valid = true;
        state.latest = pkt;
        state.last_seq = pkt.seq;
        state.last_rx_us = get_runtime_time_us(runtime_mode);
        state.received_count++;
        inet_ntop(AF_INET, &src_addr.sin_addr, state.last_src_ip, sizeof(state.last_src_ip));
        state.last_src_port = ntohs(src_addr.sin_port);
    }
}

float peer_health_rx_aoi_ms(const PeerHealthState& state, RuntimeMode runtime_mode) {
    if (!state.has_valid) return 999999.0f;
    uint64_t now_us = get_runtime_time_us(runtime_mode);
    return (now_us >= state.last_rx_us) ? static_cast<float>(now_us - state.last_rx_us) / 1000.0f : 0.0f;
}

// 包含了死区与弹性系数的控制器
float calculate_business_force(PlatoonState state, float v, float target_v, float gap, float safe_gap, float uncertainty_buffer, float max_force, float front_v, NetworkHealth net_state) {
    if (v > Safety::kSpeedGuardMps) return -max_force * 0.35f; 
    
    float final_net_f = 0.0f;
    float dynamic_uncertainty = std::min(uncertainty_buffer, Safety::kMaxUncertaintyBufferM);

    if (state == PlatoonState::DECOUPLING) {
        float decoupling_target_v = Safety::kDecouplingTargetMps; 
        float err_v = decoupling_target_v - v;
        final_net_f = std::clamp(err_v * Safety::kDecouplingSpeedGain, -max_force, max_force);
    } else {
        float err_v_cruise = target_v - v;
        float cruise_force = err_v_cruise * Safety::kCruiseSpeedGain;
        float target_gap = (state == PlatoonState::LEADER_NEW)
                               ? std::max(Safety::kLeaderMinTrackingGapM, safe_gap + Safety::kLeaderSafeGapMarginM)
                               : (safe_gap + Safety::kFollowerSafeGapMarginM + dynamic_uncertainty); 
        
        float err_p = gap - target_gap;
        float err_v_follow = front_v - v;
        
        // 速度死区保留列车的真实物理不同步特性
        float active_err_v = 0.0f;
        if (err_v_follow > Safety::kFollowVelocityDeadbandMps) active_err_v = err_v_follow - Safety::kFollowVelocityDeadbandMps;
        else if (err_v_follow < -Safety::kFollowVelocityDeadbandMps) active_err_v = err_v_follow + Safety::kFollowVelocityDeadbandMps;

        float follow_force = (err_p * Safety::kFollowPositionGain) + (active_err_v * Safety::kFollowVelocityGain);
        final_net_f = (gap < Safety::kFollowControlRangeM) ? std::min(cruise_force, follow_force) : cruise_force;
    }
    
    if (v > (target_v - 0.5f) && final_net_f > 0.0f) {
        float factor = std::max(0.0f, 1.0f - (v - (target_v - 0.5f)) / 0.5f);
        final_net_f *= factor;
    }
    return std::clamp(final_net_f, -max_force, max_force);
}

int main(int argc, char* argv[]) {
    srand(time(NULL));
    RuntimeMode runtime_mode = parse_runtime_mode(argc, argv);
    bool hil_mode = runtime_mode == RuntimeMode::HIL;
    
    // 初始化工业级基础环境 (RT & Watchdog) 
    // 假设绑定到核心1，防止被系统进程抢占
    bool rt_enabled = VCU::Infra::SystemRT::init(1); 
    if (!rt_enabled) {
        std::cout << "[RT_SYSTEM] Simulation continues in non-RT user mode.\n";
    }
    

    int id = 3; 
    TrainConfig my_cfg = get_fleet_config(id);
    TrainConfig front_cfg = get_fleet_config(id - 1);
    LocomotiveProfile my_profile = FleetDatabase::get_loco_profile(my_cfg.model);
    WagonProfile my_wagon = my_cfg.is_empty ? HeavyHaulDynamics::C80_EMPTY_PROFILE : HeavyHaulDynamics::C80_PROFILE;
    WagonProfile front_wagon = front_cfg.is_empty ? HeavyHaulDynamics::C80_EMPTY_PROFILE : HeavyHaulDynamics::C80_PROFILE;
    
    float TOTAL_MASS = my_profile.mass_kg + (my_cfg.num_wagons * my_wagon.gross_mass_kg);
    float train_len = 22.0f + my_cfg.num_wagons * my_wagon.length_m; 

    std::cout << "[SD-VCU] Node " << id << " Booting up. Mass: " << TOTAL_MASS / 1000.0f
              << " t. Mode: " << (hil_mode ? "HIL" : "SIM") << "\n";

    SDVCU_Core sdvcu(my_cfg.model, front_cfg.model, my_cfg.num_wagons);
    PlatoonState current_state = PlatoonState::STARTING;
    NetworkHealth net_state = NetworkHealth::HEALTHY_PC5;

    // 通信平面初始化
    int srv_sd = socket(AF_INET, SOCK_DGRAM, 0);
    if (srv_sd < 0) {
        std::perror("[NETWORK] socket");
        return 1;
    }
    struct sockaddr_in srv;
    std::memset(&srv, 0, sizeof(srv));
    srv.sin_family = AF_INET;
    srv.sin_port = htons(VCU_REPORT_PORT);
    const char* server_ip = std::getenv("SERVER_IP");
    if (server_ip == nullptr) {
        server_ip = hil_mode ? "192.168.1.10" : "127.0.0.1";
    }
    if (inet_pton(AF_INET, server_ip, &srv.sin_addr) != 1) {
        std::cerr << "[NETWORK] Invalid SERVER_IP: " << server_ip << "\n";
        return 1;
    }
    std::cout << "[NETWORK] UDP target server: " << server_ip << ":" << VCU_REPORT_PORT << "\n";

    int t2t_snapshot_sd = -1;
    struct sockaddr_in t2t_snapshot_addr;
    std::memset(&t2t_snapshot_addr, 0, sizeof(t2t_snapshot_addr));
    const char* t2t_snapshot_ip = std::getenv("T2T_SNAPSHOT_IP");
    int t2t_snapshot_port = get_env_int("T2T_SNAPSHOT_PORT", VCU_PEER_PORT);
    bool t2t_snapshot_enabled = (t2t_snapshot_ip != nullptr && t2t_snapshot_ip[0] != '\0');
    if (t2t_snapshot_enabled) {
        t2t_snapshot_sd = socket(AF_INET, SOCK_DGRAM, 0);
        if (t2t_snapshot_sd < 0) {
            std::perror("[T2T_FRAME] socket");
            t2t_snapshot_enabled = false;
        } else {
            t2t_snapshot_addr.sin_family = AF_INET;
            t2t_snapshot_addr.sin_port = htons(t2t_snapshot_port);
            if (inet_pton(AF_INET, t2t_snapshot_ip, &t2t_snapshot_addr.sin_addr) != 1) {
                std::cerr << "[T2T_FRAME] Invalid T2T_SNAPSHOT_IP: " << t2t_snapshot_ip << "\n";
                close(t2t_snapshot_sd);
                t2t_snapshot_sd = -1;
                t2t_snapshot_enabled = false;
            } else {
                std::cout << "[T2T_FRAME] ATP snapshot frame output: "
                          << t2t_snapshot_ip << ":" << t2t_snapshot_port << "\n";
            }
        }
    }

    PeerHealthState peer_health;
    init_peer_health_receiver(peer_health);

    T2T_Radio lora_radio;
    const char* lora_dev = std::getenv("LORA_DEV");
    if (lora_dev == nullptr) {
        lora_dev = (access("/tmp/lora-node", F_OK) == 0) ? "/tmp/lora-node" : "/dev/ttyUSB0";
    }
    bool lora_active = lora_radio.init(lora_dev, B115200, 0x17); 
    if (lora_active) {
        std::cout << "[LORA_HAL] 400MHz Safety Plane Active.\n";
    } else {
        std::cout << "[LORA_HAL] Running without physical LoRa safety plane.\n";
    }

    std::ofstream vcu_log("vcu_telemetry.csv", std::ios::trunc);
    vcu_log << "Time(s),AoI(ms),BurstLossCount,TimeJitter(us),EstimatedGap(m),SafeGap(m),State,"
            << "EffectiveDelay(s),UncertaintyBuffer(m),SafetyMargin(m),InTunnel,NlosCurve,"
            << "PlannedBlind,BlindDistance(m),BlindRiskLevel,"
            << "PeerHealthValid,PeerHealthRxAoI(ms),PeerReportedAoI(ms),PeerHealthNetwork,"
            << "PeerHealthSeq,PeerHealthRxCount,PeerHealthSeqGapCount,PeerHealthInvalidCount,"
            << "PeerHealthWrongSourceCount,PeerHealthLastInvalidSize,PeerHealthLastInvalidHeader\n";

    float TARGET_CRUISE_VEL = 24.5f; // 目标巡航速度88

    float cur_f = 0.0f, dt = 0.01f, simulated_my_pos = (4 - id) * 1000.0f + 1000.0f, simulated_my_vel = 0.0f;
    KalmanTracker kf(simulated_my_pos + 400.0f + train_len, 0.0f);
    float uncertainty_buffer = 0.0f, time_since_last_tx = 0.0f, lora_tx_timer = 0.0f;
    float blind_entry_pos = 0.0f;
    bool was_in_tunnel = false;
    const float LORA_HEARTBEAT_INTERVAL = 2.0f; 
    float current_tx_interval = 0.01f; 
    int tick_counter = 0, burst_loss_count = 0;
    uint64_t total_ticks = 0, last_rx_timestamp_us = get_runtime_time_us(runtime_mode), last_time_diff_us = 0;
    uint32_t snapshot_seq = 0;
    uint32_t t2t_snapshot_frame_seq = 0;
    AtpSnapshot local_snapshot = {};

    HardwareWatchdog hw_wd; 
    hw_wd.pet(); // 强制完成第一次喂狗，重置计时器
    bool is_fail_safe_engaged = false; 
    bool simulation_complete = false;
    bool has_received_feedback = false;
    uint64_t boot_timestamp_us = get_runtime_time_us(runtime_mode);
    float startup_grace_ms = Safety::kStartupGraceMs;
    if (const char* grace_env = std::getenv("VCU_STARTUP_GRACE_MS")) {
        startup_grace_ms = std::max(0.0f, static_cast<float>(std::atof(grace_env)));
    }
    float hil_degraded_aoi_ms = std::max(0.0f, get_env_float("VCU_HIL_DEGRADED_AOI_MS", 30.0f));

    while (!simulation_complete) {
        auto loop_start = std::chrono::steady_clock::now();
        
        
        // 先检查上一周期是否超时，再喂狗
        bool watchdog_timeout = hw_wd.is_timed_out();
        if (watchdog_timeout) {
            std::cerr << "[WATCHDOG] Main loop exceeded " << Safety::kWatchdogTimeoutUs << " us before pet.\n";
        }
        hw_wd.pet();

        kf.predict(dt);
        time_since_last_tx += dt;
        total_ticks++;
        poll_peer_health(peer_health, runtime_mode);

        
        //1.SAFETY PLANE (LoRa Asynchronous EB Polling)
        
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

      
        //2.CONTROL PLANE (5G TSN Deterministic Sync)
      
        bool in_tunnel = !hil_mode && (simulated_my_pos > 5000.0f && simulated_my_pos < 7000.0f);
        bool nlos_curve = !hil_mode && (simulated_my_pos > 10000.0f && simulated_my_pos < 11000.0f);
        bool pc5_available = hil_mode || (!in_tunnel && !nlos_curve);
        bool uu_available = hil_mode || !in_tunnel; 
        if (in_tunnel && !was_in_tunnel) {
            blind_entry_pos = simulated_my_pos;
        } else if (!in_tunnel) {
            blind_entry_pos = simulated_my_pos;
        }
        was_in_tunnel = in_tunnel;

        WorldFeedbackPacket feedback = {};
        bool raw_recv = (recv(srv_sd, &feedback, sizeof(feedback), MSG_DONTWAIT) == sizeof(feedback));
        bool packet_received = false;

        if (raw_recv) {
            if ((feedback.flags & WORLD_FEEDBACK_SIM_COMPLETE) != 0u) {
                simulation_complete = true;
                packet_received = true;
                burst_loss_count = 0; 
                uint64_t now_us = get_runtime_time_us(runtime_mode);
                last_time_diff_us = (feedback.tx_timestamp_us > 0 && now_us >= feedback.tx_timestamp_us)
                                        ? (now_us - feedback.tx_timestamp_us) : 0;
                last_rx_timestamp_us = now_us - last_time_diff_us;
            } else {
                bool accept_packet = true;
                if (!hil_mode) {
                    float dist = std::max(0.0f, kf.pos - simulated_my_pos - train_len);
                    float per = PerceptionEngine::get_packet_error_rate(dist, in_tunnel);
                    float dice = (float)rand() / (float)RAND_MAX;
                    accept_packet = dice > per;
                }
                
                if (accept_packet) { 
                    packet_received = true;
                    burst_loss_count = 0; 
                    uint64_t now_us = get_runtime_time_us(runtime_mode);
                    uint64_t base_latency = hil_mode ? 0 : (pc5_available ? 5000 : (uu_available ? 35000 : 0));
                    last_time_diff_us = (feedback.tx_timestamp_us > 0 && now_us >= feedback.tx_timestamp_us)
                                            ? (now_us - feedback.tx_timestamp_us) : base_latency;
                    last_rx_timestamp_us = now_us - last_time_diff_us;
                } else {
                    burst_loss_count++; 
                }
            }
        } else {
            burst_loss_count++; 
        }

        if (packet_received) {
            has_received_feedback = true;
            uint64_t sim_tx_time = get_runtime_time_us(runtime_mode) - last_time_diff_us;
            bool hil_degraded_feedback = hil_mode &&
                                         !simulation_complete &&
                                         (last_time_diff_us >= static_cast<uint64_t>(hil_degraded_aoi_ms * 1000.0f));
            if (simulation_complete) {
                net_state = NetworkHealth::HEALTHY_PC5;
            } else if (hil_degraded_feedback && net_state != NetworkHealth::DEGRADED_UU) {
                net_state = NetworkHealth::DEGRADED_UU;
            } else if (pc5_available && net_state != NetworkHealth::HEALTHY_PC5) {
                net_state = NetworkHealth::HEALTHY_PC5;
            } else if (!pc5_available && uu_available && net_state != NetworkHealth::DEGRADED_UU) {
                net_state = NetworkHealth::DEGRADED_UU;
            }
            last_rx_timestamp_us = sim_tx_time;
            if (!hil_mode) {
                simulated_my_pos = feedback.ego_pos;
                simulated_my_vel = feedback.ego_vel;
            }
            kf.update(simulated_my_pos + feedback.true_gap + train_len, feedback.front_vel);
        }

        float current_aoi_ms = (get_runtime_time_us(runtime_mode) - last_rx_timestamp_us) / 1000.0f;
        float since_boot_ms = (get_runtime_time_us(runtime_mode) - boot_timestamp_us) / 1000.0f;
        bool startup_grace_active = !has_received_feedback && since_boot_ms < startup_grace_ms;
        if (!startup_grace_active && current_aoi_ms > Safety::kBlindEntryAoiMs &&
            net_state != NetworkHealth::BLIND_KINEMATIC) {
            net_state = NetworkHealth::BLIND_KINEMATIC;
        }

        float blind_elapsed_s = (in_tunnel && net_state == NetworkHealth::BLIND_KINEMATIC)
                                    ? (current_aoi_ms / 1000.0f) : 0.0f;
        float blind_distance_m = in_tunnel ? std::max(0.0f, simulated_my_pos - blind_entry_pos) : 0.0f;
        int blind_risk_level = 0;
        if (blind_elapsed_s >= Safety::kBlindRestrictedAoiS) {
            blind_risk_level = 2;
        } else if (blind_elapsed_s >= Safety::kBlindCautionAoiS) {
            blind_risk_level = 1;
        }

        if (net_state == NetworkHealth::HEALTHY_PC5) uncertainty_buffer = std::max(0.0f, uncertainty_buffer - 5.0f * dt); 
        else if (net_state == NetworkHealth::DEGRADED_UU) uncertainty_buffer = std::max(30.0f, uncertainty_buffer - 2.0f * dt); 
        else if (net_state == NetworkHealth::BLIND_KINEMATIC) {
            float blind_uncertainty = (blind_elapsed_s * Safety::kBlindUncertaintyByTimeMps) +
                                      (blind_distance_m * Safety::kBlindUncertaintyByDistanceRatio);
            uncertainty_buffer = std::min(
                Safety::kMaxUncertaintyBufferM,
                std::max(uncertainty_buffer + 2.5f * dt, blind_uncertainty));
        }

        float estimated_gap = kf.pos - simulated_my_pos - train_len;
        if (estimated_gap < 0.0f) estimated_gap = 0.0f;
        
       
        // 3. 故障注入与 FAIL-SAFE 触发逻辑
        // 只要看门狗超时，或者发生极度恶劣的非计划通信中断（5秒全盲），立刻触发系统级保护
        // 计划内隧道盲区应进入 BLIND_KINEMATIC，由 KF 与 ATP 安全包络继续约束
        // 1先判断是否触发了异常
        bool planned_blind_run = in_tunnel && net_state == NetworkHealth::BLIND_KINEMATIC;
        bool aoi_timeout_fault = current_aoi_ms > Safety::kUnexpectedAoiFailSafeMs &&
                                 !planned_blind_run && !startup_grace_active;
        bool trigger_fail_safe = !is_fail_safe_engaged && (watchdog_timeout || aoi_timeout_fault);

        if (trigger_fail_safe) {
            // 2只有在真正触发时才打印原因
            if (watchdog_timeout) {
                std::cerr << "[DEBUG] FAIL-SAFE TRIGGERED BY: Watchdog Timeout (50ms)!" << std::endl;
            }
            if (aoi_timeout_fault) {
                std::cerr << "[DEBUG] FAIL-SAFE TRIGGERED BY: AoI Timeout (>5000ms)!" << std::endl;
            }

            // 3打印致命故障信息
            std::cout << "\n======================================================\n";
            std::cout << "[CRITICAL FATAL] FAIL-SAFE ENGAGED!\n";
            std::cout << "Reason: " << (watchdog_timeout ? "Watchdog Timeout." : "AoI Timeout.") << "\n";
            std::cout << "Action: Forcing Maximum Pneumatic Braking. Controller Locked.\n";
            std::cout << "======================================================\n\n";
            
            // 4锁定状态
            is_fail_safe_engaged = true; 
        }

      
        // 4. TOPOGRAPHY-AWARE SAFETY ENVELOPE
        float effective_gradient = 0.0f;
        if (simulated_my_pos > 8000.0f && simulated_my_pos <= 10000.0f) effective_gradient = -4.0f; 
        else if (simulated_my_pos > 10000.0f && simulated_my_pos < 12000.0f) effective_gradient = -4.0f + (600.0f / 400.0f); 
        else if (simulated_my_pos > 12000.0f && simulated_my_pos < 15000.0f) effective_gradient = 6.0f;

        float f_gravity = -TOTAL_MASS * 9.8f * (effective_gradient / 1000.0f);
        float effective_delay_s = current_aoi_ms / 1000.0f;
        if (net_state == NetworkHealth::BLIND_KINEMATIC) {
            effective_delay_s = std::min(
                Safety::kMaxCertifiedBlindDelayS,
                Safety::kBlindEffectiveDelayS + (blind_elapsed_s * Safety::kBlindDelayGrowthPerSecond));
        }

        sdvcu.current_safe_gap = PerceptionEngine::calculate_heavy_haul_safe_dist(
            simulated_my_vel, kf.vel, my_cfg.model, front_cfg.model, 
            my_wagon, front_wagon, my_cfg.num_wagons, front_cfg.num_wagons, false, effective_delay_s,
            effective_gradient); 
        float safety_margin = estimated_gap - sdvcu.current_safe_gap;

        // 仅在非 Fail-Safe 模式下触发常规 ATP Trip
        if (!is_fail_safe_engaged && estimated_gap > 0.1f && estimated_gap <= sdvcu.current_safe_gap && !sdvcu.is_eb()) {
            std::cout << "[FATAL] ATP TRIP! Gap: " << estimated_gap << "m | Safe Gap: " << sdvcu.current_safe_gap << "m\n";
            sdvcu.force_eb_trigger();
        }

       
        // 5. KINEMATICS & STATE MACHINE
        // 状态机流转：只有在未进入 Fail-Safe 时才允许切换业务状态
        if (!is_fail_safe_engaged) {
            float v_difference = kf.vel - simulated_my_vel; 
            if (current_state == PlatoonState::STARTING && simulated_my_vel >= 18.0f) {
                current_state = PlatoonState::CRUISING;
            }
            else if (current_state == PlatoonState::CRUISING && simulated_my_pos > 8000.0f) {
                current_state = PlatoonState::DECOUPLING;
            }
            else if (current_state == PlatoonState::DECOUPLING) {
                if (estimated_gap > 900.0f && v_difference > 3.0f) {
                    current_state = PlatoonState::LEADER_NEW; 
                }
            }
        }

        // 6. 最终输出计算 (Hardware Output Override)
        float target_net_f = 0.0f;
        
        // 优先级 1：系统级失效，输出物理极限安全制动 (1.5倍最大制动冗余)
        if (is_fail_safe_engaged) {
            target_net_f = -TOTAL_MASS * my_profile.avg_deceleration_limit * Safety::kFailSafeBrakeRedundancy;
        } 
        // 优先级 2：业务级紧急制动 (ATP Trip / LoRa 触发)
        else if (sdvcu.is_eb()) {
            target_net_f = -TOTAL_MASS * my_profile.avg_deceleration_limit;
        } 
        // 优先级 3：计划盲区风险升高时，先做限速/服务制动预备，不直接升级为 ATP Trip
        else if (planned_blind_run && blind_risk_level >= 1 &&
                 safety_margin < Safety::kBlindControlledBrakeMarginM) {
            target_net_f = -TOTAL_MASS * Safety::kBlindControlledBrakeDecelMps2;
        }
        // 优先级 4：正常业务跟车力矩计算
        else {
            float blind_target_v = TARGET_CRUISE_VEL;
            if (planned_blind_run && blind_risk_level >= 2) {
                blind_target_v = std::min(blind_target_v, Safety::kBlindRestrictedTargetMps);
            } else if (planned_blind_run && blind_risk_level >= 1) {
                blind_target_v = std::min(blind_target_v, Safety::kBlindCautionTargetMps);
            }
            target_net_f = calculate_business_force(
                current_state, simulated_my_vel, blind_target_v, estimated_gap, 
                sdvcu.current_safe_gap, uncertainty_buffer, my_profile.max_starting_tractive_effort_n, kf.vel, net_state);
        }

        // 牵引力物理爬升率钳制
        float target_motor_f = target_net_f - f_gravity;
        float max_up = (my_profile.max_starting_tractive_effort_n / my_profile.tractive_build_up_time_s) * dt;
        float max_down = ((TOTAL_MASS * my_profile.avg_deceleration_limit) / 3.0f) * dt;
        cur_f = (target_motor_f > cur_f) ? std::min(target_motor_f, cur_f + max_up) : std::max(target_motor_f, cur_f - max_down);

        float local_accel_mps2 = (cur_f + f_gravity) / TOTAL_MASS;
        simulated_my_vel += local_accel_mps2 * dt;
        if(simulated_my_vel < 0.0f) simulated_my_vel = 0.0f;
        simulated_my_pos += simulated_my_vel * dt;

        // 状态编码映射
        uint32_t report_status = 0;
        if (simulation_complete) report_status = STATUS_SIM_COMPLETE;
        else if (is_fail_safe_engaged) report_status = STATUS_FAIL_SAFE_LOCK; // 特殊编码：代表节点已死亡/锁定
        else if (sdvcu.is_eb()) report_status = STATUS_ATP_TRIP;
        else if (net_state == NetworkHealth::DEGRADED_UU) report_status = STATUS_DEGRADED_UU;
        else if (net_state == NetworkHealth::BLIND_KINEMATIC) report_status = STATUS_BLIND_RUN;
        else if (current_state == PlatoonState::DECOUPLING) report_status = STATUS_DECOUPLING;
        else if (current_state == PlatoonState::LEADER_NEW) report_status = STATUS_LEADER_NEW;

        FallbackTrainState fallback_state = {
            static_cast<uint32_t>(id),
            get_runtime_time_us(runtime_mode),
            simulated_my_pos,
            simulated_my_vel,
            local_accel_mps2,
            report_status,
            snapshot_seq++,
            hil_mode ? ATPAdapterSource::HIL_FALLBACK : ATPAdapterSource::SIM_FALLBACK
        };
        ATPAdapter::from_fallback(fallback_state, local_snapshot);

        
        // 7. 遥测与日志输出
        if (tick_counter % 10 == 0 || simulation_complete) {
            float time_s = total_ticks * dt; 
            float peer_health_rx_aoi = peer_health_rx_aoi_ms(peer_health, runtime_mode);
            float peer_reported_aoi = peer_health.has_valid ? peer_health.latest.peer_aoi_ms : 999999.0f;
            uint32_t peer_network_health = peer_health.has_valid ? peer_health.latest.network_health : STATUS_DEGRADED_UU;
            uint32_t peer_health_seq = peer_health.has_valid ? peer_health.latest.seq : 0u;
            vcu_log << time_s << "," << current_aoi_ms << "," << burst_loss_count << ","
                    << last_time_diff_us << "," << estimated_gap << "," << sdvcu.current_safe_gap << ","
                    << report_status << "," << effective_delay_s << "," << uncertainty_buffer << ","
                    << safety_margin << "," << (in_tunnel ? 1 : 0) << "," << (nlos_curve ? 1 : 0) << ","
                    << (planned_blind_run ? 1 : 0) << "," << blind_distance_m << "," << blind_risk_level << ","
                    << (peer_health.has_valid ? 1 : 0) << "," << peer_health_rx_aoi << ","
                    << peer_reported_aoi << "," << peer_network_health << ","
                    << peer_health_seq << "," << peer_health.received_count << ","
                    << peer_health.seq_gap_count << "," << peer_health.invalid_count << ","
                    << peer_health.wrong_source_count << "," << peer_health.last_invalid_size << ","
                    << peer_health.last_invalid_header << "\n";
            // warning:real sys移除flush，转为异步队列写入
            vcu_log.flush(); 
        }

        if (time_since_last_tx >= current_tx_interval) {
            ForceReportPacket rep = {
                static_cast<uint32_t>(id),
                cur_f,
                report_status,
                cmps_to_mps(local_snapshot.speed_cmps),
                cm_to_m(local_snapshot.train_pos_cm)
            };
            ssize_t sent = sendto(srv_sd, &rep, sizeof(rep), 0, (struct sockaddr*)&srv, sizeof(srv));
            if (sent != static_cast<ssize_t>(sizeof(rep))) {
                std::perror("[NETWORK] sendto ForceReportPacket");
            }
            if (t2t_snapshot_enabled) {
                T2TAtpSnapshotFrame snapshot_frame = {};
                build_t2t_atp_snapshot_frame(
                    static_cast<uint32_t>(id),
                    T2T_BROADCAST_TRAIN_ID,
                    1u,
                    t2t_snapshot_frame_seq++,
                    0u,
                    local_snapshot,
                    snapshot_frame);
                ssize_t frame_sent = sendto(t2t_snapshot_sd, &snapshot_frame, sizeof(snapshot_frame), 0,
                                            (struct sockaddr*)&t2t_snapshot_addr, sizeof(t2t_snapshot_addr));
                if (frame_sent != static_cast<ssize_t>(sizeof(snapshot_frame))) {
                    std::perror("[T2T_FRAME] sendto T2TAtpSnapshotFrame");
                }
            }
            time_since_last_tx = 0.0f; 
        }

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
            if (peer_health.enabled) {
                std::printf("[PEER_HEALTH_RX] Valid:%d | Peer:%u | Src:%s:%u | RxAoI:%6.1fms | PeerAoI:%6.1fms | Net:%u | Seq:%u | Rx:%u | Gap:%u | Invalid:%u | WrongSrc:%u\n",
                    peer_health.has_valid ? 1 : 0,
                    peer_health.has_valid ? peer_health.latest.sender_id : 0u,
                    peer_health.has_valid ? peer_health.last_src_ip : "-",
                    peer_health.has_valid ? peer_health.last_src_port : 0u,
                    peer_health_rx_aoi_ms(peer_health, runtime_mode),
                    peer_health.has_valid ? peer_health.latest.peer_aoi_ms : 999999.0f,
                    peer_health.has_valid ? peer_health.latest.network_health : STATUS_DEGRADED_UU,
                    peer_health.has_valid ? peer_health.latest.seq : 0u,
                    peer_health.received_count,
                    peer_health.seq_gap_count,
                    peer_health.invalid_count,
                    peer_health.wrong_source_count);
            }
            tick_counter = 0;
        }

        if (simulation_complete) {
            std::cout << "[SD-VCU] Simulation complete signal received. Logs flushed, node exiting cleanly.\n";
            break;
        }

        std::this_thread::sleep_until(loop_start + std::chrono::milliseconds(10));
    }
    if (peer_health.sd >= 0) close(peer_health.sd);
    if (t2t_snapshot_sd >= 0) close(t2t_snapshot_sd);
    if (srv_sd >= 0) close(srv_sd);
    return 0;
}
