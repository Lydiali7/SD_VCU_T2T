#include <iostream>
#include <cstring>
#include <cerrno>
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
#include "comm_health.hpp"
#include "comm_safety_action.hpp"
#include "comm_safety_supervisor.hpp"
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

CommSafetyThresholds get_comm_safety_thresholds() {
    CommSafetyThresholds thresholds;
    thresholds.degraded_ms = get_env_float("VCU_COMM_DEGRADED_AOI_MS", thresholds.degraded_ms);
    thresholds.blind_run_ms = get_env_float("VCU_COMM_BLIND_AOI_MS", thresholds.blind_run_ms);
    thresholds.fail_safe_ms = get_env_float("VCU_COMM_FAIL_SAFE_AOI_MS", thresholds.fail_safe_ms);
    thresholds.recover_normal_ms = get_env_float("VCU_COMM_RECOVER_NORMAL_AOI_MS", thresholds.recover_normal_ms);
    thresholds.recover_degraded_ms = get_env_float("VCU_COMM_RECOVER_DEGRADED_AOI_MS", thresholds.recover_degraded_ms);
    int recovery_count = get_env_int("VCU_COMM_RECOVERY_GOOD_SNAPSHOTS",
                                     static_cast<int>(thresholds.recovery_good_snapshots));
    thresholds.recovery_good_snapshots = recovery_count > 0 ? static_cast<uint32_t>(recovery_count) : 0u;
    return thresholds;
}

CommSafetyActionPolicy get_comm_safety_action_policy() {
    CommSafetyActionPolicy policy;
    policy.degraded_traction_limit_ratio = get_env_float(
        "VCU_COMM_ACTION_DEGRADED_TRACTION_RATIO", policy.degraded_traction_limit_ratio);
    policy.degraded_uncertainty_extra_m = get_env_float(
        "VCU_COMM_ACTION_DEGRADED_UNCERTAINTY_M", policy.degraded_uncertainty_extra_m);
    policy.blind_run_traction_limit_ratio = get_env_float(
        "VCU_COMM_ACTION_BLIND_TRACTION_RATIO", policy.blind_run_traction_limit_ratio);
    policy.blind_run_uncertainty_extra_m = get_env_float(
        "VCU_COMM_ACTION_BLIND_UNCERTAINTY_M", policy.blind_run_uncertainty_extra_m);
    policy.fail_safe_uncertainty_extra_m = get_env_float(
        "VCU_COMM_ACTION_FAIL_SAFE_UNCERTAINTY_M", policy.fail_safe_uncertainty_extra_m);
    return policy;
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

// Local observation of the peer's safety-relevant state stream. PeerHealth is
// supplementary; this receiver is the authoritative record of snapshot arrival.
struct PeerSnapshotState {
    bool enabled = false;
    bool has_valid = false;
    int sd = -1;
    uint16_t listen_port = VCU_PEER_PORT;
    uint32_t expected_sender_id = T2T_BROADCAST_TRAIN_ID;
    bool has_expected_source_ip = false;
    in_addr expected_source_addr = {};
    char expected_source_ip[INET_ADDRSTRLEN] = {0};
    uint32_t last_seq = 0;
    uint32_t received_count = 0;
    uint32_t seq_gap_count = 0;
    uint64_t missing_snapshot_count = 0;
    uint32_t current_burst_loss_count = 0;
    uint32_t max_burst_loss_count = 0;
    uint32_t duplicate_count = 0;
    uint32_t replay_count = 0;
    uint32_t invalid_count = 0;
    uint32_t crc_error_count = 0;
    uint32_t wrong_sender_count = 0;
    uint32_t wrong_source_count = 0;
    uint32_t wrong_destination_count = 0;
    uint32_t stale_timestamp_count = 0;
    uint64_t max_timestamp_age_us = 0;
    uint64_t last_rx_us = 0;
    float last_timestamp_age_ms = 999999.0f;
    T2TAtpSnapshotFrame latest = {};
    char last_src_ip[INET_ADDRSTRLEN] = {0};
    uint16_t last_src_port = 0;
};

bool init_peer_snapshot_receiver(PeerSnapshotState& state, RuntimeMode runtime_mode) {
    if (get_env_int("VCU_PEER_SNAPSHOT_RX", 1) == 0) {
        std::cout << "[PEER_SNAPSHOT_RX] Disabled by VCU_PEER_SNAPSHOT_RX=0\n";
        return false;
    }

    int listen_port = get_env_int("VCU_PEER_SNAPSHOT_PORT", VCU_PEER_PORT);
    if (listen_port <= 0 || listen_port > 65535) {
        std::cerr << "[PEER_SNAPSHOT_RX] Invalid VCU_PEER_SNAPSHOT_PORT: " << listen_port << "\n";
        return false;
    }
    state.listen_port = static_cast<uint16_t>(listen_port);

    int expected_peer = get_env_int("VCU_EXPECTED_PEER_ID", -1);
    state.expected_sender_id = (expected_peer >= 0)
                                   ? static_cast<uint32_t>(expected_peer)
                                   : T2T_BROADCAST_TRAIN_ID;
    const char* expected_ip = std::getenv("VCU_EXPECTED_PEER_IP");
    if (expected_ip != nullptr && expected_ip[0] != '\0') {
        if (inet_pton(AF_INET, expected_ip, &state.expected_source_addr) != 1) {
            std::cerr << "[PEER_SNAPSHOT_RX] Invalid VCU_EXPECTED_PEER_IP: " << expected_ip << "\n";
            return false;
        }
        state.has_expected_source_ip = true;
        std::strncpy(state.expected_source_ip, expected_ip, sizeof(state.expected_source_ip) - 1);
    }

    // Timestamp age is meaningful only when hosts have a common PTP/TAI clock.
    float max_age_ms = get_env_float("VCU_PEER_MAX_TIMESTAMP_AGE_MS", 0.0f);
    state.max_timestamp_age_us = max_age_ms > 0.0f
                                     ? static_cast<uint64_t>(max_age_ms * 1000.0f)
                                     : 0;

    state.sd = socket(AF_INET, SOCK_DGRAM, 0);
    if (state.sd < 0) {
        std::perror("[PEER_SNAPSHOT_RX] socket");
        return false;
    }
    int reuse_addr = 1;
    setsockopt(state.sd, SOL_SOCKET, SO_REUSEADDR, &reuse_addr, sizeof(reuse_addr));

    sockaddr_in bind_addr = {};
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_port = htons(state.listen_port);
    const char* bind_ip = std::getenv("VCU_PEER_SNAPSHOT_BIND_IP");
    if (bind_ip == nullptr || bind_ip[0] == '\0') {
        bind_addr.sin_addr.s_addr = INADDR_ANY;
    } else if (inet_pton(AF_INET, bind_ip, &bind_addr.sin_addr) != 1) {
        std::cerr << "[PEER_SNAPSHOT_RX] Invalid VCU_PEER_SNAPSHOT_BIND_IP: " << bind_ip << "\n";
        close(state.sd);
        state.sd = -1;
        return false;
    }
    if (bind(state.sd, reinterpret_cast<sockaddr*>(&bind_addr), sizeof(bind_addr)) != 0) {
        std::perror("[PEER_SNAPSHOT_RX] bind");
        close(state.sd);
        state.sd = -1;
        return false;
    }

    state.enabled = true;
    std::cout << "[PEER_SNAPSHOT_RX] Listening on UDP port " << state.listen_port;
    if (state.expected_sender_id != T2T_BROADCAST_TRAIN_ID) std::cout << " expected_peer=" << state.expected_sender_id;
    if (state.has_expected_source_ip) std::cout << " expected_ip=" << state.expected_source_ip;
    if (state.max_timestamp_age_us > 0) std::cout << " max_timestamp_age_ms=" << state.max_timestamp_age_us / 1000.0f;
    std::cout << "\n";
    return true;
}

void poll_peer_snapshot(PeerSnapshotState& state, RuntimeMode runtime_mode, uint32_t local_train_id) {
    if (!state.enabled || state.sd < 0) return;

    while (true) {
        T2TAtpSnapshotFrame frame = {};
        sockaddr_in src_addr = {};
        socklen_t src_len = sizeof(src_addr);
        ssize_t n = recvfrom(state.sd, &frame, sizeof(frame), MSG_DONTWAIT,
                             reinterpret_cast<sockaddr*>(&src_addr), &src_len);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) return;
            std::perror("[PEER_SNAPSHOT_RX] recvfrom");
            return;
        }
        if (n != static_cast<ssize_t>(sizeof(frame))) {
            state.invalid_count++;
            continue;
        }
        T2TFrameValidationResult validation = validate_t2t_atp_snapshot_frame_detailed(frame);
        if (validation != T2TFrameValidationResult::VALID) {
            state.invalid_count++;
            if (validation == T2TFrameValidationResult::SNAPSHOT_CRC_ERROR ||
                validation == T2TFrameValidationResult::FRAME_CRC_ERROR) {
                state.crc_error_count++;
            }
            continue;
        }
        if (state.expected_sender_id != T2T_BROADCAST_TRAIN_ID &&
            frame.header.source_train_id != state.expected_sender_id) {
            state.wrong_sender_count++;
            continue;
        }
        char src_ip[INET_ADDRSTRLEN] = {0};
        inet_ntop(AF_INET, &src_addr.sin_addr, src_ip, sizeof(src_ip));
        if (state.has_expected_source_ip &&
            src_addr.sin_addr.s_addr != state.expected_source_addr.s_addr) {
            state.wrong_source_count++;
            continue;
        }
        if (frame.header.dest_train_id != local_train_id &&
            frame.header.dest_train_id != T2T_BROADCAST_TRAIN_ID) {
            state.wrong_destination_count++;
            continue;
        }

        uint64_t now_us = get_runtime_time_us(runtime_mode);
        state.last_timestamp_age_ms = 0.0f;
        if (state.max_timestamp_age_us > 0) {
            if (now_us < frame.header.tai_timestamp_us ||
                now_us - frame.header.tai_timestamp_us > state.max_timestamp_age_us) {
                state.stale_timestamp_count++;
                continue;
            }
            state.last_timestamp_age_ms = static_cast<float>(now_us - frame.header.tai_timestamp_us) / 1000.0f;
        }

        if (state.has_valid) {
            if (frame.header.seq == state.last_seq) {
                state.duplicate_count++;
                continue;
            }
            // uint32_t subtraction preserves correct ordering across sequence wrap.
            if (static_cast<int32_t>(frame.header.seq - state.last_seq) < 0) {
                state.replay_count++;
                continue;
            }
            if (frame.header.seq != state.last_seq + 1u) {
                uint32_t missing = frame.header.seq - state.last_seq - 1u;
                state.seq_gap_count++;
                state.missing_snapshot_count += missing;
                state.current_burst_loss_count = missing;
                state.max_burst_loss_count = std::max(state.max_burst_loss_count, missing);
            } else {
                state.current_burst_loss_count = 0;
            }
        }

        state.has_valid = true;
        state.latest = frame;
        state.last_seq = frame.header.seq;
        state.last_rx_us = now_us;
        state.received_count++;
        std::strncpy(state.last_src_ip, src_ip, sizeof(state.last_src_ip) - 1);
        state.last_src_port = ntohs(src_addr.sin_port);
    }
}

float peer_snapshot_rx_aoi_ms(const PeerSnapshotState& state, RuntimeMode runtime_mode) {
    if (!state.has_valid) return 999999.0f;
    uint64_t now_us = get_runtime_time_us(runtime_mode);
    return now_us >= state.last_rx_us ? static_cast<float>(now_us - state.last_rx_us) / 1000.0f : 0.0f;
}

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

CommHealthState build_comm_health_state(const PeerSnapshotState& snapshot,
                                        const PeerHealthState& health,
                                        RuntimeMode runtime_mode) {
    CommHealthState state;
    state.peer_present = snapshot.has_valid;
    state.snapshot_valid = snapshot.has_valid;
    state.last_valid_rx_us = snapshot.last_rx_us;
    state.snapshot_rx_aoi_ms = peer_snapshot_rx_aoi_ms(snapshot, runtime_mode);
    state.last_seq = snapshot.has_valid ? snapshot.last_seq : 0u;
    state.rx_count = snapshot.received_count;
    state.seq_gap_count = snapshot.seq_gap_count;
    state.missing_snapshot_count = snapshot.missing_snapshot_count;
    state.current_burst_loss_count = snapshot.current_burst_loss_count;
    state.max_burst_loss_count = snapshot.max_burst_loss_count;
    state.invalid_count = snapshot.invalid_count;
    state.crc_error_count = snapshot.crc_error_count;
    state.wrong_sender_count = snapshot.wrong_sender_count;
    state.wrong_source_count = snapshot.wrong_source_count;
    state.wrong_destination_count = snapshot.wrong_destination_count;
    state.duplicate_count = snapshot.duplicate_count;
    state.replay_count = snapshot.replay_count;
    state.stale_timestamp_count = snapshot.stale_timestamp_count;
    state.peer_health_valid = health.has_valid;
    state.peer_health_rx_aoi_ms = peer_health_rx_aoi_ms(health, runtime_mode);
    state.peer_reported_aoi_ms = health.has_valid ? health.latest.peer_aoi_ms : 999999.0f;
    state.peer_reported_network = health.has_valid ? health.latest.network_health : STATUS_DEGRADED_UU;
    state.peer_health_seq = health.has_valid ? health.latest.seq : 0u;
    return state;
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

    PeerSnapshotState peer_snapshot;
    init_peer_snapshot_receiver(peer_snapshot, runtime_mode);

    PeerHealthState peer_health;
    init_peer_health_receiver(peer_health);

    bool comm_safety_shadow_enabled = get_env_int("VCU_COMM_SAFETY_SHADOW", 0) != 0;
    CommSafetyThresholds comm_safety_thresholds = get_comm_safety_thresholds();
    if (!comm_safety_thresholds.is_valid()) {
        std::cerr << "[COMM_SAFETY] Invalid threshold configuration; using research/shadow defaults.\n";
        comm_safety_thresholds = CommSafetyThresholds{};
    }
    CommSafetySupervisor comm_safety_supervisor(comm_safety_thresholds);
    bool comm_action_shadow_enabled = get_env_int("VCU_COMM_ACTION_SHADOW", 0) != 0;
    CommSafetyActionPolicy comm_action_policy = get_comm_safety_action_policy();
    if (!comm_action_policy.is_valid()) {
        std::cerr << "[COMM_ACTION] Invalid action policy; using research/shadow defaults.\n";
        comm_action_policy = CommSafetyActionPolicy{};
    }
    CommSafetyActionResolver comm_action_resolver(comm_action_policy);
    if (comm_action_shadow_enabled && !comm_safety_shadow_enabled) {
        std::cerr << "[COMM_ACTION] Shadow output requires VCU_COMM_SAFETY_SHADOW=1; action shadow disabled.\n";
        comm_action_shadow_enabled = false;
    }
    if (comm_safety_shadow_enabled) {
        std::cout << "[COMM_SAFETY] Shadow mode enabled: degraded=" << comm_safety_thresholds.degraded_ms
                  << "ms blind=" << comm_safety_thresholds.blind_run_ms
                  << "ms fail=" << comm_safety_thresholds.fail_safe_ms
                  << "ms recovery_good_snapshots=" << comm_safety_thresholds.recovery_good_snapshots << "\n";
    }
    if (comm_action_shadow_enabled) {
        std::cout << "[COMM_ACTION] Shadow mode enabled: degraded_traction_ratio="
                  << comm_action_policy.degraded_traction_limit_ratio
                  << " blind_traction_ratio=" << comm_action_policy.blind_run_traction_limit_ratio << "\n";
    }

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
            << "PeerHealthWrongSourceCount,PeerHealthLastInvalidSize,PeerHealthLastInvalidHeader,"
            << "PeerSnapshotValid,PeerSnapshotRxAoI(ms),PeerSnapshotTimestampAge(ms),PeerSnapshotSeq,"
            << "PeerSnapshotRxCount,PeerSnapshotSeqGapCount,PeerSnapshotDuplicateCount,PeerSnapshotReplayCount,"
            << "PeerSnapshotInvalidCount,PeerSnapshotWrongSourceCount,PeerSnapshotWrongDestCount,"
            << "PeerSnapshotStaleTimestampCount,PeerSnapshotMissingCount,PeerSnapshotCurrentBurstLoss,"
            << "PeerSnapshotMaxBurstLoss,PeerSnapshotCrcErrorCount,PeerSnapshotWrongSenderCount,"
            << "CommPeerPresent,CommSnapshotValid,CommSnapshotRxAoI(ms),CommLastValidRx(us),CommLastSeq,"
            << "CommRxCount,CommSeqGapCount,CommMissingSnapshotCount,CommCurrentBurstLoss,CommMaxBurstLoss,"
            << "CommInvalidCount,CommCrcErrorCount,CommWrongSenderCount,CommWrongSourceCount,"
            << "CommWrongDestCount,CommDuplicateCount,CommReplayCount,CommStaleTimestampCount,"
            << "CommPeerHealthValid,CommPeerHealthRxAoI(ms),CommPeerReportedAoI(ms),"
            << "CommPeerReportedNetwork,CommPeerHealthSeq,"
            << "CommSafetyShadowEnabled,CommSafetyShadowActive,CommSafetyProposedState,CommSafetyReason,"
            << "CommSafetyGoodSnapshotStreak,CommSafetyStateChanged,"
            << "CommActionShadowEnabled,CommActionShadowActive,CommActionSourceState,CommActionSourceReason,"
            << "CommActionTractionPermitted,CommActionCooperativeFollowingPermitted,CommActionBlindRunActive,"
            << "CommActionFailSafeBrakeRequested,CommActionTractionLimitRatio,CommActionUncertaintyExtra(m)\n";

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
    CommHealthState comm_health;
    CommSafetyDecision comm_safety_decision;
    CommSafetyAction comm_safety_action;

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
    float comm_safety_startup_grace_ms = std::max(
        0.0f, get_env_float("VCU_COMM_SAFETY_STARTUP_GRACE_MS", Safety::kStartupGraceMs));
    bool comm_safety_shadow_active = false;
    bool comm_action_shadow_active = false;
    if (comm_safety_shadow_enabled) {
        std::cout << "[COMM_SAFETY] Shadow startup grace=" << comm_safety_startup_grace_ms << "ms\n";
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
        poll_peer_snapshot(peer_snapshot, runtime_mode, static_cast<uint32_t>(id));
        poll_peer_health(peer_health, runtime_mode);
        comm_health = build_comm_health_state(peer_snapshot, peer_health, runtime_mode);

        
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
        comm_safety_shadow_active = comm_safety_shadow_enabled &&
                                    since_boot_ms >= comm_safety_startup_grace_ms;
        if (comm_safety_shadow_active) {
            comm_safety_decision = comm_safety_supervisor.update(comm_health);
            if (comm_action_shadow_enabled) {
                comm_safety_action = comm_action_resolver.resolve(comm_safety_decision);
                comm_action_shadow_active = true;
            }
            if (comm_safety_decision.changed) {
                std::cout << "[COMM_SAFETY] proposed=" << to_string(comm_safety_decision.state)
                          << " reason=" << to_string(comm_safety_decision.reason)
                          << " aoi_ms=" << comm_health.snapshot_rx_aoi_ms << "\n";
            }
        } else {
            comm_action_shadow_active = false;
        }
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
            vcu_log << time_s << "," << current_aoi_ms << "," << burst_loss_count << ","
                    << last_time_diff_us << "," << estimated_gap << "," << sdvcu.current_safe_gap << ","
                    << report_status << "," << effective_delay_s << "," << uncertainty_buffer << ","
                    << safety_margin << "," << (in_tunnel ? 1 : 0) << "," << (nlos_curve ? 1 : 0) << ","
                    << (planned_blind_run ? 1 : 0) << "," << blind_distance_m << "," << blind_risk_level << ","
                    << (peer_health.has_valid ? 1 : 0) << "," << comm_health.peer_health_rx_aoi_ms << ","
                    << comm_health.peer_reported_aoi_ms << "," << comm_health.peer_reported_network << ","
                    << comm_health.peer_health_seq << "," << peer_health.received_count << ","
                    << peer_health.seq_gap_count << "," << peer_health.invalid_count << ","
                    << peer_health.wrong_source_count << "," << peer_health.last_invalid_size << ","
                    << peer_health.last_invalid_header << ","
                    << (peer_snapshot.has_valid ? 1 : 0) << "," << comm_health.snapshot_rx_aoi_ms << ","
                    << peer_snapshot.last_timestamp_age_ms << "," << comm_health.last_seq << ","
                    << peer_snapshot.received_count << "," << peer_snapshot.seq_gap_count << ","
                    << peer_snapshot.duplicate_count << "," << peer_snapshot.replay_count << ","
                    << peer_snapshot.invalid_count << "," << peer_snapshot.wrong_source_count << ","
                    << peer_snapshot.wrong_destination_count << "," << peer_snapshot.stale_timestamp_count << ","
                    << peer_snapshot.missing_snapshot_count << "," << peer_snapshot.current_burst_loss_count << ","
                    << peer_snapshot.max_burst_loss_count << "," << peer_snapshot.crc_error_count << ","
                    << peer_snapshot.wrong_sender_count << ","
                    << (comm_health.peer_present ? 1 : 0) << "," << (comm_health.snapshot_valid ? 1 : 0) << ","
                    << comm_health.snapshot_rx_aoi_ms << "," << comm_health.last_valid_rx_us << ","
                    << comm_health.last_seq << "," << comm_health.rx_count << ","
                    << comm_health.seq_gap_count << "," << comm_health.missing_snapshot_count << ","
                    << comm_health.current_burst_loss_count << "," << comm_health.max_burst_loss_count << ","
                    << comm_health.invalid_count << "," << comm_health.crc_error_count << ","
                    << comm_health.wrong_sender_count << "," << comm_health.wrong_source_count << ","
                    << comm_health.wrong_destination_count << "," << comm_health.duplicate_count << ","
                    << comm_health.replay_count << "," << comm_health.stale_timestamp_count << ","
                    << (comm_health.peer_health_valid ? 1 : 0) << "," << comm_health.peer_health_rx_aoi_ms << ","
                    << comm_health.peer_reported_aoi_ms << "," << comm_health.peer_reported_network << ","
                    << comm_health.peer_health_seq << ","
                    << (comm_safety_shadow_enabled ? 1 : 0) << ","
                    << (comm_safety_shadow_active ? 1 : 0) << ","
                    << static_cast<uint32_t>(comm_safety_decision.state) << ","
                    << static_cast<uint32_t>(comm_safety_decision.reason) << ","
                    << comm_safety_decision.consecutive_good_snapshots << ","
                    << (comm_safety_decision.changed ? 1 : 0) << ","
                    << (comm_action_shadow_enabled ? 1 : 0) << ","
                    << (comm_action_shadow_active ? 1 : 0) << ","
                    << static_cast<uint32_t>(comm_safety_action.source_state) << ","
                    << static_cast<uint32_t>(comm_safety_action.source_reason) << ","
                    << (comm_safety_action.traction_permitted ? 1 : 0) << ","
                    << (comm_safety_action.cooperative_following_permitted ? 1 : 0) << ","
                    << (comm_safety_action.blind_run_active ? 1 : 0) << ","
                    << (comm_safety_action.fail_safe_brake_requested ? 1 : 0) << ","
                    << comm_safety_action.traction_limit_ratio << ","
                    << comm_safety_action.uncertainty_extra_m << "\n";
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
            if (peer_snapshot.enabled) {
                std::printf("[PEER_SNAPSHOT_RX] Valid:%d | Peer:%u | Src:%s:%u | RxAoI:%6.1fms | TsAge:%6.1fms | Seq:%u | Rx:%u | GapEvents:%u | Missing:%llu | Burst:%u/%u | Dup:%u | Replay:%u | Invalid:%u | CRC:%u | WrongSender:%u | WrongSrc:%u | WrongDest:%u | Stale:%u\n",
                    peer_snapshot.has_valid ? 1 : 0,
                    peer_snapshot.has_valid ? peer_snapshot.latest.header.source_train_id : 0u,
                    peer_snapshot.has_valid ? peer_snapshot.last_src_ip : "-",
                    peer_snapshot.has_valid ? peer_snapshot.last_src_port : 0u,
                    peer_snapshot_rx_aoi_ms(peer_snapshot, runtime_mode),
                    peer_snapshot.last_timestamp_age_ms,
                    peer_snapshot.has_valid ? peer_snapshot.last_seq : 0u,
                    peer_snapshot.received_count,
                    peer_snapshot.seq_gap_count,
                    static_cast<unsigned long long>(peer_snapshot.missing_snapshot_count),
                    peer_snapshot.current_burst_loss_count,
                    peer_snapshot.max_burst_loss_count,
                    peer_snapshot.duplicate_count,
                    peer_snapshot.replay_count,
                    peer_snapshot.invalid_count,
                    peer_snapshot.crc_error_count,
                    peer_snapshot.wrong_sender_count,
                    peer_snapshot.wrong_source_count,
                    peer_snapshot.wrong_destination_count,
                    peer_snapshot.stale_timestamp_count);
            }
            std::printf("[COMM_HEALTH] PeerPresent:%d | SnapshotValid:%d | AoI:%6.1fms | Missing:%llu | Burst:%u/%u | HealthCrossCheck:%d\n",
                comm_health.peer_present ? 1 : 0,
                comm_health.snapshot_valid ? 1 : 0,
                comm_health.snapshot_rx_aoi_ms,
                static_cast<unsigned long long>(comm_health.missing_snapshot_count),
                comm_health.current_burst_loss_count,
                comm_health.max_burst_loss_count,
                comm_health.peer_health_valid ? 1 : 0);
            if (comm_safety_shadow_enabled) {
                std::printf("[COMM_SAFETY] Proposed:%s | Reason:%s | GoodSnapshots:%u | Changed:%d\n",
                    to_string(comm_safety_decision.state),
                    to_string(comm_safety_decision.reason),
                    comm_safety_decision.consecutive_good_snapshots,
                    comm_safety_shadow_active && comm_safety_decision.changed ? 1 : 0);
            }
            if (comm_action_shadow_active) {
                std::printf("[COMM_ACTION] Source:%s | Traction:%d | Ratio:%.2f | Cooperative:%d | Blind:%d | FailBrakeReq:%d | ExtraUncert:%.1fm\n",
                    to_string(comm_safety_action.source_state),
                    comm_safety_action.traction_permitted ? 1 : 0,
                    comm_safety_action.traction_limit_ratio,
                    comm_safety_action.cooperative_following_permitted ? 1 : 0,
                    comm_safety_action.blind_run_active ? 1 : 0,
                    comm_safety_action.fail_safe_brake_requested ? 1 : 0,
                    comm_safety_action.uncertainty_extra_m);
            }
            tick_counter = 0;
        }

        if (simulation_complete) {
            std::cout << "[SD-VCU] Simulation complete signal received. Logs flushed, node exiting cleanly.\n";
            break;
        }

        std::this_thread::sleep_until(loop_start + std::chrono::milliseconds(10));
    }
    if (peer_snapshot.sd >= 0) close(peer_snapshot.sd);
    if (peer_health.sd >= 0) close(peer_health.sd);
    if (t2t_snapshot_sd >= 0) close(t2t_snapshot_sd);
    if (srv_sd >= 0) close(srv_sd);
    return 0;
}
