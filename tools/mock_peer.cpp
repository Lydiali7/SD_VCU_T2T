#include "network_proto.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <sys/socket.h>
#include <sys/time.h>
#include <thread>
#include <unistd.h>

namespace {

volatile std::sig_atomic_t keep_running = 1;

void handle_signal(int) {
    keep_running = 0;
}

uint64_t steady_now_ms() {
    auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
}

uint64_t wall_now_us() {
    auto now = std::chrono::system_clock::now().time_since_epoch();
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(now).count());
}

int get_env_int(const char* name, int default_value) {
    const char* value = std::getenv(name);
    if (value == nullptr) return default_value;
    return std::atoi(value);
}

double get_env_double(const char* name, double default_value) {
    const char* value = std::getenv(name);
    if (value == nullptr) return default_value;
    return std::atof(value);
}

std::string get_env_string(const char* name, const std::string& default_value) {
    const char* value = std::getenv(name);
    if (value == nullptr || value[0] == '\0') return default_value;
    return std::string(value);
}

uint16_t parse_listen_port(int argc, char* argv[]) {
    if (argc >= 2) {
        int value = std::atoi(argv[1]);
        if (value > 0 && value <= 65535) return static_cast<uint16_t>(value);
    }
    int env_value = get_env_int("MOCK_PEER_LISTEN_PORT", VCU_PEER_PORT);
    if (env_value > 0 && env_value <= 65535) return static_cast<uint16_t>(env_value);
    return static_cast<uint16_t>(VCU_PEER_PORT);
}

uint32_t parse_expected_source(int argc, char* argv[]) {
    if (argc >= 3) {
        int value = std::atoi(argv[2]);
        if (value >= 0) return static_cast<uint32_t>(value);
    }
    int env_value = get_env_int("MOCK_PEER_EXPECTED_SOURCE_ID", -1);
    if (env_value >= 0) return static_cast<uint32_t>(env_value);
    return T2T_BROADCAST_TRAIN_ID;
}

struct MockConfig {
    uint16_t listen_port = VCU_PEER_PORT;
    uint32_t expected_source_id = T2T_BROADCAST_TRAIN_ID;
    uint32_t mock_peer_id = 2;
    std::string health_target_ip;
    bool health_auto_target = true;
    uint16_t health_target_port = static_cast<uint16_t>(VCU_PEER_PORT + 1);
    uint32_t health_period_ms = 1000;
    uint32_t fresh_timeout_ms = 1000;
    double rx_drop_prob = 0.0;
    uint32_t rx_delay_base_ms = 0;
    uint32_t rx_delay_jitter_ms = 0;
};

MockConfig parse_config(int argc, char* argv[]) {
    MockConfig cfg;
    cfg.listen_port = parse_listen_port(argc, argv);
    cfg.expected_source_id = parse_expected_source(argc, argv);
    cfg.mock_peer_id = static_cast<uint32_t>(get_env_int("MOCK_PEER_ID", 2));
    cfg.health_target_ip = (argc >= 4) ? std::string(argv[3]) : get_env_string("MOCK_PEER_HEALTH_IP", "");
    cfg.health_auto_target = get_env_int("MOCK_PEER_HEALTH_AUTO_TARGET", 1) != 0;
    int health_port = get_env_int("MOCK_PEER_HEALTH_PORT", VCU_PEER_PORT + 1);
    if (health_port > 0 && health_port <= 65535) {
        cfg.health_target_port = static_cast<uint16_t>(health_port);
    }
    cfg.health_period_ms = static_cast<uint32_t>(std::max(50, get_env_int("MOCK_PEER_HEALTH_PERIOD_MS", 1000)));
    cfg.fresh_timeout_ms = static_cast<uint32_t>(std::max(100, get_env_int("MOCK_PEER_FRESH_TIMEOUT_MS", 1000)));
    cfg.rx_drop_prob = std::clamp(get_env_double("MOCK_PEER_RX_DROP_PROB", 0.0), 0.0, 1.0);
    cfg.rx_delay_base_ms = static_cast<uint32_t>(std::max(0, get_env_int("MOCK_PEER_RX_DELAY_BASE_MS", 0)));
    cfg.rx_delay_jitter_ms = static_cast<uint32_t>(std::max(0, get_env_int("MOCK_PEER_RX_DELAY_JITTER_MS", 0)));
    return cfg;
}

struct PeerSnapshotState {
    bool has_snapshot = false;
    uint32_t expected_source_id = T2T_BROADCAST_TRAIN_ID;
    uint32_t last_seq = 0;
    uint32_t received_count = 0;
    uint32_t seq_gap_count = 0;
    uint32_t invalid_count = 0;
    uint32_t wrong_source_count = 0;
    uint32_t injected_drop_count = 0;
    uint32_t health_send_count = 0;
    uint32_t health_send_error_count = 0;
    uint64_t last_rx_ms = 0;
    T2TAtpSnapshotFrame latest = {};
    sockaddr_in last_snapshot_src = {};
};

float peer_aoi_ms(const PeerSnapshotState& state) {
    if (!state.has_snapshot) return 999999.0f;
    return static_cast<float>(steady_now_ms() - state.last_rx_ms);
}

bool is_peer_fresh(const PeerSnapshotState& state, const MockConfig& cfg) {
    return state.has_snapshot && (steady_now_ms() - state.last_rx_ms) <= cfg.fresh_timeout_ms;
}

uint32_t network_health_code(const PeerSnapshotState& state, const MockConfig& cfg) {
    if (!state.has_snapshot) return STATUS_DEGRADED_UU;
    if (!is_peer_fresh(state, cfg)) return STATUS_BLIND_RUN;
    return 0u;
}

bool configure_health_target(int sd, const MockConfig& cfg, sockaddr_in& target) {
    if (cfg.health_target_ip.empty()) return false;
    target = {};
    target.sin_family = AF_INET;
    target.sin_port = htons(cfg.health_target_port);
    if (inet_pton(AF_INET, cfg.health_target_ip.c_str(), &target.sin_addr) != 1) {
        std::cerr << "[MOCK_PEER] Invalid MOCK_PEER_HEALTH_IP: " << cfg.health_target_ip << "\n";
        return false;
    }
    (void)sd;
    return true;
}

std::string endpoint_to_string(const sockaddr_in& addr) {
    char ip[INET_ADDRSTRLEN] = {0};
    inet_ntop(AF_INET, &addr.sin_addr, ip, sizeof(ip));
    return std::string(ip) + ":" + std::to_string(ntohs(addr.sin_port));
}

void maybe_update_auto_health_target(sockaddr_in& target, bool& has_target,
                                     const PeerSnapshotState& state, const MockConfig& cfg) {
    if (!cfg.health_auto_target || !cfg.health_target_ip.empty() || !state.has_snapshot) return;

    target = {};
    target.sin_family = AF_INET;
    target.sin_addr = state.last_snapshot_src.sin_addr;
    target.sin_port = htons(cfg.health_target_port);
    has_target = true;
}

void maybe_send_health(int sd, sockaddr_in& target, bool& has_target,
                       PeerSnapshotState& state, const MockConfig& cfg,
                       uint32_t& health_seq, uint64_t& last_health_ms) {
    uint64_t now_ms = steady_now_ms();
    if (now_ms - last_health_ms < cfg.health_period_ms) return;
    last_health_ms = now_ms;
    maybe_update_auto_health_target(target, has_target, state, cfg);

    PeerHealthPacket health = {};
    health.header = PEER_HEALTH_HEADER;
    health.sender_id = cfg.mock_peer_id;
    health.seq = ++health_seq;
    health.tx_timestamp_us = wall_now_us();
    health.peer_aoi_ms = peer_aoi_ms(state);
    health.control_aoi_ms = health.peer_aoi_ms;
    health.burst_loss_count = state.seq_gap_count;
    health.last_time_diff_us = 0;
    health.network_health = network_health_code(state, cfg);
    health.health_flags = is_peer_fresh(state, cfg) ? PEER_HEALTH_FLAG_PEER_FRESH : 0u;

    ssize_t sent = 0;
    if (has_target) {
        sent = sendto(sd, &health, sizeof(health), 0,
                      reinterpret_cast<const sockaddr*>(&target), sizeof(target));
        if (sent != static_cast<ssize_t>(sizeof(health))) {
            state.health_send_error_count++;
            std::perror("[MOCK_PEER] sendto health");
        } else {
            state.health_send_count++;
        }
    }

    std::cout << std::fixed << std::setprecision(2)
              << "[PEER_HEALTH] seq=" << health.seq
              << " target=" << (has_target ? endpoint_to_string(target) : "disabled")
              << " bytes=" << sent
              << " peer_fresh=" << ((health.health_flags & PEER_HEALTH_FLAG_PEER_FRESH) ? 1 : 0)
              << " peer_aoi_ms=" << health.peer_aoi_ms
              << " network_health=" << health.network_health
              << " rx_count=" << state.received_count
              << " seq_gap_count=" << state.seq_gap_count
              << " invalid=" << state.invalid_count
              << " injected_drop=" << state.injected_drop_count
              << " health_sent=" << state.health_send_count
              << " health_send_err=" << state.health_send_error_count << "\n";
}

void print_snapshot(const T2TAtpSnapshotFrame& frame, const sockaddr_in& src_addr,
                    const PeerSnapshotState& state, uint32_t seq_gap) {
    char src_ip[INET_ADDRSTRLEN] = {0};
    inet_ntop(AF_INET, &src_addr.sin_addr, src_ip, sizeof(src_ip));
    const AtpSnapshot& s = frame.payload.snapshot;

    std::cout << std::fixed << std::setprecision(2)
              << "[MOCK_PEER_RX] from=" << src_ip << ":" << ntohs(src_addr.sin_port)
              << " src_train=" << frame.header.source_train_id
              << " seq=" << frame.header.seq
              << " snapshot_seq=" << s.seq
              << " pos_m=" << cm_to_m(s.train_pos_cm)
              << " speed_mps=" << cmps_to_mps(s.speed_cmps)
              << " accel_mps2=" << cmps_to_mps(s.accel_cmps2)
              << " status=" << s.atp_status
              << " peer_aoi_ms=" << peer_aoi_ms(state)
              << " seq_gap=" << seq_gap
              << " rx_count=" << state.received_count
              << " gap_count=" << state.seq_gap_count << "\n";
}

void apply_injected_delay(const MockConfig& cfg, std::mt19937& rng) {
    uint32_t delay_ms = cfg.rx_delay_base_ms;
    if (cfg.rx_delay_jitter_ms > 0) {
        std::uniform_int_distribution<uint32_t> jitter_dist(0, cfg.rx_delay_jitter_ms);
        delay_ms += jitter_dist(rng);
    }
    if (delay_ms > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    }
}

} // namespace

int main(int argc, char* argv[]) {
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    MockConfig cfg = parse_config(argc, argv);
    int sd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sd < 0) {
        std::perror("[MOCK_PEER] socket");
        return 1;
    }

    int reuse_addr = 1;
    setsockopt(sd, SOL_SOCKET, SO_REUSEADDR, &reuse_addr, sizeof(reuse_addr));
    timeval timeout = {};
    timeout.tv_sec = 0;
    timeout.tv_usec = 100000;
    setsockopt(sd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    sockaddr_in bind_addr = {};
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_addr.s_addr = INADDR_ANY;
    bind_addr.sin_port = htons(cfg.listen_port);
    if (bind(sd, reinterpret_cast<sockaddr*>(&bind_addr), sizeof(bind_addr)) != 0) {
        std::perror("[MOCK_PEER] bind");
        close(sd);
        return 1;
    }

    sockaddr_in health_target = {};
    bool has_health_target = configure_health_target(sd, cfg, health_target);

    PeerSnapshotState state;
    state.expected_source_id = cfg.expected_source_id;
    uint32_t health_seq = 0;
    uint64_t last_health_ms = 0;
    std::mt19937 rng{std::random_device{}()};
    std::uniform_real_distribution<double> probability(0.0, 1.0);

    std::cout << "[MOCK_PEER] listening_port=" << cfg.listen_port
              << " expected_source="
              << (cfg.expected_source_id == T2T_BROADCAST_TRAIN_ID ? -1 : static_cast<int>(cfg.expected_source_id))
              << " mock_peer_id=" << cfg.mock_peer_id
              << " health_target="
              << (has_health_target ? cfg.health_target_ip + ":" + std::to_string(cfg.health_target_port) : "disabled")
              << " health_auto_target=" << (cfg.health_auto_target ? 1 : 0)
              << " health_port=" << cfg.health_target_port
              << " rx_drop_prob=" << cfg.rx_drop_prob
              << " rx_delay_base_ms=" << cfg.rx_delay_base_ms
              << " rx_delay_jitter_ms=" << cfg.rx_delay_jitter_ms << "\n";

    while (keep_running) {
        maybe_send_health(sd, health_target, has_health_target, state, cfg, health_seq, last_health_ms);

        T2TAtpSnapshotFrame frame = {};
        sockaddr_in src_addr = {};
        socklen_t src_len = sizeof(src_addr);
        ssize_t n = recvfrom(sd, &frame, sizeof(frame), 0,
                             reinterpret_cast<sockaddr*>(&src_addr), &src_len);
        if (n < 0) {
            if (errno == EINTR) continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
            std::perror("[MOCK_PEER] recvfrom");
            break;
        }

        if (cfg.rx_drop_prob > 0.0 && probability(rng) < cfg.rx_drop_prob) {
            state.injected_drop_count++;
            continue;
        }
        apply_injected_delay(cfg, rng);

        if (n != static_cast<ssize_t>(sizeof(frame))) {
            state.invalid_count++;
            std::cerr << "[MOCK_PEER] ignored size=" << n
                      << " expected=" << sizeof(frame) << "\n";
            continue;
        }
        if (!validate_t2t_atp_snapshot_frame(frame)) {
            state.invalid_count++;
            std::cerr << "[MOCK_PEER] invalid T2TAtpSnapshotFrame\n";
            continue;
        }
        if (state.expected_source_id != T2T_BROADCAST_TRAIN_ID &&
            frame.header.source_train_id != state.expected_source_id) {
            state.wrong_source_count++;
            std::cerr << "[MOCK_PEER] ignored source_train_id=" << frame.header.source_train_id
                      << " expected=" << state.expected_source_id << "\n";
            continue;
        }

        uint32_t seq_gap = 0;
        if (state.has_snapshot) {
            uint32_t expected_next = state.last_seq + 1u;
            if (frame.header.seq != expected_next) {
                seq_gap = frame.header.seq - expected_next;
                state.seq_gap_count++;
            }
        }

        state.has_snapshot = true;
        state.last_seq = frame.header.seq;
        state.last_rx_ms = steady_now_ms();
        state.latest = frame;
        state.last_snapshot_src = src_addr;
        state.received_count++;
        print_snapshot(frame, src_addr, state, seq_gap);
    }

    close(sd);
    std::cout << "[MOCK_PEER] stopped\n";
    return 0;
}
