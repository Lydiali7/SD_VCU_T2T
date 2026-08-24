#include "network_proto.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

namespace {

volatile std::sig_atomic_t keep_running = 1;

void handle_signal(int) {
    keep_running = 0;
}

uint16_t parse_port(int argc, char* argv[]) {
    if (argc >= 2) {
        int value = std::atoi(argv[1]);
        if (value > 0 && value <= 65535) return static_cast<uint16_t>(value);
    }
    const char* env_port = std::getenv("T2T_SNAPSHOT_LISTEN_PORT");
    if (env_port != nullptr) {
        int value = std::atoi(env_port);
        if (value > 0 && value <= 65535) return static_cast<uint16_t>(value);
    }
    return static_cast<uint16_t>(VCU_PEER_PORT);
}

uint32_t parse_expected_source(int argc, char* argv[]) {
    if (argc >= 3) {
        int value = std::atoi(argv[2]);
        if (value >= 0) return static_cast<uint32_t>(value);
    }
    const char* env_source = std::getenv("T2T_EXPECTED_SOURCE_ID");
    if (env_source != nullptr) {
        int value = std::atoi(env_source);
        if (value >= 0) return static_cast<uint32_t>(value);
    }
    return T2T_BROADCAST_TRAIN_ID;
}

uint64_t steady_now_ms() {
    auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
}

struct PeerSnapshotState {
    bool has_snapshot = false;
    uint32_t expected_source_id = T2T_BROADCAST_TRAIN_ID;
    uint32_t last_seq = 0;
    uint32_t received_count = 0;
    uint32_t seq_gap_count = 0;
    uint32_t invalid_count = 0;
    uint32_t wrong_source_count = 0;
    uint64_t last_rx_ms = 0;
    T2TAtpSnapshotFrame latest = {};
};

void print_snapshot(const T2TAtpSnapshotFrame& frame, const sockaddr_in& src_addr,
                    const PeerSnapshotState& state, uint32_t seq_gap) {
    char src_ip[INET_ADDRSTRLEN] = {0};
    inet_ntop(AF_INET, &src_addr.sin_addr, src_ip, sizeof(src_ip));
    const AtpSnapshot& s = frame.payload.snapshot;
    float peer_aoi_ms = state.has_snapshot
                            ? static_cast<float>(steady_now_ms() - state.last_rx_ms)
                            : 999999.0f;

    std::cout << std::fixed << std::setprecision(2)
              << "[T2T_FRAME] from=" << src_ip << ":" << ntohs(src_addr.sin_port)
              << " src_train=" << frame.header.source_train_id
              << " seq=" << frame.header.seq
              << " snapshot_seq=" << s.seq
              << " ts_us=" << s.timestamp_us
              << " pos_m=" << cm_to_m(s.train_pos_cm)
              << " speed_mps=" << cmps_to_mps(s.speed_cmps)
              << " accel_mps2=" << cmps_to_mps(s.accel_cmps2)
              << " permitted_mps=" << static_cast<float>(s.permitted_speed_cmps) / 100.0f
              << " target_mps=" << static_cast<float>(s.target_speed_cmps) / 100.0f
              << " status=" << s.atp_status
              << " brake_cmd=" << s.brake_command
              << " peer_aoi_ms=" << peer_aoi_ms
              << " seq_gap=" << seq_gap
              << " rx_count=" << state.received_count
              << " gap_count=" << state.seq_gap_count
              << " validity=0x" << std::hex << s.validity_mask
              << " snapshot_crc=0x" << s.crc32
              << " frame_crc=0x" << frame.frame_crc32
              << std::dec << "\n";
}

void print_idle_status(const PeerSnapshotState& state) {
    if (!state.has_snapshot) {
        std::cout << "[PEER_STATE] no valid peer snapshot yet"
                  << " invalid=" << state.invalid_count
                  << " wrong_source=" << state.wrong_source_count << "\n";
        return;
    }
    const AtpSnapshot& s = state.latest.payload.snapshot;
    uint64_t aoi_ms = steady_now_ms() - state.last_rx_ms;
    bool fresh = aoi_ms < 1000ULL;
    std::cout << std::fixed << std::setprecision(2)
              << "[PEER_STATE] fresh=" << (fresh ? 1 : 0)
              << " aoi_ms=" << aoi_ms
              << " src_train=" << state.latest.header.source_train_id
              << " last_seq=" << state.last_seq
              << " pos_m=" << cm_to_m(s.train_pos_cm)
              << " speed_mps=" << cmps_to_mps(s.speed_cmps)
              << " rx_count=" << state.received_count
              << " seq_gap_count=" << state.seq_gap_count
              << " invalid=" << state.invalid_count
              << " wrong_source=" << state.wrong_source_count << "\n";
}

} // namespace

int main(int argc, char* argv[]) {
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    uint16_t port = parse_port(argc, argv);
    uint32_t expected_source = parse_expected_source(argc, argv);
    int sd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sd < 0) {
        std::perror("[DUMP] socket");
        return 1;
    }

    int reuse_addr = 1;
    setsockopt(sd, SOL_SOCKET, SO_REUSEADDR, &reuse_addr, sizeof(reuse_addr));
    timeval timeout = {};
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    setsockopt(sd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);
    if (bind(sd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        std::perror("[DUMP] bind");
        close(sd);
        return 1;
    }

    PeerSnapshotState state;
    state.expected_source_id = expected_source;

    std::cout << "[DUMP] Listening for T2TAtpSnapshotFrame on UDP port " << port;
    if (expected_source != T2T_BROADCAST_TRAIN_ID) {
        std::cout << " expected_source=" << expected_source;
    }
    std::cout << "\n";

    while (keep_running) {
        T2TAtpSnapshotFrame frame = {};
        sockaddr_in src_addr = {};
        socklen_t src_len = sizeof(src_addr);
        ssize_t n = recvfrom(sd, &frame, sizeof(frame), 0,
                             reinterpret_cast<sockaddr*>(&src_addr), &src_len);
        if (n < 0) {
            if (errno == EINTR) continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                print_idle_status(state);
                continue;
            }
            std::perror("[DUMP] recvfrom");
            break;
        }
        if (n != static_cast<ssize_t>(sizeof(frame))) {
            state.invalid_count++;
            std::cerr << "[DUMP] Ignored packet with unexpected size: " << n
                      << " bytes, expected " << sizeof(frame) << "\n";
            continue;
        }
        if (!validate_t2t_atp_snapshot_frame(frame)) {
            state.invalid_count++;
            std::cerr << "[DUMP] Invalid T2TAtpSnapshotFrame: header or CRC check failed\n";
            continue;
        }
        if (state.expected_source_id != T2T_BROADCAST_TRAIN_ID &&
            frame.header.source_train_id != state.expected_source_id) {
            state.wrong_source_count++;
            std::cerr << "[DUMP] Ignored source_train_id=" << frame.header.source_train_id
                      << ", expected " << state.expected_source_id << "\n";
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
        state.received_count++;
        print_snapshot(frame, src_addr, state, seq_gap);
    }

    close(sd);
    std::cout << "[DUMP] stopped\n";
    return 0;
}
