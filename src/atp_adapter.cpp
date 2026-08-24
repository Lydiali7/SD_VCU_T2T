#include "atp_adapter.hpp"

#include <cstring>

bool ATPAdapter::from_fallback(const FallbackTrainState& state, AtpSnapshot& out) {
    return build_snapshot_from_sim(
        state.train_id,
        state.timestamp_us,
        state.position_m,
        state.velocity_mps,
        state.acceleration_mps2,
        state.status_flag,
        state.seq,
        static_cast<TrainDataSource>(state.source),
        out);
}

bool ATPAdapter::from_mvb(uint32_t train_id, uint64_t timestamp_us,
                          const MVB_Hardware_Frame& mvb, uint32_t seq,
                          AtpSnapshot& out) {
    return build_snapshot_from_mvb(train_id, timestamp_us, mvb, seq, out);
}

bool ATPAdapter::from_can(uint32_t train_id, const CANPacket& can, uint32_t seq,
                          AtpSnapshot& out) {
    return build_snapshot_from_can(train_id, can, seq, out);
}

bool ATPAdapter::from_atp_gateway(const uint8_t* data, size_t len, AtpSnapshot& out) {
    (void)data;
    (void)len;
    std::memset(&out, 0, sizeof(out));
    return false;
}

bool ATPAdapter::validate_snapshot_crc(const AtpSnapshot& snapshot) {
    AtpSnapshot copy = snapshot;
    uint32_t expected = copy.crc32;
    finalize_atp_snapshot_crc(copy);
    return copy.crc32 == expected;
}
