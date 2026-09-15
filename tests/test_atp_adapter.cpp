#include "atp_adapter.hpp"

#include <cstdint>
#include <iostream>

namespace {

bool expect_true(bool cond, const char* message) {
    if (!cond) {
        std::cerr << "[FAIL] " << message << "\n";
        return false;
    }
    std::cout << "[PASS] " << message << "\n";
    return true;
}

} // namespace

int main() {
    bool ok = true;

    FallbackTrainState fallback = {
        3u,
        123456789ULL,
        1234.56f,
        24.5f,
        0.12f,
        STATUS_DEGRADED_UU,
        42u,
        ATPAdapterSource::SIM_FALLBACK
    };

    AtpSnapshot snapshot = {};
    ok &= expect_true(ATPAdapter::from_fallback(fallback, snapshot), "fallback snapshot builds");
    ok &= expect_true(snapshot.timestamp_us == 123456789ULL, "timestamp copied");
    ok &= expect_true(snapshot.source_id == static_cast<uint32_t>(ATPAdapterSource::SIM_FALLBACK), "source id set");
    ok &= expect_true((snapshot.validity_mask & SNAPSHOT_VALID_POS) != 0u, "position validity set");
    ok &= expect_true((snapshot.validity_mask & SNAPSHOT_VALID_SPEED) != 0u, "speed validity set");
    ok &= expect_true((snapshot.validity_mask & SNAPSHOT_VALID_ACCEL) != 0u, "accel validity set");
    ok &= expect_true(snapshot.train_pos_cm == 123456, "position fixed-point cm conversion");
    ok &= expect_true(snapshot.speed_cmps == 2450, "speed fixed-point cm/s conversion");
    ok &= expect_true(snapshot.accel_cmps2 == 12, "accel fixed-point cm/s^2 conversion");
    ok &= expect_true(snapshot.atp_status == STATUS_DEGRADED_UU, "status copied");
    ok &= expect_true(snapshot.seq == 42u, "sequence copied");
    ok &= expect_true(ATPAdapter::validate_snapshot_crc(snapshot), "snapshot crc validates");

    AtpSnapshot corrupted = snapshot;
    corrupted.speed_cmps += 1;
    ok &= expect_true(!ATPAdapter::validate_snapshot_crc(corrupted), "snapshot crc detects corruption");

    T2TAtpSnapshotFrame frame = {};
    ok &= expect_true(build_t2t_atp_snapshot_frame(
                          3u,
                          T2T_BROADCAST_TRAIN_ID,
                          1u,
                          7u,
                          0u,
                          snapshot,
                          frame),
                      "T2T ATP snapshot frame builds");
    ok &= expect_true(validate_t2t_atp_snapshot_frame(frame), "T2T frame crc validates");

    T2TAtpSnapshotFrame corrupted_frame = frame;
    corrupted_frame.payload.snapshot.train_pos_cm += 1;
    ok &= expect_true(!validate_t2t_atp_snapshot_frame(corrupted_frame), "T2T frame crc detects corruption");
    ok &= expect_true(
        validate_t2t_atp_snapshot_frame_detailed(corrupted_frame) ==
            T2TFrameValidationResult::SNAPSHOT_CRC_ERROR,
        "T2T frame reports snapshot CRC error");

    if (!ok) {
        std::cerr << "[RESULT] ATP adapter tests failed.\n";
        return 1;
    }
    std::cout << "[RESULT] ATP adapter tests passed.\n";
    return 0;
}
