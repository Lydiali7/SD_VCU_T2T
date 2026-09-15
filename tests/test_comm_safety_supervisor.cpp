#include "comm_safety_supervisor.hpp"

#include <iostream>

namespace {

bool expect_true(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "[FAIL] " << message << "\n";
        return false;
    }
    std::cout << "[PASS] " << message << "\n";
    return true;
}

CommHealthState fresh_snapshot(uint32_t rx_count, float aoi_ms = 20.0f) {
    CommHealthState health;
    health.peer_present = true;
    health.snapshot_valid = true;
    health.rx_count = rx_count;
    health.snapshot_rx_aoi_ms = aoi_ms;
    return health;
}

} // namespace

int main() {
    bool ok = true;
    CommSafetyThresholds thresholds;
    thresholds.degraded_ms = 300.0f;
    thresholds.blind_run_ms = 1000.0f;
    thresholds.fail_safe_ms = 5000.0f;
    thresholds.recover_normal_ms = 150.0f;
    thresholds.recover_degraded_ms = 500.0f;
    thresholds.recovery_good_snapshots = 3;
    CommSafetySupervisor supervisor(thresholds);

    ok &= expect_true(supervisor.update(fresh_snapshot(1)).state == CommSafetyState::NORMAL,
                      "fresh snapshot remains normal");

    CommSafetySupervisor boundary_supervisor(thresholds);
    ok &= expect_true(boundary_supervisor.update(fresh_snapshot(1, 300.0f)).state == CommSafetyState::NORMAL,
                      "AoI at degraded boundary remains normal");
    ok &= expect_true(boundary_supervisor.update(fresh_snapshot(1, 301.0f)).state == CommSafetyState::DEGRADED,
                      "AoI above degraded boundary enters degraded");
    ok &= expect_true(boundary_supervisor.update(fresh_snapshot(1, 1000.0f)).state == CommSafetyState::DEGRADED,
                      "AoI at blind boundary remains degraded");
    ok &= expect_true(boundary_supervisor.update(fresh_snapshot(1, 1001.0f)).state == CommSafetyState::BLIND_RUN,
                      "AoI above blind boundary enters blind run");
    ok &= expect_true(boundary_supervisor.update(fresh_snapshot(1, 5000.0f)).state == CommSafetyState::BLIND_RUN,
                      "AoI at fail-safe boundary remains blind run");
    ok &= expect_true(boundary_supervisor.update(fresh_snapshot(1, 5001.0f)).state == CommSafetyState::FAIL_SAFE_LOCK,
                      "AoI above fail-safe boundary enters lock");

    CommHealthState degraded = fresh_snapshot(1, 301.0f);
    ok &= expect_true(supervisor.update(degraded).state == CommSafetyState::DEGRADED,
                      "AoI above degraded threshold enters degraded");
    ok &= expect_true(supervisor.update(fresh_snapshot(2)).state == CommSafetyState::DEGRADED,
                      "one fresh snapshot does not recover degraded");
    ok &= expect_true(supervisor.update(fresh_snapshot(3)).state == CommSafetyState::DEGRADED,
                      "two fresh snapshots do not recover degraded");
    ok &= expect_true(supervisor.update(fresh_snapshot(4)).state == CommSafetyState::NORMAL,
                      "three fresh snapshots recover normal");

    CommHealthState blind = fresh_snapshot(4, 1001.0f);
    ok &= expect_true(supervisor.update(blind).state == CommSafetyState::BLIND_RUN,
                      "AoI above blind threshold enters blind run");
    ok &= expect_true(supervisor.update(fresh_snapshot(5, 400.0f)).state == CommSafetyState::BLIND_RUN,
                      "blind run holds before recovery count is met");
    ok &= expect_true(supervisor.update(fresh_snapshot(6, 400.0f)).state == CommSafetyState::BLIND_RUN,
                      "blind run recovery requires consecutive snapshots");
    ok &= expect_true(supervisor.update(fresh_snapshot(7, 400.0f)).state == CommSafetyState::DEGRADED,
                      "blind run recovers to degraded after good snapshots");

    CommHealthState failed = fresh_snapshot(7, 5001.0f);
    ok &= expect_true(supervisor.update(failed).state == CommSafetyState::FAIL_SAFE_LOCK,
                      "AoI above fail-safe threshold enters lock");
    ok &= expect_true(supervisor.update(fresh_snapshot(100)).state == CommSafetyState::FAIL_SAFE_LOCK,
                      "fail-safe lock does not auto-recover");

    if (!ok) {
        std::cerr << "[RESULT] communication safety supervisor tests failed\n";
        return 1;
    }
    std::cout << "[RESULT] communication safety supervisor tests passed\n";
    return 0;
}
