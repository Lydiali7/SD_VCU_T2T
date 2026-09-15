#pragma once

#include <cstdint>

#include "comm_health.hpp"

enum class CommSafetyState : uint8_t {
    NORMAL = 0,
    DEGRADED,
    BLIND_RUN,
    FAIL_SAFE_LOCK
};

enum class CommSafetyReason : uint8_t {
    FRESH = 0,
    SNAPSHOT_AOI_DEGRADED,
    SNAPSHOT_AOI_BLIND_RUN,
    SNAPSHOT_AOI_FAIL_SAFE,
    RECOVERY_TO_DEGRADED,
    RECOVERY_TO_NORMAL,
    FAIL_SAFE_LATCHED
};

struct CommSafetyThresholds {
    // Research/shadow defaults. They are not certified railway safety values.
    float degraded_ms = 300.0f;
    float blind_run_ms = 1000.0f;
    float fail_safe_ms = 5000.0f;
    float recover_normal_ms = 150.0f;
    float recover_degraded_ms = 500.0f;
    uint32_t recovery_good_snapshots = 3;

    bool is_valid() const;
};

struct CommSafetyDecision {
    CommSafetyState state = CommSafetyState::NORMAL;
    CommSafetyReason reason = CommSafetyReason::FRESH;
    uint32_t consecutive_good_snapshots = 0;
    bool changed = false;
};

class CommSafetySupervisor {
public:
    explicit CommSafetySupervisor(CommSafetyThresholds thresholds = {});

    CommSafetyDecision update(const CommHealthState& health);
    void reset();

    CommSafetyState state() const { return state_; }
    const CommSafetyThresholds& thresholds() const { return thresholds_; }

private:
    bool is_new_valid_snapshot(const CommHealthState& health);
    void reset_recovery_progress();

    CommSafetyThresholds thresholds_;
    CommSafetyState state_ = CommSafetyState::NORMAL;
    uint32_t last_observed_rx_count_ = 0;
    uint32_t consecutive_good_snapshots_ = 0;
};

const char* to_string(CommSafetyState state);
const char* to_string(CommSafetyReason reason);
