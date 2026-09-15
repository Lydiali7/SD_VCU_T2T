#include "comm_safety_supervisor.hpp"

#include <stdexcept>

bool CommSafetyThresholds::is_valid() const {
    return degraded_ms > 0.0f && degraded_ms < blind_run_ms && blind_run_ms < fail_safe_ms &&
           recover_normal_ms >= 0.0f && recover_normal_ms < degraded_ms &&
           recover_degraded_ms >= 0.0f && recover_degraded_ms < blind_run_ms &&
           recovery_good_snapshots > 0;
}

CommSafetySupervisor::CommSafetySupervisor(CommSafetyThresholds thresholds)
    : thresholds_(thresholds) {
    if (!thresholds_.is_valid()) {
        throw std::invalid_argument("invalid communication safety thresholds");
    }
}

void CommSafetySupervisor::reset() {
    state_ = CommSafetyState::NORMAL;
    last_observed_rx_count_ = 0;
    consecutive_good_snapshots_ = 0;
}

bool CommSafetySupervisor::is_new_valid_snapshot(const CommHealthState& health) {
    bool new_snapshot = health.snapshot_valid && health.rx_count > last_observed_rx_count_;
    if (health.rx_count > last_observed_rx_count_) {
        last_observed_rx_count_ = health.rx_count;
    }
    return new_snapshot;
}

void CommSafetySupervisor::reset_recovery_progress() {
    consecutive_good_snapshots_ = 0;
}

CommSafetyDecision CommSafetySupervisor::update(const CommHealthState& health) {
    CommSafetyDecision result;
    result.state = state_;

    if (state_ == CommSafetyState::FAIL_SAFE_LOCK) {
        result.reason = CommSafetyReason::FAIL_SAFE_LATCHED;
        result.consecutive_good_snapshots = consecutive_good_snapshots_;
        return result;
    }

    const float aoi_ms = health.snapshot_rx_aoi_ms;
    if (aoi_ms > thresholds_.fail_safe_ms) {
        state_ = CommSafetyState::FAIL_SAFE_LOCK;
        reset_recovery_progress();
        result.state = state_;
        result.reason = CommSafetyReason::SNAPSHOT_AOI_FAIL_SAFE;
        result.changed = true;
        return result;
    }
    if (state_ == CommSafetyState::BLIND_RUN) {
        // Recovery from blind run is owned by its stricter hysteresis rule;
        // it must not be bypassed by the normal degraded-entry threshold.
        if (aoi_ms > thresholds_.blind_run_ms) {
            reset_recovery_progress();
            result.reason = CommSafetyReason::SNAPSHOT_AOI_BLIND_RUN;
            result.consecutive_good_snapshots = consecutive_good_snapshots_;
            return result;
        }
        bool new_snapshot = is_new_valid_snapshot(health);
        if (aoi_ms <= thresholds_.recover_degraded_ms && new_snapshot) {
            ++consecutive_good_snapshots_;
        } else if (aoi_ms > thresholds_.recover_degraded_ms) {
            reset_recovery_progress();
        }
        if (consecutive_good_snapshots_ >= thresholds_.recovery_good_snapshots) {
            state_ = CommSafetyState::DEGRADED;
            reset_recovery_progress();
            result.changed = true;
            result.reason = CommSafetyReason::RECOVERY_TO_DEGRADED;
        } else {
            result.reason = CommSafetyReason::SNAPSHOT_AOI_BLIND_RUN;
        }
        result.state = state_;
        result.consecutive_good_snapshots = consecutive_good_snapshots_;
        return result;
    }

    if (aoi_ms > thresholds_.blind_run_ms) {
        state_ = CommSafetyState::BLIND_RUN;
        reset_recovery_progress();
        result.state = state_;
        result.reason = CommSafetyReason::SNAPSHOT_AOI_BLIND_RUN;
        result.changed = true;
        return result;
    }
    if (aoi_ms > thresholds_.degraded_ms) {
        if (state_ != CommSafetyState::DEGRADED) {
            state_ = CommSafetyState::DEGRADED;
            result.changed = true;
        }
        reset_recovery_progress();
        result.state = state_;
        result.reason = CommSafetyReason::SNAPSHOT_AOI_DEGRADED;
        return result;
    }

    bool new_snapshot = is_new_valid_snapshot(health);
    if (state_ == CommSafetyState::DEGRADED) {
        if (aoi_ms <= thresholds_.recover_normal_ms && new_snapshot) {
            ++consecutive_good_snapshots_;
        } else if (aoi_ms > thresholds_.recover_normal_ms) {
            reset_recovery_progress();
        }
        if (consecutive_good_snapshots_ >= thresholds_.recovery_good_snapshots) {
            state_ = CommSafetyState::NORMAL;
            reset_recovery_progress();
            result.changed = true;
            result.reason = CommSafetyReason::RECOVERY_TO_NORMAL;
        } else {
            result.reason = CommSafetyReason::SNAPSHOT_AOI_DEGRADED;
        }
    } else {
        reset_recovery_progress();
        result.reason = CommSafetyReason::FRESH;
    }

    result.state = state_;
    result.consecutive_good_snapshots = consecutive_good_snapshots_;
    return result;
}

const char* to_string(CommSafetyState state) {
    switch (state) {
    case CommSafetyState::NORMAL: return "NORMAL";
    case CommSafetyState::DEGRADED: return "DEGRADED";
    case CommSafetyState::BLIND_RUN: return "BLIND_RUN";
    case CommSafetyState::FAIL_SAFE_LOCK: return "FAIL_SAFE_LOCK";
    }
    return "UNKNOWN";
}

const char* to_string(CommSafetyReason reason) {
    switch (reason) {
    case CommSafetyReason::FRESH: return "FRESH";
    case CommSafetyReason::SNAPSHOT_AOI_DEGRADED: return "SNAPSHOT_AOI_DEGRADED";
    case CommSafetyReason::SNAPSHOT_AOI_BLIND_RUN: return "SNAPSHOT_AOI_BLIND_RUN";
    case CommSafetyReason::SNAPSHOT_AOI_FAIL_SAFE: return "SNAPSHOT_AOI_FAIL_SAFE";
    case CommSafetyReason::RECOVERY_TO_DEGRADED: return "RECOVERY_TO_DEGRADED";
    case CommSafetyReason::RECOVERY_TO_NORMAL: return "RECOVERY_TO_NORMAL";
    case CommSafetyReason::FAIL_SAFE_LATCHED: return "FAIL_SAFE_LATCHED";
    }
    return "UNKNOWN";
}
