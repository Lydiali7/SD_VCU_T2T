#include "comm_safety_action.hpp"

#include <stdexcept>

bool CommSafetyActionPolicy::is_valid() const {
    return degraded_traction_limit_ratio >= 0.0f && degraded_traction_limit_ratio <= 1.0f &&
           blind_run_traction_limit_ratio >= 0.0f && blind_run_traction_limit_ratio <= 1.0f &&
           degraded_uncertainty_extra_m >= 0.0f && blind_run_uncertainty_extra_m >= 0.0f &&
           fail_safe_uncertainty_extra_m >= 0.0f;
}

CommSafetyActionResolver::CommSafetyActionResolver(CommSafetyActionPolicy policy)
    : policy_(policy) {
    if (!policy_.is_valid()) {
        throw std::invalid_argument("invalid communication safety action policy");
    }
}

CommSafetyAction CommSafetyActionResolver::resolve(const CommSafetyDecision& decision) const {
    CommSafetyAction action;
    action.source_state = decision.state;
    action.source_reason = decision.reason;

    switch (decision.state) {
    case CommSafetyState::NORMAL:
        break;
    case CommSafetyState::DEGRADED:
        action.traction_limit_ratio = policy_.degraded_traction_limit_ratio;
        action.uncertainty_extra_m = policy_.degraded_uncertainty_extra_m;
        break;
    case CommSafetyState::BLIND_RUN:
        action.traction_permitted = false;
        action.cooperative_following_permitted = false;
        action.blind_run_active = true;
        action.traction_limit_ratio = policy_.blind_run_traction_limit_ratio;
        action.uncertainty_extra_m = policy_.blind_run_uncertainty_extra_m;
        break;
    case CommSafetyState::FAIL_SAFE_LOCK:
        action.traction_permitted = false;
        action.cooperative_following_permitted = false;
        action.fail_safe_brake_requested = true;
        action.traction_limit_ratio = 0.0f;
        action.uncertainty_extra_m = policy_.fail_safe_uncertainty_extra_m;
        break;
    }
    return action;
}
