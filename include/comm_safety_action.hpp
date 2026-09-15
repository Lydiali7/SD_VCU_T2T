#pragma once

#include "comm_safety_supervisor.hpp"

// Shadow-policy action contract. No actuator or vehicle controller consumes
// this type in Phase 2C-1.
struct CommSafetyAction {
    CommSafetyState source_state = CommSafetyState::NORMAL;
    CommSafetyReason source_reason = CommSafetyReason::FRESH;
    bool traction_permitted = true;
    bool cooperative_following_permitted = true;
    bool blind_run_active = false;
    bool fail_safe_brake_requested = false;
    float traction_limit_ratio = 1.0f;
    float uncertainty_extra_m = 0.0f;
};

struct CommSafetyActionPolicy {
    // Research/shadow defaults. These values are not safety-certified limits.
    float degraded_traction_limit_ratio = 0.60f;
    float degraded_uncertainty_extra_m = 30.0f;
    float blind_run_traction_limit_ratio = 0.0f;
    float blind_run_uncertainty_extra_m = 100.0f;
    float fail_safe_uncertainty_extra_m = 150.0f;

    bool is_valid() const;
};

class CommSafetyActionResolver {
public:
    explicit CommSafetyActionResolver(CommSafetyActionPolicy policy = {});

    CommSafetyAction resolve(const CommSafetyDecision& decision) const;
    const CommSafetyActionPolicy& policy() const { return policy_; }

private:
    CommSafetyActionPolicy policy_;
};
