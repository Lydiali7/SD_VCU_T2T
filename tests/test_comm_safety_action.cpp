#include "comm_safety_action.hpp"

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

CommSafetyDecision decision(CommSafetyState state) {
    CommSafetyDecision value;
    value.state = state;
    return value;
}

} // namespace

int main() {
    bool ok = true;
    CommSafetyActionResolver resolver;

    CommSafetyAction normal = resolver.resolve(decision(CommSafetyState::NORMAL));
    ok &= expect_true(normal.traction_permitted && normal.cooperative_following_permitted &&
                          !normal.blind_run_active && !normal.fail_safe_brake_requested &&
                          normal.traction_limit_ratio == 1.0f && normal.uncertainty_extra_m == 0.0f,
                      "normal action permits nominal cooperative operation");

    CommSafetyAction degraded = resolver.resolve(decision(CommSafetyState::DEGRADED));
    ok &= expect_true(degraded.traction_permitted && degraded.cooperative_following_permitted &&
                          degraded.traction_limit_ratio < 1.0f && degraded.uncertainty_extra_m > 0.0f,
                      "degraded action limits traction and expands uncertainty");

    CommSafetyAction blind = resolver.resolve(decision(CommSafetyState::BLIND_RUN));
    ok &= expect_true(!blind.traction_permitted && !blind.cooperative_following_permitted &&
                          blind.blind_run_active && !blind.fail_safe_brake_requested &&
                          blind.traction_limit_ratio == 0.0f && blind.uncertainty_extra_m > degraded.uncertainty_extra_m,
                      "blind-run action inhibits traction and cooperative following");

    CommSafetyAction failed = resolver.resolve(decision(CommSafetyState::FAIL_SAFE_LOCK));
    ok &= expect_true(!failed.traction_permitted && !failed.cooperative_following_permitted &&
                          !failed.blind_run_active && failed.fail_safe_brake_requested &&
                          failed.traction_limit_ratio == 0.0f && failed.uncertainty_extra_m >= blind.uncertainty_extra_m,
                      "fail-safe action requests braking and remains restrictive");

    if (!ok) {
        std::cerr << "[RESULT] communication safety action tests failed\n";
        return 1;
    }
    std::cout << "[RESULT] communication safety action tests passed\n";
    return 0;
}
