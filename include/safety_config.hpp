#pragma once
#include <cstdint>

namespace VCU::SafetyConfig {

// Simulation defaults. These constants are centralized so each value
// can be traced, swept, and replaced by project-specific safety evidence.
constexpr uint64_t kWatchdogTimeoutUs = 50000ULL;
constexpr float kStartupGraceMs = 8000.0f;
constexpr float kSpeedGuardMps = 26.5f;
constexpr float kMaxUncertaintyBufferM = 150.0f;
constexpr float kDecouplingTargetMps = 14.0f;
constexpr float kDecouplingSpeedGain = 60000.0f;
constexpr float kCruiseSpeedGain = 50000.0f;
constexpr float kLeaderMinTrackingGapM = 850.0f;
constexpr float kLeaderSafeGapMarginM = 100.0f;
constexpr float kFollowerSafeGapMarginM = 80.0f;
constexpr float kFollowVelocityDeadbandMps = 1.5f;
constexpr float kFollowPositionGain = 8000.0f;
constexpr float kFollowVelocityGain = 55000.0f;
constexpr float kFollowControlRangeM = 2500.0f;
constexpr float kBlindEntryAoiMs = 2000.0f;
constexpr float kUnexpectedAoiFailSafeMs = 5000.0f;
constexpr float kBlindEffectiveDelayS = 2.0f;
constexpr float kBlindDelayGrowthPerSecond = 0.06f;
constexpr float kMaxCertifiedBlindDelayS = 8.0f;
constexpr float kBlindUncertaintyByTimeMps = 0.75f;
constexpr float kBlindUncertaintyByDistanceRatio = 0.03f;
constexpr float kBlindCautionAoiS = 30.0f;
constexpr float kBlindRestrictedAoiS = 60.0f;
constexpr float kBlindCautionTargetMps = 22.0f;
constexpr float kBlindRestrictedTargetMps = 18.0f;
constexpr float kBlindControlledBrakeMarginM = 80.0f;
constexpr float kBlindControlledBrakeDecelMps2 = 0.15f;
constexpr float kFailSafeBrakeRedundancy = 1.5f;

} // namespace VCU::SafetyConfig
