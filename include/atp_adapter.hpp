#pragma once

#include <cstdint>
#include "network_proto.hpp"

enum class ATPAdapterSource : uint32_t {
    SIM_FALLBACK = static_cast<uint32_t>(TrainDataSource::SIM_FALLBACK),
    HIL_FALLBACK = static_cast<uint32_t>(TrainDataSource::HIL_FALLBACK),
    MVB_TRAIN_LINE = static_cast<uint32_t>(TrainDataSource::MVB_TRAIN_LINE),
    CAN_GATEWAY = static_cast<uint32_t>(TrainDataSource::CAN_GATEWAY),
    ATP_GATEWAY = static_cast<uint32_t>(TrainDataSource::ATP_GATEWAY),
    TCMS_GATEWAY = static_cast<uint32_t>(TrainDataSource::TCMS_GATEWAY)
};

struct FallbackTrainState {
    uint32_t train_id;
    uint64_t timestamp_us;
    float position_m;
    float velocity_mps;
    float acceleration_mps2;
    uint32_t status_flag;
    uint32_t seq;
    ATPAdapterSource source;
};

class ATPAdapter {
public:
    static bool from_fallback(const FallbackTrainState& state, AtpSnapshot& out);
    static bool from_mvb(uint32_t train_id, uint64_t timestamp_us,
                         const MVB_Hardware_Frame& mvb, uint32_t seq,
                         AtpSnapshot& out);
    static bool from_can(uint32_t train_id, const CANPacket& can, uint32_t seq,
                         AtpSnapshot& out);

    // Placeholder for the real ATP/TCMS gateway mapping.
    // This must be implemented from the authorized ICD, not guessed from simulation fields.
    static bool from_atp_gateway(const uint8_t* data, size_t len, AtpSnapshot& out);

    static bool validate_snapshot_crc(const AtpSnapshot& snapshot);
};
