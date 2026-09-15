#pragma once
#include <cstdint>
#include <cstring>
#include <algorithm>
#include <cmath>
#include <cstddef>

#define MULTICAST_GROUP "239.0.0.1"
#define MULTICAST_PORT 8888
#define VCU_REPORT_PORT 9000
#define VCU_PEER_PORT 9100

#define STATUS_ATP_TRIP 1
#define STATUS_DEGRADED_UU 2
#define STATUS_BLIND_RUN 3
#define STATUS_DECOUPLING 4
#define STATUS_LEADER_NEW 5
#define STATUS_SIM_COMPLETE 6
#define STATUS_FAIL_SAFE_LOCK 99

enum class PlatoonState {
    STARTING,        
    CRUISING,        
    DECOUPLING,      
    LEADER_NEW,
    SAFE_STOP     
};

enum class NodeRole : uint32_t {
    AUTO = 0,
    LEADER = 1,
    FOLLOWER = 2
};

enum SnapshotValidity : uint32_t {
    SNAPSHOT_VALID_POS = 1u << 0,
    SNAPSHOT_VALID_SPEED = 1u << 1,
    SNAPSHOT_VALID_ACCEL = 1u << 2,
    SNAPSHOT_VALID_BRAKE_PRESS = 1u << 3,
    SNAPSHOT_VALID_ATP_MODE = 1u << 4,
    SNAPSHOT_VALID_EB_STATUS = 1u << 5,
    SNAPSHOT_VALID_MOVEMENT_AUTHORITY = 1u << 6,
    SNAPSHOT_VALID_TARGET_SPEED = 1u << 7
};

enum class T2TMsgType : uint8_t {
    ATP_SNAPSHOT = 1,
    TRAIN_STATE = 2,
    SAFETY_STATE = 3,
    FORMATION_STATE = 4,
    HEALTH_STATE = 5,
    EMERGENCY_BRAKE = 6
};

enum class TrainDataSource : uint32_t {
    SIM_FALLBACK = 1,
    HIL_FALLBACK = 2,
    MVB_TRAIN_LINE = 3,
    CAN_GATEWAY = 4,
    ATP_GATEWAY = 5,
    TCMS_GATEWAY = 6
};


// UDP (V2V and World Server Communication)

#pragma pack(push, 1)

struct RawT2TPacket {
    uint32_t sender_id; 
    uint32_t header;    // 0x55AA55AA
    uint32_t seq;       
    uint64_t global_timestamp_us; // [PTP SYNC] IEEE 1588 Microsecond Timestamp
    uint64_t payload;   
    uint32_t crc;       
};

struct WorldUpdatePacket {
    uint32_t header; 
    double timestamp; 
    uint32_t seq; 
    uint32_t train_id;
    double pos; 
    double vel; 
    double accel; 
    uint32_t status_flag;
};

struct ForceReportPacket {
    uint32_t train_id; 
    float cmd_force; 
    uint32_t status_flag;
    float current_vel;
    float current_pos;
};

struct WorldFeedbackPacket {
    float true_gap;
    float front_vel;
    float ego_pos;
    float ego_vel;
    uint32_t flags;
    uint64_t tx_timestamp_us;
};

#define WORLD_FEEDBACK_SIM_COMPLETE 0x1u
#define WORLD_FEEDBACK_HIL_IMPAIRED 0x2u

struct T2TFrameHeader {
    uint16_t magic;
    uint8_t version;
    uint8_t msg_type;
    uint16_t header_len;
    uint16_t payload_len;
    uint32_t source_train_id;
    uint32_t dest_train_id;
    uint32_t formation_id;
    uint32_t seq;
    uint64_t tai_timestamp_us;
    uint32_t validity_mask;
    uint32_t flags;
};

struct AtpSnapshot {
    uint64_t timestamp_us;
    uint32_t source_id;
    uint32_t validity_mask;
    int32_t train_pos_cm;
    int32_t speed_cmps;
    int32_t accel_cmps2;
    uint16_t permitted_speed_cmps;
    uint16_t target_speed_cmps;
    int32_t movement_authority_end_cm;
    uint16_t brake_cylinder_pressure_kpa;
    uint16_t brake_pipe_pressure_kpa;
    uint32_t atp_mode;
    uint32_t atp_status;
    uint32_t brake_command;
    uint32_t eb_reason;
    uint32_t seq;
    uint32_t crc32;
};

struct T2TAtpSnapshotPayload {
    AtpSnapshot snapshot;
};

struct T2TAtpSnapshotFrame {
    T2TFrameHeader header;
    T2TAtpSnapshotPayload payload;
    uint32_t frame_crc32;
};

#define T2T_FRAME_MAGIC 0x5432u
#define T2T_FRAME_VERSION 1u
#define T2T_BROADCAST_TRAIN_ID 0xFFFFFFFFu

struct PeerStatePacket {
    uint32_t header; 
    uint32_t sender_id;
    uint32_t role;
    uint32_t status_flag;
    uint32_t seq;
    uint64_t tx_timestamp_us;
    float track_pos;
    float velocity;
    float acceleration;
    float cmd_force;
};

#define PEER_STATE_HEADER 0x54525450u

struct PeerSafetyPacket {
    uint32_t header;
    uint32_t sender_id;
    uint32_t role;
    uint32_t status_flag;
    uint32_t seq;
    uint64_t tx_timestamp_us;
    float estimated_gap;
    float safe_gap;
    float safety_margin;
    float uncertainty_buffer;
    float control_aoi_ms;
    uint32_t safety_flags;
};

struct PeerFormationPacket {
    uint32_t header;
    uint32_t sender_id;
    uint32_t role;
    uint32_t seq;
    uint64_t tx_timestamp_us;
    uint32_t formation_id;
    uint32_t front_id;
    float target_gap;
    uint32_t formation_phase;
};

struct PeerHealthPacket {
    uint32_t header;
    uint32_t sender_id;
    uint32_t seq;
    uint64_t tx_timestamp_us;
    float peer_aoi_ms;
    float control_aoi_ms;
    uint32_t burst_loss_count;
    uint64_t last_time_diff_us;
    uint32_t network_health;
    uint32_t health_flags;
};

#define PEER_SAFETY_HEADER 0x54525346u
#define PEER_FORMATION_HEADER 0x5452464fu
#define PEER_HEALTH_HEADER 0x5452484cu

#define PEER_SAFETY_FLAG_EB_ACTIVE 0x1u
#define PEER_SAFETY_FLAG_FAIL_SAFE 0x2u
#define PEER_HEALTH_FLAG_WATCHDOG_TIMEOUT 0x1u
#define PEER_HEALTH_FLAG_PEER_FRESH 0x2u

inline uint32_t t2t_crc32_fnv1a(const uint8_t* data, size_t len) {
    uint32_t hash = 2166136261u;
    for (size_t i = 0; i < len; ++i) {
        hash ^= data[i];
        hash *= 16777619u;
    }
    return hash;
}

inline int32_t meters_to_cm(float meters) {
    return static_cast<int32_t>(std::lround(meters * 100.0f));
}

inline int32_t mps_to_cmps(float mps) {
    return static_cast<int32_t>(std::lround(mps * 100.0f));
}

inline uint16_t mps_to_u16_cmps(float mps) {
    int32_t cmps = mps_to_cmps(mps);
    return static_cast<uint16_t>(std::clamp(cmps, 0, 65535));
}

inline float cm_to_m(int32_t cm) {
    return static_cast<float>(cm) / 100.0f;
}

inline float cmps_to_mps(int32_t cmps) {
    return static_cast<float>(cmps) / 100.0f;
}

inline void finalize_atp_snapshot_crc(AtpSnapshot& snapshot) {
    snapshot.crc32 = 0;
    snapshot.crc32 = t2t_crc32_fnv1a(
        reinterpret_cast<const uint8_t*>(&snapshot),
        offsetof(AtpSnapshot, crc32));
}

inline bool build_snapshot_from_sim(uint32_t train_id, uint64_t timestamp_us,
                                    float pos_m, float vel_mps, float accel_mps2,
                                    uint32_t status_flag, uint32_t seq,
                                    TrainDataSource source, AtpSnapshot& out) {
    std::memset(&out, 0, sizeof(out));
    out.timestamp_us = timestamp_us;
    out.source_id = static_cast<uint32_t>(source);
    out.validity_mask = SNAPSHOT_VALID_POS | SNAPSHOT_VALID_SPEED | SNAPSHOT_VALID_ACCEL |
                        SNAPSHOT_VALID_EB_STATUS | SNAPSHOT_VALID_TARGET_SPEED;
    out.train_pos_cm = meters_to_cm(pos_m);
    out.speed_cmps = mps_to_cmps(vel_mps);
    out.accel_cmps2 = mps_to_cmps(accel_mps2);
    out.permitted_speed_cmps = mps_to_u16_cmps(26.5f);
    out.target_speed_cmps = mps_to_u16_cmps(24.5f);
    out.atp_status = status_flag;
    out.brake_command = (status_flag == STATUS_ATP_TRIP || status_flag == STATUS_FAIL_SAFE_LOCK) ? 1u : 0u;
    out.eb_reason = status_flag;
    out.seq = seq;
    (void)train_id;
    finalize_atp_snapshot_crc(out);
    return true;
}

inline bool build_t2t_atp_snapshot_frame(uint32_t source_train_id, uint32_t dest_train_id,
                                         uint32_t formation_id, uint32_t seq,
                                         uint32_t flags, const AtpSnapshot& snapshot,
                                         T2TAtpSnapshotFrame& out) {
    std::memset(&out, 0, sizeof(out));
    out.header.magic = T2T_FRAME_MAGIC;
    out.header.version = T2T_FRAME_VERSION;
    out.header.msg_type = static_cast<uint8_t>(T2TMsgType::ATP_SNAPSHOT);
    out.header.header_len = sizeof(T2TFrameHeader);
    out.header.payload_len = sizeof(T2TAtpSnapshotPayload);
    out.header.source_train_id = source_train_id;
    out.header.dest_train_id = dest_train_id;
    out.header.formation_id = formation_id;
    out.header.seq = seq;
    out.header.tai_timestamp_us = snapshot.timestamp_us;
    out.header.validity_mask = snapshot.validity_mask;
    out.header.flags = flags;
    out.payload.snapshot = snapshot;
    out.frame_crc32 = t2t_crc32_fnv1a(
        reinterpret_cast<const uint8_t*>(&out),
        offsetof(T2TAtpSnapshotFrame, frame_crc32));
    return true;
}

enum class T2TFrameValidationResult : uint8_t {
    VALID = 0,
    MAGIC_ERROR,
    VERSION_ERROR,
    MESSAGE_TYPE_ERROR,
    HEADER_LENGTH_ERROR,
    PAYLOAD_LENGTH_ERROR,
    SNAPSHOT_CRC_ERROR,
    FRAME_CRC_ERROR
};

inline T2TFrameValidationResult validate_t2t_atp_snapshot_frame_detailed(
    const T2TAtpSnapshotFrame& frame) {
    if (frame.header.magic != T2T_FRAME_MAGIC) return T2TFrameValidationResult::MAGIC_ERROR;
    if (frame.header.version != T2T_FRAME_VERSION) return T2TFrameValidationResult::VERSION_ERROR;
    if (frame.header.msg_type != static_cast<uint8_t>(T2TMsgType::ATP_SNAPSHOT)) return T2TFrameValidationResult::MESSAGE_TYPE_ERROR;
    if (frame.header.header_len != sizeof(T2TFrameHeader)) return T2TFrameValidationResult::HEADER_LENGTH_ERROR;
    if (frame.header.payload_len != sizeof(T2TAtpSnapshotPayload)) return T2TFrameValidationResult::PAYLOAD_LENGTH_ERROR;

    AtpSnapshot snapshot = frame.payload.snapshot;
    uint32_t expected_snapshot_crc = snapshot.crc32;
    finalize_atp_snapshot_crc(snapshot);
    if (snapshot.crc32 != expected_snapshot_crc) return T2TFrameValidationResult::SNAPSHOT_CRC_ERROR;

    uint32_t expected_frame_crc = t2t_crc32_fnv1a(
        reinterpret_cast<const uint8_t*>(&frame),
        offsetof(T2TAtpSnapshotFrame, frame_crc32));
    return expected_frame_crc == frame.frame_crc32
               ? T2TFrameValidationResult::VALID
               : T2TFrameValidationResult::FRAME_CRC_ERROR;
}

inline bool validate_t2t_atp_snapshot_frame(const T2TAtpSnapshotFrame& frame) {
    return validate_t2t_atp_snapshot_frame_detailed(frame) == T2TFrameValidationResult::VALID;
}

enum class TrainBusType : uint8_t {
    T2T_RADIO = 1,
    MVB = 2,
    CAN = 3
};

enum class TrainBusService : uint8_t {
    PROCESS_DATA = 1,
    EMERGENCY_BRAKE = 2,
    HEARTBEAT = 3,
    SIM_CONTROL = 4
};

// IEC 61375-inspired process-data envelope for VCU-internal HAL boundaries.
// It is intentionally explicit and byte-stable: HAL drivers move this frame,
// while perception/control code consumes the decoded payload.
struct TrainBusFrame {
    uint16_t sof;              // 0xFEFA, aligned with MVB board command style
    uint8_t version;
    uint8_t bus_type;
    uint8_t service;
    uint8_t flags;
    uint16_t source_addr;
    uint16_t dest_addr;
    uint32_t seq;
    uint64_t timestamp_us;
    uint16_t payload_len;
    uint8_t payload[64];
    uint32_t crc32;
    uint16_t eof;              // 0xE4FF, aligned with uploaded MVB examples
};

class TrainBusFraming {
public:
    static constexpr uint16_t SOF = 0xFEFA;
    static constexpr uint16_t EOF_MARK = 0xE4FF;
    static constexpr uint8_t VERSION = 1;

    static uint32_t crc32_fnv1a(const uint8_t* data, size_t len) {
        uint32_t hash = 2166136261u;
        for (size_t i = 0; i < len; ++i) {
            hash ^= data[i];
            hash *= 16777619u;
        }
        return hash;
    }

    static bool encode_payload(TrainBusFrame& frame, TrainBusType bus, TrainBusService service,
                               uint16_t source, uint16_t dest, uint32_t seq, uint64_t ts_us,
                               const void* payload, size_t payload_len, uint8_t flags = 0) {
        if (payload_len > sizeof(frame.payload)) return false;
        std::memset(&frame, 0, sizeof(frame));
        frame.sof = SOF;
        frame.version = VERSION;
        frame.bus_type = static_cast<uint8_t>(bus);
        frame.service = static_cast<uint8_t>(service);
        frame.flags = flags;
        frame.source_addr = source;
        frame.dest_addr = dest;
        frame.seq = seq;
        frame.timestamp_us = ts_us;
        frame.payload_len = static_cast<uint16_t>(payload_len);
        if (payload_len > 0 && payload != nullptr) {
            std::memcpy(frame.payload, payload, payload_len);
        }
        frame.crc32 = crc32_fnv1a(frame.payload, frame.payload_len);
        frame.eof = EOF_MARK;
        return true;
    }

    static bool encode_raw_t2t(TrainBusFrame& frame, const RawT2TPacket& pkt) {
        return encode_payload(frame, TrainBusType::T2T_RADIO, TrainBusService::PROCESS_DATA,
                              static_cast<uint16_t>(pkt.sender_id), 0xFFFF, pkt.seq,
                              pkt.global_timestamp_us, &pkt, sizeof(pkt));
    }

    static bool decode_raw_t2t(const TrainBusFrame& frame, RawT2TPacket& pkt) {
        if (!validate(frame) || frame.bus_type != static_cast<uint8_t>(TrainBusType::T2T_RADIO) ||
            frame.payload_len != sizeof(RawT2TPacket)) {
            return false;
        }
        std::memcpy(&pkt, frame.payload, sizeof(pkt));
        return true;
    }

    static bool validate(const TrainBusFrame& frame) {
        if (frame.sof != SOF || frame.eof != EOF_MARK || frame.version != VERSION) return false;
        if (frame.payload_len > sizeof(frame.payload)) return false;
        return crc32_fnv1a(frame.payload, frame.payload_len) == frame.crc32;
    }
};


// Hardware Interface Protocols (MVB / LoRa)


struct MVB_Hardware_Frame {
    uint8_t head[2];      
    uint8_t cmd_type;     
    uint8_t port_addr;    
    
    uint16_t raw_speed;    
    uint16_t brake_press;  
    uint32_t io_status;    
    
    uint8_t reserved[240]; 
    uint8_t checksum;      
};
/*
struct LoRa_Physical_Packet {
    uint16_t target_addr;  
    uint8_t channel;       
    uint8_t payload[128];  
};
*/

#pragma pack(pop)


// Sensor Redundancy Protocols (CAN / AVX-512 aligned)


struct alignas(64) CANPacket {
    uint32_t header;       
    uint16_t source_id;    
    uint16_t seq_num;      
    uint64_t gptp_timestamp_us; // TSN Time Sync for onboard sensors
    float payload[10];          
    uint32_t crc32;        
    uint32_t padding;      
};
struct DualSensorBroadcast {
    uint16_t target_train_id; 
    CANPacket path_a;         
    CANPacket path_b;         
};

inline bool build_snapshot_from_mvb(uint32_t train_id, uint64_t timestamp_us,
                                    const MVB_Hardware_Frame& mvb, uint32_t seq,
                                    AtpSnapshot& out) {
    std::memset(&out, 0, sizeof(out));
    out.timestamp_us = timestamp_us;
    out.source_id = static_cast<uint32_t>(TrainDataSource::MVB_TRAIN_LINE);
    out.validity_mask = SNAPSHOT_VALID_SPEED | SNAPSHOT_VALID_BRAKE_PRESS;
    out.speed_cmps = static_cast<int32_t>(mvb.raw_speed);
    out.brake_pipe_pressure_kpa = mvb.brake_press;
    out.atp_status = mvb.io_status;
    out.seq = seq;
    (void)train_id;
    finalize_atp_snapshot_crc(out);
    return true;
}

inline bool build_snapshot_from_can(uint32_t train_id, const CANPacket& can, uint32_t seq,
                                    AtpSnapshot& out) {
    std::memset(&out, 0, sizeof(out));
    out.timestamp_us = can.gptp_timestamp_us;
    out.source_id = static_cast<uint32_t>(TrainDataSource::CAN_GATEWAY);
    out.validity_mask = SNAPSHOT_VALID_SPEED | SNAPSHOT_VALID_ACCEL;
    out.speed_cmps = mps_to_cmps(can.payload[1]);
    out.accel_cmps2 = mps_to_cmps(can.payload[2]);
    out.atp_status = can.header;
    out.seq = seq;
    (void)train_id;
    finalize_atp_snapshot_crc(out);
    return true;
}

class ProtocolConverter {
public:
    static void encode_mvb(MVB_Hardware_Frame& frame, double speed, double pressure) {
        frame.head[0] = 0xFE;
        frame.head[1] = 0xFA;
        frame.cmd_type = 0x05;
        frame.raw_speed = static_cast<uint16_t>(speed * 100.0);
        frame.brake_press = static_cast<uint16_t>(pressure * 10.0);
    }

    static bool validate_mvb(const uint8_t* buffer, size_t len) {
        if (len < sizeof(MVB_Hardware_Frame)) return false;
        return (buffer[0] == 0xFE && buffer[1] == 0xFA);
    }
};
