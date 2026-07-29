#pragma once
#include <immintrin.h>
#include <nmmintrin.h>
#include <cstdint>
#include <cstddef>
#include <vector>
#include <algorithm>
#include <cstring>
#include "network_proto.hpp"
#include "train_dynamics.hpp"


// 1. Legacy & Compatibility Definitions 
enum class TrainType {
    DF4D,
    DF8B,
    HXN3,
    EMU_DISTRIBUTED
};

struct TrainState {
    int id;
    double pos;         
    double vel;         
    double accel;       
    float cmd_force;    
    float mass; 
    float smooth_brake_pressure = 0.0f; 
    uint16_t lora_target_addr = 0xFFFF;
};


// 2. Hardware Accelerated Perception Structures

struct alignas(64) SensorData {
    float distances[16]; 
};

enum class HalBusSource : uint8_t {
    UNKNOWN = 0,
    CAN_RADAR = 1,
    MVB_TRAIN_LINE = 2,
    T2T_RADIO = 3
};

struct StandardInputBuffer {
    HalBusSource source = HalBusSource::UNKNOWN;
    uint16_t channel = 0;
    uint64_t timestamp_us = 0;
    size_t len = 0;
    uint8_t data[256] = {0};
};


// 3. Kalman Filter for Tunnel Blind Run

class KalmanTracker {
public:
    float pos;
    float vel;
    float p_err;
    float v_err;

    KalmanTracker(float init_p, float init_v) 
        : pos(init_p), vel(init_v), p_err(10.0f), v_err(2.0f) {}

    void predict(float dt) {
        pos += vel * dt;
        p_err += 5.0f * dt; 
        v_err += 1.0f * dt;
    }

    void update(float meas_p, float meas_v) {
        float k_p = p_err / (p_err + 20.0f); 
        pos = pos + k_p * (meas_p - pos);
        p_err = (1.0f - k_p) * p_err;

        float k_v = v_err / (v_err + 5.0f);  
        vel = vel + k_v * (meas_v - vel);
        v_err = (1.0f - k_v) * v_err;
    }
};


// 4. Core Logic & Math Engines

class SDVCU_Core {
private:
    int consecutive_errors;
    const int ERROR_THRESHOLD = 3; 
    uint16_t last_seq;
    bool is_eb_triggered;
    
    LocoModel my_loco_model;
    LocoModel front_loco_model;
    int num_wagons;

    bool avx2_payload_match(const SensorData& a, const SensorData& b); //to be implemented with prototype for fast comparison

public:
    float current_safe_gap;
    float current_vel;

    SDVCU_Core(LocoModel me, LocoModel front, int wagons);
    
    bool process_sensors(uint16_t seq_num, const SensorData& path_a, const SensorData& path_b, 
                         uint64_t ts_a_us, uint64_t ts_b_us, float current_aoi_s);
    
    bool is_eb() const;
    void force_eb_trigger();
};

class PerceptionEngine {
public:
    static bool fast_unpack(const RawT2TPacket& raw, SensorData& out, uint32_t& last_seq);
    static bool decode_input_buffer(const StandardInputBuffer& input, SensorData& out);
    static bool decode_can_radar_buffer(const StandardInputBuffer& input, SensorData& out);
    static bool decode_mvb_trainline_buffer(const StandardInputBuffer& input, SensorData& out);
    
    //Added current_gradient_permille for topography awareness
    static float calculate_heavy_haul_safe_dist(
        float v_rear_mps, float v_front_mps, 
        LocoModel rear_model, LocoModel front_model, 
        const WagonProfile& rear_wagon, const WagonProfile& front_wagon,
        int rear_num, int front_num, 
        bool is_ecp_active, float actual_comm_delay_s,
        float current_gradient_permille = 0.0f); 
    
    static float get_packet_error_rate(float distance_m, bool is_in_tunnel);

    static float calculate_safe_dist(float v_rear_mps, float v_front_mps, TrainType type);
};
