#include "perception.hpp"
#include <iostream>
#include <algorithm>
#include <random>
#include <cmath>

//Static Random Number Generators for Fading Models
static std::random_device rd;
static std::mt19937 rf_gen(rd());
static std::normal_distribution<float> gauss_dist(0.0f, 1.0f);

bool PerceptionEngine::fast_unpack(const RawT2TPacket& raw, SensorData& out, uint32_t& last_seq) {
    if (raw.header != 0x55AA55AA || raw.seq <= last_seq) return false;
    if (_mm_crc32_u64(0, raw.payload) != raw.crc) return false;

    last_seq = raw.seq;
    uint16_t v_raw = raw.payload & 0xFFFF;
    uint32_t p_raw = (raw.payload >> 16) & 0xFFFFFFFF;
    int16_t a_raw = (int16_t)(raw.payload >> 48);

    out.distances[0] = p_raw / 100.0f; 
    out.distances[1] = v_raw / 100.0f; 
    out.distances[2] = a_raw / 100.0f; 
    return true;
}

bool PerceptionEngine::decode_input_buffer(const StandardInputBuffer& input, SensorData& out) {
    switch (input.source) {
        case HalBusSource::CAN_RADAR:
            return decode_can_radar_buffer(input, out);
        case HalBusSource::MVB_TRAIN_LINE:
            return decode_mvb_trainline_buffer(input, out);
        case HalBusSource::T2T_RADIO: {
            if (input.len < sizeof(TrainBusFrame)) return false;
            TrainBusFrame frame;
            std::memcpy(&frame, input.data, sizeof(frame));
            RawT2TPacket pkt;
            uint32_t seq = 0;
            return TrainBusFraming::decode_raw_t2t(frame, pkt) && fast_unpack(pkt, out, seq);
        }
        default:
            return false;
    }
}

bool PerceptionEngine::decode_can_radar_buffer(const StandardInputBuffer& input, SensorData& out) {
    std::memset(&out, 0, sizeof(out));

    if (input.len >= sizeof(CANPacket)) {
        CANPacket pkt;
        std::memcpy(&pkt, input.data, sizeof(pkt));
        if (pkt.header == 0x55AA55AA) {
            size_t count = std::min<size_t>(10, sizeof(out.distances) / sizeof(out.distances[0]));
            std::memcpy(out.distances, pkt.payload, count * sizeof(float));
            return true;
        }
    }

    // Compact radar fallback: distance, ego velocity, front velocity as float32.
    if (input.len >= 3 * sizeof(float)) {
        std::memcpy(out.distances, input.data, 3 * sizeof(float));
        return true;
    }

    return false;
}

bool PerceptionEngine::decode_mvb_trainline_buffer(const StandardInputBuffer& input, SensorData& out) {
    std::memset(&out, 0, sizeof(out));
    if (input.len < sizeof(MVB_Hardware_Frame)) return false;

    MVB_Hardware_Frame frame;
    std::memcpy(&frame, input.data, sizeof(frame));
    if (!ProtocolConverter::validate_mvb(reinterpret_cast<const uint8_t*>(&frame), sizeof(frame))) {
        return false;
    }

    out.distances[0] = 0.0f;                         // No radar gap in MVB train-line frame
    out.distances[1] = frame.raw_speed / 100.0f;     // ego speed
    out.distances[2] = -1.0f;                        // front speed unavailable
    out.distances[3] = frame.brake_press / 10.0f;    // brake pipe/cylinder pressure
    out.distances[4] = static_cast<float>(frame.io_status);
    return true;
}

float PerceptionEngine::calculate_heavy_haul_safe_dist(
    float v_rear_mps, float v_front_mps, 
    LocoModel rear_model, LocoModel front_model, 
    const WagonProfile& rear_wagon, const WagonProfile& front_wagon,
    int rear_num, int front_num, 
    bool is_ecp_active, float actual_comm_delay_s,
    float current_gradient_permille)
{
    float d_buffer = 150.0f; 
    LocomotiveProfile rear_loco = FleetDatabase::get_loco_profile(rear_model);
    LocomotiveProfile front_loco = FleetDatabase::get_loco_profile(front_model);

    // Gravity acceleration component based on track gradient
    float g_component = 9.8f * (current_gradient_permille / 1000.0f);

    float s_front = 0.0f;
    float temp_v_front = v_front_mps;
    float dt = 0.1f; 
    
    // Front train braking distance integration
    while (temp_v_front > 0.0f) {
        float inst_decel = HeavyHaulDynamics::get_instant_deceleration(temp_v_front, front_wagon, front_num, front_loco);
        inst_decel += g_component; 
        if (inst_decel < 0.05f) inst_decel = 0.05f; 
        
        temp_v_front -= inst_decel * dt;
        if (temp_v_front < 0.0f) temp_v_front = 0.0f;
        s_front += temp_v_front * dt;
    }

    float loco_pneumatic_delay = rear_loco.air_brake_propagation_time_s; 
    float wagon_wave_delay = is_ecp_active ? 0.1f : HeavyHaulDynamics::calc_pneumatic_delay(rear_wagon, rear_num);

    float total_delay_time = actual_comm_delay_s + loco_pneumatic_delay + wagon_wave_delay;
    float s_rear_delay = v_rear_mps * total_delay_time;

    float s_rear_brake = 0.0f;
    float temp_v_rear = v_rear_mps;
    
    // Rear train braking distance integration
    while (temp_v_rear > 0.0f) {
        float inst_decel = HeavyHaulDynamics::get_instant_deceleration(temp_v_rear, rear_wagon, rear_num, rear_loco);
        inst_decel += g_component; 
        if (inst_decel < 0.05f) inst_decel = 0.05f;

        temp_v_rear -= inst_decel * dt;
        if (temp_v_rear < 0.0f) temp_v_rear = 0.0f;
        s_rear_brake += temp_v_rear * dt;
    }

    float required_gap = (s_rear_delay + s_rear_brake) - s_front + d_buffer;
    return std::max(d_buffer, required_gap);
}

float PerceptionEngine::get_packet_error_rate(float distance_m, bool is_in_tunnel) {
    const float P_tx_dbm = 23.0f;           
    const float Noise_floor_dbm = -110.0f;  
    const float Reference_path_loss = 40.0f; 
    const float Path_loss_exponent = 2.4f;   
    const float Tunnel_occlusion_db = 25.0f; 

    // 1. Large-scale Fading (Path Loss)
    float path_loss = Reference_path_loss + 10.0f * Path_loss_exponent * std::log10(std::max(distance_m, 1.0f));
    if (is_in_tunnel) path_loss += Tunnel_occlusion_db;

    // 2. Small-scale Fast Fading (Multipath)
    float fading_db = 0.0f;
    if (is_in_tunnel) {
        // NLOS (Tunnel): Rayleigh Fading
        float x = gauss_dist(rf_gen);
        float y = gauss_dist(rf_gen);
        float rayleigh_linear = (x * x + y * y) / 2.0f; 
        fading_db = 10.0f * std::log10(std::max(rayleigh_linear, 0.0001f)); 
    } else {
        // LOS (Open Area): Rician Fading (K-factor = 4.0 ~ 6dB)
        float K_linear = 4.0f; 
        float mean = std::sqrt(K_linear / (K_linear + 1.0f));
        float sigma = std::sqrt(1.0f / (2.0f * (K_linear + 1.0f)));
        
        float x = mean + sigma * gauss_dist(rf_gen);
        float y = sigma * gauss_dist(rf_gen);
        float rician_linear = (x * x + y * y);
        fading_db = 10.0f * std::log10(std::max(rician_linear, 0.0001f));
    }

    // 3. Final SNR calculation with Multipath Perturbation
    float snr = P_tx_dbm - path_loss + fading_db - Noise_floor_dbm;

    // 4. Map SNR to PER
    float k = 0.8f; 
    float threshold = 10.0f; 
    float per = 1.0f / (1.0f + std::exp(k * (snr - threshold)));

    return std::clamp(per, 0.0f, 1.0f);
}

SDVCU_Core::SDVCU_Core(LocoModel me, LocoModel front, int wagons) {
    consecutive_errors = 0;
    last_seq = 0;
    is_eb_triggered = false;
    current_safe_gap = 9999.0f;
    current_vel = 0.0f;
    my_loco_model = me;
    front_loco_model = front;
    num_wagons = wagons;
}

bool SDVCU_Core::process_sensors(uint16_t seq_num, const SensorData& path_a, const SensorData& path_b, 
                                 uint64_t ts_a_us, uint64_t ts_b_us, float current_aoi_s) {
    
    uint64_t time_diff = (ts_a_us > ts_b_us) ? (ts_a_us - ts_b_us) : (ts_b_us - ts_a_us);
    const uint64_t TSN_TOLERANCE_US = 50000; 

    if (time_diff > TSN_TOLERANCE_US) {
        consecutive_errors++;
        std::cout << "[PERCEPTION_WARN] TSN Time-Window Check Failed! Desync: " << time_diff << "us. Link degraded.\n";
    } 
    else {
        consecutive_errors = 0; 
        if (seq_num != 0) last_seq = seq_num;
        
        float distance_to_obstacle = (path_a.distances[0] + path_b.distances[0]) / 2.0f; 
        current_vel = (path_a.distances[1] + path_b.distances[1]) / 2.0f; 
        float front_vel = (path_a.distances[2] + path_b.distances[2]) / 2.0f; 

        if (front_vel < 0.0f) {
            front_vel = current_vel; 
        }
        
        TrainConfig my_cfg = get_fleet_config(3);
        TrainConfig front_cfg = get_fleet_config(2);
        WagonProfile my_wagon = my_cfg.is_empty ? HeavyHaulDynamics::C80_EMPTY_PROFILE : HeavyHaulDynamics::C80_PROFILE;
        WagonProfile front_wagon = front_cfg.is_empty ? HeavyHaulDynamics::C80_EMPTY_PROFILE : HeavyHaulDynamics::C80_PROFILE;

        current_safe_gap = PerceptionEngine::calculate_heavy_haul_safe_dist(
            current_vel, front_vel, my_cfg.model, front_cfg.model, 
            my_wagon, front_wagon, my_cfg.num_wagons, front_cfg.num_wagons, false, current_aoi_s, 0.0f);
        
        if (distance_to_obstacle > 0.1f && distance_to_obstacle < current_safe_gap) {
            std::cout << "[PERCEPTION_FATAL] ATP Trip! Radar Gap < Safe Gap. Triggering EB Lock!\n";
            is_eb_triggered = true; 
        }
    }
    
    if (consecutive_errors >= ERROR_THRESHOLD) {
        std::cout << "[PERCEPTION_FATAL] Sensor Paths Diverged > Threshold. Triggering Fail-safe EB!\n";
        is_eb_triggered = true;
    }
    
    return !is_eb_triggered;
}

bool SDVCU_Core::is_eb() const { return is_eb_triggered; }
void SDVCU_Core::force_eb_trigger() { is_eb_triggered = true; }
