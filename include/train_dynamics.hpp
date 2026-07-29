#pragma once
#include <cmath>
#include <algorithm>
#include <cstdint>

enum class LocoModel : uint8_t {
    DF4D = 1,
    DF8B = 2,
    HXN3 = 3
};

struct LocomotiveProfile {
    LocoModel model;
    float mass_kg;
    float max_starting_tractive_effort_n;
    float tractive_build_up_time_s;       
    float air_brake_propagation_time_s;   
    float avg_deceleration_limit;         
};

struct WagonProfile {
    float gross_mass_kg;
    float length_m;
    float shoe_pressure_kn; 
    int total_shoes_per_wagon;
};

class FleetDatabase {
public:
    static LocomotiveProfile get_loco_profile(LocoModel model) {
        LocomotiveProfile profile;
        profile.model = model;
        profile.mass_kg = 150000.0f; // Standardized to 150t with 25t axle load consideration

        switch(model) {
            case LocoModel::DF4D:
                profile.max_starting_tractive_effort_n = 435000.0f;
                profile.tractive_build_up_time_s = 15.0f; 
                profile.air_brake_propagation_time_s = 1.2f; 
                profile.avg_deceleration_limit = 0.9f; 
                break;
            case LocoModel::DF8B:
                profile.max_starting_tractive_effort_n = 520000.0f;
                profile.tractive_build_up_time_s = 12.0f; 
                profile.air_brake_propagation_time_s = 1.1f; 
                profile.avg_deceleration_limit = 0.85f; 
                break;
            case LocoModel::HXN3:
                profile.max_starting_tractive_effort_n = 620000.0f;
                profile.tractive_build_up_time_s = 10.0f; 
                profile.air_brake_propagation_time_s = 1.0f; 
                profile.avg_deceleration_limit = 0.8f; 
                break;
        }
        return profile;
    }
};

class HeavyHaulDynamics {
public:
    static constexpr WagonProfile C80_PROFILE = {
        104000.0f,  // 满载约 104 吨
        12.0f,      
        20.0f,      
        8           
    };

    static constexpr WagonProfile C80_EMPTY_PROFILE = {
        24000.0f,   // 空载仅约 24 吨 (自重)
        12.0f,
        20.0f,
        8
    };

    static float calc_friction_coeff(float v_mps, float k_kn) {
        float v_kmh = v_mps * 3.6f; 
        if (v_kmh < 0.1f) v_kmh = 0.1f; 
        
        float term_k = (k_kn + 100.0f) / (5.0f * k_kn + 100.0f);
        float term_v = (2.0f * v_kmh + 100.0f) / (5.0f * v_kmh + 100.0f);
        return 0.32f * term_k * term_v;
    }

    static float get_instant_deceleration(float v_mps, const WagonProfile& wagon, int num_wagons, const LocomotiveProfile& loco) {
        float phi_h = calc_friction_coeff(v_mps, wagon.shoe_pressure_kn);
        float total_wagon_brake_n = num_wagons * wagon.total_shoes_per_wagon * wagon.shoe_pressure_kn * 1000.0f * phi_h;
        float loco_brake_n = loco.mass_kg * loco.avg_deceleration_limit; 
        float total_mass = loco.mass_kg + (num_wagons * wagon.gross_mass_kg);
        return (total_wagon_brake_n + loco_brake_n) / total_mass;
    }

    static float calc_pneumatic_delay(const WagonProfile& wagon, int num_wagons) {
        // Roughly 250m/s propagation speed
        float total_length = num_wagons * wagon.length_m;
        return total_length / 250.0f;
    }
};

// Heterogeneous Platoon Configuration
\
struct TrainConfig {
    LocoModel model;
    int num_wagons;
    bool is_empty;
};

inline TrainConfig get_fleet_config(int id) {
    // 强制定义 0-4 号车的异构特性
    switch(id) {
        case 0: return {LocoModel::HXN3, 48, false}; // 0号: 头车，重载满载
        case 1: return {LocoModel::DF8B, 48, true};  // 1号: 空载轻车 (刹车极快)
        case 2: return {LocoModel::DF4D, 50, false}; // 2号: 超长重载满载 (刹车极慢)
        case 3: return {LocoModel::DF8B, 48, false}; // 3号: DUT真实测试节点，满载
        case 4: return {LocoModel::DF8B, 40, true};  // 4号: 较短的空载轻车
        default: return {LocoModel::DF8B, 48, false};
    }
}