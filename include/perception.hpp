#pragma once
#include <immintrin.h>
#include <iostream>

// Defined train composition types
enum class TrainType {
    LOCO_HAULED,    // Locomotive-hauled (Single power car at front, slower response)
    EMU_DISTRIBUTED // Electric Multiple Unit (Distributed power/brake, fast response)
};

struct alignas(64) SensorData {
    float distances[16]; 
    //[0] = distance to front obstacle (m)
    //[1] = current speed (m/s)
    //[2] = former speed (m/s) 
    //[3] = former acceleration (m/s^2)
};

class PerceptionEngine {
public:
    // Calculate safety distance based on train dynamics
    static float calculate_safe_envelope(float v, TrainType type);
    
    // Core safety check combining 2oo2 and dynamics
    static bool is_system_safe(const SensorData& pathA, const SensorData& pathB, TrainType type,bool cooperative);
};