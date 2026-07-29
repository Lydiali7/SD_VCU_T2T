#pragma once
#include <string>

namespace VCU::Infra {
    // 实时系统配置类
    class SystemRT {
    public:
        
        static bool init(int cpu_core_id, int priority = 80);
    };
}