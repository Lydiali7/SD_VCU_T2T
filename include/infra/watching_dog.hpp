#pragma once
#include <chrono>
#include <iostream>
#include "safety_config.hpp"

/*
class HardwareWatchdog {
private:
    uint64_t last_pet_time_us;
    const uint64_t timeout_us = VCU::SafetyConfig::kWatchdogTimeoutUs;

public:
    HardwareWatchdog() : last_pet_time_us(0) {}

    // 喂狗函数：这在底层通常对应操作 GPIO
    void pet() {
        last_pet_time_us = std::chrono::steady_clock::now().time_since_epoch().count();
        // [此处插入底层物理 GPIO 翻转逻辑，例如写 /sys/class/gpio]
        // std::cout << "[WATCHDOG] Petting the dog...\n";
    }

    // 检查逻辑：如果在主循环检测到未及时喂狗，应立即触发软件内部的最高级 EB
    bool is_timed_out() {
        uint64_t now = std::chrono::steady_clock::now().time_since_epoch().count();
        return (now - last_pet_time_us) > timeout_us;
    }
};

*/
class HardwareWatchdog {
private:
    uint64_t last_pet_time_us;
    const uint64_t timeout_us = 50000; // 50ms

public:
    HardwareWatchdog() : last_pet_time_us(0) {}

    void pet() {
        auto now = std::chrono::steady_clock::now();
        last_pet_time_us = std::chrono::duration_cast<std::chrono::microseconds>(
                               now.time_since_epoch()
                           ).count();
    }

    bool is_timed_out() {
        auto now = std::chrono::steady_clock::now();
        uint64_t now_us = std::chrono::duration_cast<std::chrono::microseconds>(
                              now.time_since_epoch()
                          ).count();
        return (now_us - last_pet_time_us) > timeout_us;
    }

    // 新增 getter
    uint64_t get_last_pet_time_us() const {
        return last_pet_time_us;
    }
};
