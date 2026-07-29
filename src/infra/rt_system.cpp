#include "infra/rt_system.hpp"
#include <cerrno>
#include <cstring>
#include <iostream>
#include <sched.h>
#include <pthread.h>
#include <sys/mman.h>

namespace VCU::Infra {
    bool SystemRT::init(int cpu_core_id, int priority) {
        // 1. 设置 SCHED_FIFO 优先级
        struct sched_param param;
        param.sched_priority = priority;
        if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
            std::cerr << "[RT_WARN] SCHED_FIFO unavailable (" << std::strerror(errno)
                      << "). Continuing with normal scheduler.\n";
            return false;
        }

        // 2. 绑定 CPU 核心
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(cpu_core_id, &cpuset);
        if (pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset) != 0) {
            std::cerr << "[RT_WARN] CPU affinity unavailable. Continuing without pinning.\n";
            return false;
        }

        // 3. 锁定内存，防止 Swap 抖动
        if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
            std::cerr << "[RT_WARN] mlockall unavailable (" << std::strerror(errno)
                      << "). Continuing without memory lock.\n";
            return false;
        }

        std::cout << "[RT_SYSTEM] Real-time environment initialized on Core " << cpu_core_id << "\n";
        return true;
    }
}
