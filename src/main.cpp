#include <iostream>
#include <thread>
#include <pthread.h>
#include <sys/mman.h>
#include <nmmintrin.h> // 用于生产者计算 CRC
#include "perception.hpp"
#include "spsc_queue.hpp"

//队列现在传输原始二进制报文，模拟真实的 T2T 总线
SPSCQueue<RawT2TPacket> data_bus(128);

void pin_thread_to_core(int core_id) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core_id, &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
}

// PRODUCER: 模拟 Core 3 从无线电接收并打包数据
void communication_thread() {
    pin_thread_to_core(3);
    uint32_t seq_counter = 0;
    float sim_dist = 50.0f;
    float sim_speed = 33.0f;

    while (true) {
        RawT2TPacket raw;
        raw.header = 0x55AA55AA;
        raw.seq = ++seq_counter;
        
        uint64_t v_self_enc = static_cast<uint64_t>(sim_speed * 100);
        uint64_t d_enc = static_cast<uint64_t>(sim_dist * 100);
        uint64_t v_front_enc = static_cast<uint64_t>(30.0f * 100); 
        raw.payload = v_self_enc | (d_enc << 16) | (v_front_enc << 48);

        // 先计算正确的 CRC
        raw.crc = _mm_crc32_u64(0, raw.payload);

        // 在 push 之前注入干扰
        /*if (seq_counter % 500 == 0) {
            raw.crc ^= 0xFFFFFFFF; // 故意破坏校验码
            std::cout << "\n[Core 3] !!! INJECTING NETWORK NOISE (Corrupting CRC) !!!" << std::endl;
        }*/
       if (seq_counter % 1000 >= 500 && seq_counter % 1000 < 503) {
            raw.crc ^= 0xFFFFFFFF; // 破坏校验
            std::cout << "[Core 3] !!! INJECTING NETWORK INTERFERENCE !!! Seq: " << seq_counter << std::endl;
        }

        // 最后再推送到总线
        data_bus.push(raw);

        if (sim_dist > 4.5f) sim_dist -= 0.1f;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

TrainType my_train_type = TrainType::EMU_DISTRIBUTED;

// CONSUMER: Core 2 执行安全解析与决策
void control_thread() {
    pin_thread_to_core(2);
    uint32_t last_seq = 0; // 用于防重放校验
    RawT2TPacket raw_in;
    SensorData current_packet;

    int consecutive_errors = 0; // 连续错误计数
    const int MAX_ALLOWED_ERRORS = 3; 

    while (true) {
        if (data_bus.pop(raw_in)) {
            //高速解包 & 协议安全校验
            if (PerceptionEngine::fast_unpack(raw_in, current_packet, last_seq)) {

                consecutive_errors = 0; // 成功解析置0
                
                //执行 SIL4 级安全决策,模拟 2oo2，传入两份一样的解析结果进行比对
                PerceptionEngine::is_system_safe(current_packet, current_packet, my_train_type, true);
                
                // 更新序列号基准
                last_seq = raw_in.seq;
            } else {
                consecutive_errors++;
                std::cout << "![WARNING] Packet error detected! count: " << consecutive_errors << std::endl;
            }
            if (consecutive_errors >= MAX_ALLOWED_ERRORS) {
                std::cout << "!!! CRITICAL: Too many consecutive errors, initiating emergency protocols! !!!" << std::endl;
                // 在真实系统中，这里会触发紧急制动等安全措施
                break; // For this simulation, we just exit the loop
            }
        }
        asm("pause");
    }
}

int main() {
    mlockall(MCL_CURRENT | MCL_FUTURE);
    std::cout << "System Launch: T2T Protocol Acceleration Test (" 
              << (my_train_type == TrainType::EMU_DISTRIBUTED ? "EMU" : "LOCO") << ")" << std::endl;

    std::thread comm_task(communication_thread);
    std::thread ctrl_task(control_thread);

    comm_task.join();
    ctrl_task.join();
    return 0;
}