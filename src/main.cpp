#include <iostream>
#include <thread>
#include <pthread.h>
#include <sys/mman.h>
#include "perception.hpp"
#include "spsc_queue.hpp"

// Global SPSC Queue acting as the "Data Bus" between Core 3 and Core 2
SPSCQueue<SensorData> data_bus(128);

void pin_thread_to_core(int core_id) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core_id, &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
}

// PRODUCER: Mimics high-speed data acquisition on Core 3
/*void communication_thread() {
    pin_thread_to_core(3);
    std::cout << "Communication Thread: Started on Core 3" << std::endl;

    SensorData packet;
    float counter = 0.0f;

    while (true) {
        // Simulate data coming from T2T network
        for(int i=0; i<16; i++) {
            packet.distances[i] = 100.0f + counter;
            if (counter > 5.0f) { packet.distances[0] = 999.9f; } // 运行一段时间后故意制造 A/B 路径差异
        }
        
        if (!data_bus.push(packet)) {
            // Queue full - usually shouldn't happen if consumer is fast enough
        }
        
        counter += 0.1f;
        std::this_thread::sleep_for(std::chrono::milliseconds(2)); // 500Hz
    }
}

void communication_thread() {
    pin_thread_to_core(3);
    SensorData pathA_packet, pathB_packet;
    float counter = 0.0f;

    while (true) {
        // 正常填充数据
        for(int i=0; i<16; i++) {
            pathA_packet.distances[i] = 100.0f + counter;
            pathB_packet.distances[i] = 100.0f + counter;
        }

        // 【故障注入】：每运行到一定程度，让 Path B 产生跳变
        if (counter > 5.0f && counter < 5.2f) { 
            pathB_packet.distances[0] = 999.9f; 
        }
        
        // 我们需要修改 SPSC 队列来传两个包，或者连续 push 两次
        // 为了演示方便，我们直接把 A 和 B 传给校验函数（如果它们在同一个核）
        PerceptionEngine::process_redundant_data(pathA_packet, pathB_packet);
        
        counter += 0.1f;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

// CONSUMER: Runs SIL4 Perception Logic on Core 2
void control_thread() {
    pin_thread_to_core(2);
    std::cout << "Control Thread: Started on Core 2" << std::endl;

    SensorData current_packet;
    while (true) {
        if (data_bus.pop(current_packet)) {
            // Use AVX-512 to process the incoming packet
            // Here we compare it against itself to simulate redundancy check
            PerceptionEngine::process_redundant_data(current_packet, current_packet);
        }
        // Small hint to CPU to reduce power during busy-wait
        asm("pause"); 
    }
}
    */

void communication_thread() {
    pin_thread_to_core(3);
    SensorData packet;
    // 初始化整个数组为 0，防止垃圾值
    for(int i=0; i<16; i++) packet.distances[i] = 0.0f;

    float sim_dist = 50.0f;
    float sim_speed = 33.0f; 

    while (true) {
        packet.distances[0] = sim_dist;
        packet.distances[1] = sim_speed;
        
        // 模拟前车数据（假设前车和本车速度一样，加速度也是 0.8）
        packet.distances[2] = 30.0f; // zhizaozhuiwei
        packet.distances[3] = 0.8f;

        data_bus.push(packet);
        if (sim_dist > 5.0f) sim_dist -= 0.1f;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

// Choose train type for this VCU instance
TrainType my_train_type = TrainType::EMU_DISTRIBUTED; // Change 
void control_thread() {
    pin_thread_to_core(2);

    SensorData current_packet;
    while (true) {
        if (data_bus.pop(current_packet)) {
            // Check safety with the selected train dynamics
            PerceptionEngine::is_system_safe(current_packet, current_packet, my_train_type, true);
        }
        asm("pause");
    }
}

int main() {
    // Tuning: Lock memory to prevent page faults
    mlockall(MCL_CURRENT | MCL_FUTURE);

    std::cout << "System Launch: Test for " << (my_train_type == TrainType::EMU_DISTRIBUTED ? "EMU" : "LOCO") << std::endl;

    std::thread comm_task(communication_thread);
    std::thread ctrl_task(control_thread);

    comm_task.join();
    ctrl_task.join();

    return 0;
}