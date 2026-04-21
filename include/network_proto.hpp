#pragma once
#include <cstdint>
#include <cstring>

#define MULTICAST_GROUP "239.0.0.1"
#define MULTICAST_PORT 8888
#define VCU_REPORT_PORT 9000


enum class PlatoonState {
    FOLLOWER,        // 1正常跟车模式 (默认，依赖前车雷达和T2T通信保持间距)
    LEADER_NOMINAL,  // 2默认全局头车 (0号车专用，执行上帝指令)
    DECOUPLING,      // 3正在解编 (断开与前车的跟车约束，加速拉开距离)
    LEADER_NEW       // 4解散完成，成为新车队的领导者 (不再受制于前方车厢)
};

// =================================================================
// UDP (用于 V2V 和 World Server 通信)
// =================================================================
#pragma pack(push, 1)

struct RawT2TPacket {
    uint32_t sender_id; 
    uint32_t header;    // 0x55AA55AA
    uint32_t seq;       
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
};

/**
 * MVB 硬件帧结构
 * 用于 VCU 与 MVB 模块之间的串口通信
 */
struct MVB_Hardware_Frame {
    uint8_t head[2];      // 必须是 0xFE, 0xFA
    uint8_t cmd_type;     // 配置/数据类型，例如 0x05
    uint8_t port_addr;    // MVB 端口地址
    
    // 物理数据区 (从资料 250 字节负载中抽象出的关键字段)
    uint16_t raw_speed;    // 原始速度值 (需除以 100 转换)
    uint16_t brake_press;  // 制动缸压力 (用于模拟空气制动延迟)
    uint32_t io_status;    // 物理 IO 状态位
    
    uint8_t reserved[240]; // 填充至资料要求的 250 字节左右
    uint8_t checksum;      // 校验位
};

/**
 * LoRa 物理封包
 * 用于模拟 E22 模块在“定点模式”下的无线传输格式
 */
struct LoRa_Physical_Packet {
    uint16_t target_addr;  // 目标模块地址 (高 8 位 + 低 8 位)
    uint8_t channel;       // 通信频率信道
    
    // 负载部分：通常嵌套 RawT2TPacket 或控制指令
    uint8_t payload[128];  
};

#pragma pack(pop)

// 64字节对齐的 CAN 包结构
struct alignas(64) CANPacket {
    uint32_t header;       // 魔数 0xAA55
    uint16_t source_id;    // 传感器节点地址
    uint16_t seq_num;      // 防重放攻击序列号
    float payload[12];     // [0]=障碍物距离(Gap), [1]=本车速度, [2]=前车速度
    uint32_t crc32;        // 硬件校验和
    uint32_t padding;      // 补齐 64 字节
};

struct DualSensorBroadcast {
    uint16_t target_train_id; // 目标车厢 ID
    CANPacket path_a;         // 冗余物理线路 A
    CANPacket path_b;         // 冗余物理线路 B
};

// 数据转换
class ProtocolConverter {
public:
    // 将物理世界模拟的速度转换为 MVB 硬件帧格式
    static void encode_mvb(MVB_Hardware_Frame& frame, double speed, double pressure) {
        frame.head[0] = 0xFE;
        frame.head[1] = 0xFA;
        frame.cmd_type = 0x05;
        frame.raw_speed = static_cast<uint16_t>(speed * 100.0);
        frame.brake_press = static_cast<uint16_t>(pressure * 10.0);
    }

    // 从 MVB 原始字节流中提取数据 (用于 vcu_node 的感知)
    static bool validate_mvb(const uint8_t* buffer, size_t len) {
        if (len < sizeof(MVB_Hardware_Frame)) return false;
        return (buffer[0] == 0xFE && buffer[1] == 0xFA);
    }
};