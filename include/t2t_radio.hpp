#pragma once
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <vector>
#include "network_proto.hpp"

class T2T_Radio {
private:
    int uart_fd;
    uint8_t target_channel;
    
    // 串口流缓冲区，用于处理粘包和断包
    std::vector<uint8_t> rx_stream_buffer;

public:
    T2T_Radio() : uart_fd(-1), target_channel(0x17) {}
    
    ~T2T_Radio() {
        if (uart_fd >= 0) close(uart_fd);
    }

    /**
     * 初始化 E22 LoRa 模块串口
     * @param dev_node: 驱动生成的节点，如 "/dev/ttyWCH0"
     * @param channel: 目标通信信道 (需与硬件配置一致)
     */
    bool init(const char* dev_node, int baud_rate = B115200, uint8_t channel = 0x17) {
        target_channel = channel;
        
        // 1. 以非阻塞读写模式打开串口
        uart_fd = open(dev_node, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (uart_fd < 0) {
            perror("[T2T_Radio] Open Serial Failed");
            return false;
        }

        // 2. 配置 termios 硬件参数 (8N1)
        struct termios options;
        tcgetattr(uart_fd, &options);
        cfsetispeed(&options, baud_rate);
        cfsetospeed(&options, baud_rate);

        options.c_cflag |= (CLOCAL | CREAD | CS8);
        options.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
        
        // 设置为 Raw Mode (原始透传)，禁止回显和特殊字符处理
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_oflag &= ~OPOST;

        tcsetattr(uart_fd, TCSANOW, &options);
        tcflush(uart_fd, TCIOFLUSH);

        std::cout << "[T2T_Radio] Hardware Initialized on " << dev_node << std::endl;
        return true;
    }

    /**
     * E22 定点广播发射
     * 格式: ADDH(FF) + ADDL(FF) + CHAN(target_channel) + DATA
     */
    bool broadcast(const RawT2TPacket& pkt) {
        if (uart_fd < 0) return false;

        uint8_t tx_buf[128];
        tx_buf[0] = 0xFF; // 广播目标地址高位
        tx_buf[1] = 0xFF; // 广播目标地址低位
        tx_buf[2] = target_channel;
        
        std::memcpy(tx_buf + 3, &pkt, sizeof(RawT2TPacket));
        int total_len = 3 + sizeof(RawT2TPacket);

        ssize_t sent = write(uart_fd, tx_buf, total_len);
        return sent == total_len;
    }

    /**
     * 基于滑动窗口的串口流数据解析
     */
    bool receive(RawT2TPacket& pkt) {
        if (uart_fd < 0) return false;

        uint8_t temp_buf[256];
        ssize_t len = read(uart_fd, temp_buf, sizeof(temp_buf));
        
        if (len > 0) {
            rx_stream_buffer.insert(rx_stream_buffer.end(), temp_buf, temp_buf + len);
        }

        // 寻找帧头 0x55AA55AA 并对齐
        while (rx_stream_buffer.size() >= sizeof(RawT2TPacket)) {
            // 快速扫描帧头
            uint32_t header;
            std::memcpy(&header, rx_stream_buffer.data() + 4, 4); // 偏移4字节看 header 字段

            if (header == 0x55AA55AA) {
                // 校验通过，提取全包
                std::memcpy(&pkt, rx_stream_buffer.data(), sizeof(RawT2TPacket));
                rx_stream_buffer.erase(rx_stream_buffer.begin(), rx_stream_buffer.begin() + sizeof(RawT2TPacket));
                return true;
            } else {
                // 没对齐，弹出首字节继续寻找
                rx_stream_buffer.erase(rx_stream_buffer.begin());
            }
        }
        return false;
    }
};