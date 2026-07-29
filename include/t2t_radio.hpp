#pragma once
#include <cerrno>
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
    
    // Serial stream buffer for handling packet fragmentation and concatenation
    std::vector<uint8_t> rx_stream_buffer;

public:
    T2T_Radio() : uart_fd(-1), target_channel(0x17) {}
    
    ~T2T_Radio() {
        if (uart_fd >= 0) close(uart_fd);
    }

    int get_fd() const {
        return uart_fd;
    }

    /**
     * Initialize E22 LoRa module serial port
     * @param dev_node: Device node mapped by OS, e.g., "/dev/ttyWCH0"
     * @param channel: Target frequency channel (must match hardware config)
     */
    bool init(const char* dev_node, int baud_rate = B115200, uint8_t channel = 0x17) {
        target_channel = channel;
        
        // 1. Open serial port in non-blocking read/write mode
        uart_fd = open(dev_node, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (uart_fd < 0) {
            std::cerr << "[LORA_WARN] " << dev_node << " unavailable ("
                      << std::strerror(errno)
                      << "). Safety radio disabled for simulation.\n";
            return false;
        }

        // 2. Configure termios hardware parameters (8N1)
        struct termios options;
        tcgetattr(uart_fd, &options);
        cfsetispeed(&options, baud_rate);
        cfsetospeed(&options, baud_rate);

        options.c_cflag |= (CLOCAL | CREAD | CS8);
        options.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
        
        // Enable Raw Mode, disabling echo and special character processing
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_oflag &= ~OPOST;

        tcsetattr(uart_fd, TCSANOW, &options);
        tcflush(uart_fd, TCIOFLUSH);

        std::cout << "[T2T_Radio] Hardware Initialized on " << dev_node << std::endl;
        return true;
    }

    /**
     * E22 Fixed-point Broadcast Transmission
     * Format: ADDH(FF) + ADDL(FF) + CHAN(target_channel) + DATA
     */
    bool broadcast(const RawT2TPacket& pkt) {
        if (uart_fd < 0) return false;

        TrainBusFrame frame;
        if (!TrainBusFraming::encode_raw_t2t(frame, pkt)) return false;

        uint8_t tx_buf[256];
        tx_buf[0] = 0xFF; // Broadcast target address HIGH
        tx_buf[1] = 0xFF; // Broadcast target address LOW
        tx_buf[2] = target_channel;
        
        std::memcpy(tx_buf + 3, &frame, sizeof(frame));
        int total_len = 3 + sizeof(frame);

        ssize_t sent = write(uart_fd, tx_buf, total_len);
        return sent == total_len;
    }

    /**
     * Serial stream parsing based on a jitter-free sliding window
     */
    bool receive(RawT2TPacket& pkt) {
        if (uart_fd < 0) return false;

        uint8_t temp_buf[256];
        ssize_t len = read(uart_fd, temp_buf, sizeof(temp_buf));
        
        if (len > 0) {
            rx_stream_buffer.insert(rx_stream_buffer.end(), temp_buf, temp_buf + len);
        }

        // Search for header 0x55AA55AA and align payload
        // Using sliding cursor to replace O(N) erase, eliminating memory jitter
        size_t search_idx = 0;
        bool found = false;

        while (rx_stream_buffer.size() - search_idx >= sizeof(TrainBusFrame)) {
            uint16_t sof;
            std::memcpy(&sof, rx_stream_buffer.data() + search_idx, sizeof(sof));

            if (sof == TrainBusFraming::SOF) {
                TrainBusFrame frame;
                std::memcpy(&frame, rx_stream_buffer.data() + search_idx, sizeof(frame));
                
                if (TrainBusFraming::decode_raw_t2t(frame, pkt)) {
                    rx_stream_buffer.erase(rx_stream_buffer.begin(), rx_stream_buffer.begin() + search_idx + sizeof(frame));
                    found = true;
                    break;
                }
                search_idx++;
            } else {
                // Misaligned, advance cursor without moving memory
                search_idx++;
            }
        }

        // Prevent infinite memory growth by clearing confirmed garbage data
        if (!found && search_idx > 0) {
            rx_stream_buffer.erase(rx_stream_buffer.begin(), rx_stream_buffer.begin() + search_idx);
        }

        return found; 
    }
};
