#ifndef MVB_HAL_HPP
#define MVB_HAL_HPP

#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <vector>
#include <cstdint>

class MVB_Device {
private:
    int fd;
    bool is_initialized;

public:
    MVB_Device() : fd(-1), is_initialized(false) {}

    bool init(const char* port_name) {
        fd = open(port_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd < 0) {
            std::cerr << "[MVB_HAL] Error: Failed to open port " << port_name << "\n";
            return false;
        }

        struct termios options;
        tcgetattr(fd, &options);
        cfsetispeed(&options, B115200);
        cfsetospeed(&options, B115200);
        options.c_cflag |= (CLOCAL | CREAD | CS8);
        options.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_oflag &= ~OPOST;
        tcsetattr(fd, TCSANOW, &options);

        if (!cold_boot()) {
            std::cerr << "[MVB_HAL] Error: Cold boot sequence failed.\n";
            close(fd);
            fd = -1;
            return false;
        }

        is_initialized = true;
        std::cout << "[MVB_HAL] Initialization and cold boot successful on " << port_name << "\n";
        return true;
    }

    bool cold_boot() {
        std::cout << "[MVB_HAL] Initiating hardware cold boot sequence...\n";
        
        // Configuration payload (264 bytes) derived from MVB test commands
        uint8_t config_seq[264] = {0};
        // Line 1
        uint8_t l1[] = {0xFE, 0xFA, 0x05, 0x00, 0x71, 0x05, 0x07, 0x10, 0x04, 0x00, 0x07, 0x20, 0x04, 0x00, 0x07, 0x30, 0x04, 0x00, 0x07, 0x40, 0x04, 0x00, 0x07, 0x50, 0x04, 0x00};
        std::memcpy(config_seq, l1, sizeof(l1));
        // Line 2
        uint8_t l2[] = {0x05, 0x07, 0x18, 0x04, 0x00, 0x07, 0x28, 0x04, 0x00, 0x07, 0x38, 0x04, 0x00, 0x07, 0x48, 0x04, 0x00, 0x07, 0x58, 0x04, 0x00};
        std::memcpy(config_seq + 88 + 26, l2, sizeof(l2));
        // Line 3 tail
        config_seq[261] = 0xA0; config_seq[262] = 0xE4; config_seq[263] = 0xFF;

        std::cout << "[MVB_HAL] Sending configuration payload...\n";
        if (write(fd, config_seq, sizeof(config_seq)) != sizeof(config_seq)) return false;
        usleep(300000); 

        // Enable periodic upload payload
        uint8_t enable_seq[] = {0xFE, 0x08, 0x07, 0x20, 0x01, 0xE6, 0x6A, 0xFF};
        std::cout << "[MVB_HAL] Sending periodic upload enable command...\n";
        if (write(fd, enable_seq, sizeof(enable_seq)) != sizeof(enable_seq)) return false;
        usleep(100000);

        tcflush(fd, TCIOFLUSH);
        return true; 
    }

    int read_frame(void* buffer, size_t size) {
        if (!is_initialized) return -1;
        return read(fd, buffer, size);
    }

    int write_frame(const void* buffer, size_t size) {
        if (!is_initialized) return -1;
        return write(fd, buffer, size);
    }

    ~MVB_Device() {
        if (fd >= 0) close(fd);
    }
};

#endif