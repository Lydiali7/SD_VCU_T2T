#ifndef CAN_HAL_HPP
#define CAN_HAL_HPP

#include <iostream>
#include <cstring>
#include <memory>
#include <dlfcn.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <fcntl.h>
#include <unistd.h>
#include <algorithm> 
#include "network_proto.hpp" 

// 抽象基类，定义统一的 CAN 接口
class ICanDevice {
public:
    virtual bool init() = 0;
    virtual int receive(CANPacket& pkt) = 0;
    virtual int send(const CANPacket& pkt) = 0;
    virtual ~ICanDevice() = default;
};

// =================================================================
// 1. SocketCAN 实现 (用于纯软件仿真, 如 vcan0)
// =================================================================
class SocketCanDevice : public ICanDevice {
private:
    int sd;
    std::string iface;

public:
    SocketCanDevice(const std::string& interface_name) : sd(-1), iface(interface_name) {}

    bool init() override {
        struct sockaddr_can addr;
        struct ifreq ifr;
        if ((sd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) return false;
        
        int enable_canfd = 1;
        setsockopt(sd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd));
        
        strncpy(ifr.ifr_name, iface.c_str(), IFNAMSIZ - 1);
        ioctl(sd, SIOCGIFINDEX, &ifr);
        memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        
        fcntl(sd, F_SETFL, fcntl(sd, F_GETFL, 0) | O_NONBLOCK);
        
        if (bind(sd, (struct sockaddr *)&addr, sizeof(addr)) < 0) return false;
        
        std::cout << "[CAN_HAL] SocketCAN initialized on virtual interface: " << iface << "\n";
        return true;
    }

    int receive(CANPacket& pkt) override {
        struct canfd_frame frame;
        if (read(sd, &frame, sizeof(frame)) > 0) {
            std::memcpy(&pkt, frame.data, sizeof(CANPacket));
            return 1;
        }
        return 0;
    }

    int send(const CANPacket& pkt) override {
        struct canfd_frame frame;
        memset(&frame, 0, sizeof(frame));
        // 【修正】：完全对齐 network_proto.hpp 中的 source_id
        frame.can_id = pkt.source_id; 
        frame.len = 64; 
        std::memcpy(frame.data, &pkt, sizeof(CANPacket));
        return write(sd, &frame, sizeof(struct canfd_frame));
    }

    ~SocketCanDevice() {
        if (sd >= 0) close(sd);
    }
};

// =================================================================
// 2. 广成 USBCAN 实现 (动态加载 libcontrolcan.so)
// =================================================================
class UsbCanDevice : public ICanDevice {
private:
    // CANalyst-II 厂家标准结构体 (对照 controlcan.h)
    typedef struct _VCI_CAN_OBJ {
        uint32_t ID;
        uint32_t TimeStamp;
        uint8_t  TimeFlag;
        uint8_t  SendType;
        uint8_t  RemoteFlag;
        uint8_t  ExternFlag;
        uint8_t  DataLen;
        uint8_t  Data[8];
        uint8_t  Reserved[3];
    } VCI_CAN_OBJ;

    void* dylib_handle;
    
    typedef uint32_t (*VCI_OpenDevice_Func)(uint32_t, uint32_t, uint32_t);
    typedef uint32_t (*VCI_CloseDevice_Func)(uint32_t, uint32_t);
    typedef uint32_t (*VCI_InitCAN_Func)(uint32_t, uint32_t, uint32_t, void*);
    typedef uint32_t (*VCI_StartCAN_Func)(uint32_t, uint32_t, uint32_t);
    typedef uint32_t (*VCI_Receive_Func)(uint32_t, uint32_t, uint32_t, void*, uint32_t, int);
    typedef uint32_t (*VCI_Transmit_Func)(uint32_t, uint32_t, uint32_t, void*, uint32_t);
    
    VCI_OpenDevice_Func vci_open;
    VCI_CloseDevice_Func vci_close;
    VCI_InitCAN_Func vci_init;
    VCI_StartCAN_Func vci_start;
    VCI_Receive_Func vci_receive;
    VCI_Transmit_Func vci_transmit;

public:
    UsbCanDevice() : dylib_handle(nullptr), vci_open(nullptr), vci_close(nullptr), 
                     vci_init(nullptr), vci_start(nullptr), vci_receive(nullptr), vci_transmit(nullptr) {}

    bool init() override {
        dylib_handle = dlopen("libcontrolcan.so", RTLD_LAZY);
        if (!dylib_handle) {
            dylib_handle = dlopen("./libcontrolcan.so", RTLD_LAZY);
        }
        
        if (!dylib_handle) {
            std::cerr << "[CAN_HAL] Error: Cannot load libcontrolcan.so. Please ensure the library is in the current directory or /usr/lib.\n";
            return false;
        }

        vci_open = (VCI_OpenDevice_Func)dlsym(dylib_handle, "VCI_OpenDevice");
        vci_close = (VCI_CloseDevice_Func)dlsym(dylib_handle, "VCI_CloseDevice");
        vci_init = (VCI_InitCAN_Func)dlsym(dylib_handle, "VCI_InitCAN");
        vci_start = (VCI_StartCAN_Func)dlsym(dylib_handle, "VCI_StartCAN");
        vci_receive = (VCI_Receive_Func)dlsym(dylib_handle, "VCI_Receive");
        vci_transmit = (VCI_Transmit_Func)dlsym(dylib_handle, "VCI_Transmit");

        if (!vci_open || !vci_close || !vci_receive || !vci_transmit) {
            std::cerr << "[CAN_HAL] Error: VCI function mapping failed. The .so file might be incompatible.\n";
            return false;
        }

        if (vci_open(4, 0, 0) != 1) { 
            std::cerr << "[CAN_HAL] Error: Failed to open CANalyst-II. Root privileges (sudo) required!\n";
            return false;
        }
        
        std::cout << "[CAN_HAL] Physical CANalyst-II device opened successfully.\n";
        return true;
    }

    int receive(CANPacket& pkt) override {
        VCI_CAN_OBJ vci_obj;
        if (vci_receive && vci_receive(4, 0, 0, &vci_obj, 1, 0) > 0) {
            std::memcpy(&pkt, vci_obj.Data, std::min(sizeof(CANPacket), (size_t)vci_obj.DataLen));
            // 【修正】：将收到的硬件 ID 赋值给应用层的 source_id
            pkt.source_id = vci_obj.ID; 
            return 1;
        }
        return 0;
    }

    int send(const CANPacket& pkt) override {
        VCI_CAN_OBJ vci_obj;
        memset(&vci_obj, 0, sizeof(VCI_CAN_OBJ));
        
        // 【修正】：完全对齐 network_proto.hpp 中的 source_id
        vci_obj.ID = pkt.source_id; 
        vci_obj.SendType = 0;       
        vci_obj.RemoteFlag = 0;     
        vci_obj.ExternFlag = 1;     
        vci_obj.DataLen = 8;        
        
        std::memcpy(vci_obj.Data, &pkt, 8);

        if (vci_transmit) {
            return vci_transmit(4, 0, 0, &vci_obj, 1);
        }
        return 0;
    }

    ~UsbCanDevice() {
        if (vci_close) {
            vci_close(4, 0);
            std::cout << "[CAN_HAL] Hardware device closed.\n";
        }
        if (dylib_handle) {
            dlclose(dylib_handle);
        }
    }
};

#endif