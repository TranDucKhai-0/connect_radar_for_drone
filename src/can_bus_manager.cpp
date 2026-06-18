#include "can_bus_manager.hpp"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>

CanBusManager::CanBusManager(const std::string& interfaceName)
    : m_interfaceName(interfaceName), m_socketFd(-1), m_isConnected(false) {} // Khởi tạo biến interface và đặt fd bằng -1

CanBusManager::~CanBusManager() {
    Disconnect(); // Đóng socket khi hủy đối tượng
}

// Khởi tạo và liên kết socket CAN
bool CanBusManager::_InitSocket() {
    struct sockaddr_can addr;
    struct ifreq ifr;

    // Tạo socket chuẩn RAW CAN
    m_socketFd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (m_socketFd < 0) {
        std::cerr << "Error while opening socket\n";
        return false;
    }

    // Lấy chỉ số index của interface CAN (VD: can0)
    std::strcpy(ifr.ifr_name, m_interfaceName.c_str());
    ioctl(m_socketFd, SIOCGIFINDEX, &ifr);

    // Chuẩn bị địa chỉ để bind socket
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    // Gắn kết (bind) socket với interface đã cho
    if (bind(m_socketFd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        std::cerr << "Error in socket bind\n";
        close(m_socketFd);
        m_socketFd = -1;
        return false;
    }

    return true; // Khởi tạo thành công
}

// Kết nối đến CAN bus
bool CanBusManager::Connect() {
    if (m_isConnected) return true; // Đã kết nối thì không làm gì thêm
    
    // Thử khởi tạo socket
    if (_InitSocket()) {
        m_isConnected = true;
        std::cout << "Connected to CAN interface: " << m_interfaceName << "\n";
        return true;
    }
    return false;
}

// Ngắt kết nối và đóng file descriptor
void CanBusManager::Disconnect() {
    if (m_socketFd >= 0) {
        close(m_socketFd);
        m_socketFd = -1;
    }
    m_isConnected = false;
}

// Kiểm tra trạng thái kết nối
bool CanBusManager::IsConnected() const {
    return m_isConnected;
}

// Đọc một khung (frame) từ CAN bus
bool CanBusManager::ReadCanFrame(struct can_frame& frame) {
    if (!m_isConnected) return false;
    
    // Sử dụng hàm read() của hệ thống để nhận dữ liệu từ socket
    int nbytes = read(m_socketFd, &frame, sizeof(struct can_frame));
    if (nbytes < 0) {
        return false; // Lỗi đọc
    }
    return true;
}

// Ghi một khung (frame) lên CAN bus
bool CanBusManager::WriteCanFrame(const struct can_frame& frame) {
    if (!m_isConnected) return false;

    // Sử dụng hàm write() của hệ thống để đẩy dữ liệu lên socket
    int nbytes = write(m_socketFd, &frame, sizeof(struct can_frame));
    if (nbytes != sizeof(struct can_frame)) {
        return false; // Lỗi ghi không đủ byte
    }
    return true;
}
