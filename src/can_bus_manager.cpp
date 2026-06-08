#include "can_bus_manager.hpp"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>

CanBusManager::CanBusManager(const std::string& interfaceName)
    : m_interfaceName(interfaceName), m_socketFd(-1), m_isConnected(false) {}

CanBusManager::~CanBusManager() {
    Disconnect();
}

bool CanBusManager::_InitSocket() {
    struct sockaddr_can addr;
    struct ifreq ifr;

    m_socketFd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (m_socketFd < 0) {
        std::cerr << "Error while opening socket\n";
        return false;
    }

    std::strcpy(ifr.ifr_name, m_interfaceName.c_str());
    ioctl(m_socketFd, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(m_socketFd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        std::cerr << "Error in socket bind\n";
        close(m_socketFd);
        m_socketFd = -1;
        return false;
    }

    return true;
}

bool CanBusManager::Connect() {
    if (m_isConnected) return true;
    
    if (_InitSocket()) {
        m_isConnected = true;
        std::cout << "Connected to CAN interface: " << m_interfaceName << "\n";
        return true;
    }
    return false;
}

void CanBusManager::Disconnect() {
    if (m_socketFd >= 0) {
        close(m_socketFd);
        m_socketFd = -1;
    }
    m_isConnected = false;
}

bool CanBusManager::IsConnected() const {
    return m_isConnected;
}

bool CanBusManager::ReadCanFrame(struct can_frame& frame) {
    if (!m_isConnected) return false;
    
    int nbytes = read(m_socketFd, &frame, sizeof(struct can_frame));
    if (nbytes < 0) {
        return false;
    }
    return true;
}

bool CanBusManager::WriteCanFrame(const struct can_frame& frame) {
    if (!m_isConnected) return false;

    int nbytes = write(m_socketFd, &frame, sizeof(struct can_frame));
    if (nbytes != sizeof(struct can_frame)) {
        return false;
    }
    return true;
}
