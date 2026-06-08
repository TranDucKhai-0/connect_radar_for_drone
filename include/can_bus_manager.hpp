#ifndef CANBUSMANAGER_HPP
#define CANBUSMANAGER_HPP

#include "i_data_link.hpp"
#include <string>

class CanBusManager : public IDataLink {
private:
    std::string m_interfaceName;
    int m_socketFd;
    bool m_isConnected;

    // Các hàm private tuân theo quy tắc: _PascalCase
    bool _InitSocket();

public:
    explicit CanBusManager(const std::string& interfaceName);
    ~CanBusManager() override;

    bool Connect() override;
    void Disconnect() override;
    bool IsConnected() const override;

    bool ReadCanFrame(struct can_frame& frame) override;
    bool WriteCanFrame(const struct can_frame& frame) override;
};

#endif // CANBUSMANAGER_HPP
