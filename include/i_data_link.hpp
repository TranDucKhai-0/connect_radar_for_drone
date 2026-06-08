#ifndef IDATALINK_HPP
#define IDATALINK_HPP

#include <vector>
#include <cstdint>
#include <linux/can.h>

class IDataLink {
public:
    virtual ~IDataLink() = default;

    virtual bool Connect() = 0;
    virtual void Disconnect() = 0;
    virtual bool IsConnected() const = 0;
    
    // Đọc frame CAN chuẩn vì radar hiện tại dùng SocketCAN
    virtual bool ReadCanFrame(struct can_frame& frame) = 0;
    
    // Ghi frame CAN
    virtual bool WriteCanFrame(const struct can_frame& frame) = 0;
};

#endif // IDATALINK_HPP
