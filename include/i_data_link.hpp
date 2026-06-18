#ifndef IDATALINK_HPP
#define IDATALINK_HPP

#include <vector>
#include <cstdint>
#include <linux/can.h>

// ---------------------------------------------------------
// Interface (Giao diện) định nghĩa chuẩn giao tiếp liên kết dữ liệu
// Các lớp như CanBusManager, UartManager (nếu có) sẽ kế thừa từ đây
// ---------------------------------------------------------
class IDataLink {
public:
    virtual ~IDataLink() = default;

    // Các hàm giao tiếp cơ bản
    virtual bool Connect() = 0; // Khởi tạo và mở kết nối
    virtual void Disconnect() = 0; // Đóng kết nối
    virtual bool IsConnected() const = 0; // Kiểm tra trạng thái kết nối
    
    // Đọc frame CAN chuẩn vì radar hiện tại dùng SocketCAN
    virtual bool ReadCanFrame(struct can_frame& frame) = 0;
    
    // Ghi frame CAN xuống đường truyền
    virtual bool WriteCanFrame(const struct can_frame& frame) = 0;
};

#endif // IDATALINK_HPP
