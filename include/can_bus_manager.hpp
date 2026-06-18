#ifndef CANBUSMANAGER_HPP
#define CANBUSMANAGER_HPP

#include "i_data_link.hpp"
#include <string>

class CanBusManager : public IDataLink {
private:
    std::string m_interfaceName; // Tên interface CAN (ví dụ: "can0")
    int m_socketFd;              // File descriptor của socket kết nối CAN
    bool m_isConnected;          // Cờ trạng thái kết nối

    // Các hàm private tuân theo quy tắc: thêm tiền tố _ cho hàm private
    // Khởi tạo socket RAW CAN ở mức hệ thống
    bool _InitSocket();

public:
    explicit CanBusManager(const std::string& interfaceName);
    ~CanBusManager() override;

    // Giao tiếp kết nối
    bool Connect() override;
    void Disconnect() override;
    bool IsConnected() const override;

    // Các hàm đọc ghi dữ liệu CAN frame
    bool ReadCanFrame(struct can_frame& frame) override;
    bool WriteCanFrame(const struct can_frame& frame) override;
};

#endif // CANBUSMANAGER_HPP
