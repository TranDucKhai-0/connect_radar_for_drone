#ifndef U10_RADAR_HPP
#define U10_RADAR_HPP

#include "i_radar.hpp"
#include <vector>
#include <cstdint>

// Định nghĩa các trạng thái phân tích cú pháp dữ liệu dòng byte
typedef enum
{
    PARSE_STATE_FIND_SYNC,      // Trạng thái tìm kiếm mã đồng bộ Magic Word
    PARSE_STATE_READ_HEADER,    // Trạng thái đọc Header (độ dài gói tin)
    PARSE_STATE_READ_PAYLOAD    // Trạng thái đọc và lưu trữ phần thân dữ liệu
} parseState_t;

class U10Radar : public IRadar
{
public:
    explicit U10Radar(int id = 0, uint32_t canId = 0x123);
    ~U10Radar() override = default;

    // Cấu hình radar (ví dụ: góc yaw bù trừ)
    void Init(float mountingYaw) override;
    
    // Bóc tách dữ liệu từ Frame CAN của radar U10
    bool ParseCanFrame(const struct can_frame &frame, float droneVForward, float droneVRight) override;
    
    // Trả về danh sách tọa độ vật cản tương đối trong hệ tọa độ FRD
    std::vector<obstacleRelative_t> GetObstaclesRelative() const;

    // Trả về tổng số lượng vật cản radar đang nhìn thấy
    int GetObstacleCount() const override;

private:
    int m_id; // ID của radar (1: Front, 2: Right, 3: Back, 4: Left)
    uint32_t m_canId; // CAN ID tương ứng để radar này lọc frame
    float m_mountingYaw; // Góc gắn radar bù trừ (rad)
    std::vector<obstacleRelative_t> m_obstacles; // Danh sách vật cản chính thức của một frame hoàn chỉnh
    
    // Bộ đệm byte tích lũy dữ liệu từ các CAN frame
    std::vector<uint8_t> m_byteBuffer;

    // Máy trạng thái phân tích cú pháp
    parseState_t m_state;
    uint32_t m_expectedTotalLength;

    // Các hàm private phụ trợ (bắt đầu bằng gạch dưới và PascalCase)
    void _ResetParser();
    bool _ParsePacket(const uint8_t* pData, size_t length);
};

#endif // U10_RADAR_HPP