#ifndef MR72RADAR_HPP
#define MR72RADAR_HPP

#include "i_radar.hpp"
#include <vector>
#include <cstdint>

constexpr uint32_t MR72_OBJECT_LIST_STATUS = 0x60AUL;
constexpr uint32_t MR72_OBJECT_GENERAL_INFO = 0x60BUL;

class MR72Radar : public IRadar
{
public:
    explicit MR72Radar(int id = 0);
    ~MR72Radar() override = default;

    // Cấu hình radar (ví dụ: góc yaw bù trừ)
    void Init(float mountingYaw) override;
    
    // ParseCanFrame giờ đây không nhận vận tốc drone nữa vì nó tính relative thôi
    // Bóc tách dữ liệu từ Frame CAN của radar MR72
    bool ParseCanFrame(const struct can_frame &frame, float droneVForward, float droneVRight) override;
    
    // Trả về danh sách tọa độ vật cản tương đối
    std::vector<obstacleRelative_t> GetObstaclesRelative() const;

    // Trả về tổng số lượng vật cản radar đang nhìn thấy
    int GetObstacleCount() const override;

private:
    int m_id; // ID của radar (để phân biệt khi có nhiều radar)
    float m_mountingYaw; // Góc gắn radar bù trừ (rad)
    int m_expectedObstacles; // Số lượng vật cản dự kiến (dành cho logic ghép frame)
    std::vector<obstacleRelative_t> m_tempObstacles; // Bộ đệm tạm thời chứa vật cản trong quá trình parse
    std::vector<obstacleRelative_t> m_obstacles; // Danh sách vật cản chính thức của một frame hoàn chỉnh
};

#endif // MR72RADAR_HPP
