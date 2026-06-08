#ifndef MR72RADAR_HPP
#define MR72RADAR_HPP

#include "IRadar.hpp"
#include <vector>
#include <cstdint>

// Hằng số CAN ID của MR72 (từ logic cũ)
constexpr uint32_t MR72_MESSAGE_CONFIG = 0x200UL;
constexpr uint32_t MR72_RADAR_STATUS   = 0x201UL;
constexpr uint32_t MR72_VERSION_INFO   = 0x700UL;
constexpr uint32_t MR72_OBJECT_LIST_STATUS = 0x60AUL;
constexpr uint32_t MR72_OBJECT_GENERAL_INFO = 0x60BUL;

class MR72Radar : public IRadar {
private:
    int m_id;
    float m_mountingYaw;
    std::vector<obstacleData_t> m_obstacles;
    std::vector<obstacleData_t> m_tempObstacles; // Buffer tạm trong chu kỳ nhận
    int m_expectedObstacles; // Số điểm dự kiến nhận được từ gói 0x60A

public:
    explicit MR72Radar(int id = 0);
    ~MR72Radar() override = default;

    void Init(float mountingYaw) override;
    bool ParseCanFrame(const struct can_frame& frame, float droneVForward, float droneVRight) override;
    std::vector<obstacleData_t> GetObstacles() const override;
    int GetObstacleCount() const override;
};

#endif // MR72RADAR_HPP
