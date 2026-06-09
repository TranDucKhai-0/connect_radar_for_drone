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

    void Init(float mountingYaw) override;
    
    // ParseCanFrame giờ đây không nhận vận tốc drone nữa vì nó tính relative thôi
    bool ParseCanFrame(const struct can_frame &frame, float droneVForward, float droneVRight) override;
    
    std::vector<obstacleRelative_t> GetObstaclesRelative() const;

    int GetObstacleCount() const override;

private:
    int m_id;
    float m_mountingYaw;
    int m_expectedObstacles;
    std::vector<obstacleRelative_t> m_tempObstacles;
    std::vector<obstacleRelative_t> m_obstacles;
};

#endif // MR72RADAR_HPP
