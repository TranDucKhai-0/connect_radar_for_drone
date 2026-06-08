#include "mr72_radar.hpp"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

MR72Radar::MR72Radar(int id)
    : m_id(id), m_mountingYaw(0.0f), m_expectedObstacles(0)
{
    m_obstacles.reserve(64);
    m_tempObstacles.reserve(64);
}

void MR72Radar::Init(float mountingYaw)
{
    m_mountingYaw = mountingYaw;
}

bool MR72Radar::ParseCanFrame(const struct can_frame &frame, float droneVForward, float droneVRight)
{
    // Triết lý: Nhận 1 khung hình điểm vật cản (0x60B) là xử lý và gửi đi luôn, không chờ gói Status (0x60A)
    if (frame.can_id == (MR72_OBJECT_LIST_STATUS + m_id * 0x10))
    {
        // Bỏ qua gói báo chu kỳ, vì ta xử lý real-time trên từng điểm
        return false; 
    }
    else if (frame.can_id == (MR72_OBJECT_GENERAL_INFO + m_id * 0x10))
    {
        obstacle_relative_t obs;
        obs.id = frame.data[0];

        uint16_t distLongRaw = (frame.data[1] << 5) | (frame.data[2] >> 3);
        uint16_t distLatRaw = ((frame.data[2] & 0x7) << 8) | frame.data[3];
        uint16_t vrelLongRaw = (frame.data[4] << 2) | ((frame.data[5] >> 6) & 0x3);
        uint16_t vrelLatRaw = (frame.data[5] << 3) | ((frame.data[6] >> 5) & 0x7);

        uint8_t sectorNumber = (frame.data[6] >> 3) & 0x3;

        // Chỉ lọc lấy những vật thể đã được xác nhận chắc chắn (Measured == 0x02)
        // (Sau này có thể mở rộng lấy 0x01, 0x03 tại đây)
        if (sectorNumber != 0x02)
        {
            return false;
        }

        // Chuyển đổi từ Raw Value sang giá trị vật lý
        obs.y = distLatRaw * 0.2f - 204.6f;     // Chuyển sang hệ tọa độ FRD (Y=Right)
        obs.x = distLongRaw * 0.2f - 500.0f;    // X=Forward
        obs.z = 0.0f;                           // MR72 không cung cấp thông tin độ cao, giả định z=0
        obs.v_x = vrelLongRaw * 0.25f - 128.0f; // Vận tốc dọc trục x (Forward)
        obs.v_y = vrelLatRaw * 0.25f - 64.0f;   // Vận tốc dọc trục y (Right)
        obs.v_z = 0.0f;                         // Giả định v_z=0

        // Lưu trực tiếp vào m_obstacles (1 điểm duy nhất) và báo true để gửi đi luôn
        m_obstacles.clear();
        m_obstacles.push_back(obs);
        
        return true; 
    }

    return false;
}

std::vector<obstacle_relative_t> MR72Radar::GetObstaclesRelative() const
{
    return m_obstacles;
}

int MR72Radar::GetObstacleCount() const
{
    return m_obstacles.size();
}
