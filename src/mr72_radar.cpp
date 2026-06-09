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
    // Bỏ qua gói STATUS (0x60A), xử lý real-time trên từng gói object (0x60B)
    if (frame.can_id == (MR72_OBJECT_LIST_STATUS + m_id * 0x10))
    {
        return false; // Không cần xử lý gì, chỉ dùng để đồng bộ khi nào có gói object mới sẽ đọc dữ liệu
    }
    else if (frame.can_id == (MR72_OBJECT_GENERAL_INFO + m_id * 0x10))
    {
        uint8_t sectorNumber = (frame.data[6] >> 3) & 0x3;

        // Chỉ lấy vật thể đã được xác nhận (Measured == 0x02)
        if (sectorNumber != 0x02) return false;

        obstacleRelative_t obs;
        obs.id = frame.data[0]; // Giữ nguyên object ID gốc do radar gán, GCS dùng để phân biệt các điểm

        uint16_t distLongRaw = (frame.data[1] << 5) | (frame.data[2] >> 3);
        uint16_t distLatRaw  = ((frame.data[2] & 0x7) << 8) | frame.data[3];
        uint16_t vrelLongRaw = (frame.data[4] << 2) | ((frame.data[5] >> 6) & 0x3);
        uint16_t vrelLatRaw  = (frame.data[5] << 3) | ((frame.data[6] >> 5) & 0x7);

        obs.x   = distLongRaw * 0.2f - 500.0f;    // X = Forward (m)
        obs.y   = distLatRaw  * 0.2f - 204.6f;    // Y = Right   (m)
        obs.z   = 0.0f;                            // MR72 không đo độ cao
        obs.vx = vrelLongRaw * 0.25f - 128.0f;   // Vận tốc dọc trục X (m/s)
        obs.vy = vrelLatRaw  * 0.25f - 64.0f;    // Vận tốc dọc trục Y (m/s)
        obs.vz = 0.0f;

        m_obstacles.clear();
        m_obstacles.push_back(obs);
        return true;
    }

    return false;
}

std::vector<obstacleRelative_t> MR72Radar::GetObstaclesRelative() const
{
    return m_obstacles;
}

int MR72Radar::GetObstacleCount() const
{
    return m_obstacles.size();
}
