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
    // Gói tin 0x60A đánh dấu điểm bắt đầu chu kỳ truyền mới
    if (frame.can_id == (MR72_OBJECT_LIST_STATUS + m_id * 0x10))
    {
        // Ghi nhận chu kỳ trước đó đã xong
        m_obstacles = m_tempObstacles;
        m_tempObstacles.clear();

        m_expectedObstacles = frame.data[0];

        return true; // Trả về true báo hiệu kết thúc 1 frame của radar (để có thể lấy dữ liệu)
    }
    // Các gói tin điểm vật cản 0x60B
    else if (frame.can_id == (MR72_OBJECT_GENERAL_INFO + m_id * 0x10))
    {
        if (m_tempObstacles.size() >= 64)
        {
            return false; // Tránh tràn buffer
        }

        obstacle_relative_t obs;
        obs.id = frame.data[0];

        uint16_t distLongRaw = (frame.data[1] << 5) | (frame.data[2] >> 3);
        uint16_t distLatRaw = ((frame.data[2] & 0x7) << 8) | frame.data[3];
        uint16_t vrelLongRaw = (frame.data[4] << 2) | ((frame.data[5] >> 6) & 0x3);
        uint16_t vrelLatRaw = (frame.data[5] << 3) | ((frame.data[6] >> 5) & 0x7);

        uint8_t sectorNumber = (frame.data[6] >> 3) & 0x3;

        // Chỉ lọc lấy những vật thể đã được xác nhận chắc chắn (Measured == 0x02)
        if (sectorNumber != 0x02)
        {
            return false;
        }

        // Chuyển đổi từ Raw Value sang giá trị vật lý
        obs.y = distLatRaw * 0.2f - 204.6f;     // Chuyển sang hệ tọa độ FRD (Y=Right)
        obs.x = distLongRaw * 0.2f - 500.0f;    // X=Forward
        obs.z = 0.0f;                           // MR72 không cung cấp thông tin độ cao, giả định z=0
        obs.v_x = vrelLongRaw * 0.25f - 128.0f; // Vận tốc dọc trục x (Forward), bù trừ vận tốc drone sẽ được xử lý ở luồng DataProcessingThread
        obs.v_y = vrelLatRaw * 0.25f - 64.0f;   // Vận tốc dọc trục y (Right), bù trừ vận tốc drone sẽ được xử lý ở luồng DataProcessingThread
        obs.v_z = 0.0f;                         // MR72 không cung cấp thông tin vận tốc dọc trục z, giả định v_z=0

        m_tempObstacles.push_back(obs);
    }

    // Nếu đã nhận đủ số lượng object_count báo từ gói list_status
    if (m_tempObstacles.size() >= static_cast<size_t>(m_expectedObstacles) && m_expectedObstacles > 0)
    {
        m_obstacles = m_tempObstacles;
        m_tempObstacles.clear();
        return true; // Báo hiệu đã xong 1 chu kỳ quét
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
