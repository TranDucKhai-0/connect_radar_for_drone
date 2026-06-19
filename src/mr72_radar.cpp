#include "mr72_radar.hpp"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

MR72Radar::MR72Radar(int id)
    : m_id(id), m_mountingYaw(0.0f), m_expectedObstacles(0)
{
    // Dành trước không gian trong bộ nhớ để tránh cấp phát lại động liên tục
    // MR72 tối đa hỗ trợ 64 điểm ảnh trong 1 frame
    m_obstacles.reserve(64);
    m_tempObstacles.reserve(64);
}

// Khởi tạo thông số (góc lắp đặt yaw nếu cần xoay toạ độ)
void MR72Radar::Init(float mountingYaw)
{
    m_mountingYaw = mountingYaw;
}

// Phân tích cú pháp khung dữ liệu CAN của MR72 Radar
bool MR72Radar::ParseCanFrame(const struct can_frame &frame, float droneVForward, float droneVRight)
{
    // Bỏ qua gói STATUS (0x60A), xử lý real-time trên từng gói object (0x60B)
    if (frame.can_id == (MR72_OBJECT_LIST_STATUS + m_id * 0x10))
    {
        return false; // Không cần xử lý gì, chỉ dùng để đồng bộ khi nào có gói object mới sẽ đọc dữ liệu
    }
    // Gói dữ liệu vật thể chung (0x60B)
    else if (frame.can_id == (MR72_OBJECT_GENERAL_INFO + m_id * 0x10))
    {
        uint8_t sectorNumber = (frame.data[6] >> 3) & 0x03;

        // Theo protocol MR72, (data[6] >> 3) & 0x3 là Sector Number.
        // Sector 1 là khu vực chính giữa, Sector 2 và 3 là hai bên.
        // Chỉ lấy vật thể ở Sector 1 (chính giữa) để giảm nhiễu 2 bên
        if (sectorNumber != 0x02) return false;

        obstacleRelative_t obs;
        obs.id = frame.data[0]; // Giữ nguyên object ID gốc do radar gán, GCS dùng để phân biệt các điểm

        // Cấu trúc Byte của thông tin chung vật thể (theo datasheet MR72)
        // Dịch bit để gom lại thành raw value (13 bit, 11 bit...)
        uint16_t distLongRaw = (frame.data[1] << 5) | (frame.data[2] >> 3);
        uint16_t distLatRaw  = ((frame.data[2] & 0x7) << 8) | frame.data[3];
        uint16_t vrelLongRaw = (frame.data[4] << 2) | ((frame.data[5] >> 6) & 0x3);
        uint16_t vrelLatRaw  = ((frame.data[5] & 0x3F) << 3) | ((frame.data[6] >> 5) & 0x7);

        // Chuyển đổi sang đơn vị chuẩn hệ mét (meter, m/s)
        obs.x   = distLongRaw * 0.2f - 500.0f;    // X = Khoảng cách dọc (Forward) (m)
        obs.y   = distLatRaw  * 0.2f - 204.6f;    // Y = Khoảng cách ngang (Right) (m)
        obs.z   = 0.0f;                            // MR72 là radar 2D, không đo độ cao Z
        obs.vx = vrelLongRaw * 0.25f - 128.0f;   // Vận tốc tương đối dọc trục X (m/s)
        obs.vy = vrelLatRaw  * 0.25f - 64.0f;    // Vận tốc tương đối dọc trục Y (m/s)
        obs.vz = 0.0f;

        // x,y,z -> range,angle
        obs.angle = std::atan2(obs.y, obs.x); // rad
        obs.range = std::sqrtf(obs.x * obs.x + obs.y * obs.y); // m

        // lọc bỏ data rác
        if (obs.range < 2.0f || obs.range > 40.0f) return false;
        
        // Lọc khoảng cách radar ID 2 và 4 trong phạm vi 2.0m - 20.0m
        if ((m_id == 2 || m_id == 4) && (obs.range < 2.0f || obs.range > 20.0f)) return false;

        // Chuyển đổi từ range, angle ngược lại x, y, z
        obs.x = obs.range * std::cosf(obs.angle);
        obs.y = obs.range * std::sinf(obs.angle);
        obs.z = 0.0f;

        m_obstacles.clear();
        m_obstacles.push_back(obs); // Ghi nhận vật cản
        return true;
    }

    return false; // Không thuộc gói tin nào cần thiết
}

// Lấy danh sách vật cản (Toạ độ tương đối)
std::vector<obstacleRelative_t> MR72Radar::GetObstaclesRelative() const
{
    return m_obstacles;
}

// Lấy số lượng điểm ảnh vật cản hiện có
int MR72Radar::GetObstacleCount() const
{
    return m_obstacles.size();
}
