#include "mr72_radar.hpp"
#include <cmath>
#include <chrono>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define ANGLE_LIMIT_FOV_RAD 0.3490658F // 20 độ (rad) - Giới hạn FOV của radar MR72 (±20 độ quanh trục chính giữa)

#define RANGE_LIMIT_MIN 2.0F
#define RANGE_LIMIT_MAX 40.0F

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
    static std::chrono::steady_clock::time_point lastTimes[5] = {};
    bool isFrameParsed = false;

    // Gói trạng thái danh sách vật thể (0x60A) - Bắt đầu chu kỳ đo mới
    if (frame.can_id == (MR72_OBJECT_LIST_STATUS + m_id * 0x10))
    {
        // 1. Kiểm tra xem có dữ liệu của chu kỳ trước chưa được gửi đi hoàn chỉnh
        if (!m_tempObstacles.empty())
        {
            m_obstacles = std::move(m_tempObstacles);
            m_tempObstacles.clear();
            isFrameParsed = true;
        }
        return isFrameParsed;
    }
    // Gói dữ liệu vật thể chung (0x60B)
    else if (frame.can_id == (MR72_OBJECT_GENERAL_INFO + m_id * 0x10))
    {
        auto now = std::chrono::steady_clock::now();
        
        // Tính toán khoảng thời gian từ lần nhận gói 0x60B trước đó
        auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTimes[m_id]).count();
        lastTimes[m_id] = now;

        // Nếu khoảng thời gian > 20ms, chứng tỏ đây là một chu kỳ mới
        // (Dùng làm cơ chế dự phòng cực kỳ robust trong trường hợp gói 0x60A bị mất hoặc không được gửi)
        if (dt > 20 && !m_tempObstacles.empty())
        {
            m_obstacles = std::move(m_tempObstacles);
            m_tempObstacles.clear();
            isFrameParsed = true;
        }

        uint8_t sectorNumber = (frame.data[6] >> 3) & 0x03;
        obstacleRelative_t obs;
        obs.id = frame.data[0]; // Giữ nguyên object ID gốc do radar gán

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
        obs.vx  = vrelLongRaw * 0.25f - 128.0f;   // Vận tốc tương đối dọc trục X (m/s)
        obs.vy  = vrelLatRaw  * 0.25f - 64.0f;    // Vận tốc tương đối dọc trục Y (m/s)
        obs.vz  = 0.0f;

        // Tính toán khoảng cách (range) và góc (angle) tương đối trong hệ FRD
        obs.angle = atan2f(obs.y, obs.x); // rad
        obs.range = sqrtf(obs.x * obs.x + obs.y * obs.y); // m

        // Theo tài liệu kỹ thuật của MR72, Sector 2 là khu vực chính giữa (Sector 1 bên trái, Sector 3 bên phải).
        // Chỉ lấy vật thể ở Sector 2 (chính giữa) để giảm nhiễu từ hai bên sườn.
        bool isValid = true;
        if (sectorNumber != 0x02)
        {
            isValid = false;
        }
        else
        {
            // Lọc khoảng cách tùy thuộc vào vị trí lắp đặt của radar (m_id)
            if (m_id == 1 || m_id == 3) // Front, Back (2m - 40m)
            {
                if (obs.range < RANGE_LIMIT_MIN || obs.range > RANGE_LIMIT_MAX)
                {
                    isValid = false;
                }
            }
            else if (m_id == 2 || m_id == 4) // Right, Left (2m - 20m)
            {
                if (obs.range < RANGE_LIMIT_MIN || obs.range > RANGE_LIMIT_MAX/2)
                {
                    isValid = false;
                }
            }
            else
            {
                // Tránh trường hợp m_id bất thường
                if (obs.range < RANGE_LIMIT_MIN || obs.range > RANGE_LIMIT_MAX)
                {
                    isValid = false;
                }
            }
        }

        // Lọc theo góc FOV của từng radar (m_id)
        if(m_id == 1 && std::abs(obs.angle) > ANGLE_LIMIT_FOV_RAD)
            isValid = false;
        else if (m_id == 2 && std::abs(obs.angle) > ANGLE_LIMIT_FOV_RAD)
            isValid = false;

        if (isValid)
        {
            // Đồng bộ tọa độ x, y từ range và angle sau khi lọc nhiễu
            obs.x = obs.range * cosf(obs.angle);
            obs.y = obs.range * sinf(obs.angle);
            obs.z = 0.0f;

            m_tempObstacles.push_back(obs);
        }

        return isFrameParsed;
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
