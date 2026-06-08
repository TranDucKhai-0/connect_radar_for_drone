#include "MR72Radar.hpp"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

MR72Radar::MR72Radar(int id) 
    : m_id(id), m_mountingYaw(0.0f), m_expectedObstacles(0) {
    m_obstacles.reserve(64);
    m_tempObstacles.reserve(64);
}

void MR72Radar::Init(float mountingYaw) {
    m_mountingYaw = mountingYaw;
}

bool MR72Radar::ParseCanFrame(const struct can_frame& frame, float droneVForward, float droneVRight) {
    // Gói tin 0x60A đánh dấu điểm bắt đầu chu kỳ truyền mới
    if (frame.can_id == (MR72_OBJECT_LIST_STATUS + m_id * 0x10)) {
        // Ghi nhận chu kỳ trước đó đã xong
        m_obstacles = m_tempObstacles; 
        m_tempObstacles.clear();
        
        m_expectedObstacles = frame.data[0];
        
        return true; // Trả về true báo hiệu kết thúc 1 frame của radar (để có thể lấy dữ liệu)
    }
    // Các gói tin điểm vật cản 0x60B
    else if (frame.can_id == (MR72_OBJECT_GENERAL_INFO + m_id * 0x10)) {
        if (m_tempObstacles.size() >= 64) {
            return false; // Tránh tràn buffer
        }

        obstacleData_t obs;
        obs.id = frame.data[0];
        
        uint16_t distLongRaw = (frame.data[1] << 5) | (frame.data[2] >> 3);
        uint16_t distLatRaw = ((frame.data[2] & 0x7) << 8) | frame.data[3];
        uint16_t vrelLongRaw = (frame.data[4] << 2) | ((frame.data[5] >> 6) & 0x3);
        uint16_t vrelLatRaw = (frame.data[5] << 3) | ((frame.data[6] >> 5) & 0x7);
        
        uint8_t sectorNumber = (frame.data[6] >> 3) & 0x3;
        
        // Chỉ lọc lấy những vật thể đã được xác nhận chắc chắn (Measured == 0x02)
        if (sectorNumber != 0x02) {
            return false;
        }

        // Chuyển đổi từ Raw Value sang giá trị vật lý
        obs.y = distLatRaw * 0.2f - 204.6f;
        obs.x = distLongRaw * 0.2f - 500.0f;
        obs.z = 0.0f;
        obs.v_x = vrelLongRaw * 0.25f - 128.0f;
        obs.v_y = vrelLatRaw * 0.25f - 64.0f;
        obs.v_z = 0.0f;

        // Tính toán vận tốc thân Drone chiếu lên hệ toạ độ nội bộ của Radar này
        float yawRad = m_mountingYaw * (M_PI / 180.0f);
        float vRadarX = droneVForward * std::cos(yawRad) + droneVRight * std::sin(yawRad);
        float vRadarY = -droneVForward * std::sin(yawRad) + droneVRight * std::cos(yawRad);

        // Tính vận tốc tuyệt đối (Absolute Velocity) của điểm mục tiêu so với mặt đất
        obs.vabs_x = obs.v_x + vRadarX;
        obs.vabs_y = obs.v_y + vRadarY;
        obs.vabs_z = 0.0f;

        // BÙ TRỪ VỊ TRÍ (Latency Compensation): Bù 100ms độ trễ
        obs.x += obs.vabs_x * 0.1f;
        obs.y += obs.vabs_y * 0.1f;

        // Cập nhật vị trí tuyệt đối (giả sử radar nằm ở tâm drone hoặc hệ toạ độ đã được gán)
        obs.absolute_value_x = obs.x;
        obs.absolute_value_y = obs.y;
        obs.absolute_value_z = obs.z;

        // Chuyển từ Cartesian sang Polar (dùng x, y đã cập nhật)
        obs.angle = std::atan2(obs.y, obs.x);
        obs.range = std::sqrt(obs.x * obs.x + obs.y * obs.y);

        m_tempObstacles.push_back(obs);
    }
    
    return false;
}

std::vector<obstacleData_t> MR72Radar::GetObstacles() const {
    return m_obstacles;
}

int MR72Radar::GetObstacleCount() const {
    return m_obstacles.size();
}
