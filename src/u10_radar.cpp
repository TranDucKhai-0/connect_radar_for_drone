#include "u10_radar.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>

U10Radar::U10Radar(int id, uint32_t canId)
    : m_id(id), m_canId(canId), m_mountingYaw(0.0f), m_state(PARSE_STATE_FIND_SYNC), m_expectedTotalLength(0)
{
    // Nếu canId giữ giá trị mặc định là 0x123, tự động chuyển về CAN ID chuẩn của U10
    if (canId == 0x123 && id >= 1 && id <= 4)
    {
        m_canId = 0x0D0 + id; // 0x0D1, 0x0D2, 0x0D3, 0x0D4
    }
    m_obstacles.reserve(64);
    m_byteBuffer.reserve(1024);
}

// Khởi tạo thông số (góc lắp đặt yaw nếu cần xoay toạ độ)
void U10Radar::Init(float mountingYaw)
{
    m_mountingYaw = mountingYaw;
}

// Phân tích cú pháp khung dữ liệu CAN của U10 Radar
bool U10Radar::ParseCanFrame(const struct can_frame &frame, float droneVForward, float droneVRight)
{
    // Chỉ xử lý các frame CAN có ID khớp với m_canId
    if (frame.can_id != m_canId)
    {
        return false;
    }

    // Nối dữ liệu từ CAN frame (tối đa 8 bytes) vào bộ đệm tích lũy
    m_byteBuffer.insert(m_byteBuffer.end(), frame.data, frame.data + frame.can_dlc);

    bool isFrameParsed = false;
    bool isParsing = true;

    while (isParsing)
    {
        switch (m_state)
        {
            case PARSE_STATE_FIND_SYNC:
            {
                if (m_byteBuffer.size() < 8)
                {
                    isParsing = false;
                    break;
                }

                // Tìm Magic Word đồng bộ: 02 01 04 03 06 05 08 07
                const uint8_t magicWord[8] = {0x02, 0x01, 0x04, 0x03, 0x06, 0x05, 0x08, 0x07};
                auto it = std::search(m_byteBuffer.begin(), m_byteBuffer.end(), std::begin(magicWord), std::end(magicWord));
                if (it != m_byteBuffer.end())
                {
                    // Đã tìm thấy Magic Word, xóa các byte rác phía trước nó
                    m_byteBuffer.erase(m_byteBuffer.begin(), it);
                    m_state = PARSE_STATE_READ_HEADER;
                }
                else
                {
                    // Không tìm thấy, giữ lại tối đa 7 byte cuối cùng phòng trường hợp Magic Word bị chia cắt giữa các CAN frame
                    if (m_byteBuffer.size() > 7)
                    {
                        m_byteBuffer.erase(m_byteBuffer.begin(), m_byteBuffer.end() - 7);
                    }
                    isParsing = false;
                }
                break;
            }

            case PARSE_STATE_READ_HEADER:
            {
                if (m_byteBuffer.size() < 16)
                {
                    isParsing = false;
                    break;
                }

                // Lấy độ dài gói tin (bytes 12-15) dạng little-endian từ header
                uint32_t totalLength = m_byteBuffer[12] |
                                      (m_byteBuffer[13] << 8) |
                                      (m_byteBuffer[14] << 16) |
                                      (m_byteBuffer[15] << 24);

                // Kiểm tra tính hợp lệ sơ bộ của độ dài gói tin
                // Một gói tin tối thiểu phải chứa header (40 bytes)
                if (totalLength < 40 || totalLength > 8192)
                {
                    // Độ dài không hợp lệ, xóa byte đầu tiên để dịch chuyển và tìm Magic Word tiếp theo
                    m_byteBuffer.erase(m_byteBuffer.begin());
                    m_state = PARSE_STATE_FIND_SYNC;
                }
                else
                {
                    m_expectedTotalLength = totalLength;
                    m_state = PARSE_STATE_READ_PAYLOAD;
                }
                break;
            }

            case PARSE_STATE_READ_PAYLOAD:
            {
                if (m_byteBuffer.size() < m_expectedTotalLength)
                {
                    isParsing = false;
                    break;
                }

                // Đã nhận đủ toàn bộ gói tin, tiến hành bóc tách dữ liệu
                if (_ParsePacket(m_byteBuffer.data(), m_expectedTotalLength))
                {
                    isFrameParsed = true;
                }

                // Xóa gói tin đã xử lý khỏi bộ đệm tích lũy
                m_byteBuffer.erase(m_byteBuffer.begin(), m_byteBuffer.begin() + m_expectedTotalLength);
                m_state = PARSE_STATE_FIND_SYNC;
                break;
            }
        }
    }

    return isFrameParsed;
}

// Bóc tách dữ liệu từ một gói tin hoàn chỉnh đã tích lũy
bool U10Radar::_ParsePacket(const uint8_t* pData, size_t length)
{
    if (length < 40)
    {
        return false;
    }

    // Đọc thông tin Header
    uint32_t frameNum = pData[20] | (pData[21] << 8) | (pData[22] << 16) | (pData[23] << 24);
    uint32_t pointsNum = pData[28] | (pData[29] << 8) | (pData[30] << 16) | (pData[31] << 24);
    uint32_t tlvNum = pData[32] | (pData[33] << 8) | (pData[34] << 16) | (pData[35] << 24);

    std::vector<obstacleRelative_t> newObstacles;
    newObstacles.reserve(pointsNum);

    size_t tlvIndex = 40;

    for (uint32_t i = 0; i < tlvNum; ++i)
    {
        if (tlvIndex + 8 > length)
        {
            break; // Tránh tràn bộ nhớ
        }

        uint32_t tlvType = pData[tlvIndex] | 
                           (pData[tlvIndex + 1] << 8) | 
                           (pData[tlvIndex + 2] << 16) | 
                           (pData[tlvIndex + 3] << 24);
                           
         uint32_t tlvLength = pData[tlvIndex + 4] | 
                             (pData[tlvIndex + 5] << 8) | 
                             (pData[tlvIndex + 6] << 16) | 
                             (pData[tlvIndex + 7] << 24);

        if (tlvIndex + 8 + tlvLength > length)
        {
            break; // Lỗi gói tin bị cắt cụt
        }

        if (tlvType == 1) // Point Cloud Data
        {
            for (uint32_t k = 0; k < pointsNum; ++k)
            {
                size_t m = tlvIndex + 8 + k * 16;
                if (m + 16 > tlvIndex + 8 + tlvLength)
                {
                    break; // Vượt quá chiều dài TLV
                }

                float xU10, yU10, zU10, vDoppler;
                std::memcpy(&xU10, &pData[m], sizeof(float));
                std::memcpy(&yU10, &pData[m + 4], sizeof(float));
                std::memcpy(&zU10, &pData[m + 8], sizeof(float));
                std::memcpy(&vDoppler, &pData[m + 12], sizeof(float));

                // Chuyển đổi sang hệ tọa độ FRD của drone:
                // X_frd = Y_u10 (hướng tới trước)
                // Y_frd = X_u10 (hướng sang phải)
                // Z_frd = -Z_u10 (hướng xuống)
                obstacleRelative_t obs;
                obs.id = -32768; // Gán giá trị int16_t cực tiểu để khi cộng offset ở main.cpp vẫn ra giá trị âm
                obs.x = yU10;
                obs.y = xU10;
                obs.z = -zU10;

                // Tính toán khoảng cách (range) và góc (angle) trong hệ FRD
                obs.range = std::sqrt(obs.x * obs.x + obs.y * obs.y + obs.z * obs.z);
                obs.angle = std::atan2(obs.y, obs.x);

                // Chiếu vận tốc Doppler hướng tâm lên các trục tọa độ
                if (obs.range > 0.0f)
                {
                    obs.vx = vDoppler * (obs.x / obs.range);
                    obs.vy = vDoppler * (obs.y / obs.range);
                    obs.vz = vDoppler * (obs.z / obs.range);
                }
                else
                {
                    obs.vx = 0.0f;
                    obs.vy = 0.0f;
                    obs.vz = 0.0f;
                }

                // Lọc nhiễu khoảng cách giống như MR72
                // Radar ID 1, 3 (Front, Back): khoảng cách 2m - 40m
                if ((m_id == 1 || m_id == 3) && (obs.range < 2.0f || obs.range > 40.0f))
                {
                    continue;
                }
                // Radar ID 2, 4 (Right, Left): khoảng cách 2m - 20m
                else if ((m_id == 2 || m_id == 4) && (obs.range < 2.0f || obs.range > 20.0f))
                {
                    continue;
                }

                newObstacles.push_back(obs);
            }
        }

        // Di chuyển sang TLV tiếp theo
        tlvIndex += 8 + tlvLength;
    }

    m_obstacles = std::move(newObstacles);
    return true;
}

// Lấy danh sách vật cản (Toạ độ tương đối)
std::vector<obstacleRelative_t> U10Radar::GetObstaclesRelative() const
{
    return m_obstacles;
}

// Lấy số lượng điểm ảnh vật cản hiện có
int U10Radar::GetObstacleCount() const
{
    return m_obstacles.size();
}

// Đặt lại trạng thái bộ đệm
void U10Radar::_ResetParser()
{
    m_state = PARSE_STATE_FIND_SYNC;
    m_expectedTotalLength = 0;
    if (m_byteBuffer.size() > 4096)
    {
        m_byteBuffer.clear();
    }
}
