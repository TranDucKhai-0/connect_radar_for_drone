#include "csv_logger.hpp"
#include <iostream>
#include <iomanip>
#include <filesystem>
#include <cerrno>
#include <cstring>

CsvLogger::CsvLogger(const std::string &filename, LoggerType type) : m_filename(filename), m_type(type) {}

CsvLogger::~CsvLogger()
{
    Close(); // Đảm bảo tệp được đóng khi hủy logger
}

// Hàm mở file để ghi dữ liệu log (tạo thư mục nếu cần)
bool CsvLogger::Open()
{
    if (m_fileStream.is_open())
    {
        m_fileStream.close(); // Đóng file cũ trước khi tạo mới
    }

    // Tự động tạo cây thư mục nếu nó chưa tồn tại (ví dụ: blackbox/)
    try {
        std::filesystem::path dir = std::filesystem::path(m_filename).parent_path();
        if (!dir.empty()) {
            std::filesystem::create_directories(dir);
        }
    } catch (const std::exception &e) {
        std::cerr << "CsvLogger: Cannot create directory for " << m_filename << ": " << e.what() << "\n";
        return false;
    }

    // Mở file ở chế độ ghi đè (trunc) — mỗi phiên bay sẽ ghi đè lên phiên cũ 
    // để tránh đầy bộ nhớ không kiểm soát
    m_fileStream.open(m_filename, std::ios::out | std::ios::trunc);
    if (!m_fileStream.is_open())
    {
        std::cerr << "CsvLogger: Cannot open file " << m_filename
                  << " — " << std::strerror(errno) << "\n";
        return false;
    }

    // Ghi Header của file CSV dựa trên loại logger
    if (m_type == LoggerType::RADAR)
    {
        m_fileStream << "TimestampUsec,X,Y,Z,Range,Angle,Vx,Vy,Vz,DroneAlt\n";
    }
    else
    {
        m_fileStream << "TimestampUsec,SectorIdx,DistanceCm\n";
    }
    m_fileStream.flush(); // Đẩy ngay xuống đĩa cứng

    std::cout << "CsvLogger: Logging to " << m_filename << "\n";
    return true;
}

// Đóng luồng ghi file
void CsvLogger::Close()
{
    if (m_fileStream.is_open())
    {
        m_fileStream.close();
    }
}

// Ghi thông tin các vật cản (toạ độ tuyệt đối) tại một thời điểm (timestamp)
void CsvLogger::LogObstacles(long long timestampUsec, const std::vector<obstacleAbsolute_t>& obstacles, float droneAlt)
{
    if (!m_fileStream.is_open())
        return;

    for (const auto &obs : obstacles)
    {
        // Lọc bỏ data rác ngoài dải hoạt động MR72 (2m - 40m)
        // if (obs.range < 2.0f || obs.range > 40.0f) continue;

        // Ghi các thuộc tính của vật thể ra dạng phân cách bằng dấu phẩy
        m_fileStream << std::fixed << std::setprecision(3)
                     << timestampUsec << ","
                     << obs.x << ","
                     << obs.y << ","
                     << obs.z << ","
                     << obs.range << ","
                     << obs.angle << ","
                     << obs.vx << ","
                     << obs.vy << ","
                     << obs.vz << ","
                     << droneAlt << "\n";
    }

    m_fileStream.flush(); // Đảm bảo dữ liệu không bị mất nếu chương trình crash
}

// Ghi thông tin các cung gửi sang FC (được đồng bộ)
void CsvLogger::LogFcDistances(long long timestampUsec, const uint16_t distances[72])
{
    if (!m_fileStream.is_open())
        return;

    for (uint8_t i = 0; i < 72; i++)
    {
        if (distances[i] < 4001) // Chỉ ghi các cung có vật cản (dưới ngưỡng tối đa 4000cm)
        {
            m_fileStream << timestampUsec << ","
                         << (int)i << ","
                         << distances[i] << "\n";
        }
    }
    m_fileStream.flush();
}
