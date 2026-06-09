#include "csv_logger.hpp"
#include <iostream>
#include <iomanip>
#include <filesystem>
#include <cerrno>
#include <cstring>

CsvLogger::CsvLogger(const std::string &filename) : m_filename(filename) {}

CsvLogger::~CsvLogger()
{
    Close();
}

bool CsvLogger::Open()
{
    if (m_fileStream.is_open())
    {
        m_fileStream.close();
    }

    // Tự động tạo thư mục nếu chưa tồn tại
    try {
        std::filesystem::path dir = std::filesystem::path(m_filename).parent_path();
        if (!dir.empty()) {
            std::filesystem::create_directories(dir);
        }
    } catch (const std::exception &e) {
        std::cerr << "CsvLogger: Cannot create directory for " << m_filename << ": " << e.what() << "\n";
        return false;
    }

    // Mở ở chế độ ghi đè (trunc) — mỗi phiên bay ghi đè lên phiên cũ
    m_fileStream.open(m_filename, std::ios::out | std::ios::trunc);
    if (!m_fileStream.is_open())
    {
        std::cerr << "CsvLogger: Cannot open file " << m_filename
                  << " — " << std::strerror(errno) << "\n";
        return false;
    }

    // Header
    m_fileStream << "TimestampMs,X,Y,Z,Range,Angle,Vx,Vy,Vz,DroneAlt\n";
    m_fileStream.flush();

    std::cout << "CsvLogger: Logging to " << m_filename << "\n";
    return true;
}

void CsvLogger::Close()
{
    if (m_fileStream.is_open())
    {
        m_fileStream.close();
    }
}

void CsvLogger::LogObstacles(long long timestampMs, const std::vector<obstacleAbsolute_t>& obstacles, float droneAlt)
{
    if (!m_fileStream.is_open())
        return;

    for (const auto &obs : obstacles)
    {
        // Lọc bỏ data rác ngoài dải hoạt động MR72 (2m - 40m)
        if (obs.range < 2.0f || obs.range > 40.0f) continue;

        m_fileStream << std::fixed << std::setprecision(3)
                     << timestampMs << ","
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

    m_fileStream.flush();
}
