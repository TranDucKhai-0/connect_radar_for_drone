#include "CsvLogger.hpp"
#include <iostream>

CsvLogger::CsvLogger(const std::string& fileName) : m_fileName(fileName) {}

CsvLogger::~CsvLogger() {
    Close();
}

bool CsvLogger::Open() {
    m_fileStream.open(m_fileName, std::ios::out | std::ios::app);
    if (!m_fileStream.is_open()) {
        std::cerr << "Failed to open log file: " << m_fileName << "\n";
        return false;
    }
    
    // Nếu file trống, ghi header
    m_fileStream.seekp(0, std::ios::end);
    if (m_fileStream.tellp() == 0) {
        m_fileStream << "TimestampMs,ID,X,Y,Z,Range,Angle,Vx,Vy,Vz,Elevation\n";
    }
    
    return true;
}

void CsvLogger::Close() {
    if (m_fileStream.is_open()) {
        m_fileStream.close();
    }
}

void CsvLogger::LogObstacles(long long timestampMs, const std::vector<obstacleData_t>& obstacles, float Elevation) {
    if (!m_fileStream.is_open()) return;

    for (const auto& obs : obstacles) {
        m_fileStream << timestampMs << ","
                     << obs.id << "," // ID của vật thể không phải của radar
                     << obs.x << ","
                     << obs.y << ","
                     << obs.z << ","
                     << obs.range << ","
                     << obs.angle << ","
                     << obs.v_x << ","
                     << obs.v_y << ","
                     << obs.v_z << ","
                     << Elevation << "\n";
    }
    
    // Đảm bảo dữ liệu được ghi xuống đĩa
    m_fileStream.flush();
}
