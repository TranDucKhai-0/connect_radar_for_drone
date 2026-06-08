#include "csv_logger.hpp"
#include <iostream>

CsvLogger::CsvLogger(const std::string& filename) : m_filename(filename) {}

CsvLogger::~CsvLogger() {
    Close();
}

bool CsvLogger::Open() {
    if (m_fileStream.is_open()) {
        m_fileStream.close();
    }
    // Mở ở chế độ ghi đè (trunc) để xóa dữ liệu cũ mỗi khi bắt đầu phiên mới
    m_fileStream.open(m_filename, std::ios::out | std::ios::trunc);
    if (!m_fileStream.is_open()) {
        std::cerr << "CsvLogger: Could not open file " << m_filename << " for writing.\n";
        return false;
    }
    
    // Ghi header mỗi khi khởi tạo 
    m_fileStream << "TimestampMs,ID_obj,X,Y,Z,Range,Angle,Vx,Vy,Vz,DroneAlt\n";
    
    return true;
}

void CsvLogger::Close() {
    if (m_fileStream.is_open()) {
        m_fileStream.close();
    }
}

void CsvLogger::LogObstacles(long long timestampMs, const std::vector<obstacle_absolute_t>& obstacles, float droneAlt) {
    if (!m_fileStream.is_open()) return;

    for (const auto& obs : obstacles) {
        m_fileStream << timestampMs << ","
                     << obs.id << ","
                     << obs.x << ","
                     << obs.y << ","
                     << obs.z << ","
                     << obs.range << ","
                     << obs.angle << ","
                     << obs.v_x << ","
                     << obs.v_y << ","
                     << obs.v_z << ","
                     << droneAlt << "\n";
    }
    
    m_fileStream.flush();
}
