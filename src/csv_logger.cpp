#include "csv_logger.hpp"
#include <iostream>

CsvLogger::CsvLogger(const std::string& filename) : m_filename(filename) {}

CsvLogger::~CsvLogger() {
    Close();
}

bool CsvLogger::Open() {
    m_fileStream.open(m_filename, std::ios::out | std::ios::app);
    if (!m_fileStream.is_open()) {
        std::cerr << "CsvLogger: Could not open file " << m_filename << " for writing.\n";
        return false;
    }
    
    // Nếu file trống, ghi header
    m_fileStream.seekp(0, std::ios::end);
    if (m_fileStream.tellp() == 0) {
        m_fileStream << "TimestampMs,ID,X,Y,Z,AbsX,AbsY,AbsZ,Range,Angle,Vx,Vy,Vz,VabsX,VabsY,VabsZ,DroneAlt\n";
    }
    
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
