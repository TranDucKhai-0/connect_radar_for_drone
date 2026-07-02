#ifndef CSVLOGGER_HPP
#define CSVLOGGER_HPP

#include "i_radar.hpp"
#include <string>
#include <fstream>
#include <vector>

enum class LoggerType {
    RADAR,
    FC
};

class CsvLogger {
public:
    CsvLogger(const std::string& filename, LoggerType type = LoggerType::RADAR);
    ~CsvLogger();

    // Mở file và ghi header, tự tạo thư mục nếu chưa có
    bool Open();
    
    // Đóng luồng ghi file
    void Close();
    
    // Hàm thực hiện ghi thông tin từng vật cản cùng timestamp và độ cao drone ra file
    void LogObstacles(long long timestampUsec, const std::vector<obstacleAbsolute_t>& obstacles, float droneAlt);

    // Hàm thực hiện ghi thông tin các cung gửi sang FC (được đồng bộ)
    void LogFcDistances(long long timestampUsec, const uint16_t distances[72]);

private:
    std::string m_filename; // Đường dẫn và tên file CSV
    std::ofstream m_fileStream; // Luồng ghi file (ofstream)
    LoggerType m_type;
};

#endif // CSVLOGGER_HPP
