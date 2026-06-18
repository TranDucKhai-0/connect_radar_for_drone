#ifndef CSVLOGGER_HPP
#define CSVLOGGER_HPP

#include "i_radar.hpp"
#include <string>
#include <fstream>
#include <vector>

class CsvLogger {
public:
    CsvLogger(const std::string& filename);
    ~CsvLogger();

    // Mở file và ghi header, tự tạo thư mục nếu chưa có
    bool Open();
    
    // Đóng luồng ghi file
    void Close();
    
    // Hàm thực hiện ghi thông tin từng vật cản cùng timestamp và độ cao drone ra file
    void LogObstacles(long long timestampUsec, const std::vector<obstacleAbsolute_t>& obstacles, float droneAlt);

private:
    std::string m_filename; // Đường dẫn và tên file CSV
    std::ofstream m_fileStream; // Luồng ghi file (ofstream)
};

#endif // CSVLOGGER_HPP
