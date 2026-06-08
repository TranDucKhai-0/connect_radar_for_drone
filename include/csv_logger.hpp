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

    bool Open();
    void Close();
    void LogObstacles(long long timestampMs, const std::vector<obstacle_absolute_t>& obstacles, float droneAlt);

private:
    std::string m_filename;
    std::ofstream m_fileStream;
};

#endif // CSVLOGGER_HPP
