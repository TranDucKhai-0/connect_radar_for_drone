#ifndef CSVLOGGER_HPP
#define CSVLOGGER_HPP

#include "IRadar.hpp"
#include <string>
#include <fstream>
#include <vector>

class CsvLogger {
private:
    std::string m_fileName;
    std::ofstream m_fileStream;

public:
    explicit CsvLogger(const std::string& fileName);
    ~CsvLogger();

    bool Open();
    void Close();
    void LogObstacles(long long timestampMs, const std::vector<obstacleData_t>& obstacles);
};

#endif // CSVLOGGER_HPP
