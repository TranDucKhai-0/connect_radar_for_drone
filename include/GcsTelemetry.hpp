#ifndef GCSTELEMETRY_HPP
#define GCSTELEMETRY_HPP

#include "IRadar.hpp"
#include <vector>

class GcsTelemetry {
public:
    GcsTelemetry() = default;
    ~GcsTelemetry() = default;

    // Khởi tạo kết nối UDP hoặc Serial tới GCS (tạm thời để mô phỏng)
    bool Init();

    // Gửi danh sách vật cản qua MAVLink tới GCS
    void SendObstacles(const std::vector<obstacleData_t>& obstacles);
};

#endif // GCSTELEMETRY_HPP
