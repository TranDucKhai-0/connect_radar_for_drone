#include "GcsTelemetry.hpp"
#include <iostream>

bool GcsTelemetry::Init() {
    // TODO: Thiết lập socket UDP để gửi MAVLink tới GCS (QGroundControl)
    std::cout << "GcsTelemetry initialized.\n";
    return true;
}

void GcsTelemetry::SendObstacles(const std::vector<obstacleData_t>& obstacles) {
    if (obstacles.empty()) return;

    // TODO: Đóng gói dữ liệu obstacles thành bản tin OBSTACLE_DISTANCE (MAVLink)
    // Giả lập log in ra console thay vì gửi MAVLink thực sự do chưa có thư viện mavlink c_library_v2
    std::cout << "[GCS Telemetry] Sent " << obstacles.size() << " obstacles to GCS.\n";
}
