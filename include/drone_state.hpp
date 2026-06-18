#ifndef DRONE_STATE_HPP
#define DRONE_STATE_HPP

#include <mutex>

// ---------------------------------------------------------
// Lớp quản lý trạng thái của Drone (Thread-safe)
// Dùng để chia sẻ dữ liệu vận tốc, độ cao giữa luồng đọc FC và luồng tính toán
// ---------------------------------------------------------
class DroneState {
private:
    float velocity_x_ = 0.0f; // Vận tốc trục North hoặc Front tuỳ bản tin FC (m/s)
    float velocity_y_ = 0.0f; // Vận tốc trục East hoặc Right (m/s)
    float altitude_m_ = 0.0f; // Độ cao so với mặt đất (m)
    mutable std::mutex mutex_; // Mutex đảm bảo an toàn khi đọc/ghi đa luồng

public:
    DroneState() = default;

    // Cập nhật trạng thái mới nhất từ FC
    void Update(float vx, float vy, float alt) {
        std::lock_guard<std::mutex> lock(mutex_);
        velocity_x_ = vx;
        velocity_y_ = vy;
        altitude_m_ = alt;
    }

    // Lấy trạng thái hiện tại (Copy an toàn qua tham chiếu)
    void GetState(float& vx, float& vy, float& alt) const {
        std::lock_guard<std::mutex> lock(mutex_);
        vx = velocity_x_;
        vy = velocity_y_;
        alt = altitude_m_;
    }
    
    // Lấy độ cao hiện tại
    float GetAltitude() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return altitude_m_;
    }
};

#endif // DRONE_STATE_HPP
