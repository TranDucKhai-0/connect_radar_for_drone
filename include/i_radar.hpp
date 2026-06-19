#ifndef IRADAR_HPP
#define IRADAR_HPP

#include <vector>
#include <stdint.h>
#include <linux/can.h>

// Định nghĩa chung cho một vật cản (áp dụng chung cho mọi Radar)
typedef struct
{
    uint8_t id;            // ID của vật cản (do radar cung cấp, không phải ID riêng của radar)
    float x;           // Vị trí tương đối trục x (Longitudinal distance) (m)
    float y;           // Vị trí tương đối trục y (Lateral distance) (m)
    float z;           // Vị trí tương đối trục z (Vertical distance) (m)
    float range;       // Khoảng cách tương đối trực tiếp từ radar (m)
    float angle;       // Góc tương đối đối giữa trục x (Front) và vật cản (rad)
    float vx;          // Vận tốc tương đối dọc trục x (m/s)
    float vy;          // Vận tốc tương đối dọc trục y (m/s)
    float vz;          // Vận tốc tương đối dọc trục z (m/s)
} obstacleRelative_t; // Dữ liệu vật cản tổng hợp tương đối (từ radar)

typedef struct
{
    uint8_t id;            // ID của vật cản
    float x;           // Vị trí tuyệt đối trục x (m)
    float y;           // Vị trí tuyệt đối trục y (m)
    float z;           // Vị trí tuyệt đối trục z (m)
    float range;       // Khoảng cách tuyệt đối trực tiếp từ radar (m)
    float angle;       // Góc tuyệt đối giữa trục x (Front) và vật cản (rad)
    float vx;          // Vận tốc tuyệt đối dọc trục x (m/s)
    float vy;          // Vận tốc tuyệt đối dọc trục y (m/s)
    float vz;          // Vận tốc tuyệt đối dọc trục z (m/s)
} obstacleAbsolute_t; // Dữ liệu vật cản tổng hợp tuyệt đối

// ---------------------------------------------------------
// Interface (Giao diện) dùng chung cho tất cả các loại Radar
// Giúp dễ dàng mở rộng và hỗ trợ đa dạng radar (MR72, NRA24, v.v...)
// ---------------------------------------------------------
class IRadar
{
public:
    virtual ~IRadar() = default;

    // Khởi tạo các tham số, cấu hình radar (ví dụ angle offset / mounting yaw)
    virtual void Init(float mountingYaw) = 0;

    // Parse data từ CAN frame hoặc các dạng data khác tuỳ radar
    // Trả về true nếu hoàn thành việc bóc tách thành công một điểm ảnh
    virtual bool ParseCanFrame(const struct can_frame &frame, float droneVForward, float droneVRight) = 0;

    // Lấy số lượng vật cản hiện tại mà radar này phát hiện được
    virtual int GetObstacleCount() const = 0;
};

#endif // IRADAR_HPP
