#ifndef IRADAR_HPP
#define IRADAR_HPP

#include <vector>
#include <linux/can.h>

// Định nghĩa chung cho một vật cản (áp dụng chung cho mọi Radar)
typedef struct
{
    int id;            // ID của vật cản (do radar cung cấp, không phải ID riêng của radar)
    float x;           // Vị trí tương đối trục x (Longitudinal distance) (m)
    float y;           // Vị trí tương đối trục y (Lateral distance) (m)
    float z;           // Vị trí tương đối trục z (Vertical distance) (m)
    float v_x;         // Vận tốc tương đối dọc trục x (m/s)
    float v_y;         // Vận tốc tương đối dọc trục y (m/s)
    float v_z;         // Vận tốc tương đối dọc trục z (m/s)
} obstacle_relative_t; // Dữ liệu vật cản tổng hợp tương đối (từ radar)

typedef struct
{
    int id;            // ID của vật cản
    float x;           // Vị trí tuyệt đối trục x (m)
    float y;           // Vị trí tuyệt đối trục y (m)
    float z;           // Vị trí tuyệt đối trục z (m)
    float range;       // Khoảng cách tuyệt đối trực tiếp từ radar (m)
    float angle;       // Góc tuyệt đối giữa trục x (Front) và vật cản (rad)
    float v_x;         // Vận tốc tuyệt đối dọc trục x (m/s)
    float v_y;         // Vận tốc tuyệt đối dọc trục y (m/s)
    float v_z;         // Vận tốc tuyệt đối dọc trục z (m/s)
} obstacle_absolute_t; // Dữ liệu vật cản tổng hợp tuyệt đối

class IRadar
{
public:
    virtual ~IRadar() = default;

    // Khởi tạo các tham số, cấu hình radar (ví dụ angle offset)
    virtual void Init(float mountingYaw) = 0;

    // Parse data từ CAN frame, trả về true nếu hoàn thành 1 chu kỳ parse
    virtual bool ParseCanFrame(const struct can_frame &frame, float droneVForward, float droneVRight) = 0;

    // Lấy số lượng vật cản hiện tại
    virtual int GetObstacleCount() const = 0;
};

#endif // IRADAR_HPP
