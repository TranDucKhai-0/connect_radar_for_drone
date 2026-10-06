#ifndef LASER_DRIVER_H
#define LASER_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SF20_START_BYTE     0xAA   // Start Byte cố định của gói binary lwNx

/**
 * @brief Danh sách các Command ID của cảm biến SF20/LW20
 */
typedef enum {
    SF20_CMD_PROD_NAME        = 0U,    // Command ID 0: Product Name
    SF20_CMD_DIST_OUT         = 27U,   // Command ID 27: Distance Output Mask
    SF20_CMD_COMM_MODE        = 28U,   // Command ID 28: Communication / Startup Mode
    SF20_CMD_STREAM           = 30U,   // Command ID 30: Stream Configuration
    SF20_CMD_DIST_CM          = 44U,   // Command ID 44: Distance in cm
    SF20_CMD_DIST_MM          = 45U,   // Command ID 45: Distance in mm
    SF20_CMD_MEDIAN_ENABLE    = 137U,  // Command ID 137: Median Filter Enable
    SF20_CMD_MEDIAN_SIZE      = 138U,  // Command ID 138: Median Filter Size (3..32)
    SF20_CMD_SMOOTHING_ENABLE = 139U,  // Command ID 139: Smoothing Filter Enable
    SF20_CMD_SMOOTHING_FACTOR = 140U,  // Command ID 140: Smoothing Factor (1..99)
    SF20_CMD_ROLLING_ENABLE   = 141U,  // Command ID 141: Rolling Average Enable
    SF20_CMD_ROLLING_SIZE     = 142U   // Command ID 142: Rolling Average Size (2..32)
} SF20_CmdID_t;

/**
 * @brief Định nghĩa các Bitmask cho Command ID 27 (Distance Output)
 */
typedef enum {
    SF20_OUT_FIRST_RAW       = (1U << 0),  // Bit 0: First return raw
    SF20_OUT_FIRST_CLOSEST   = (1U << 1),  // Bit 1: First return closest
    SF20_OUT_FIRST_FILTERED  = (1U << 2),  // Bit 2: First return (Filtered)
    SF20_OUT_FIRST_FURTHEST  = (1U << 3),  // Bit 3: First return furthest
    SF20_OUT_FIRST_STRENGTH  = (1U << 4),  // Bit 4: First return strength (%)
    SF20_OUT_LAST_RAW        = (1U << 5),  // Bit 5: Last return raw
    SF20_OUT_LAST_CLOSEST    = (1U << 6),  // Bit 6: Last return closest
    SF20_OUT_LAST_FILTERED   = (1U << 7),  // Bit 7: Last return median
    SF20_OUT_LAST_FURTHEST   = (1U << 8)   // Bit 8: Last return furthest
} SF20_DistanceOutput_t;

/**
 * @brief Trạng thái Bật/Tắt các bộ lọc
 */
typedef enum {
    SF20_FILTER_DISABLE = 0U,
    SF20_FILTER_ENABLE  = 1U
} SF20_FilterState_t;

/**
 * @brief Chế độ tự động Stream dữ liệu (Command ID 30)
 */
typedef enum {
    SF20_STREAM_DISABLED    = 0U,  // Tắt stream
    SF20_STREAM_DISTANCE_CM = 5U,  // Stream bản tin khoảng cách cm (Cmd 44)
    SF20_STREAM_DISTANCE_MM = 6U   // Stream bản tin khoảng cách mm (Cmd 45)
} SF20_StreamMode_t;

// LiDAR SF20
typedef struct {
    int32_t  distance_mm;
    int16_t  distance_cm;
    float    distance_m;           // Giá trị quy đổi ra mét (m)
    int16_t  signal_strength_raw;  // Giá trị Strength thô từ SF20 (%)
    uint8_t  command_id;           // Command ID của bản tin vừa received
    bool     is_valid;             // Cờ báo dữ liệu hợp lệ
} LaserObject_t;

/* --- Cấu trúc hướng đối tượng cho Laser Driver --- */

/**
 * @brief Function pointer cho việc truyền dữ liệu (Hardware Abstraction)
 * @param data Con trỏ trỏ tới mảng dữ liệu cần truyền
 * @param size Chiều dài mảng dữ liệu
 * @return 0 nếu thành công, giá trị khác nếu lỗi
 */
typedef int8_t (*Laser_TxCallback)(uint8_t *data, uint16_t size);

/**
 * @brief Cấu hình Laser Driver
 */
typedef struct {
    Laser_TxCallback transmit_cb;  /* Hàm callback gửi dữ liệu UART */
} Laser_Config_t;

/**
 * @brief Đối tượng (Handle) của Laser Driver
 */
typedef struct {
    Laser_Config_t config;
    bool           is_initialized;
} Laser_Handle_t;

/* ============================================================================
   CORE APIs (Low-Level)
   ============================================================================ */

/**
 * @brief  Khởi tạo Laser Driver
 * @param  handle: Con trỏ trỏ tới đối tượng Laser
 * @param  config: Cấu hình của Laser
 */
void Laser_SF20_Init(Laser_Handle_t *handle, const Laser_Config_t *config);

/* --- HÀM GENERIC ĐỌC / GHI REGISTER DÙNG CHUNG (MULTI-PURPOSE API) --- */
void Laser_SF20_Read_Cmd(Laser_Handle_t *handle, SF20_CmdID_t cmd_id);
void Laser_SF20_Write_UInt8(Laser_Handle_t *handle, SF20_CmdID_t cmd_id, uint8_t value);
void Laser_SF20_Write_UInt32(Laser_Handle_t *handle, SF20_CmdID_t cmd_id, uint32_t value);

/* --- BỘ PARSER DỮ LIỆU ĐA NĂNG & HÀM CAO CẤP --- */
bool Laser_SF20_Parse_DMABuffer(Laser_Handle_t *handle, const uint8_t *buf, uint16_t length, LaserObject_t *out_data);
void Laser_SF20_Send_Handshake(Laser_Handle_t *handle);
void Laser_SF20_Set_StreamMode(Laser_Handle_t *handle, uint32_t stream_mode);

void Laser_SF20_Set_DistanceOutput(Laser_Handle_t *handle, uint32_t mask);
void Laser_SF20_Set_Filter_Mode(Laser_Handle_t *handle, SF20_CmdID_t filterMode, uint32_t filterSize, SF20_FilterState_t state);

#ifdef __cplusplus
}
#endif

#endif // LASER_DRIVER_H
