#include "laser_driver.h"
#include <stddef.h>

/**
 * @brief  Thuật toán CRC-16-CCITT (đa thức 0x1021) chuẩn của SF20/C
 * @param  Data: Mảng byte cần tính (từ Start byte đến hết Payload)
 * @param  Size: Chiều dài mảng cần tính
 * @return Giá trị Checksum CRC 16-bit
 */
static uint16_t _Laser_SF20_CreateCRC(const uint8_t *Data, uint16_t Size)
{
    uint16_t crc = 0;
    for (uint32_t i = 0; i < Size; ++i)
    {
        uint16_t code = crc >> 8;
        code ^= Data[i];
        code ^= code >> 4;
        crc = (uint16_t)(crc << 8);
        crc ^= code;
        code = (uint16_t)(code << 5);
        crc ^= code;
        code = (uint16_t)(code << 7);
        crc ^= code;
    }
    return crc;
}

/* ============================================================================
   CORE APIs (Low-Level)
   ============================================================================ */

void Laser_SF20_Init(Laser_Handle_t *handle, const Laser_Config_t *config) {
    if (handle != NULL && config != NULL) {
        handle->config = *config;
        handle->is_initialized = true;
    }
}

/* ============================================================================
   HÀM GENERIC GIAO TIẾP ĐỌC / GHI ĐA NĂNG CHO BẤT KỲ REGISTER NÀO CỦA SF20
   ============================================================================ */

/**
 * @brief  Hàm generic gửi lệnh ĐỌC (Read Command) cho bất kỳ Command ID nào
 *         - Frame = 6 bytes: Start(1) + Flags(2) + CmdID(1) + CRC(2)
 *         - Flags = 0x0040 (Bit 0 Read = 0, Payload len = 1)
 */
void Laser_SF20_Read_Cmd(Laser_Handle_t *handle, SF20_CmdID_t cmd_id) {
    if (handle == NULL || handle->config.transmit_cb == NULL) return;

    uint8_t packet[6];

    packet[0] = SF20_START_BYTE;
    packet[1] = 0x40;            // Flags Low (Read=0, PayloadLen=1)
    packet[2] = 0x00;            // Flags High
    packet[3] = (uint8_t)cmd_id; // Command ID

    uint16_t crc = _Laser_SF20_CreateCRC(packet, 4);
    packet[4] = (uint8_t)(crc & 0xFF);
    packet[5] = (uint8_t)((crc >> 8) & 0xFF);

    handle->config.transmit_cb(packet, sizeof(packet));
}

/**
 * @brief  Hàm generic gửi lệnh GHI (Write Command) dạng 1 byte dữ liệu (uint8_t) cho bất kỳ Register nào
 *         - Frame = 7 bytes: Start(1) + Flags(2) + CmdID(1) + Data(1) + CRC(2)
 *         - Flags = 0x0081 (Bit 0 Write = 1, Payload len = 2)
 */
void Laser_SF20_Write_UInt8(Laser_Handle_t *handle, SF20_CmdID_t cmd_id, uint8_t value) {
    if (handle == NULL || handle->config.transmit_cb == NULL) return;

    uint8_t packet[7];

    packet[0] = SF20_START_BYTE;
    packet[1] = 0x81;            // Flags Low (Write=1, PayloadLen=2)
    packet[2] = 0x00;            // Flags High
    packet[3] = (uint8_t)cmd_id; // Command ID
    packet[4] = value;           // Data

    uint16_t crc = _Laser_SF20_CreateCRC(packet, 5);
    packet[5] = (uint8_t)(crc & 0xFF);
    packet[6] = (uint8_t)((crc >> 8) & 0xFF);

    handle->config.transmit_cb(packet, sizeof(packet));
}

/**
 * @brief  Hàm generic gửi lệnh GHI (Write Command) dạng 4 bytes dữ liệu (uint32_t / int32_t) cho bất kỳ Register nào
 *         - Frame = 10 bytes: Start(1) + Flags(2) + CmdID(1) + Data(4) + CRC(2)
 *         - Flags = 0x0141 (Bit 0 Write = 1, Payload len = 5)
 */
void Laser_SF20_Write_UInt32(Laser_Handle_t *handle, SF20_CmdID_t cmd_id, uint32_t value) {
    if (handle == NULL || handle->config.transmit_cb == NULL) return;

    uint8_t packet[10];

    packet[0] = SF20_START_BYTE;
    packet[1] = 0x41;            // Flags Low (Write=1, PayloadLen=5)
    packet[2] = 0x01;            // Flags High
    packet[3] = (uint8_t)cmd_id; // Command ID

    /* 4 bytes Little-Endian */
    packet[4] = (uint8_t)(value & 0xFF);
    packet[5] = (uint8_t)((value >> 8) & 0xFF);
    packet[6] = (uint8_t)((value >> 16) & 0xFF);
    packet[7] = (uint8_t)((value >> 24) & 0xFF);

    uint16_t crc = _Laser_SF20_CreateCRC(packet, 8);
    packet[8] = (uint8_t)(crc & 0xFF);
    packet[9] = (uint8_t)((crc >> 8) & 0xFF);

    handle->config.transmit_cb(packet, sizeof(packet));
}

/* ============================================================================
   BỘ PARSER DỮ LIỆU ĐA NĂNG (GENERIC PARSER FOR ANY DISTANCE PACKET / PAYLOAD)
   ============================================================================ */

/**
 * @brief  Quét và trích xuất dữ liệu SF20 đa năng từ bộ đệm DMA
 *         - Tự động tính toán tổng chiều dài gói tin dựa theo Payload Length.
 *         - Hỗ trợ cả bản tin khoảng cách mm (Command 45) và cm (Command 44).
 *         - Hỗ trợ gói tin chứa 1 hoặc nhiều trường dữ liệu trả về.
 * @param  buf: Con trỏ mảng byte DMA thu được
 * @param  length: Số byte thực tế nhận được
 * @param  out_data: Con trỏ struct lưu kết quả đầu ra
 * @return true nếu parse thành công gói tin hợp lệ
 */
bool Laser_SF20_Parse_DMABuffer(Laser_Handle_t *handle, const uint8_t *buf, uint16_t length, LaserObject_t *out_data)
{
    if (buf == NULL || out_data == NULL || length < 10)
        return false;

    // Quét ngược từ cuối buffer để lấy gói tin mới nhất
    for (int32_t i = (int32_t)length - 10; i >= 0; i--)
    {
        // 1. Tìm Start byte (0xAA)
        if (buf[i] != SF20_START_BYTE)
            continue;

        // 2. Trích xuất Payload length từ Flags (Bit 6-15)
        uint16_t flags = (uint16_t)buf[i + 1] | ((uint16_t)buf[i + 2] << 8);
        uint16_t payload_len = (flags >> 6) & 0x03FF;

        // Payload phải ít nhất 1 byte (CmdID), kiểm tra chi tiết tùy command bên dưới
        uint16_t total_packet_len = payload_len + 5; // Start(1) + Flags(2) + Payload + CRC(2)
        if (payload_len < 1 || ((uint16_t)i + total_packet_len) > length)
            continue;

        uint8_t cmd_id = buf[i + 3];

        // 3. Kiểm tra CRC-16-CCITT động dựa theo chiều dài gói tin
        uint16_t crc_calc_len = payload_len + 3; // Start + Flags + Payload
        uint16_t received_crc = (uint16_t)buf[i + crc_calc_len] | ((uint16_t)buf[i + crc_calc_len + 1] << 8);
        if (received_crc != _Laser_SF20_CreateCRC(&buf[i], crc_calc_len))
            continue;

        if (cmd_id == SF20_CMD_DIST_MM)
        {
            // Bản tin 45 (mm) cần ít nhất 5 bytes Payload (1B Cmd + 4B Int32)
            if (payload_len < 5) continue;

            int32_t dist_raw = (int32_t)((uint32_t)buf[i + 4] |
                                         ((uint32_t)buf[i + 5] << 8) |
                                         ((uint32_t)buf[i + 6] << 16) |
                                         ((uint32_t)buf[i + 7] << 24));

            out_data->distance_mm         = dist_raw;
            out_data->distance_m          = (float)dist_raw * 0.001f;
            out_data->signal_strength_raw = 0;
            out_data->command_id          = cmd_id;
            out_data->is_valid            = true;
            return true; // Parse thành công gói tin mới nhất
        }
        else if (cmd_id == SF20_CMD_DIST_CM)
        {
            if (payload_len >= 5)
            {
                // Multi-field: bit 4 (First Strength) + bit 7 (Last Filtered)
                // Thứ tự theo bit index tăng dần:
                //   Trường 1 (bit 4): First Strength (int16_t, offset [i+4..i+5])
                //   Trường 2 (bit 7): Last Filtered distance (int16_t, offset [i+6..i+7])
                int16_t strength_raw = (int16_t)((uint16_t)buf[i + 4] |
                                              ((uint16_t)buf[i + 5] << 8));

                int16_t dist_raw = (int16_t)((uint16_t)buf[i + 6] |
                                              ((uint16_t)buf[i + 7] << 8));

                out_data->distance_cm         = dist_raw;
                out_data->distance_m          = (float)dist_raw * 0.01f;
                out_data->signal_strength_raw = strength_raw;
                out_data->command_id          = cmd_id;
                out_data->is_valid            = true;
                return true; // Parse thành công gói tin mới nhất
            }
            else if (payload_len >= 3)
            {
                // Single-field fallback (tương thích ngược)
                int16_t dist_raw = (int16_t)((uint16_t)buf[i + 4] |
                                              ((uint16_t)buf[i + 5] << 8));

                out_data->distance_cm         = dist_raw;
                out_data->distance_m          = (float)dist_raw * 0.01f;
                out_data->signal_strength_raw = 0;
                out_data->command_id          = cmd_id;
                out_data->is_valid            = true;
                return true; // Parse thành công gói tin mới nhất
            }
            continue;
        }
    }

    return false;
}

/* ============================================================================
   HÀM CẤU HÌNH TIỆN ÍCH CAO CẤP (HIGH-LEVEL HELPER FUNCTIONS)
   ============================================================================ */

void Laser_SF20_Send_Handshake(Laser_Handle_t *handle) {
    Laser_SF20_Read_Cmd(handle, SF20_CMD_PROD_NAME);
}

void Laser_SF20_Set_StreamMode(Laser_Handle_t *handle, uint32_t stream_mode) {

    Laser_SF20_Write_UInt32(handle, SF20_CMD_STREAM, stream_mode);
}

void Laser_SF20_Set_DistanceOutput(Laser_Handle_t *handle, uint32_t mask) {
    Laser_SF20_Write_UInt32(handle, SF20_CMD_DIST_OUT, mask);
}

void Laser_SF20_Set_Filter_Mode(Laser_Handle_t *handle, SF20_CmdID_t filterMode, uint32_t filterSize, SF20_FilterState_t state) {
    Laser_SF20_Write_UInt8(handle, filterMode, (uint8_t)state);

    // Nếu là filter Median
    if (filterMode == SF20_CMD_MEDIAN_ENABLE) {
        if (filterSize < 3)  filterSize = 3;
        if (filterSize > 32) filterSize = 32;
        Laser_SF20_Write_UInt32(handle, SF20_CMD_MEDIAN_SIZE, filterSize);
    } else if (filterMode == SF20_CMD_SMOOTHING_ENABLE) {
        if (filterSize < 1)  filterSize = 1;
        if (filterSize > 99) filterSize = 99;
        Laser_SF20_Write_UInt32(handle, SF20_CMD_SMOOTHING_FACTOR, filterSize);
    } else {
        if (filterSize < 2)  filterSize = 2;
        if (filterSize > 32) filterSize = 32;
        Laser_SF20_Write_UInt32(handle, SF20_CMD_ROLLING_SIZE, filterSize);
    }
}


