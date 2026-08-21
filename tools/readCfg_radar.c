#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

int main() {
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;
    char seen_ids[2048] = {0};

    // Khởi tạo socket RAW cho CAN
    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Tiểu Đệ không mở được socket");
        return 1;
    }

    // Chỉ định interface là can0
    strcpy(ifr.ifr_name, "can0");
    ioctl(s, SIOCGIFINDEX, &ifr);

    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    // Bind socket vào can0
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Tiểu Đệ không bind được vào can0");
        return 1;
    }

    printf("Tiểu Đệ đang lắng nghe dữ liệu MR72 trên can0 (Nhấn Ctrl+C để thoát)...\n");

    // Vòng lặp đọc dữ liệu vắt kiệt hiệu suất
    while (1) {
        int nbytes = read(s, &frame, sizeof(struct can_frame));
        if (nbytes < 0) {
            perror("Lỗi đọc dữ liệu");
            break;
        }

        // Tự động phát hiện và in các CAN ID hoạt động trên bus để chẩn đoán
        if (frame.can_id < 2048) {
            if (!seen_ids[frame.can_id]) {
                seen_ids[frame.can_id] = 1;
                printf("[Chẩn đoán] Phát hiện CAN ID hoạt động trên bus: 0x%03X (Độ dài: %d byte)\n", frame.can_id, frame.can_dlc);
            }
        }

        // Lọc và xử lý bản tin trạng thái (RadarState: 0x201, 0x211, 0x221, ..., 0x271)
        if ((frame.can_id & 0xFF0F) == 0x0201) {
            int current_radar_id = (frame.can_id - 0x201) / 0x10;
            printf("\n[0x%03X] Nhận được cấu hình (RadarState) từ Radar ID %d\n", frame.can_id, current_radar_id);
            
            // 1. NVMReadStatus: Start bit 60, dài 1 bit -> Byte 7, bit 4
            int nvm_read_status = (frame.data[7] >> 4) & 0x01;
            printf("  -> NVM Read Status (Đọc từ NVM): %s (0x%X)\n", 
                   nvm_read_status ? "Successful (Thành công)" : "failed (Thất bại)", nvm_read_status);

            // 2. NVMWriteStatus: Start bit 7, dài 1 bit -> Byte 0, bit 7
            int nvm_write_status = (frame.data[0] >> 7) & 0x01;
            printf("  -> NVM Write Status (Ghi vào NVM): %s (0x%X)\n", 
                   nvm_write_status ? "Successful (Thành công)" : "failed (Thất bại)", nvm_write_status);

            // 3. MaxDistanceCfg: Start bit 22, dài 10 bit -> Byte 1 và Byte 2, Res = 2 mét
            int max_distance = ((frame.data[1] << 2) | (frame.data[2] >> 6)) * 2;
            printf("  -> Max Distance Cfg (Khoảng cách tối đa): %d mét\n", max_distance);

            // 4. SensorID: Start bit 32, dài 3 bit -> Gộp Byte 3 bits 7-6 và Byte 4 bit 0
            int sensor_id = ((frame.data[3] & 0xC0) >> 5) | (frame.data[4] & 0x01);
            printf("  -> Sensor ID (ID của Radar): %d\n", sensor_id);

            // 5. SortIndex: Start bit 36, dài 3 bit -> Byte 4 bits 4-2
            int sort_index = (frame.data[4] >> 2) & 0x07;
            printf("  -> Sort Index (Sắp xếp mục tiêu): ");
            switch(sort_index) {
                case 0: printf("no sorting (Không sắp xếp) (0x0)\n"); break;
                case 1: printf("sorted by range (Sắp xếp theo khoảng cách) (0x1)\n"); break;
                case 2: printf("sorted by RCS (Sắp xếp theo RCS) (0x2)\n"); break;
                default: printf("Unknown (Không xác định) (0x%X)\n", sort_index); break;
            }

            // 6. RadarPowerCfg: Start bit 39, dài 3 bit -> Byte 4 bits 7-5
            int power_cfg = (frame.data[4] >> 5) & 0x07;
            printf("  -> Radar Power Cfg (Công suất phát radar): ");
            switch(power_cfg) {
                case 0: printf("Standard (Chuẩn) (0x0)\n"); break;
                case 1: printf("-3dB Tx Gain (0x1)\n"); break;
                case 2: printf("-6dB Tx Gain (0x2)\n"); break;
                case 3: printf("-9dB Tx Gain (0x3)\n"); break;
                default: printf("Unknown (Không xác định) (0x%X)\n", power_cfg); break;
            }

            // 7. OutputTypeCfg: Start bit 42, dài 2 bit -> Byte 5 bits 2-1
            int output_type = (frame.data[5] >> 1) & 0x03;
            printf("  -> Output Type Cfg (Kiểu dữ liệu đầu ra): ");
            switch(output_type) {
                case 0: printf("none (Không xuất) (0x0)\n"); break;
                case 1: printf("Objects (Vật thể) (0x1)\n"); break;
                case 2: printf("Clusters (Cụm điểm) (0x2)\n"); break;
                default: printf("Unknown (Không xác định) (0x%X)\n", output_type); break;
            }

            // 8. RCS_threshold: Start bit 58, dài 3 bit -> Byte 7 bits 2-0
            int rcs_threshold = frame.data[7] & 0x07;
            printf("  -> RCS Threshold (Ngưỡng RCS độ nhạy): ");
            switch(rcs_threshold) {
                case 0: printf("Standard (Chuẩn) (0x0)\n"); break;
                case 1: printf("high sensitivity (Độ nhạy cao) (0x1)\n"); break;
                default: printf("Unknown (Không xác định) (0x%X)\n", rcs_threshold); break;
            }
        }
        
        // Lọc và xử lý bản tin số lượng mục tiêu (Base ID 0x60A)
        else if ((frame.can_id & 0xFF0F) == 0x060A) {
            int current_radar_id = (frame.can_id - 0x60A) / 0x10;
            // Objects_NofObjects (Độ dài 8 bit) bắt đầu ở Byte 0
            printf("\n[0x%03X] Frame mới từ Radar ID %d - Số lượng Point phát hiện được: %d\n", 
                   frame.can_id, current_radar_id, frame.data[0]);
        }
    }

    close(s);
    return 0;
}